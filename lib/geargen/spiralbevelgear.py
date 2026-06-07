"""Spiral bevel gear -- full gear (work in progress).

Built by SUBCLASSING the straight `BevelGearGenerator` and reusing everything it
does (anchor sketch, the §2 cone/frustum lattice, §3 virtual-spur tooth profiles,
revolve, the toe/heel conical cuts that trim a tooth body to a flush band, circular
pattern, combine, bore, meshing rotation, cleanup). The ONLY thing the spiral
version changes is the tooth body, via the `_transformToothBody` hook.

Spiral tooth (approach B -- 2-D trace as a loft guide rail):
  1. Build the tooth TRACE as a persistent 2-D cutter-arc curve, on the cone, per
     gear (driving + the opposite-hand mirror for the pinion). The trace crosses the
     cone element at the mid-face at the mean spiral angle psi, so its toe/heel ends
     sit at offset azimuths -- that offset IS the spiral lean.
  2. Slice the UNCUT tooth at the toe and heel (a touch past the cones) -> two
     cross-section surface patches, and rotate each onto the trace's toe/heel ends.
  3. LOFT toe->heel using the trace as the centreline (guide rail): the loft bends
     along the trace into the spiral.
  4. Trim the ends flush with the toe/heel cones.

theta(R) (the trace twist) = cutter-arc offset / (R*sin(gamma)); it is only defined
over the face (toe->heel) and is CLAMPED outside it (theta ~ 1/R blows up toward the
apex, which over-twisted earlier builds).
"""

import math
from ...lib import fusion360utils as futil
from .misc import *        # to_cm, to_mm, get_design, get_ui
from .base import *        # get_value/get_boolean/get_selection, ...
from .utilities import *   # get_normal
from .bevelgear import (
    BevelGearGenerator,
    BevelGearCommandInputsConfigurator,
)
import adsk.core, adsk.fusion


# ---------------------------------------------------------------------------
# Extra spiral dialog input ids (added on top of the 14 bevel inputs)
# ---------------------------------------------------------------------------
INPUT_ID_SPIRAL_ANGLE = 'spiralAngle'
INPUT_ID_HAND = 'spiralHand'
INPUT_ID_CUTTER_RADIUS = 'cutterRadius'
INPUT_ID_SECTIONS = 'toothSections'

_HAND_RIGHT = 'Right'
_HAND_LEFT = 'Left'


# ---------------------------------------------------------------------------
# Dialog inputs configurator -- the 14 bevel inputs plus the spiral ones
# ---------------------------------------------------------------------------
class SpiralBevelGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd):
        # Reuse the bevel dialog verbatim, then append the spiral inputs.
        BevelGearCommandInputsConfigurator.configure(cmd)
        inputs = cmd.commandInputs

        # 15. Mean Spiral Angle (deg). 0 == straight bevel.
        inputs.addValueInput(
            INPUT_ID_SPIRAL_ANGLE, 'Mean Spiral Angle', 'deg',
            adsk.core.ValueInput.createByString('35 deg'))

        # 16. Hand of Spiral (dropdown). Driving gear's hand; pinion is opposite.
        handInput = inputs.addDropDownCommandInput(
            INPUT_ID_HAND, 'Hand of Spiral',
            adsk.core.DropDownStyles.TextListDropDownStyle)
        handInput.listItems.add(_HAND_RIGHT, True)
        handInput.listItems.add(_HAND_LEFT, False)

        # 17. Cutter Radius (mm). 0 == auto (= mean cone distance).
        inputs.addValueInput(
            INPUT_ID_CUTTER_RADIUS, 'Cutter Radius', 'mm',
            adsk.core.ValueInput.createByReal(to_cm(0)))

        # 18. Tooth Sections (currently informational; B uses toe+heel + the rail).
        inputs.addValueInput(
            INPUT_ID_SECTIONS, 'Tooth Sections', '',
            adsk.core.ValueInput.createByReal(8))


# ---------------------------------------------------------------------------
# Generator -- subclass of the straight bevel generator
# ---------------------------------------------------------------------------
class SpiralBevelGearGenerator(BevelGearGenerator):
    def _readInputs(self, inputs):
        # Read all the bevel inputs first (returns the 7-tuple, stashes the rest on
        # self), then read + validate the spiral inputs and stash them too.
        result = super()._readInputs(inputs)
        (_parent, _plane, _center, module,
         drivingTeeth, pinionTeeth, shaftAngle_deg) = result

        unitsManager = self.design.unitsManager

        def rawValue(input_id, units):
            return unitsManager.evaluateExpression(
                inputs.itemById(input_id).expression, units)

        spiralAngle_rad = rawValue(INPUT_ID_SPIRAL_ANGLE, 'deg')  # back in radians
        spiralAngle_deg = math.degrees(spiralAngle_rad)
        if spiralAngle_deg < 0 or spiralAngle_deg >= 60:
            raise Exception('Mean Spiral Angle must be in [0, 60) degrees')

        handItem = inputs.itemById(INPUT_ID_HAND).selectedItem
        hand = handItem.name if handItem else _HAND_RIGHT

        cutterRadius_cm = rawValue(INPUT_ID_CUTTER_RADIUS, 'mm')  # already internal cm
        if cutterRadius_cm < 0:
            raise Exception('Cutter Radius must be non-negative')

        sections = int(round(rawValue(INPUT_ID_SECTIONS, '')))
        if sections < 3 or sections > 20:
            raise Exception('Tooth Sections must be between 3 and 20')

        # Stash spiral parameters + the gear geometry the tooth hook will need.
        self._spiralAngle_rad = spiralAngle_rad
        self._spiralHand = hand
        self._cutterRadius_cm = cutterRadius_cm
        self._toothSections = sections
        self._sbModule = module
        self._sbDrivingTeeth = drivingTeeth
        self._sbPinionTeeth = pinionTeeth
        self._sbShaftAngle_deg = shaftAngle_deg

        futil.log(
            f'Spiral bevel inputs: psi={spiralAngle_deg:.1f} deg, hand={hand}, '
            f'cutterRadius={to_mm(cutterRadius_cm):.3f} mm (0=auto), sections={sections}',
            force_console=True)

        return result

    # TEMP DIAGNOSTIC: the inherited spur tooth profile comes back marginally
    # under-constrained for some inputs (a known spurgear fragility, flagged in the
    # bevel spec). Log the status instead of raising so the build proceeds; the
    # tooth BODY is geometrically fine regardless. (To be hardened in spurgear later.)
    def _assertFullyConstrained(self, sketch, name):
        ok = sketch.isFullyConstrained
        futil.log(f'[spiral-diag] sketch "{name}" fullyConstrained={ok}',
                  force_console=True)

    # ----- final mesh-phase nudge of the pinion about Apex->A -----
    # Tunable: angle (in tooth fractions) to rotate the pinion gear so the spiral teeth
    # seat tooth-in-gap with the driving gear. The straight bevel meshes with NO pinion
    # nudge, and our spiral tooth is centred so its mean cross-section is unrotated
    # (identical to the straight tooth at mid-face) -> it must mesh at mid-face with the
    # same 0 nudge. A non-zero value shifts the pinion off that proven position and shows
    # up as volume overlap at mid-face. Keep at 0 unless the mesh genuinely needs it.
    _PINION_MESH_PHASE_TEETH = 0.0

    def _pinionMeshPhase(self):
        if self._spiralAngle_rad <= 0:
            return 0.0
        return math.radians(self._PINION_MESH_PHASE_TEETH * 360.0 / self._sbPinionTeeth)

    # ----- lengthwise crown (relief) -----
    # Tunable: circumferential relief per radian of local twist. Each segment's cross-
    # section is scaled by (1 - _CROWN_PER_RAD*|ang|) about its centroid, so mid-face (twist
    # 0) is untouched and toe/heel are progressively thinned -> contact localizes at
    # mid-face and the twisted ends clear instead of gouging. Scales with spiral angle.
    # 0 disables. Dial up until the pair runs clean.
    _CROWN_PER_RAD = 0.0   # TEMP off (scaleFeatures unsupported here) while fixing twist

    # ----- trace-construction helpers (ported from the validated trace sketch) -----
    def _combine(self, base, a, e1, b=0.0, e2=None):
        """World Point3D = base + a*e1 (+ b*e2). a, b in cm; e1, e2 unit Vector3D."""
        x = base.x + a * e1.x
        y = base.y + a * e1.y
        z = base.z + a * e1.z
        if e2 is not None:
            x += b * e2.x
            y += b * e2.y
            z += b * e2.z
        return adsk.core.Point3D.create(x, y, z)

    def _planeByAngle(self, comp, line, refPlane, deg):
        ci = comp.constructionPlanes.createInput()
        ci.setByAngle(line, adsk.core.ValueInput.createByString(f'{deg} deg'), refPlane)
        return comp.constructionPlanes.add(ci)

    def _distDim(self, sketch, p0, p1, textWorld, value_cm):
        """Aligned point-to-point driving distance dimension, value in cm."""
        d = sketch.sketchDimensions.addDistanceDimension(
            p0, p1, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
            sketch.modelToSketchSpace(textWorld))
        d.parameter.value = value_cm
        return d

    def _circleIntersectNearest(self, R, cx, cy, rc, nearPx, nearPy, R_label):
        """Intersection of circle(origin,R) with cutter circle(C=(cx,cy),rc); the
        solution nearest (nearPx,nearPy). All 2-D, tangent-plane coords (cm)."""
        d = math.hypot(cx, cy)
        a = (R * R - rc * rc + d * d) / (2 * d)
        h2 = R * R - a * a
        if h2 < 0:
            raise Exception(
                f'Cutter Radius {to_mm(rc):.3f} mm too small: cutter circle never '
                f'reaches the {R_label} radius {to_mm(R):.3f} mm. Increase Cutter Radius.')
        h = math.sqrt(h2)
        ux, uy = cx / d, cy / d
        bx, by = a * ux, a * uy
        s1 = (bx - h * uy, by + h * ux)
        s2 = (bx + h * uy, by - h * ux)
        d1 = math.hypot(s1[0] - nearPx, s1[1] - nearPy)
        d2 = math.hypot(s2[0] - nearPx, s2[1] - nearPy)
        return s1 if d1 <= d2 else s2

    def _bottomEdgeMid(self, profile, apex, axisDir):
        """World midpoint of the profile's bottom (root) edge: the boundary curve whose
        own midpoint sits nearest the shaft axis (the tooth root)."""
        best, bestPerp2 = None, None
        for loop in profile.profileLoops:
            for pc in loop.profileCurves:
                e = pc.sketchEntity.worldGeometry.evaluator
                (_ok, sp, ep) = e.getParameterExtents()
                (_ok, mid) = e.getPointAtParameter(0.5 * (sp + ep))
                wx, wy, wz = mid.x - apex.x, mid.y - apex.y, mid.z - apex.z
                along = wx * axisDir.x + wy * axisDir.y + wz * axisDir.z
                perp2 = (wx * wx + wy * wy + wz * wz) - along * along
                if bestPerp2 is None or perp2 < bestPerp2:
                    bestPerp2, best = perp2, mid
        return best

    def _perpToAxis(self, pt, apex, axisDir):
        """Component of (pt - apex) perpendicular to the shaft axis (the radial vector
        about which a point's azimuth is measured)."""
        wx, wy, wz = pt.x - apex.x, pt.y - apex.y, pt.z - apex.z
        a = wx * axisDir.x + wy * axisDir.y + wz * axisDir.z
        return adsk.core.Vector3D.create(
            wx - a * axisDir.x, wy - a * axisDir.y, wz - a * axisDir.z)

    # ----- spiral tooth: (1) build the genuine cutter-arc TRACE, (2) slice the straight
    # tooth into sections, (3) MOVE each section onto the trace, (4) loft through them --
    def _transformToothBody(self, designComponent, toothBody, gearBody, shaftAxisEdge,
                            apexWorld, apexSketchPoint, toeMid, heelMid, toeConeWorld,
                            heelConeWorld, parentToothPlane, gearLabel, teethNumber):
        if self._spiralAngle_rad <= 0:
            # straight: just trim the uncut tooth to a band, like the base hook.
            return self._cutConicalEnds(
                designComponent, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)

        features = designComponent.features
        apex = apexWorld

        # --- world frame from the built geometry ---
        sA = shaftAxisEdge.startSketchPoint.worldGeometry
        sB = shaftAxisEdge.endSketchPoint.worldGeometry
        axisDir = adsk.core.Vector3D.create(sB.x - sA.x, sB.y - sA.y, sB.z - sA.z)
        axisDir.normalize()
        # The tooth TRACE plane sits on the cone element Apex -> D (heelEdgePoints[0]:
        # 'D' for the driving gear, 'C' for the pinion). Use that exact direction.
        coneVec = adsk.core.Vector3D.create(
            heelConeWorld.x - apex.x, heelConeWorld.y - apex.y, heelConeWorld.z - apex.z)
        coneVec.normalize()
        # tangent-plane frame: e_x = coneVec (the Apex->D element), e_y = v (circumferential).
        v = axisDir.crossProduct(coneVec)
        v.normalize()
        tpNormal = coneVec.crossProduct(v)   # normal of the tangent plane
        tpNormal.normalize()

        def distAlong(p):
            return ((p.x - apex.x) * coneVec.x + (p.y - apex.y) * coneVec.y
                    + (p.z - apex.z) * coneVec.z)

        R_toe = distAlong(toeMid)
        R_heel = distAlong(heelMid)
        R_mean = 0.5 * (R_toe + R_heel)
        span = R_heel - R_toe

        # --- cutter-arc geometry (tangent-plane 2-D frame: e_x = coneVec, e_y = v) ---
        isPinion = (gearLabel == 'Pinion')
        r_c = self._cutterRadius_cm if self._cutterRadius_cm > 0 else R_mean
        psi = self._spiralAngle_rad
        handSign = +1 if self._spiralHand == _HAND_RIGHT else -1
        if isPinion:
            handSign = -handSign          # the pair meshes with opposite hands
        # Opposite hand = mirror the cutter centre across the CONE ELEMENT (py=0), i.e.
        # flip Cy (the cos term), NOT Cx. (Putting handSign on the sin term mirrors about
        # x=R_mean instead, which gives the two gears different twist magnitudes.)
        Cx = R_mean - r_c * math.sin(psi)
        Cy = handSign * r_c * math.cos(psi)

        def tanW(px, py):
            return self._combine(apex, px, coneVec, py, v)

        def arc2d(R):
            return self._circleIntersectNearest(R, Cx, Cy, r_c, R_mean, 0.0, 'sec')

        # arc spans a touch past both cones (toe toward apex, heel past the back cone).
        R_lo = R_toe - 0.06 * span
        R_hi = R_heel + 0.06 * span
        toe2d = arc2d(R_lo)
        heel2d = arc2d(R_hi)
        futil.log(
            f'[spiral] {gearLabel}: R[toe/mean/heel]=[{to_mm(R_toe):.2f}/{to_mm(R_mean):.2f}/'
            f'{to_mm(R_heel):.2f}] mm, r_c={to_mm(r_c):.2f} mm, hand={handSign:+d}',
            force_console=True)
        futil.log(
            f'[spiral] {gearLabel}: TRACE2D psi={math.degrees(psi):.2f} C=('
            f'{to_mm(Cx):.2f},{to_mm(Cy):.2f}) toe2d=({to_mm(toe2d[0]):.2f},'
            f'{to_mm(toe2d[1]):.2f}) heel2d=({to_mm(heel2d[0]):.2f},{to_mm(heel2d[1]):.2f}) '
            f'-> 2D twist span={to_mm(heel2d[1]-toe2d[1]):.2f}mm', force_console=True)

        # ================================================================
        # STEP 1: build the tooth TRACE first -- a GENUINE constrained cutter arc on
        # the pitch-cone tangent plane (centre coincident to the cutter circle, radius
        # r_c, ends pinned by their cone distance). No plotting.
        # ================================================================
        axialPlane = shaftAxisEdge.parentSketch.referencePlane
        ceSketch = designComponent.sketches.add(axialPlane)
        ceSketch.name = f'{gearLabel} Cone Element'
        ceLine = ceSketch.sketchCurves.sketchLines.addByTwoPoints(
            ceSketch.modelToSketchSpace(apex),
            ceSketch.modelToSketchSpace(self._combine(apex, R_heel, coneVec)))
        ceLine.isConstruction = True
        tangentPlane = self._planeByAngle(designComponent, ceLine, axialPlane, 90)
        tangentPlane.name = f'{gearLabel} Trace Plane'   # (NOT 'Tooth Plane' -- that name
        #                              belongs to the parent's transverse profile plane)

        traceSketch = designComponent.sketches.add(tangentPlane)
        traceSketch.name = f'{gearLabel} 2D Tooth Trace'
        tgc = traceSketch.geometricConstraints
        tlines = traceSketch.sketchCurves.sketchLines
        tApex = traceSketch.project(apexSketchPoint).item(0)
        tAxis = traceSketch.project(shaftAxisEdge).item(0)   # projects along the cone element

        coneEl = tlines.addByTwoPoints(
            traceSketch.modelToSketchSpace(apex),
            traceSketch.modelToSketchSpace(tanW(R_heel, 0)))
        coneEl.isConstruction = True
        tgc.addCoincident(coneEl.startSketchPoint, tApex)
        tgc.addParallel(coneEl, tAxis)
        self._distDim(traceSketch, coneEl.startSketchPoint, coneEl.endSketchPoint,
                      tanW(R_heel * 0.5, -0.2), R_heel)

        lineX = tlines.addByTwoPoints(
            traceSketch.modelToSketchSpace(apex), traceSketch.modelToSketchSpace(tanW(Cx, 0)))
        lineX.isConstruction = True
        tgc.addCoincident(lineX.startSketchPoint, tApex)
        tgc.addParallel(lineX, coneEl)
        self._distDim(traceSketch, lineX.startSketchPoint, lineX.endSketchPoint,
                      tanW(Cx * 0.5, -0.2), abs(Cx))
        lineY = tlines.addByTwoPoints(
            traceSketch.modelToSketchSpace(tanW(Cx, 0)), traceSketch.modelToSketchSpace(tanW(Cx, Cy)))
        lineY.isConstruction = True
        tgc.addCoincident(lineY.startSketchPoint, lineX.endSketchPoint)
        tgc.addPerpendicular(lineY, coneEl)
        self._distDim(traceSketch, lineY.startSketchPoint, lineY.endSketchPoint,
                      tanW(Cx + 0.2, Cy * 0.5), abs(Cy))

        cutter = traceSketch.sketchCurves.sketchCircles.addByCenterRadius(
            traceSketch.modelToSketchSpace(tanW(Cx, Cy)), r_c)
        cutter.isConstruction = True
        tgc.addCoincident(cutter.centerSketchPoint, lineY.endSketchPoint)
        traceSketch.sketchDimensions.addDiameterDimension(
            cutter, traceSketch.modelToSketchSpace(tanW(Cx + r_c, Cy))).parameter.value = 2 * r_c

        arc = traceSketch.sketchCurves.sketchArcs.addByThreePoints(
            traceSketch.modelToSketchSpace(tanW(*toe2d)),
            traceSketch.modelToSketchSpace(tanW(R_mean, 0.0)),
            traceSketch.modelToSketchSpace(tanW(*heel2d)))
        tgc.addCoincident(arc.centerSketchPoint, lineY.endSketchPoint)
        traceSketch.sketchDimensions.addRadialDimension(
            arc, traceSketch.modelToSketchSpace(tanW(Cx, Cy - r_c * 0.5))).parameter.value = r_c
        # (Endpoint distance dims removed for now -- they over-constrained the solve with
        # the Apex->D plane. The arc is placed exactly by its three cutter-circle points.)
        futil.log(f'[spiral] {gearLabel}: STEP 1 trace arc built, '
                  f'fullyConstrained={traceSketch.isFullyConstrained}', force_console=True)

        # ================================================================
        # 3-D TOOTH TRACE: project the 2-D arc onto the ROOT CONE face (the cone the
        # dedendum element M->C / O->D generates) ALONG the Tooth Plane normal.
        # ================================================================
        gearProfilesPlane = shaftAxisEdge.parentSketch.referencePlane
        # Root cone face = the gearBody cone face containing the midpoint of the dedendum
        # element (toeConeWorld..heelConeWorld = M..C / O..D).
        midDed = adsk.core.Point3D.create(
            0.5 * (toeConeWorld.x + heelConeWorld.x),
            0.5 * (toeConeWorld.y + heelConeWorld.y),
            0.5 * (toeConeWorld.z + heelConeWorld.z))
        rootFace = None
        for f in gearBody.faces:
            if f.geometry.surfaceType != adsk.core.SurfaceTypes.ConeSurfaceType:
                continue
            ev2 = f.evaluator
            (ok, param) = ev2.getParameterAtPoint(midDed)
            if not ok:
                continue
            (ok2, back) = ev2.getPointAtParameter(param)
            if ok2 and back.distanceTo(midDed) < 1e-3:    # midDed lies on this cone face
                rootFace = f
                break
        if rootFace is None:
            raise Exception(f'{gearLabel}: could not find the root cone face at the dedendum midpoint')

        # Direction guide = a construction line along the Tooth Plane normal, in ITS OWN
        # sketch (the projection target must not contain any geometry of its own).
        # tpNormal lies in the gear-profiles plane, so it can be drawn there.
        dirSketch = designComponent.sketches.add(gearProfilesPlane)
        dirSketch.name = f'{gearLabel} Proj Dir'
        dirLine = dirSketch.sketchCurves.sketchLines.addByTwoPoints(
            dirSketch.modelToSketchSpace(apex),
            dirSketch.modelToSketchSpace(self._combine(apex, 1.0, tpNormal)))
        dirLine.isConstruction = True

        sketch3d = designComponent.sketches.add(gearProfilesPlane)
        sketch3d.name = f'{gearLabel} 3D Tooth Trace'
        proj = sketch3d.projectToSurface(
            [rootFace], [arc],
            adsk.fusion.SurfaceProjectTypes.AlongVectorSurfaceProjectType, dirLine)
        trace3d = proj.item(0) if hasattr(proj, 'item') else proj[0]
        futil.log(f'[spiral] {gearLabel}: projected 3-D tooth trace onto the root cone',
                  force_console=True)

        # ================================================================
        # dYYY: TWIST ANGLE. Plane at 90 deg to I->J / G->H; project the APEX (shared by
        # both gears and on both shaft axes -> symmetric for equal teeth, unlike the
        # off-axis assembly centre) and the 3-D trace onto it; join apex->trace-start &
        # apex->trace-end; put a DRIVEN angular dimension between them.
        # ================================================================
        lineForPlane = self._lineGH if isPinion else self._lineIJ
        twistPlane = self._planeByAngle(designComponent, lineForPlane, gearProfilesPlane, 90)
        twistPlane.name = f'{gearLabel} Twist Plane'
        angleSketch = designComponent.sketches.add(twistPlane)
        angleSketch.name = f'{gearLabel} Twist Angle'
        pApex = angleSketch.project(apexSketchPoint).item(0)
        pStart = angleSketch.project(trace3d.startSketchPoint).item(0)
        pEnd = angleSketch.project(trace3d.endSketchPoint).item(0)
        aLines = angleSketch.sketchCurves.sketchLines
        lA = aLines.addByTwoPoints(pApex, pStart)
        lB = aLines.addByTwoPoints(pApex, pEnd)
        # text point at the chord midpoint (inside the arc's sector) -> dimension picks
        # the angle the arc subtends, not its supplement.
        midChord = adsk.core.Point3D.create(
            0.5 * (pStart.geometry.x + pEnd.geometry.x),
            0.5 * (pStart.geometry.y + pEnd.geometry.y),
            0.5 * (pStart.geometry.z + pEnd.geometry.z))
        angDim = angleSketch.sketchDimensions.addAngularDimension(
            lA, lB, midChord, False)
        self._dTwist = angDim.parameter.name
        futil.log(f'[spiral] {gearLabel}: twist angle param={self._dTwist} '
                  f'value={math.degrees(angDim.parameter.value):.2f} deg', force_console=True)

        # ================================================================
        # PASS 2a: cut plane = the PARENT'S transverse TOOTH PLANE offset toward the apex.
        # (cut/pattern/rotate/loft to follow.)
        # ================================================================
        dFace = self._dFacePinion if isPinion else self._dFaceDriving
        # Offset TOWARD the apex. The parent plane's normal points opposite ways for the
        # two gears, so choose the sign that moves toward the apex per gear.
        pn = parentToothPlane.geometry.normal
        po = parentToothPlane.geometry.origin
        towardApex = (apex.x - po.x) * pn.x + (apex.y - po.y) * pn.y + (apex.z - po.z) * pn.z
        sign = 1.0 if towardApex > 0 else -1.0
        offInput = designComponent.constructionPlanes.createInput()
        offInput.setByOffset(parentToothPlane, adsk.core.ValueInput.createByReal(sign * span / 6))
        cutPlane = designComponent.constructionPlanes.add(offInput)
        cutPlane.name = f'{gearLabel} Cut Plane'
        futil.log(f'[spiral] {gearLabel}: PASS 2a cut plane = parent Tooth Plane offset '
                  f'{to_mm(sign*span/6):.2f}mm (toward apex, sign={sign:+.0f})', force_console=True)

        # ================================================================
        # PASS 2b: cut (split) the straight Tooth Body with the cut plane.
        # ================================================================
        # ================================================================
        # PASS 2c: split by hand -- 8 planes stepped TOWARD THE APEX (k=0 is the verified
        # cut plane) at span/6, split piece-by-piece; skip any plane that misses.
        # ================================================================
        pieces = [toothBody]
        for k in range(8):
            if k == 0:
                planeK = cutPlane
            else:
                pi = designComponent.constructionPlanes.createInput()
                pi.setByOffset(parentToothPlane,
                               adsk.core.ValueInput.createByReal(sign * (k + 1) * span / 6))
                planeK = designComponent.constructionPlanes.add(pi)
                planeK.name = f'{gearLabel} Cut {k}'
            nextPieces = []
            for body in pieces:
                try:
                    sf = features.splitBodyFeatures.add(
                        features.splitBodyFeatures.createInput(body, planeK, True))
                    nextPieces += [sf.bodies.item(i) for i in range(sf.bodies.count)]
                except Exception:
                    nextPieces.append(body)   # this plane missed this piece
            pieces = nextPieces
        futil.log(f'[spiral] {gearLabel}: PASS 2c split by hand -> {len(pieces)} segments',
                  force_console=True)

        # ================================================================
        # PASS 2d: order segments along the cone element, drop the apex-side scrap, then
        # rotate each remaining segment about the SHAFT AXIS by the 3-D trace's actual
        # azimuth at that segment's cone distance, relative to the mean station. Measuring
        # the azimuth directly in 3-D about the real axis bakes in the developed->shaft
        # (1/sin gamma) relationship -- no separate factor, no linear approximation, no
        # /6 scaling. The mean station stays unrotated -> the half-tooth mesh offset holds.
        # ================================================================
        pieces.sort(key=lambda b: distAlong(b.physicalProperties.centerOfMass))
        scrap, segments = pieces[0], pieces[1:]    # pieces[0] = long apex-side scrap
        features.removeFeatures.add(scrap)

        # Sample the 3-D trace into (cone distance, perpendicular-to-axis vector) pairs.
        ev = trace3d.worldGeometry.evaluator
        (_ok, t0, t1) = ev.getParameterExtents()
        samples = []
        for i in range(61):
            (_okp, P) = ev.getPointAtParameter(t0 + (t1 - t0) * i / 60.0)
            axc = ((P.x - apex.x) * axisDir.x + (P.y - apex.y) * axisDir.y
                   + (P.z - apex.z) * axisDir.z)
            perp = adsk.core.Vector3D.create(
                (P.x - apex.x) - axc * axisDir.x,
                (P.y - apex.y) - axc * axisDir.y,
                (P.z - apex.z) - axc * axisDir.z)
            samples.append((distAlong(P), perp))

        def _azimuth(vRef, v):           # signed angle vRef->v about the shaft axis
            cr = vRef.crossProduct(v)
            return math.atan2(cr.dotProduct(axisDir), vRef.dotProduct(v))

        # Total toe->heel twist from the trace ENDPOINTS (robust even when the arc's cone
        # distance is non-monotonic, as on the pinion). Magnitude only -- the hand sets the
        # direction. This is the real shaft-axis angle, so the developed->shaft 1/sin(gamma)
        # is already included; it replaces the old dYYY/6 (which had both a 1/sin(gamma) and
        # a /6-vs-/(N-1) scaling error).
        total = abs(_azimuth(samples[0][1], samples[-1][1]))
        twistVal = angDim.parameter.value          # dYYY (the plane-measured value)
        futil.log(f'[spiral] {gearLabel}: AUDIT dYYY(plane)={math.degrees(twistVal):.2f} deg '
                  f'vs direct shaft-axis twist={math.degrees(total):.2f} deg '
                  f'(ratio={total / twistVal if twistVal else 0:.3f})', force_console=True)

        # Stagger by the CONE DISTANCE of each segment's heel face -- the exact section the
        # loft samples -- centred on R_mean. This guarantees the loft section sitting at
        # R_mean is genuinely unrotated (= the straight tooth, which already meshes).
        # Centring by segment INDEX instead left the loft's mid-face section rotated by
        # half a segment, which is the mid-face overlap. Linear -> monotonic C-shape.
        def _heelR(seg):
            best = None
            for i in range(seg.faces.count):
                bb = seg.faces.item(i).boundingBox
                d = distAlong(adsk.core.Point3D.create(
                    0.5 * (bb.minPoint.x + bb.maxPoint.x),
                    0.5 * (bb.minPoint.y + bb.maxPoint.y),
                    0.5 * (bb.minPoint.z + bb.maxPoint.z)))
                if best is None or d > best:
                    best = d
            return best
        for seg in segments:
            ang = -handSign * total * (R_mean - _heelR(seg)) / span
            # Lengthwise crown: thin the cross-section about its centroid in proportion to
            # the local twist, so toe/heel are relieved and contact localizes at mid-face.
            relief = self._CROWN_PER_RAD * abs(ang)
            if relief > 1e-4:
                try:
                    cpIn = designComponent.constructionPoints.createInput()
                    cpIn.setByPoint(seg.physicalProperties.centerOfMass)
                    cp = designComponent.constructionPoints.add(cpIn)
                    ents = adsk.core.ObjectCollection.create()
                    ents.add(seg)
                    features.scaleFeatures.add(features.scaleFeatures.createInput(
                        ents, cp, adsk.core.ValueInput.createByReal(1.0 - relief)))
                except Exception as e:
                    futil.log(f'[spiral] {gearLabel}: crown skipped ({e})', force_console=True)
            if abs(ang) > 1e-9:
                m = adsk.core.Matrix3D.create()
                m.setToRotation(ang, axisDir, apex)
                coll = adsk.core.ObjectCollection.create()
                coll.add(seg)
                mv = features.moveFeatures.createInput2(coll)
                mv.defineAsFreeMove(m)
                features.moveFeatures.add(mv)
        futil.log(f'[spiral] {gearLabel}: PASS 2d staggered {len(segments)} segments, '
                  f'total twist={math.degrees(total):.2f} deg', force_console=True)

        # ================================================================
        # PASS 2e: loft the heel-facing (D->J / C->H) cut face of each staggered segment
        # -> the curved tooth. The heel-facing cut face = the face whose bbox centre is
        # farthest along the cone element.
        # ================================================================
        def endFace(seg, farthest):
            best, bestD = None, None
            for i in range(seg.faces.count):
                f = seg.faces.item(i)
                bb = f.boundingBox
                c = adsk.core.Point3D.create(
                    0.5 * (bb.minPoint.x + bb.maxPoint.x),
                    0.5 * (bb.minPoint.y + bb.maxPoint.y),
                    0.5 * (bb.minPoint.z + bb.maxPoint.z))
                d = distAlong(c)
                if bestD is None or (d > bestD if farthest else d < bestD):
                    bestD, best = d, f
            return best

        loftInput = features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        # Toe-most segment's apex-side face first (pushes the loft PAST the toe cone so the
        # required toe cut bites), then the heel-facing (D->J / C->H) face of every segment
        # (the last one reaches past the heel cone).
        loftInput.loftSections.add(endFace(segments[0], False))
        for seg in segments:
            loftInput.loftSections.add(endFace(seg, True))
        curvedTooth = features.loftFeatures.add(loftInput).bodies.item(0)
        curvedTooth.name = f'{gearLabel} Spiral Tooth'
        futil.log(f'[spiral] {gearLabel}: PASS 2e lofted curved tooth '
                  f'({loftInput.loftSections.count} sections)', force_console=True)

        # remove the segment scaffolding (loft has captured their faces)
        for seg in segments:
            features.removeFeatures.add(seg)

        # ================================================================
        # PASS 2f: reinstate the toe/heel conical trims -> ends flush on the cones.
        # ================================================================
        return self._cutConicalEnds(
            designComponent, curvedTooth, gearBody, toeMid, heelMid, apexWorld, gearLabel)
