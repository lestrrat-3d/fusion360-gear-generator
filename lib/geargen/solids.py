# Gear-agnostic solid-body helpers shared by the gear generators.
#
# These functions encode hard-won Fusion behaviors (face-by-midpoint cone
# selection, keeper selection, lenient non-intersecting cuts, piece-wise
# slicing) that were proven in the bevel generator. They are framework code:
# gear generators MUST call them rather than re-implement the patterns — see
# PLAYBOOK.md "Shared geargen helper library".

import math
import adsk.core, adsk.fusion
from ...lib import fusion360utils as futil


# Fusion reports a split whose tool does not touch the target with this code
# (message text may be localized — '交差' is the Japanese fragment).
_NON_INTERSECT_MARKERS = ('SPLIT_TARGET_TOOL_NOT_INTERSECT', '交差')


class NonIntersectError(RuntimeError):
    """A conical cut found cone faces, but every attempt failed with Fusion's
    tool-does-not-intersect signal. Callers for which a non-intersecting cut
    is a legitimate no-op (e.g. a heel cone that never overshoots the tooth)
    catch THIS type; any other split failure stays a plain RuntimeError."""


def surface_distance(surface, worldPoint):
    # Distance from worldPoint to its projection onto the (infinite) surface.
    # Returns float('inf') when the evaluator cannot project the point (e.g.
    # near a cone's apex singularity) — "try last", never "disqualified"
    # ([PB-FACE-BY-MIDPOINT]).
    try:
        evaluator = surface.evaluator
        (ok, param) = evaluator.getParameterAtPoint(worldPoint)
        if not ok:
            return float('inf')
        (ok2, projected) = evaluator.getPointAtParameter(param)
        if not ok2:
            return float('inf')
        return projected.distanceTo(worldPoint)
    except Exception:
        return float('inf')


def find_cone_faces_by_midpoint(frustumBody, edgeMidWorld):
    # All ConeSurfaceType faces of frustumBody as (distance, face) pairs,
    # ordered best-first by the distance of edgeMidWorld to the face's
    # surface ([PB-FACE-BY-MIDPOINT]).
    coneFaces = []
    for face in frustumBody.faces:
        if face.geometry.surfaceType == adsk.core.SurfaceTypes.ConeSurfaceType:
            dist = surface_distance(face.geometry, edgeMidWorld)
            coneFaces.append((dist, face))
    coneFaces.sort(key=lambda t: t[0])
    return coneFaces


def select_keeper(component, pieces, apexWorld, label):
    # Keeper rule after a conical cut: remove every piece that contains the
    # apex, keep the single largest remaining piece, remove the rest
    # ([PB-REMOVE-PIECES]). Raises when no non-apex piece exists.
    nonApex = []
    for piece in pieces:
        containment = piece.pointContainment(apexWorld)
        if containment == adsk.fusion.PointContainment.PointInsidePointContainment:
            component.features.removeFeatures.add(piece)
        else:
            nonApex.append(piece)
    if len(nonApex) == 0:
        raise RuntimeError(f'{label}: no non-apex piece after cut')
    nonApex.sort(key=lambda b: b.physicalProperties.volume, reverse=True)
    keeper = nonApex[0]
    for extra in nonApex[1:]:
        component.features.removeFeatures.add(extra)
    return keeper


def apply_conical_cut(component, targetBody, frustumBody, edgeMidWorld,
                      apexWorld, label):
    # Split targetBody with a cone face of frustumBody: order the candidate
    # faces best-first by edge-midpoint distance and try each as the split
    # tool (isSplittingToolExtended=True), keeping the first that actually
    # splits (>1 piece); returns the keeper piece per select_keeper.
    #
    # When no face splits the target: raises NonIntersectError if the
    # per-face failures carried Fusion's non-intersect signal, else a plain
    # RuntimeError. Either way the message preserves the full per-face
    # distance/error history ([PB-SELF-DIAGNOSING]).
    features = component.features
    coneFaces = find_cone_faces_by_midpoint(frustumBody, edgeMidWorld)
    history = []
    for (dist, face) in coneFaces:
        try:
            splitInput = features.splitBodyFeatures.createInput(
                targetBody, face, True)
            split = features.splitBodyFeatures.add(splitInput)
            if split.bodies.count > 1:
                pieces = [split.bodies.item(i) for i in range(split.bodies.count)]
                keeper = select_keeper(component, pieces, apexWorld, label)
                futil.log(
                    f'{label}: split into {split.bodies.count} '
                    f'pieces (face dist={dist})', force_console=True)
                return keeper
            else:
                history.append(f'dist={dist}: split produced <=1 piece')
        except RuntimeError as e:
            history.append(f'dist={dist}: {str(e)}')
    message = (f'{label}: no cone face split the tooth body '
               f'(faces found={len(coneFaces)}; history={history})')
    if any(marker in entry for entry in history for marker in _NON_INTERSECT_MARKERS):
        raise NonIntersectError(message)
    raise RuntimeError(message)


def cut_conical_ends(component, toothBody, gearBody, toeMid, heelMid,
                     apexWorld, label):
    # The toe-then-heel two-cone flush trim. The toe cut must split (any
    # failure propagates); the heel cone may legitimately not reach the
    # keeper (common on ratio pairs) — a NonIntersectError there keeps the
    # tooth intact.
    keeper = apply_conical_cut(
        component, toothBody, gearBody, toeMid, apexWorld,
        f'{label} toe cut (#1)')
    try:
        keeper = apply_conical_cut(
            component, keeper, gearBody, heelMid, apexWorld,
            f'{label} heel cut (#2)')
    except NonIntersectError:
        futil.log(
            f'{label} heel cut (#2): tool did not intersect, kept intact',
            force_console=True)
    return keeper


def slice_body_by_offset_planes(component, body, basePlane, offsets):
    # Split `body` piece-by-piece with construction planes offset from
    # basePlane by each value in `offsets` (cm). A plane that misses a piece
    # leaves that piece whole (the split's RuntimeError is absorbed).
    # Returns the resulting pieces; the caller asserts the expected count
    # ([PB-EMPTY-RESULT]).
    planes = component.constructionPlanes
    features = component.features
    pieces = [body]
    for offset in offsets:
        cpInput = planes.createInput()
        cpInput.setByOffset(basePlane, adsk.core.ValueInput.createByReal(offset))
        cutPlane = planes.add(cpInput)
        newPieces = []
        for piece in pieces:
            try:
                splitInput = features.splitBodyFeatures.createInput(
                    piece, cutPlane, True)
                split = features.splitBodyFeatures.add(splitInput)
                for b in split.bodies:
                    newPieces.append(b)
            except RuntimeError:
                newPieces.append(piece)
        pieces = newPieces
    return pieces


def rotate_body_about_edge(component, body, edge, angle):
    # Rotate `body` by `angle` (radians) about the axis through the sketch
    # edge's WORLD endpoints, via a free-move matrix ([PB-MOVE-ROTATE]).
    startW = edge.startSketchPoint.worldGeometry
    endW = edge.endSketchPoint.worldGeometry
    axisVec = adsk.core.Vector3D.create(
        endW.x - startW.x, endW.y - startW.y, endW.z - startW.z)
    axisVec.normalize()
    matrix = adsk.core.Matrix3D.create()
    matrix.setToRotation(angle, axisVec, startW)
    bodies = adsk.core.ObjectCollection.create()
    bodies.add(body)
    moveInput = component.features.moveFeatures.createInput2(bodies)
    moveInput.defineAsFreeMove(matrix)
    component.features.moveFeatures.add(moveInput)


def plane_by_angle(component, line, refPlane, angleDeg):
    # Construction plane through the sketch line, at angleDeg to refPlane.
    # Pass the line directly — never wrap it in Path.create
    # ([PB-CONSTRUCTION-PLANES]).
    planes = component.constructionPlanes
    pinput = planes.createInput()
    pinput.setByAngle(line,
                      adsk.core.ValueInput.createByString(f'{angleDeg} deg'),
                      refPlane)
    return planes.add(pinput)


def combine_point(base, a, e1, b=0.0, e2=None):
    # World point base + a*e1 (+ b*e2); lengths in internal cm.
    x = base.x + a * e1.x
    y = base.y + a * e1.y
    z = base.z + a * e1.z
    if e2 is not None:
        x += b * e2.x
        y += b * e2.y
        z += b * e2.z
    return adsk.core.Point3D.create(x, y, z)


def circle_intersect_nearest(R, Cx, Cy, r_c, refX, refY):
    # Intersect the apex-centred circle of radius R (centre at the origin)
    # with the cutter circle (centre (Cx, Cy), radius r_c); return the
    # intersection (x, y) nearest the reference point (refX, refY). A
    # non-overlapping pair is clamped to the radical-line tangency point.
    d = math.hypot(Cx, Cy)
    if d == 0:
        raise Exception('cutter circle centered at apex; cannot intersect')
    a = (R * R - r_c * r_c + d * d) / (2.0 * d)
    h2 = R * R - a * a
    if h2 < 0:
        h2 = 0.0
    h = math.sqrt(h2)
    ux, uy = Cx / d, Cy / d
    px = a * ux
    py = a * uy
    s1 = (px - h * uy, py + h * ux)
    s2 = (px + h * uy, py - h * ux)
    d1 = math.hypot(s1[0] - refX, s1[1] - refY)
    d2 = math.hypot(s2[0] - refX, s2[1] - refY)
    return s1 if d1 <= d2 else s2


def hide_construction_geometry(component):
    # Recursively hide every sketch, construction plane, and construction
    # axis under `component` (dedupe by entityToken), leaving only bodies
    # visible ([PB-TREE-CLEANUP]).
    seen = set()

    def walk(comp):
        for sketch in comp.sketches:
            token = sketch.entityToken
            if token not in seen:
                seen.add(token)
                sketch.isLightBulbOn = False
        for plane in comp.constructionPlanes:
            token = plane.entityToken
            if token not in seen:
                seen.add(token)
                plane.isLightBulbOn = False
        for axis in comp.constructionAxes:
            token = axis.entityToken
            if token not in seen:
                seen.add(token)
                axis.isLightBulbOn = False
        for occ in comp.occurrences:
            walk(occ.component)

    walk(component)
