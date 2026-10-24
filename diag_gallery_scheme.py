# Fusion diagnostic: does the decad-gallery spur tooth-section sketch scheme work in Fusion?
#
# The gallery draws one tooth with every computed point dimensioned from the sketch origin (a horizontal and a
# vertical distance), fit splines through those points, two lines and two arcs about the origin. The Fusion add-in
# draws the same tooth with "ribs" instead (docs/fusion/spurgear.md, [SPUR-F-RIBS]). This script builds the gallery
# scheme in Fusion and reports whether it is fully constrained, over-constrained, and yields one closed profile.
# Run it as a script in Fusion; it writes diag_gallery_scheme_out.txt beside the deployed add-in.
import collections
import math
import os
import traceback
import adsk.core, adsk.fusion

app = adsk.core.Application.get()
lines = []


def log(s):
    lines.append(s)


# ---- the involute helpers, copied from the generated spurgear.py ----
_Point = collections.namedtuple('Point', ['x', 'y'])

def _rotate(x, y, theta):
    s, c = math.sin(theta), math.cos(theta)
    return _Point((x * c) - (y * s), (x * s) + (y * c))

def _involute_point(base, r):
    ratio = base / r
    if ratio > 1:
        ratio = 1
    t = math.tan(math.acos(ratio))
    s, c = math.sin(t), math.cos(t)
    return _Point(base * (c + (t * s)), -(base * (s - (t * c))))

def _flank_beta(pitch, base, teeth):
    pp = _involute_point(base, pitch)
    return (math.pi / (2 * teeth)) - math.atan2(pp.y, pp.x)

def _sample_flanks(pitch, base, start, tip, teeth, samples, angle):
    left = []
    right = []
    beta = _flank_beta(pitch, base, teeth)
    left = [None] * (samples)
    right = [None] * (samples)
    for i in range(0, samples):
        r = start + (((tip - start) * float(i)) / float(samples - 1))
        m = _involute_point(base, r)
        l = _rotate(m.x, m.y, beta)
        rt = _Point(l.x, -l.y)
        left[i] = _rotate(l.x, l.y, angle)
        right[i] = _rotate(rt.x, rt.y, angle)
    return left, right


H = adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation
V = adsk.fusion.DimensionOrientations.VerticalDimensionOrientation


def p3(x, y):
    return adsk.core.Point3D.create(x, y, 0)


class Tally:
    ok = 0
    failed = []


def add_dim(sk, origin, pt, orient, tag):
    g = pt.geometry
    text = p3(g.x, g.y - 0.3) if orient == H else p3(g.x + 0.3, g.y)
    try:
        sk.sketchDimensions.addDistanceDimension(origin, pt, orient, text)
        Tally.ok += 1
        return True
    except Exception as e:
        Tally.failed.append('%s %s at (%.5f, %.5f): %s' % (tag, 'H' if orient == H else 'V', g.x, g.y, str(e).strip()[:60]))
        return False


def dim_point(sk, origin, pt, tag, arc_end=False):
    """The gallery's DimensionPoint (H and V distance from the origin) or DimensionArcEnd (one coordinate)."""
    g = pt.geometry
    o = origin.geometry
    if not arc_end:
        add_dim(sk, origin, pt, H, tag)
        add_dim(sk, origin, pt, V, tag)
    elif abs(g.x - o.x) >= abs(g.y - o.y):
        add_dim(sk, origin, pt, V, tag)
    else:
        add_dim(sk, origin, pt, H, tag)


def list_constraints(sk, header):
    kinds = {}
    for i in range(sk.geometricConstraints.count):
        t = sk.geometricConstraints.item(i).objectType.split('::')[-1]
        kinds[t] = kinds.get(t, 0) + 1
    log('%s: %d geometric constraints %s, %d dimensions' % (header, sk.geometricConstraints.count, kinds, sk.sketchDimensions.count))


def entity_report(sk):
    free = []
    for i in range(sk.sketchCurves.count):
        c = sk.sketchCurves.item(i)
        if not c.isFullyConstrained:
            free.append('curve %d type %s' % (i, c.geometry.curveType))
    for i in range(sk.sketchPoints.count):
        p = sk.sketchPoints.item(i)
        if not p.isFullyConstrained:
            g = p.geometry
            free.append('point %d (%.5f, %.5f)' % (i, g.x, g.y))
    log('entities not fully constrained: %d' % len(free))
    for f in free[:12]:
        log('   ' + f)


def build(label, module_mm, teeth, pressure_deg, samples, angle_rad, first='right', arcs_first=False, anchor=True,
          centres=True, skip_redundant=True, extrude=True, fix='none', anchor_first=False):
    log('--- %s: module %g mm, %d teeth, angle %.5f, fix=%s, anchor_first=%s'
        % (label, module_mm, teeth, angle_rad, fix, anchor_first))
    Tally.ok = 0
    Tally.failed = []
    root = app.activeProduct.rootComponent
    sk = root.sketches.add(root.xYConstructionPlane)
    sk.name = 'Gallery Scheme ' + label
    try:
        m = module_mm / 10.0
        pitch = m * teeth / 2
        base = pitch * math.cos(math.radians(pressure_deg))
        rootr = (m * teeth - 2.5 * m) / 2
        tip = (m * teeth + 2 * m) / 2
        left, right = _sample_flanks(pitch, base, base, tip, teeth, samples, angle_rad)
        local = sk.sketchPoints.add(p3(0, 0))
        if anchor_first:
            sk.geometricConstraints.addCoincident(local, sk.originPoint)
        def spline(points):
            c = adsk.core.ObjectCollection.create()
            for q in points:
                c.add(p3(q.x, q.y))
            return sk.sketchCurves.sketchFittedSplines.add(c)
        rspline = spline(right)
        lspline = spline(left)
        def rc(q):
            return p3(q.x * rootr / base, q.y * rootr / base)
        rr = sk.sketchPoints.add(rc(right[0]))
        lr = sk.sketchPoints.add(rc(left[0]))
        list_constraints(sk, 'after splines and root points')
        def lines_and_arcs():
            sk.sketchCurves.sketchLines.addByTwoPoints(rr, rspline.startSketchPoint)
            sk.sketchCurves.sketchLines.addByTwoPoints(lspline.startSketchPoint, lr)
            tip_arc = sk.sketchCurves.sketchArcs.addByCenterStartEnd(local, rspline.endSketchPoint, lspline.endSketchPoint)
            root_arc = sk.sketchCurves.sketchArcs.addByCenterStartEnd(local, rr, lr)
            if centres:
                for arc in (tip_arc, root_arc):
                    cp = arc.centerSketchPoint
                    log('arc centre is the local origin point: %s' % (cp.entityToken == local.entityToken))
                    sk.geometricConstraints.addCoincident(cp, local)
        if arcs_first:
            lines_and_arcs()
        order = [('R', rspline, False), ('L', lspline, True)]
        if first == 'left':
            order.reverse()
        for side, sp, has_arc_end in order:
            for i in range(samples):
                pt = sp.fitPoints.item(i)
                tag = '%s%d' % (side, i)
                second_first_point = (i == 0 and side == order[1][0])
                if second_first_point and fix in ('vertical', 'fallback'):
                    # x equal to the other flank's first point by alignment, then only the vertical distance
                    other = order[0][1].fitPoints.item(0)
                    if fix == 'fallback':
                        before = len(Tally.failed)
                        add_dim(sk, local, pt, H, tag)
                        if len(Tally.failed) > before:
                            log('fallback: H dimension of %s refused, aligning vertically instead' % tag)
                            Tally.failed.pop()
                            sk.geometricConstraints.addVerticalPoints(pt, other)
                        before = len(Tally.failed)
                        add_dim(sk, local, pt, V, tag)
                        if len(Tally.failed) > before:
                            log('fallback: V dimension of %s refused, aligning horizontally instead' % tag)
                            Tally.failed.pop()
                            sk.geometricConstraints.addHorizontalPoints(pt, other)
                        continue
                    else:
                        try:
                            sk.geometricConstraints.addVerticalPoints(pt, other)
                            Tally.ok += 1
                        except Exception as e:
                            Tally.failed.append('%s vertical align: %s' % (tag, str(e).strip()[:60]))
                    add_dim(sk, local, pt, V, tag)
                elif second_first_point and fix == 'vh':
                    add_dim(sk, local, pt, V, tag)
                    add_dim(sk, local, pt, H, tag)
                else:
                    dim_point(sk, local, pt, tag, arc_end=(has_arc_end and i == samples - 1))
        dim_point(sk, local, lr, 'Lroot', arc_end=True)
        dim_point(sk, local, rr, 'Rroot')
        if not arcs_first:
            lines_and_arcs()
        list_constraints(sk, 'after dimensions, lines and arcs')
        log('dimensions created: %d, failed: %d' % (Tally.ok, len(Tally.failed)))
        for f in Tally.failed[:6]:
            log('   FAILED ' + f)
        log('isFullyConstrained before anchoring = %s' % sk.isFullyConstrained)
        if anchor:
            try:
                sk.geometricConstraints.addCoincident(local, sk.originPoint)
                log('after coincident with origin: isFullyConstrained = %s' % sk.isFullyConstrained)
            except Exception as e:
                log('anchoring RAISED: %s' % str(e).strip()[:80])
        entity_report(sk)
        worst = 0.0
        for sp, pts in ((rspline, right), (lspline, left)):
            for i, q in enumerate(pts):
                g = sp.fitPoints.item(i).geometry
                worst = max(worst, math.hypot(g.x - q.x, g.y - q.y))
        log('worst fit-point displacement: %.3e cm; profiles: %d' % (worst, sk.profiles.count))
        for i in range(sk.profiles.count):
            prof = sk.profiles.item(i)
            for j in range(prof.profileLoops.count):
                loop = prof.profileLoops.item(j)
                log('  profile %d loop %d: %d curves' % (i, j, loop.profileCurves.count))
        if extrude and sk.profiles.count == 1:
            try:
                ext = root.features.extrudeFeatures
                inp = ext.createInput(sk.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
                inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(0.5))
                body = ext.add(inp).bodies.item(0)
                log('extruded: %d faces, volume %.4f cm3' % (body.faces.count, body.physicalProperties.volume))
            except Exception:
                log('extrude RAISED:\n' + traceback.format_exc())
            try:
                d = sk.sketchDimensions.item(4)
                old = d.parameter.value
                d.parameter.value = old * 1.001
                log('edit probe: dimension 4 %.6f -> %.6f, profiles still %d, isFullyConstrained %s'
                    % (old, d.parameter.value, sk.profiles.count, sk.isFullyConstrained))
                d.parameter.value = old
            except Exception:
                log('edit probe RAISED:\n' + traceback.format_exc())
    except Exception:
        log('RAISED:\n' + traceback.format_exc())
    return sk


def build_ribs(label, module_mm, teeth, pressure_deg, samples, angle_rad):
    """The Fusion add-in's own scheme (drawCircles + drawTooth + flank-to-root), for comparison."""
    log('--- RIBS %s: module %g mm, %d teeth, angle %.5f' % (label, module_mm, teeth, angle_rad))
    root = app.activeProduct.rootComponent
    sk = root.sketches.add(root.xYConstructionPlane)
    sk.name = 'Ribs ' + label
    try:
        m = module_mm / 10.0
        pitchR = m * teeth / 2
        baseR = pitchR * math.cos(math.radians(pressure_deg))
        rootR = (m * teeth - 2.5 * m) / 2
        tipR = (m * teeth + 2 * m) / 2
        cons = sk.geometricConstraints
        dims = sk.sketchDimensions
        lines_ = sk.sketchCurves.sketchLines
        arcs_ = sk.sketchCurves.sketchArcs
        local = sk.sketchPoints.add(p3(0, 0))
        circles = {}
        for name, r, construction in (('root', rootR, False), ('tip', tipR, True)):
            c = sk.sketchCurves.sketchCircles.addByCenterRadius(local, r)
            c.isConstruction = construction
            d = dims.addDiameterDimension(c, p3(r, r))
            d.parameter.value = 2 * r
            circles[name] = c
        ca, sa = math.cos(angle_rad), math.sin(angle_rad)
        left, right = _sample_flanks(pitchR, baseR, baseR, tipR, teeth, samples, angle_rad)
        lc = adsk.core.ObjectCollection.create()
        rc = adsk.core.ObjectCollection.create()
        for q in left:
            lc.add(p3(q.x, q.y))
        for q in right:
            rc.add(p3(q.x, q.y))
        lsp = sk.sketchCurves.sketchFittedSplines.add(lc)
        rsp = sk.sketchCurves.sketchFittedSplines.add(rc)
        lfits, rfits = lsp.fitPoints, rsp.fitPoints
        top = sk.sketchPoints.add(p3(tipR * ca, tipR * sa))
        cons.addCoincident(top, circles['tip'])
        arc = arcs_.addByCenterStartEnd(local, rfits.item(rfits.count - 1), lfits.item(lfits.count - 1))
        cons.addCoincident(arc.centerSketchPoint, local)
        spine = lines_.addByTwoPoints(local, top)
        spine.isConstruction = True
        refEnd = sk.sketchPoints.add(p3(tipR, 0))
        tp = p3(tipR, tipR / 2)
        hd = dims.addDistanceDimension(local, refEnd, H, tp)
        hd.parameter.value = tipR
        vd = dims.addDistanceDimension(local, refEnd, V, tp)
        vd.parameter.value = 0
        ref = lines_.addByTwoPoints(local, refEnd)
        ref.isConstruction = True
        angdim = dims.addAngularDimension(ref, spine, p3(tipR / 2 * math.cos(angle_rad / 2), tipR / 2 * math.sin(angle_rad / 2)))
        across = V if abs(ca) >= abs(sa) else H
        along = H if abs(ca) >= abs(sa) else V
        prev = local
        for i in range(lfits.count):
            lf, rf = lfits.item(i), rfits.item(i)
            rib = lines_.addByTwoPoints(lf, rf)
            rib.isConstruction = True
            fit = lf.geometry
            tpt = p3(fit.x, fit.y)
            dims.addDistanceDimension(lf, rf, across, tpt)
            t = fit.x * ca + fit.y * sa
            mid = sk.sketchPoints.add(p3(t * ca, t * sa))
            cons.addCoincident(mid, spine)
            cons.addMidPoint(mid, rib)
            if i != lfits.count - 1:
                cons.addPerpendicular(spine, rib)
            dims.addDistanceDimension(prev, mid, along, tpt)
            prev = mid
        for fp in (lfits.item(0), rfits.item(0)):
            g = fp.geometry
            k = rootR / math.hypot(g.x, g.y)
            end = p3(g.x * k, g.y * k)
            ln = lines_.addByTwoPoints(end, fp)
            rend = ln.startSketchPoint
            hdm = dims.addDistanceDimension(local, rend, H, end)
            hdm.parameter.value = abs(end.x)
            vdm = dims.addDistanceDimension(local, rend, V, end)
            vdm.parameter.value = abs(end.y)
        if angle_rad != 0:
            angdim.parameter.value = angle_rad
        proj_ok = True
        cons.addCoincident(local, sk.originPoint)
        log('constraints %d, dimensions %d; isFullyConstrained = %s' % (cons.count, dims.count, sk.isFullyConstrained))
        entity_report(sk)
        log('profiles: %d' % sk.profiles.count)
        for i in range(sk.profiles.count):
            prof = sk.profiles.item(i)
            for j in range(prof.profileLoops.count):
                log('  profile %d loop %d: %d curves' % (i, j, prof.profileLoops.item(j).profileCurves.count))
    except Exception:
        log('RAISED:\n' + traceback.format_exc())


def probe_pair(label, qx, qy, order='HV', second_first='H'):
    """Two free points p, q dimensioned from a free local origin: does the second point's first dimension raise?"""
    root = app.activeProduct.rootComponent
    sk = root.sketches.add(root.xYConstructionPlane)
    sk.name = 'Probe ' + label
    a, b = 1.58829, 0.17109
    local = sk.sketchPoints.add(p3(0, 0))
    p = sk.sketchPoints.add(p3(a, b))
    q = sk.sketchPoints.add(p3(qx, qy))
    res = []
    for o in (H, V) if order == 'HV' else (V, H):
        res.append('p%s=%s' % ('H' if o == H else 'V', add_dim(sk, local, p, o, 'p')))
    for o in ((H, V) if second_first == 'H' else (V, H)):
        res.append('q%s=%s' % ('H' if o == H else 'V', add_dim(sk, local, q, o, 'q')))
    log('probe %-34s %s' % (label, ' '.join(res)))


doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
try:
    Tally.failed = []
    log('=== COMPARISON: gallery dimension scheme (with arc-centre and duplicate-dimension fixes) versus the add-in ribs')
    for name, mod, teeth, pr, smp, deg in (
            ('17 teeth', 2.0, 17, 20.0, 15, 0.0),
            ('17 teeth', 2.0, 17, 20.0, 15, 14.5),
            ('17 teeth', 2.0, 17, 20.0, 15, 45.0),
            ('17 teeth', 2.0, 17, 20.0, 15, 90.0),
            ('40 teeth', 1.0, 40, 20.0, 15, 0.0),
            ('24 teeth, 25 deg, 9 samples', 3.0, 24, 25.0, 9, 0.0)):
        ang = math.radians(deg)
        build('DIM %s @ %g deg' % (name, deg), mod, teeth, pr, smp, ang, fix='fallback')
        build_ribs('%s @ %g deg' % (name, deg), mod, teeth, pr, smp, ang)
finally:
    text = '\n'.join(lines)
    appdata = os.environ.get('APPDATA')
    if appdata:
        addin = os.path.join(appdata, 'Autodesk', 'Autodesk Fusion 360', 'API', 'AddIns', 'Gear Generator')
    else:
        addin = os.path.expanduser('~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/AddIns/Gear Generator')
    try:
        with open(os.path.join(addin, 'diag_gallery_scheme_out.txt'), 'w') as f:
            f.write(text)
    except OSError:
        pass
    print(text)
    doc.close(False)
