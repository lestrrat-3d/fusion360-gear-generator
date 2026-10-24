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


def build(label, module_mm, teeth, pressure_deg, samples, angle_rad, first='right', arcs_first=False, anchor=True):
    log('--- %s: module %g mm, %d teeth, pressure %g, %d samples, angle %.5f, first=%s, arcs_first=%s'
        % (label, module_mm, teeth, pressure_deg, samples, angle_rad, first, arcs_first))
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
        def lines_and_arcs():
            sk.sketchCurves.sketchLines.addByTwoPoints(rr, rspline.startSketchPoint)
            sk.sketchCurves.sketchLines.addByTwoPoints(lspline.startSketchPoint, lr)
            sk.sketchCurves.sketchArcs.addByCenterStartEnd(local, rspline.endSketchPoint, lspline.endSketchPoint)
            sk.sketchCurves.sketchArcs.addByCenterStartEnd(local, rr, lr)
        if arcs_first:
            lines_and_arcs()
        order = [('R', rspline, False), ('L', lspline, True)]
        if first == 'left':
            order.reverse()
        for side, sp, has_arc_end in order:
            for i in range(samples):
                dim_point(sk, local, sp.fitPoints.item(i), '%s%d' % (side, i), arc_end=(has_arc_end and i == samples - 1))
        dim_point(sk, local, lr, 'Lroot', arc_end=True)
        dim_point(sk, local, rr, 'Rroot')
        if not arcs_first:
            lines_and_arcs()
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
    except Exception:
        log('RAISED:\n' + traceback.format_exc())
    return sk


doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
try:
    build('A0 17 teeth, angle 0', 2.0, 17, 20.0, 15, 0.0)
    build('A1 17 teeth, angle 1e-4', 2.0, 17, 20.0, 15, 1e-4)
    build('A2 17 teeth, angle 0, left first', 2.0, 17, 20.0, 15, 0.0, first='left')
    build('A3 17 teeth, angle 0, arcs first', 2.0, 17, 20.0, 15, 0.0, arcs_first=True)
    build('B 17 teeth, angle 14.5 deg', 2.0, 17, 20.0, 15, math.radians(14.5))
    build('B2 17 teeth, angle 14.5 deg, arcs first', 2.0, 17, 20.0, 15, math.radians(14.5), arcs_first=True)
    build('C 40 teeth, angle 0', 1.0, 40, 20.0, 15, 0.0)
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
