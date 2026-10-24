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


def dim_point(sk, origin, pt, full=True, arc_end=False):
    """Dimension pt from origin the way the gallery's DimensionPoint / DimensionArcEnd do."""
    g = pt.geometry
    ox = origin.geometry
    x, y = g.x - ox.x, g.y - ox.y
    n = 0
    if full or not arc_end:
        sk.sketchDimensions.addDistanceDimension(origin, pt, H, p3(g.x, g.y - 0.3)); n += 1
        sk.sketchDimensions.addDistanceDimension(origin, pt, V, p3(g.x + 0.3, g.y)); n += 1
        return n
    # arc end: one coordinate only, the radial one comes from the arc
    if abs(x) >= abs(y):
        sk.sketchDimensions.addDistanceDimension(origin, pt, V, p3(g.x + 0.3, g.y))
    else:
        sk.sketchDimensions.addDistanceDimension(origin, pt, H, p3(g.x, g.y - 0.3))
    return 1


def build(label, module_mm, teeth, pressure_deg, samples, angle_rad, anchor_on_origin=True, probe=None):
    log('--- %s: module %g mm, %d teeth, pressure %g deg, %d samples, angle %.4f rad'
        % (label, module_mm, teeth, pressure_deg, samples, angle_rad))
    root = app.activeProduct.rootComponent
    sk = root.sketches.add(root.xYConstructionPlane)
    sk.name = 'Gallery Scheme ' + label
    try:
        m = module_mm / 10.0                       # cm
        pitch = m * teeth / 2
        base = pitch * math.cos(math.radians(pressure_deg))
        rootr = (m * teeth - 2.5 * m) / 2
        tip = (m * teeth + 2 * m) / 2
        left, right = _sample_flanks(pitch, base, base, tip, teeth, samples, angle_rad)
        local = sk.sketchPoints.add(p3(0, 0))      # movable local origin, as the add-in keeps ([SPUR-F-LOCAL-ORIGIN])
        # fit splines through Point3Ds; the spline owns its fit SketchPoints
        rc = adsk.core.ObjectCollection.create()
        for q in right:
            rc.add(p3(q.x, q.y))
        rspline = sk.sketchCurves.sketchFittedSplines.add(rc)
        lc = adsk.core.ObjectCollection.create()
        for q in left:
            lc.add(p3(q.x, q.y))
        lspline = sk.sketchCurves.sketchFittedSplines.add(lc)
        ndim = 0
        for i in range(samples):
            ndim += dim_point(sk, local, rspline.fitPoints.item(i))
        for i in range(samples):
            ndim += dim_point(sk, local, lspline.fitPoints.item(i), full=(i != samples - 1), arc_end=(i == samples - 1))
        log('fit-point dimensions created: %d' % ndim)
        # root contacts: base sample scaled to the root circle
        def root_contact(q):
            return p3(q.x * rootr / base, q.y * rootr / base)
        rr = sk.sketchPoints.add(root_contact(right[0]))
        lr = sk.sketchPoints.add(root_contact(left[0]))
        ndim += dim_point(sk, local, lr, full=False, arc_end=True)   # the gallery creates the left root first
        ndim += dim_point(sk, local, rr)
        lines_ = sk.sketchCurves.sketchLines
        lines_.addByTwoPoints(rr, rspline.startSketchPoint)
        lines_.addByTwoPoints(lspline.startSketchPoint, lr)
        arcs = sk.sketchCurves.sketchArcs
        arcs.addByCenterStartEnd(local, rspline.endSketchPoint, lspline.endSketchPoint)   # tip arc
        arcs.addByCenterStartEnd(local, rr, lr)                                            # root arc
        log('before anchoring: isFullyConstrained = %s' % sk.isFullyConstrained)
        if anchor_on_origin:
            sk.geometricConstraints.addCoincident(local, sk.originPoint)
            log('after coincident with origin: isFullyConstrained = %s' % sk.isFullyConstrained)
        # how far did the solver move the fit points from their targets?
        worst = 0.0
        for spline, pts in ((rspline, right), (lspline, left)):
            for i, q in enumerate(pts):
                g = spline.fitPoints.item(i).geometry
                worst = max(worst, math.hypot(g.x - q.x, g.y - q.y))
        log('worst fit-point displacement from target: %.3e cm' % worst)
        log('profiles: %d' % sk.profiles.count)
        for i in range(sk.profiles.count):
            prof = sk.profiles.item(i)
            for j in range(prof.profileLoops.count):
                loop = prof.profileLoops.item(j)
                kinds = {}
                for k in range(loop.profileCurves.count):
                    t = str(loop.profileCurves.item(k).geometry.curveType)
                    kinds[t] = kinds.get(t, 0) + 1
                log('  profile %d loop %d: %d curves %s' % (i, j, loop.profileCurves.count, kinds))
        if probe:
            probe(sk, local, rspline, lspline)
    except Exception:
        log('RAISED:\n' + traceback.format_exc())
    return sk


def probe_edit(sk, local, rspline, lspline):
    """Is the sketch really determined? Nudge one interior fit-point dimension and see the profile survive."""
    try:
        d = sk.sketchDimensions.item(4)
        old = d.parameter.value
        d.parameter.value = old * 1.0001
        log('probe: edited dimension 4 %.6f -> %.6f; isFullyConstrained = %s; profiles = %d'
            % (old, d.parameter.value, sk.isFullyConstrained, sk.profiles.count))
        d.parameter.value = old
    except Exception:
        log('probe RAISED:\n' + traceback.format_exc())


doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
try:
    build('A default 17 teeth', 2.0, 17, 20.0, 15, 0.0, probe=probe_edit)
    build('B rotated 14.5 deg', 2.0, 17, 20.0, 15, math.radians(14.5))
    build('C 40 teeth', 1.0, 40, 20.0, 15, 0.0)
    build('D 24 teeth, 25 deg pressure, 9 samples', 3.0, 24, 25.0, 9, 0.0)
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
