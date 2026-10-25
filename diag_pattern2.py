import math
import os
import time
import traceback
import adsk.core, adsk.fusion

# Why is the patterned cycloidal disc sketch not fully constrained? The add-in's disc sketch leaves, in every copy of
# the lobe, one spline end and one standalone point at the same place free: the junction between two lobes. diag_pattern
# closed the ring fully constrained, but it dimensioned a shared junction on the EARLIER spline's last fit point,
# while the add-in dimensions it on the LATER spline's first fit point, and it patterned about the sketch origin. This
# draws the add-in's structure (3 splines of 11, 10 and 11 fit points, a bore circle at (1, 0) as the pattern centre)
# and tries these variants, printing whether the sketch is fully constrained and what stays free:
#   later      the add-in's way: the shared point dimensioned on the later spline
#   earlier    the shared point dimensioned on the earlier spline
#   origin     as later, patterned about the sketch origin instead of the circle centre
#   nocircle   as later, with no circle: the pattern centre is a dimensioned point
#   explicit   as later, with a coincident between each seed junction's two points
#   joined     as later, plus a coincident between each pair of neighbouring copies' junction points
# Run it from the Text Commands window in Py mode:
#   exec(open(r'<AddIns>\Gear Generator\diag_pattern2.py').read())
# The results are written to diag_pattern2_out.txt beside this file after every variant.

app = adsk.core.Application.get()
HORIZONTAL = adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation
VERTICAL = adsk.fusion.DimensionOrientations.VerticalDimensionOrientation

LOBES = 15
COUNTS = [11, 10, 11]
RADIUS = 5.0
DEPTH = 0.3
CX = 0.1

lines = []


def appdata_dir():
    appdata = os.environ.get('APPDATA')
    if appdata:
        return os.path.join(appdata, 'Autodesk', 'Autodesk Fusion 360', 'API', 'AddIns', 'Gear Generator')
    return os.path.expanduser('~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/AddIns/Gear Generator')


def log(s):
    lines.append(s)
    print(s)
    try:
        with open(os.path.join(appdata_dir(), 'diag_pattern2_out.txt'), 'w') as f:
            f.write('\n'.join(lines))
    except OSError:
        pass


def lobe_runs():
    steps = sum(c - 1 for c in COUNTS)
    coords = []
    for j in range(steps + 1):
        theta = 2 * math.pi * (j / steps) / LOBES + 0.2
        r = RADIUS + DEPTH * math.cos(LOBES * theta)
        coords.append((CX + r * math.cos(theta), r * math.sin(theta)))
    runs, at = [], 0
    for c in COUNTS:
        runs.append(coords[at:at + c])
        at += c - 1
    return runs


def add_spline(sketch, coords):
    collection = adsk.core.ObjectCollection.create()
    for x, y in coords:
        collection.add(adsk.core.Point3D.create(x, y, 0))
    return sketch.sketchCurves.sketchFittedSplines.add(collection)


def free_entities(sketch):
    found = []
    curves = sketch.sketchCurves
    for i in range(curves.count):
        curve = curves.item(i)
        if curve.isFullyConstrained:
            continue
        fitted = adsk.fusion.SketchFittedSpline.cast(curve)
        loose = []
        if fitted is not None:
            for j in range(fitted.fitPoints.count):
                if not fitted.fitPoints.item(j).isFullyConstrained:
                    loose.append(j)
        found.append('%s %s' % (curve.objectType.split('::')[-1], loose))
    points = sketch.sketchPoints
    loose_points = sum(1 for i in range(points.count) if not points.item(i).isFullyConstrained)
    return '%d curves free %s; %d points free' % (len(found), sorted(set(found))[:3], loose_points)


def variant(name, body):
    doc = None
    try:
        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.rootComponent
        sketch = root.sketches.add(root.xYConstructionPlane)
        origin = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))
        sketch.geometricConstraints.addCoincident(origin, sketch.originPoint)
        start = time.perf_counter()
        text = body(sketch, origin)
        log('%-9s %5.2f s  %s' % (name, time.perf_counter() - start, text))
    except Exception:
        log('%-9s FAILED %s' % (name, traceback.format_exc().splitlines()[-1]))
    finally:
        if doc is not None:
            doc.close(False)


def dim(sketch, origin, point, x, y):
    text = adsk.core.Point3D.create(x, y, 0)
    dims = sketch.sketchDimensions
    dims.addDistanceDimension(origin, point, HORIZONTAL, text)
    dims.addDistanceDimension(origin, point, VERTICAL, text)


def build(sketch, origin, shared_on, circle, center_at_origin, explicit, joined):
    centre = origin
    if circle:
        c = sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(CX, 0, 0), 3.0)
        centre = c.centerSketchPoint
        dim(sketch, origin, centre, CX, 0.0)
        sketch.sketchDimensions.addDiameterDimension(c, adsk.core.Point3D.create(1, 1, 0))
    elif not center_at_origin:
        centre = sketch.sketchPoints.add(adsk.core.Point3D.create(CX, 0, 0))
        dim(sketch, origin, centre, CX, 0.0)
    runs = lobe_runs()
    splines = [add_spline(sketch, run) for run in runs]
    for index, (spline, run) in enumerate(zip(splines, runs)):
        fits = spline.fitPoints
        for j, (x, y) in enumerate(run):
            last_of_run = j == len(run) - 1
            first_of_run = j == 0
            if index == len(runs) - 1 and last_of_run:
                continue
            if last_of_run and shared_on == 'later':
                continue
            if first_of_run and index > 0 and shared_on == 'earlier':
                continue
            dim(sketch, origin, fits.item(j), x, y)
    if explicit:
        for a, b in zip(splines, splines[1:]):
            try:
                sketch.geometricConstraints.addCoincident(a.fitPoints.item(a.fitPoints.count - 1), b.fitPoints.item(0))
            except RuntimeError:
                pass
    constraints = sketch.geometricConstraints
    patternInput = constraints.createCircularPatternInput(list(splines), centre)
    patternInput.quantity = adsk.core.ValueInput.createByReal(LOBES)
    patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
    pattern = constraints.addCircularPattern(patternInput)
    created = [e for e in pattern.createdEntities if e.objectType.endswith('SketchFittedSpline')]
    first_start = adsk.fusion.SketchFittedSpline.cast(splines[0]).fitPoints.item(0).geometry
    cg = centre.geometry
    step = 2 * math.pi / LOBES
    dx, dy = first_start.x - cg.x, first_start.y - cg.y
    target = (cg.x + dx * math.cos(step) - dy * math.sin(step), cg.y + dx * math.sin(step) + dy * math.cos(step))
    nearest = min(created, key=lambda s: math.hypot(s.fitPoints.item(0).geometry.x - target[0],
                                                    s.fitPoints.item(0).geometry.y - target[1]))
    end = splines[-1].fitPoints.item(splines[-1].fitPoints.count - 1)
    constraints.addCoincident(end, nearest.fitPoints.item(0))
    problems = 0
    if joined:
        ordered = list(splines) + [s for s in created]
        ordered.sort(key=lambda s: math.atan2(s.fitPoints.item(0).geometry.y - cg.y,
                                              s.fitPoints.item(0).geometry.x - cg.x) % (2 * math.pi))
        per = len(COUNTS)
        for k in range(LOBES):
            a = ordered[k * per + per - 1]
            b = ordered[((k + 1) * per) % len(ordered)]
            try:
                constraints.addCoincident(a.fitPoints.item(a.fitPoints.count - 1), b.fitPoints.item(0))
            except RuntimeError:
                problems += 1
    return 'constrained %s, %d junction failures; %s' % (sketch.isFullyConstrained, problems, free_entities(sketch))


try:
    variant('later', lambda s, o: build(s, o, 'later', True, False, False, False))
    variant('earlier', lambda s, o: build(s, o, 'earlier', True, False, False, False))
    variant('origin', lambda s, o: build(s, o, 'later', False, True, False, False))
    variant('nocircle', lambda s, o: build(s, o, 'later', False, False, False, False))
    variant('explicit', lambda s, o: build(s, o, 'later', True, False, True, False))
    variant('joined', lambda s, o: build(s, o, 'later', True, False, False, True))
finally:
    log('done')
