import math
import os
import time
import traceback
import adsk.core, adsk.fusion

# Does a Fusion circular pattern of a spline lobe close into one outline, and what does it take? The cycloidal
# gallery model now draws one lobe of 3 fitted splines (each ends where the next starts, the last ends free) and
# patterns it 15 times about a centre. Decad shares one point at every junction. Fusion's pattern copies each get
# their own points, so this asks which of these makes one closed, fully constrained outer profile:
#   control   the whole ring drawn spline by spline, as the add-in draws it today, no pattern
#   pattern   the pattern alone
#   one       the pattern and one coincident from the seed's end to the first copy's start
#   all       the pattern and a coincident between every neighbouring pair
#   merge     the pattern and every neighbouring pair of end points merged
# Run it from the Text Commands window in Py mode:
#   exec(open(r'<AddIns>\Gear Generator\diag_pattern.py').read())
# The results are written to diag_pattern_out.txt beside this file after every variant.

app = adsk.core.Application.get()
HORIZONTAL = adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation
VERTICAL = adsk.fusion.DimensionOrientations.VerticalDimensionOrientation

LOBES = 15
SPLINES = 3
PER_SPLINE = 10
RADIUS = 5.0
DEPTH = 0.3

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
        with open(os.path.join(appdata_dir(), 'diag_pattern_out.txt'), 'w') as f:
            f.write('\n'.join(lines))
    except OSError:
        pass


def lobe_runs(k):
    """The fit-point coordinates of the SPLINES splines of lobe k, each ending where the next starts."""
    steps = SPLINES * (PER_SPLINE - 1)
    coords = []
    for j in range(steps + 1):
        theta = 2 * math.pi * (k + j / steps) / LOBES
        r = RADIUS + DEPTH * math.cos(LOBES * theta)
        coords.append((r * math.cos(theta), r * math.sin(theta)))
    return [coords[i * (PER_SPLINE - 1):i * (PER_SPLINE - 1) + PER_SPLINE] for i in range(SPLINES)]


def add_spline(sketch, coords):
    collection = adsk.core.ObjectCollection.create()
    for x, y in coords:
        collection.add(adsk.core.Point3D.create(x, y, 0))
    return sketch.sketchCurves.sketchFittedSplines.add(collection)


def new_sketch():
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    sketch = root.sketches.add(root.xYConstructionPlane)
    origin = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))
    sketch.geometricConstraints.addCoincident(origin, sketch.originPoint)
    return doc, sketch, origin


def end_points(spline):
    fits = spline.fitPoints
    return fits.item(0), fits.item(fits.count - 1)


def near(point, x, y):
    return math.hypot(point.geometry.x - x, point.geometry.y - y) < 1e-6


def report(sketch):
    profiles = sketch.profiles
    outer = 0
    area = 0.0
    for i in range(profiles.count):
        profile = profiles.item(i)
        if any(loop.isOuter for loop in profile.profileLoops):
            outer += 1
            area = max(area, profile.areaProperties().area)
    return 'profiles %d, outer loops %d, largest area %.3f, constrained %s' % (
        profiles.count, outer, area, sketch.isFullyConstrained)


def variant(name, body):
    doc = None
    try:
        doc, sketch, origin = new_sketch()
        start = time.perf_counter()
        text = body(sketch, origin)
        log('%-10s %5.2f s  %s' % (name, time.perf_counter() - start, text))
    except Exception:
        log('%-10s FAILED %s' % (name, traceback.format_exc().splitlines()[-1]))
    finally:
        if doc is not None:
            doc.close(False)


def draw_seed(sketch, origin):
    """The seed lobe: its splines drawn, every fit point dimensioned from the origin except the last point."""
    runs = lobe_runs(0)
    splines = [add_spline(sketch, run) for run in runs]
    dims = sketch.sketchDimensions
    done = []
    for index, (spline, run) in enumerate(zip(splines, runs)):
        fits = spline.fitPoints
        for j, (x, y) in enumerate(run):
            if index == len(runs) - 1 and j == len(run) - 1:
                continue
            if any(abs(x - u) < 1e-9 and abs(y - v) < 1e-9 for u, v in done):
                continue
            done.append((x, y))
            text = adsk.core.Point3D.create(x, y, 0)
            dims.addDistanceDimension(origin, fits.item(j), HORIZONTAL, text)
            dims.addDistanceDimension(origin, fits.item(j), VERTICAL, text)
    return splines


def pattern_of(sketch, origin, splines):
    constraints = sketch.geometricConstraints
    patternInput = constraints.createCircularPatternInput(list(splines), origin)
    patternInput.quantity = adsk.core.ValueInput.createByReal(LOBES)
    patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
    return constraints.addCircularPattern(patternInput)


def copies_of(pattern):
    """The created splines, ordered by the lobe they sit in: the angle of their first fit point."""
    created = [e for e in pattern.createdEntities if e.objectType.endswith('SketchFittedSpline')]

    def angle(spline):
        p = spline.fitPoints.item(0).geometry
        return math.atan2(p.y, p.x) % (2 * math.pi)
    return sorted(created, key=angle)


try:
    # control: the whole ring, spline by spline, no pattern.
    def control(sketch, origin):
        sketch.isComputeDeferred = True
        for k in range(LOBES):
            for run in lobe_runs(k):
                add_spline(sketch, run)
        sketch.isComputeDeferred = False
        return report(sketch)
    variant('control', control)

    def only_pattern(sketch, origin):
        sketch.isComputeDeferred = True
        splines = draw_seed(sketch, origin)
        pattern = pattern_of(sketch, origin, splines)
        sketch.isComputeDeferred = False
        return '%d created; %s' % (len(pattern.createdEntities), report(sketch))
    variant('pattern', only_pattern)

    def seed_to_first(sketch, origin):
        sketch.isComputeDeferred = True
        splines = draw_seed(sketch, origin)
        pattern = pattern_of(sketch, origin, splines)
        first = [s for s in copies_of(pattern)][0]
        sketch.geometricConstraints.addCoincident(end_points(splines[-1])[1], end_points(first)[0])
        sketch.isComputeDeferred = False
        return '%d created; %s' % (len(pattern.createdEntities), report(sketch))
    variant('one', seed_to_first)

    def neighbours(sketch, origin, how):
        sketch.isComputeDeferred = True
        splines = draw_seed(sketch, origin)
        pattern = pattern_of(sketch, origin, splines)
        created = copies_of(pattern)
        # Lobe k's splines are created[k*SPLINES:(k+1)*SPLINES] once ordered by angle, the seed being lobe 0.
        ordered = list(splines) + [s for s in created if not any(s.entityToken == t.entityToken for t in splines)]
        ordered.sort(key=lambda s: math.atan2(s.fitPoints.item(0).geometry.y, s.fitPoints.item(0).geometry.x) % (2 * math.pi))
        problems = 0
        for i in range(len(ordered)):
            a = end_points(ordered[i])[1]
            b = end_points(ordered[(i + 1) % len(ordered)])[0]
            try:
                if how == 'all':
                    sketch.geometricConstraints.addCoincident(a, b)
                else:
                    a.merge(b)
            except RuntimeError:
                problems += 1
        sketch.isComputeDeferred = False
        return '%d splines, %d junction failures; %s' % (len(ordered), problems, report(sketch))
    variant('all', lambda s, o: neighbours(s, o, 'all'))
    variant('merge', lambda s, o: neighbours(s, o, 'merge'))
finally:
    log('done')
