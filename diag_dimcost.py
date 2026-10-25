import math
import os
import time
import traceback
import adsk.core, adsk.fusion

# Where does the time of one addDistanceDimension call go? The cycloidal build spent 359 s dimensioning 457 points of
# one sketch and 0.3 s on a one-point sketch, with the solver deferred, so the cost is per call and not the solve.
# Each variant dimensions the same points the way the add-in does (a horizontal and a vertical distance from the local
# origin) and reports seconds per dimension. Run it from the Text Commands window in Py mode:
#   exec(open(r'<AddIns>\Gear Generator\diag_dimcost.py').read())
# The results are written to diag_dimcost_out.txt beside this file after every variant.

app = adsk.core.Application.get()
HORIZONTAL = adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation
VERTICAL = adsk.fusion.DimensionOrientations.VerticalDimensionOrientation

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
        with open(os.path.join(appdata_dir(), 'diag_dimcost_out.txt'), 'w') as f:
            f.write('\n'.join(lines))
    except OSError:
        pass


def ring(n, radius=5.0):
    return [(radius * math.cos(2 * math.pi * i / n), radius * math.sin(2 * math.pi * i / n)) for i in range(n)]


def fresh_sketch(design_type=None):
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    if design_type is not None:
        design.designType = design_type
    root = design.rootComponent
    sketch = root.sketches.add(root.xYConstructionPlane)
    origin = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))
    return doc, sketch, origin


def dimension_all(sketch, origin, points, text_at_point=True):
    """Seconds per dimension, with the points already in the sketch."""
    dims = sketch.sketchDimensions
    start = time.perf_counter()
    for point, (x, y) in points:
        text = adsk.core.Point3D.create(x, y, 0) if text_at_point else adsk.core.Point3D.create(0.1, 0.1, 0)
        dims.addDistanceDimension(origin, point, HORIZONTAL, text)
        dims.addDistanceDimension(origin, point, VERTICAL, text)
    return (time.perf_counter() - start) / (2 * len(points))


def loose_points(sketch, coords):
    return [(sketch.sketchPoints.add(adsk.core.Point3D.create(x, y, 0)), (x, y)) for x, y in coords]


def spline_points(sketch, coords):
    collection = adsk.core.ObjectCollection.create()
    for x, y in coords:
        collection.add(adsk.core.Point3D.create(x, y, 0))
    fits = sketch.sketchCurves.sketchFittedSplines.add(collection).fitPoints
    return [(fits.item(i), coords[i]) for i in range(len(coords))]


def variant(name, body, design_type=None):
    doc = None
    try:
        doc, sketch, origin = fresh_sketch(design_type)
        log('%-52s %s' % (name, body(sketch, origin)))
    except Exception:
        log('%-52s FAILED %s' % (name, traceback.format_exc().splitlines()[-1]))
    finally:
        if doc is not None:
            doc.close(False)


N = 40
coords = ring(N)


def seconds(value):
    return '%.3f s/dim' % value


try:
    # A. The baseline the add-in sees: points in a sketch, solver running.
    variant('A  loose points, solver on', lambda s, o: seconds(dimension_all(s, o, loose_points(s, coords))))

    # B. The same with the solver deferred, as the add-in does above 200 points.
    def deferred(s, o):
        s.isComputeDeferred = True
        points = loose_points(s, coords)
        value = dimension_all(s, o, points)
        start = time.perf_counter()
        s.isComputeDeferred = False
        return '%s, release %.2f s' % (seconds(value), time.perf_counter() - start)
    variant('B  loose points, solver deferred', deferred)

    # C. Deferred, and the sketch hidden, so no graphics need updating.
    def hidden(s, o):
        s.isComputeDeferred = True
        s.isVisible = False
        points = loose_points(s, coords)
        value = dimension_all(s, o, points)
        s.isComputeDeferred = False
        s.isVisible = True
        return seconds(value)
    variant('C  deferred + sketch hidden', hidden)

    # D. Deferred, and the sketch's dimension/constraint/point/profile display switched off.
    def display_off(s, o):
        s.isComputeDeferred = True
        s.areDimensionsShown = False
        s.areConstraintsShown = False
        s.arePointsShown = False
        s.areProfilesShown = False
        points = loose_points(s, coords)
        value = dimension_all(s, o, points)
        s.isComputeDeferred = False
        s.areDimensionsShown = True
        s.areConstraintsShown = True
        s.arePointsShown = True
        s.areProfilesShown = True
        return seconds(value)
    variant('D  deferred + dimensions/constraints/points/profiles off', display_off)

    # E. Deferred, hidden and display off together.
    def everything_off(s, o):
        s.isComputeDeferred = True
        s.isVisible = False
        s.areDimensionsShown = False
        s.areConstraintsShown = False
        s.arePointsShown = False
        s.areProfilesShown = False
        points = loose_points(s, coords)
        value = dimension_all(s, o, points)
        s.isComputeDeferred = False
        return seconds(value)
    variant('E  deferred + hidden + display off', everything_off)

    # F. Dimension text far from the points, in one place.
    def text_far(s, o):
        s.isComputeDeferred = True
        points = loose_points(s, coords)
        value = dimension_all(s, o, points, text_at_point=False)
        s.isComputeDeferred = False
        return seconds(value)
    variant('F  deferred, dimension text at one place', text_far)

    # G. Fit points of a spline, as the add-in dimensions, deferred.
    def fit_points(s, o):
        s.isComputeDeferred = True
        points = spline_points(s, coords)
        value = dimension_all(s, o, points)
        s.isComputeDeferred = False
        return seconds(value)
    variant('G  spline fit points, deferred', fit_points)

    # H. Direct modelling: no timeline to record into.
    def direct(s, o):
        s.isComputeDeferred = True
        points = loose_points(s, coords)
        value = dimension_all(s, o, points)
        s.isComputeDeferred = False
        return seconds(value)
    variant('H  deferred, direct design type', direct, adsk.fusion.DesignTypes.DirectDesignType)

    # I. Does the cost grow with the number of dimensions already in the sketch? Deferred, in four equal rounds.
    def growth(s, o):
        s.isComputeDeferred = True
        rounds = []
        for k in range(4):
            part = ring(N, 5.0 + k)
            rounds.append(seconds(dimension_all(s, o, loose_points(s, part))))
        s.isComputeDeferred = False
        return ' -> '.join(rounds)
    variant('I  deferred, 4 rounds of %d points (cost over time)' % N, growth)

    # J. Fixing a point instead of dimensioning it: what one alternative would cost per point.
    def fixed(s, _origin):
        s.isComputeDeferred = True
        points = loose_points(s, coords)
        start = time.perf_counter()
        for point, _ in points:
            point.isFixed = True
        value = (time.perf_counter() - start) / len(points)
        s.isComputeDeferred = False
        return '%.3f s/point (isFixed)' % value
    variant('J  deferred, isFixed per point', fixed)
finally:
    log('done')
