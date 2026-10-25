import math
import os
import time
import traceback
import adsk.core, adsk.fusion

# Second round on the cost of one addDistanceDimension. Round one (diag_dimcost.py) found 0.003 s per dimension for
# 40 loose points in an empty document, rising to 0.053 s after 160 points, and 0.023 s for spline fit points. The
# cycloidal build measured 0.15 s per dimension on a 1-point sketch and 0.39 s on the 457-point disc sketch. This
# round asks what a document adds, how the cost grows with spline fit points, and what a circular pattern of one lobe
# would cost. Run it from the Text Commands window in Py mode:
#   exec(open(r'<AddIns>\Gear Generator\diag_dimcost2.py').read())
# The results are written to diag_dimcost2_out.txt beside this file after every variant.

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
        with open(os.path.join(appdata_dir(), 'diag_dimcost2_out.txt'), 'w') as f:
            f.write('\n'.join(lines))
    except OSError:
        pass


def new_document():
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    return doc, design, design.rootComponent


def new_sketch(root, plane=None):
    sketch = root.sketches.add(plane if plane is not None else root.xYConstructionPlane)
    origin = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))
    return sketch, origin


def rosette(lobes, points, arc, radius=5.0, depth=0.3):
    """The coordinates of one spline per lobe: `points` fit points over `arc` of each lobe's angle, gaps between."""
    out = []
    for k in range(lobes):
        spline = []
        for j in range(points):
            theta = 2 * math.pi * (k + arc * j / (points - 1)) / lobes
            r = radius + depth * math.cos(lobes * theta)
            spline.append((r * math.cos(theta), r * math.sin(theta)))
        out.append(spline)
    return out


def add_spline(sketch, coords):
    collection = adsk.core.ObjectCollection.create()
    for x, y in coords:
        collection.add(adsk.core.Point3D.create(x, y, 0))
    spline = sketch.sketchCurves.sketchFittedSplines.add(collection)
    fits = spline.fitPoints
    return spline, [(fits.item(i), coords[i]) for i in range(len(coords))]


def dimension(sketch, origin, points):
    dims = sketch.sketchDimensions
    start = time.perf_counter()
    for point, (x, y) in points:
        text = adsk.core.Point3D.create(x, y, 0)
        dims.addDistanceDimension(origin, point, HORIZONTAL, text)
        dims.addDistanceDimension(origin, point, VERTICAL, text)
    return time.perf_counter() - start


def per_dim(seconds, points):
    return '%.3f s/dim' % (seconds / (2 * len(points)))


def variant(name, body):
    doc = None
    try:
        doc, design, root = new_document()
        log('%-58s %s' % (name, body(design, root)))
    except Exception:
        log('%-58s FAILED %s' % (name, traceback.format_exc().splitlines()[-1]))
    finally:
        if doc is not None:
            doc.close(False)


LOBES = 15
PER_SPLINE = 10


try:
    # K. Spline fit points in rounds of 3 splines, solver deferred: how the cost grows with the spline count.
    def spline_growth(design, root):
        sketch, origin = new_sketch(root)
        sketch.isComputeDeferred = True
        rounds = []
        data = rosette(LOBES, PER_SPLINE, 0.9)
        for start in range(0, LOBES, 3):
            points = []
            for coords in data[start:start + 3]:
                points += add_spline(sketch, coords)[1]
            rounds.append(per_dim(dimension(sketch, origin, points), points))
        sketch.isComputeDeferred = False
        return ' -> '.join(rounds)
    variant('K  15 splines x 10 fit points, 5 rounds of 3 splines', spline_growth)

    # L. The same sketch hidden.
    def spline_hidden(design, root):
        sketch, origin = new_sketch(root)
        sketch.isComputeDeferred = True
        sketch.isVisible = False
        rounds = []
        data = rosette(LOBES, PER_SPLINE, 0.9)
        for start in range(0, LOBES, 3):
            points = []
            for coords in data[start:start + 3]:
                points += add_spline(sketch, coords)[1]
            rounds.append(per_dim(dimension(sketch, origin, points), points))
        sketch.isComputeDeferred = False
        return ' -> '.join(rounds)
    variant('L  as K, sketch hidden', spline_hidden)

    # M. Each spline dimensioned right after it is drawn, instead of all splines first.
    def interleaved(design, root):
        sketch, origin = new_sketch(root)
        sketch.isComputeDeferred = True
        total = 0.0
        count = 0
        for coords in rosette(LOBES, PER_SPLINE, 0.9):
            points = add_spline(sketch, coords)[1]
            total += dimension(sketch, origin, points)
            count += len(points)
        sketch.isComputeDeferred = False
        return '%.3f s/dim over %d points' % (total / (2 * count), count)
    variant('M  as K, each spline dimensioned as drawn', interleaved)

    # N. One lobe dimensioned, then a circular pattern of it: the cost the gallery would pay.
    def pattern(design, root):
        sketch, origin = new_sketch(root)
        sketch.isComputeDeferred = True
        spline, points = add_spline(sketch, rosette(LOBES, PER_SPLINE, 0.9)[0])
        seconds = dimension(sketch, origin, points)
        constraints = sketch.geometricConstraints
        patternInput = constraints.createCircularPatternInput([spline], origin)
        patternInput.quantity = adsk.core.ValueInput.createByReal(LOBES)
        patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
        start = time.perf_counter()
        constraints.addCircularPattern(patternInput)
        made = time.perf_counter() - start
        sketch.isComputeDeferred = False
        return ('dimension %.2f s, pattern %.2f s, splines %d, constrained %s'
                % (seconds, made, sketch.sketchCurves.sketchFittedSplines.count, sketch.isFullyConstrained))
    variant('N  one spline of 10 fit points + circular pattern x15', pattern)

    # O. What a heavy body in the document does to a later small sketch's dimensions.
    def heavy_body(design, root):
        sketch, origin = new_sketch(root)
        sketch.isComputeDeferred = True
        coords = [p for spline in rosette(LOBES, PER_SPLINE, 0.9) for p in spline]
        collection = adsk.core.ObjectCollection.create()
        for x, y in coords:
            collection.add(adsk.core.Point3D.create(x, y, 0))
        curve = sketch.sketchCurves.sketchFittedSplines.add(collection)
        sketch.sketchCurves.sketchLines.addByTwoPoints(curve.startSketchPoint, curve.endSketchPoint)
        sketch.isComputeDeferred = False
        extrudes = root.features.extrudeFeatures
        extrudeInput = extrudes.createInput(sketch.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extrudeInput.setDistanceExtent(False, adsk.core.ValueInput.createByReal(1.0))
        body = extrudes.add(extrudeInput).bodies.item(0)
        out = []
        for label, visible in (('body visible', True), ('body hidden', False)):
            body.isVisible = visible
            small, small_origin = new_sketch(root)
            points = [(small.sketchPoints.add(adsk.core.Point3D.create(1 + i, 1, 0)), (1 + i, 1)) for i in range(7)]
            out.append('%s %s' % (label, per_dim(dimension(small, small_origin, points), points)))
        body.isVisible = True
        return ', '.join(out)
    variant('O  7-point sketch after a 150-point spline body', heavy_body)

    # P. The same small sketch in an empty document, for comparison with O.
    def empty_small(design, root):
        small, small_origin = new_sketch(root)
        points = [(small.sketchPoints.add(adsk.core.Point3D.create(1 + i, 1, 0)), (1 + i, 1)) for i in range(7)]
        return per_dim(dimension(small, small_origin, points), points)
    variant('P  7-point sketch in an empty document', empty_small)

    # Q. Many user parameters in the design.
    def parameters(design, root):
        for i in range(150):
            design.userParameters.add('P%d' % i, adsk.core.ValueInput.createByReal(1.0 + i), 'mm', '')
        small, small_origin = new_sketch(root)
        points = [(small.sketchPoints.add(adsk.core.Point3D.create(1 + i, 1, 0)), (1 + i, 1)) for i in range(7)]
        return per_dim(dimension(small, small_origin, points), points)
    variant('Q  7-point sketch with 150 user parameters', parameters)

    # R. A sketch on an offset construction plane.
    def offset_plane(design, root):
        planes = root.constructionPlanes
        planeInput = planes.createInput()
        planeInput.setByOffset(root.xYConstructionPlane, adsk.core.ValueInput.createByReal(2.0))
        plane = planes.add(planeInput)
        small, small_origin = new_sketch(root, plane)
        points = [(small.sketchPoints.add(adsk.core.Point3D.create(1 + i, 1, 0)), (1 + i, 1)) for i in range(7)]
        return per_dim(dimension(small, small_origin, points), points)
    variant('R  7-point sketch on an offset plane', offset_plane)
finally:
    log('done')
