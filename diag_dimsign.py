import traceback
import adsk.core, adsk.fusion

app = adsk.core.Application.get()

lines = []

def log(s):
    lines.append(s)

doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
try:
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    sk = root.sketches.add(root.xYConstructionPlane)
    origin = sk.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))
    sk.geometricConstraints.addCoincident(origin, sk.originPoint)

    # Probe A: point below the X axis, vertical distance dimension from origin.
    pA = sk.sketchPoints.add(adsk.core.Point3D.create(1.0, -0.5, 0))
    dA = sk.sketchDimensions.addDistanceDimension(
        origin, pA,
        adsk.fusion.DimensionOrientations.VerticalDimensionOrientation,
        adsk.core.Point3D.create(1.0, -0.25, 0))
    log('A: created vertical dim to point (1.0,-0.5); parameter.value = %.6f' % dA.parameter.value)

    # Probe B: assign the signed (negative) delta, as the generated code does.
    try:
        dA.parameter.value = -0.5
        g = pA.geometry
        log('B: set value=-0.5 OK; point now (%.6f, %.6f); parameter.value = %.6f'
            % (g.x, g.y, dA.parameter.value))
    except Exception:
        log('B: set value=-0.5 RAISED:\n' + traceback.format_exc())

    # Probe C: assign the positive magnitude; where does the point go?
    try:
        dA.parameter.value = 0.5
        g = pA.geometry
        log('C: set value=0.5 OK; point now (%.6f, %.6f); parameter.value = %.6f'
            % (g.x, g.y, dA.parameter.value))
    except Exception:
        log('C: set value=0.5 RAISED:\n' + traceback.format_exc())

    # Probe D: a zero-value vertical dimension (the +X reference endpoint scheme).
    pD = sk.sketchPoints.add(adsk.core.Point3D.create(2.0, 0.0, 0))
    try:
        dDh = sk.sketchDimensions.addDistanceDimension(
            origin, pD,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
            adsk.core.Point3D.create(1.0, 0.1, 0))
        dDh.parameter.value = 2.0
        dDv = sk.sketchDimensions.addDistanceDimension(
            origin, pD,
            adsk.fusion.DimensionOrientations.VerticalDimensionOrientation,
            adsk.core.Point3D.create(2.0, 0.1, 0))
        dDv.parameter.value = 0.0
        g = pD.geometry
        log('D: zero vertical dim OK; point now (%.6f, %.6f)' % (g.x, g.y))
    except Exception:
        log('D: zero vertical dim RAISED:\n' + traceback.format_exc())

    # Probe E: line endpoint held on a circle by numbers only (no coincident):
    # does a profile still form through the circle?
    circle = sk.sketchCurves.sketchCircles.addByCenterRadius(
        adsk.core.Point3D.create(0, 0, 0), 1.0)
    sk.geometricConstraints.addCoincident(circle.centerSketchPoint, sk.originPoint)
    tp = adsk.core.Point3D.create(0.2, 1.2, 0)
    sk.sketchDimensions.addRadialDimension(circle, tp)
    import math
    ang = 0.3
    e1 = adsk.core.Point3D.create(math.cos(ang), math.sin(ang), 0)
    e2 = adsk.core.Point3D.create(math.cos(-ang), math.sin(-ang), 0)
    far = adsk.core.Point3D.create(2.0, 0, 0)
    l1 = sk.sketchCurves.sketchLines.addByTwoPoints(e1, far)
    l2 = sk.sketchCurves.sketchLines.addByTwoPoints(e2, l1.endSketchPoint)
    log('E: profiles with 2 lines meeting circle by numbers only: %d' % sk.profiles.count)
    for i in range(sk.profiles.count):
        prof = sk.profiles.item(i)
        for j in range(prof.profileLoops.count):
            loop = prof.profileLoops.item(j)
            kinds = []
            for k in range(loop.profileCurves.count):
                kinds.append(str(loop.profileCurves.item(k).geometry.curveType))
            log('   profile %d loop %d curve types: %s' % (i, j, kinds))
finally:
    text = '\n'.join(lines)
    out = 'C:/Users/lestr/AppData/Roaming/Autodesk/Autodesk Fusion 360/API/AddIns/Gear Generator/diag_dimsign_out.txt'
    with open(out, 'w') as f:
        f.write(text)
    print(text)
    doc.close(False)
