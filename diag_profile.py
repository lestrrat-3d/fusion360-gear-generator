import math
import adsk.core, adsk.fusion

app = adsk.core.Application.get()
design = adsk.fusion.Design.cast(app.activeProduct)
ui = app.userInterface

lines = []

def log(s):
    lines.append(s)

target = None
for comp in design.allComponents:
    for sk in comp.sketches:
        if sk.name == 'Gear Profile':
            target = sk
if target is None:
    log('No sketch named "Gear Profile" found')
else:
    sk = target
    log('sketch: %s (component %s) fullyConstrained=%s' % (
        sk.name, sk.parentComponent.name, sk.isFullyConstrained))
    log('profiles: %d' % sk.profiles.count)
    tmap = {
        adsk.core.Curve3DTypes.NurbsCurve3DCurveType: 'nurbs',
        adsk.core.Curve3DTypes.Arc3DCurveType: 'arc',
        adsk.core.Curve3DTypes.Line3DCurveType: 'line',
        adsk.core.Curve3DTypes.Circle3DCurveType: 'circle',
    }
    for i in range(sk.profiles.count):
        prof = sk.profiles.item(i)
        for j in range(prof.profileLoops.count):
            loop = prof.profileLoops.item(j)
            counts = {}
            for k in range(loop.profileCurves.count):
                ct = loop.profileCurves.item(k).geometry.curveType
                name = tmap.get(ct, 'other(%s)' % ct)
                counts[name] = counts.get(name, 0) + 1
            log('  profile %d loop %d (outer=%s): %s' % (
                i, j, loop.isOuter, sorted(counts.items())))
    log('-- circles --')
    for i in range(sk.sketchCurves.sketchCircles.count):
        c = sk.sketchCurves.sketchCircles.item(i)
        g = c.centerSketchPoint.geometry
        log('  circle r=%.6f center=(%.6f,%.6f)' % (c.radius, g.x, g.y))
    log('-- non-construction lines --')
    for i in range(sk.sketchCurves.sketchLines.count):
        l = sk.sketchCurves.sketchLines.item(i)
        if l.isConstruction:
            continue
        s, e = l.startSketchPoint.geometry, l.endSketchPoint.geometry
        log('  line (%.6f,%.6f) r=%.6f -> (%.6f,%.6f) r=%.6f' % (
            s.x, s.y, math.hypot(s.x, s.y), e.x, e.y, math.hypot(e.x, e.y)))
    log('-- arcs --')
    for i in range(sk.sketchCurves.sketchArcs.count):
        a = sk.sketchCurves.sketchArcs.item(i)
        s, e = a.startSketchPoint.geometry, a.endSketchPoint.geometry
        c = a.centerSketchPoint.geometry
        log('  arc center=(%.6f,%.6f) r=%.6f start=(%.6f,%.6f) end=(%.6f,%.6f) sweep=%.4f' % (
            c.x, c.y, a.radius, s.x, s.y, e.x, e.y, a.geometry.endAngle - a.geometry.startAngle))
    log('-- splines --')
    for i in range(sk.sketchCurves.sketchFittedSplines.count):
        sp = sk.sketchCurves.sketchFittedSplines.item(i)
        s, e = sp.startSketchPoint.geometry, sp.endSketchPoint.geometry
        log('  spline start=(%.6f,%.6f) r=%.6f end=(%.6f,%.6f) r=%.6f' % (
            s.x, s.y, math.hypot(s.x, s.y), e.x, e.y, math.hypot(e.x, e.y)))

text = '\n'.join(lines)
# The console print is the primary output; the file beside the deployed
# add-in (found via the standard per-OS AddIns location) is a convenience.
import os
appdata = os.environ.get('APPDATA')
if appdata:
    addin = os.path.join(appdata, 'Autodesk', 'Autodesk Fusion 360', 'API',
                         'AddIns', 'Gear Generator')
else:
    addin = os.path.expanduser('~/Library/Application Support/Autodesk/'
                               'Autodesk Fusion 360/API/AddIns/Gear Generator')
try:
    with open(os.path.join(addin, 'diag_profile_out.txt'), 'w') as f:
        f.write(text)
except OSError:
    pass
print(text)
