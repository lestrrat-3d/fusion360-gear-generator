import math
import os
import traceback
import adsk.core, adsk.fusion

# Why does the pinion of a built bevel pair look different from the driving gear? Run this with the pair open in Fusion.
# For every body in the design it prints the volume, the area, the counts of faces by surface kind, and each cone and
# plane face that is not a tooth flank: the cone faces' apex, axis and half angle, so a trimming cone that does not lie on
# the blank's own face shows as two nearby cones. Run it from the Text Commands window in Py mode:
#   exec(open(r'<AddIns>\Gear Generator\diag_bevel_bodies.py').read())
# The results are written to diag_bevel_bodies_out.txt beside this file.

app = adsk.core.Application.get()
lines = []


def appdata_dir():
    appdata = os.environ.get('APPDATA')
    if appdata:
        return os.path.join(appdata, 'Autodesk', 'Autodesk Fusion 360', 'API', 'AddIns', 'Gear Generator')
    return os.path.expanduser('~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/AddIns/Gear Generator')


def log(s):
    lines.append(s)
    print(s)


def fmt(v):
    return '(%.4f, %.4f, %.4f)' % (v.x * 10, v.y * 10, v.z * 10)


def bodies_of(component, path):
    for body in component.bRepBodies:
        yield path + '/' + component.name, body
    for occurrence in component.occurrences:
        yield from bodies_of(occurrence.component, path + '/' + component.name)


def describe(path, body):
    props = body.physicalProperties
    log('%s  body "%s"  volume %.3f mm3  area %.3f mm2  faces %d  edges %d  solid %s' % (
        path, body.name, props.volume * 1000, props.area * 100, body.faces.count, body.edges.count, body.isSolid))
    box = body.boundingBox
    log('   box %s .. %s' % (fmt(box.minPoint), fmt(box.maxPoint)))
    kinds = {}
    cones = []
    for face in body.faces:
        kind = face.geometry.surfaceType
        kinds[kind] = kinds.get(kind, 0) + 1
        if kind == adsk.core.SurfaceTypes.ConeSurfaceType:
            cone = adsk.core.Cone.cast(face.geometry)
            cones.append((fmt(cone.origin), fmt(cone.axis), math.degrees(cone.halfAngle), face.area * 100))
    names = {adsk.core.SurfaceTypes.PlaneSurfaceType: 'plane', adsk.core.SurfaceTypes.ConeSurfaceType: 'cone',
             adsk.core.SurfaceTypes.CylinderSurfaceType: 'cylinder', adsk.core.SurfaceTypes.NurbsSurfaceType: 'nurbs',
             adsk.core.SurfaceTypes.SphereSurfaceType: 'sphere', adsk.core.SurfaceTypes.TorusSurfaceType: 'torus'}
    log('   faces by kind: ' + ', '.join('%s %d' % (names.get(k, str(k)), n) for k, n in sorted(kinds.items())))
    big = sorted(cones, key=lambda c: -c[3])[:6]
    for origin, axis, half, area in big:
        log('   cone  apex %s  axis %s  half angle %.5f deg  area %.2f mm2' % (origin, axis, half, area))


try:
    design = adsk.fusion.Design.cast(app.activeProduct)
    found = list(bodies_of(design.rootComponent, ''))
    log('%d bodies' % len(found))
    for path, body in found:
        describe(path, body)
except Exception:
    log(traceback.format_exc())
finally:
    try:
        with open(os.path.join(appdata_dir(), 'diag_bevel_bodies_out.txt'), 'w') as f:
            f.write('\n'.join(lines))
    except OSError:
        pass
