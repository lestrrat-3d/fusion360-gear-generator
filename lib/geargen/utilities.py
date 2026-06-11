import adsk.core, adsk.fusion

def find_profile_by_curve_counts(sketch, nurbs=0, arcs=0, lines=0):
    """Return the profile whose loop has exactly the given curve-type counts
    (NURBS / arcs / lines, and no curves of any other type). Raises with a
    self-diagnosing message when no loop matches ([PB-PROFILE-MATCH]) — never
    falls back to a wrong profile."""
    Curve3DTypes = adsk.core.Curve3DTypes
    for profile in sketch.profiles:
        for loop in profile.profileLoops:
            nurbsN = arcsN = linesN = othersN = 0
            for pc in loop.profileCurves:
                ct = pc.geometry.curveType
                if ct == Curve3DTypes.NurbsCurve3DCurveType:
                    nurbsN += 1
                elif ct == Curve3DTypes.Arc3DCurveType:
                    arcsN += 1
                elif ct == Curve3DTypes.Line3DCurveType:
                    linesN += 1
                else:
                    othersN += 1
            if nurbsN == nurbs and arcsN == arcs and linesN == lines and othersN == 0:
                return profile
    raise Exception(
        f'Could not find profile in sketch "{sketch.name}" '
        f'({nurbs} NURBS + {arcs} arcs + {lines} lines)')

def find_circle_by_radius(sketch, radius, tolerance=0.0001):
    """Return the sketch circle whose radius matches within tolerance (cm).
    Raises when no circle matches — never falls back to a wrong circle."""
    circles = sketch.sketchCurves.sketchCircles
    for i in range(0, circles.count):
        c = circles.item(i)
        if abs(c.radius - radius) < tolerance:
            return c
    raise Exception(
        f'No circle of radius {radius} (tolerance {tolerance}) in sketch "{sketch.name}"')

def get_normal(entity) -> adsk.core.Vector3D:
    otyp = entity.objectType 
    if otyp == adsk.fusion.BRepFace.classType():
        return get_normal(entity.geometry)
    elif otyp == adsk.fusion.ConstructionPlane.classType():
        return entity.geometry.normal
    elif otyp == adsk.core.Plane.classType():
        return entity.normal
    elif otyp == adsk.fusion.Sketch.classType():
        # Need to recursively go into the geometry of the
        # reference plane, as it may be a construction plane
        # or a planar surface
        return get_normal(entity.referencePlane)
    else:
        return None
    

