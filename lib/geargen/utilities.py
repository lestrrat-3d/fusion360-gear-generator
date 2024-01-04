import adsk.core, adsk.fusion

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
    

