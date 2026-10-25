# Gear-agnostic Fusion helpers shared by the runtime and the generated gear add-ins.

from ...lib import fusion360utils as futil


def name_feature(obj, name):
    # Name a timeline feature, construction plane or sketch by its role. A refused
    # name must never fail a build, so the failure is logged and ignored.
    try:
        obj.name = name
    except Exception as e:
        futil.log(f'could not name "{name}": {e}')


def hide_construction_geometry(component):
    # Recursively hide every sketch, construction plane, and construction
    # axis under `component`, leaving only bodies visible ([PB-TREE-CLEANUP]).
    # A component reached through two occurrences is walked once; components
    # are compared with ==, never by entityToken, whose string can differ over
    # time for one entity.
    walked = []

    def walk(comp):
        if any(comp == seen for seen in walked):
            return
        walked.append(comp)
        for sketch in comp.sketches:
            sketch.isLightBulbOn = False
        for plane in comp.constructionPlanes:
            plane.isLightBulbOn = False
        for axis in comp.constructionAxes:
            axis.isLightBulbOn = False
        for occ in comp.occurrences:
            walk(occ.component)

    walk(component)
