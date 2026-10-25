import adsk.core

def to_cm(mm):
    return mm/10

def to_mm(cm):
    return cm*10

def get_design(app=adsk.core.Application.get()):
    des = adsk.fusion.Design.cast(app.activeProduct)
    if not des:
        raise Exception('A Fusion design must be active when invoking this command.')
    return des


def existing_sketches(design):
    """Every sketch that already exists in the design. Paired with settle_sketch_display to tell the sketches a
    generator authored from the ones the user already had (see [PB-SETTLE-DISPLAY]). The sketches are kept as objects
    and compared with ==: one entity can return different entityToken strings over time, so tokens are never
    compared."""
    sketches = []
    for component in design.allComponents:
        for sketch in component.sketches:
            sketches.append(sketch)
    return sketches


def _is_known(sketch, known):
    for other in known:
        try:
            if sketch == other:
                return True
        except Exception:
            pass
    return False


def settle_sketch_display(design, known=None):
    """[PB-SETTLE-DISPLAY] Make Fusion's display agree with the solver once generation is done.

    A sketch authored through the API ends up with a browser node whose constraint icon still
    shows the state the sketch had when it was created, which is before any of its constraints
    existed. Opening and closing the sketch editor refreshes it, and so does this: each sketch
    is re-read, and one adsk.doEvents() lets Fusion process the updates it has queued. Measured
    on the bevel gear, this changes no geometry, no constraint and no dimension, only what the
    browser shows. The sketches are not recomputed (isComputeDeferred toggled): that re-solves
    every sketch, which costs as much as building it, and the control measurement in the
    playbook shows adsk.doEvents() alone refreshed a sketch it was not applied to.

    Sketches in known (from existing_sketches) are left alone, so only what the generator just
    authored is touched. Returns how many sketches were settled."""
    settled = 0
    for component in design.allComponents:
        for sketch in component.sketches:
            if known and _is_known(sketch, known):
                continue
            try:
                # The read is part of the refresh: it is what asks Fusion for the current
                # verdict rather than the one cached on the node.
                sketch.isFullyConstrained
            except Exception:
                continue
            settled += 1
    try:
        adsk.doEvents()
    except Exception:
        pass
    return settled
