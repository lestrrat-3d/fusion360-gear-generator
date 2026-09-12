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


def sketch_tokens(design):
    """Every sketch that already exists in the design, by entity token. Paired with
    settle_sketch_display to tell the sketches a generator authored from the ones the user
    already had (see [PB-SETTLE-DISPLAY])."""
    tokens = set()
    for component in design.allComponents:
        for sketch in component.sketches:
            try:
                tokens.add(sketch.entityToken)
            except Exception:
                pass
    return tokens


def settle_sketch_display(design, known_tokens=None):
    """[PB-SETTLE-DISPLAY] Make Fusion's display agree with the solver once generation is done.

    A sketch authored through the API ends up with a browser node whose constraint icon still
    shows the state the sketch had when it was created, which is before any of its constraints
    existed. Opening and closing the sketch editor refreshes it, and so does this: each sketch
    is recomputed and re-read, and one adsk.doEvents() lets Fusion process the updates it has
    queued. Measured on the bevel gear, this changes no geometry, no constraint and no
    dimension, only what the browser shows.

    Sketches whose token is in known_tokens are left alone, so only what the generator just
    authored is touched. Returns how many sketches were settled."""
    settled = 0
    for component in design.allComponents:
        for sketch in component.sketches:
            if known_tokens:
                try:
                    if sketch.entityToken in known_tokens:
                        continue
                except Exception:
                    pass
            try:
                sketch.isComputeDeferred = True
                sketch.isComputeDeferred = False
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