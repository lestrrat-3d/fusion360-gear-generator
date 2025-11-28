import math
import adsk.core

def to_cm(mm: float) -> float:
    """Convert millimeters to centimeters."""
    return mm / 10.0

def get_ui(app=adsk.core.Application.get()):
    ui = app.userInterface
    if not ui:
        raise Exception('No UI object available. Please run this script from within Fusion 360')
    return ui

def get_design(app=adsk.core.Application.get()):
    des = adsk.fusion.Design.cast(app.activeProduct)
    if not des:
        raise Exception('A Fusion design must be active when invoking this command.')
    return des