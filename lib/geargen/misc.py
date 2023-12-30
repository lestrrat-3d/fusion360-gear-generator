import math
import adsk.core

def to_cm(mm):
    return mm/10

def to_mm(cm):
    return cm*10

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