# Author-
# Description-Creates 3d labprint-like spars


import datetime as dt
import traceback
from functools import partial

from adsk.core import Application, Matrix3D, ValueInput
from adsk.fusion import Design, Component
from .utils import log_func

"""
Creates 3DLabprint-like spars on a wing solid.

Inputs:
    name of wing body sketch - assumed solid at this stage - use shell-tool on it later if required
    name of spars sketch
"""

WING_BODY = 'skin'
SPARS_SKETCH = 'spars'
CREATE_COMPONENT_NAME = 'spars'
SPAR_THICKNESS_CM = 0.1


LOGFILE = '/Users/andy/logs/create-spars.log'
log = partial(log_func, LOGFILE)


def horizontal_plane(component):
    return component.xYConstructionPlane


def create_spar_from_line(component, component_occurrence, sketch, line, spar_width):
    log(line)
    start_point = sketch.sketchToModelSpace(line.startSketchPoint.geometry)
    end_point = sketch.sketchToModelSpace(line.endSketchPoint.geometry)
    log(start_point, end_point)

    # create construction plane, centered on line, at 90 degrees to horizontal plan
    planes = component.constructionPlanes
    plane_input = planes.createInput()

    angle = ValueInput.createByString('90.0 deg')
    plane_input.setByAngle(line, angle, horizontal_plane(component))
    center_plane = planes.add(plane_input)

    # create offset planes one either side of the center plane
    plane_input = planes.createInput()
    offset = ValueInput.createByReal(spar_width)
    plane_input.setByOffset(center_plane, offset)
    plane1 = planes.add(plane_input)

    plane_input = planes.createInput()
    offset = ValueInput.createByReal(-1 * spar_width)
    plane_input.setByOffset(center_plane, offset)
    plane2 = planes.add(plane_input)

    # now use boundary fill to create spar between construction planes and wing body/


def run(context):
    global ui
    try:

        app = Application.get()
        des = Design.cast(app.activeProduct)
        ui = app.userInterface

        log('--------------------------------')
        log(dt.datetime.now(), __file__)
        log()

        root = Component.cast(des.rootComponent)

        # locate the spars sketch
        sketch = root.sketches.itemByName(SPARS_SKETCH)
        if sketch is None:
            raise ValueError('Sketch "{}" not found'.format(SPARS_SKETCH))

        # locate the wing body
        wing_body = root.bRepBodies.itemByName(WING_BODY)
        if wing_body is None:
            raise ValueError('Wing body "{}" not found'.format(WING_BODY))

        # create new component
        component_occurrence = root.occurrences.addNewComponent(Matrix3D.create())
        component = Component.cast(component_occurrence.component)
        component.name = CREATE_COMPONENT_NAME

        log('created component')

        # now create the spars, one for each line on the sketch.

        lines = sketch.sketchCurves.sketchLines
        log('num lines:', lines.count)
        for line in lines:
            create_spar_from_line(component, component_occurrence, sketch, line, SPAR_THICKNESS_CM)


        ui.messageBox('done')

    except:
        msg = 'Failed:\n{}'.format(traceback.format_exc())
        log(msg)
        if ui:
            ui.messageBox(msg)


if __name__ == '__main__':
    import doctest
    doctest.testmod()

