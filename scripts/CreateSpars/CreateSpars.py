# Author-
# Description-Creates 3d labprint-like spars


import datetime as dt
import traceback
from functools import partial

from adsk.core import Application, Matrix3D, ValueInput, Point3D, ObjectCollection
from adsk.fusion import Design, Component, FeatureOperations
from .f360lib.orientation import VERTICAL_UP_DIRECTION, SPANWISE_DIRECTION
from .f360lib.utils import boundary_fill_between_planes, project_coord, load_settings, item_by_name
from .f360lib.utils import log_func

"""
Creates 3DLabprint-like spars on a wing solid.

Inputs:
    name of wing body sketch - assumed solid at this stage - use shell-tool on it later if required
    name of spars sketch
"""

app = Application.get()
design = Design.cast(app.activeProduct)
ui = app.userInterface
settings = load_settings(design.userParameters, 'ribSettings', ui)
log = partial(log_func, settings.LOGFILE)


def horizontal_plane(component):
    return component.xYConstructionPlane


def create_spar_from_line(component, component_occurrence, wing_body, spar_lines_sketch, line, spar_width):
    log(line)
    start_point = spar_lines_sketch.sketchToModelSpace(line.startSketchPoint.geometry)
    end_point = spar_lines_sketch.sketchToModelSpace(line.endSketchPoint.geometry)
    log(start_point, end_point)

    # create construction plane, centered on line, at 90 degrees to horizontal plan
    planes = component.constructionPlanes
    plane_input = planes.createInput()

    angle = ValueInput.createByString('90.0 deg')
    plane_input.setByAngle(line, angle, horizontal_plane(component))
    center_plane = planes.add(plane_input)

    # create offset planes one either side of the center plane
    plane_input = planes.createInput()
    offset = ValueInput.createByReal(spar_width / 2)
    plane_input.setByOffset(center_plane, offset)
    plane1 = planes.add(plane_input)

    plane_input = planes.createInput()
    offset = ValueInput.createByReal(-1 * spar_width / 2)
    plane_input.setByOffset(center_plane, offset)
    plane2 = planes.add(plane_input)

    for p in [center_plane, plane1, plane2]:
        p.isLightBulbOn = False

    # now use boundary fill to create spar between construction planes and wing body
    spar = boundary_fill_between_planes(component, component_occurrence, wing_body, plane1, plane2)

    # now get bounding box of spar and find min an max spanwise dimensions

    min_point = spar.boundingBox.minPoint
    max_point = spar.boundingBox.maxPoint
    sw1 = project_coord(min_point.asArray(), SPANWISE_DIRECTION.asArray())
    sw2 = project_coord(max_point.asArray(), SPANWISE_DIRECTION.asArray())
    min_spanwise = min(sw1, sw2)
    max_spanwise = max(sw1, sw2)

    v1 = project_coord(min_point.asArray(), VERTICAL_UP_DIRECTION.asArray())
    v2 = project_coord(max_point.asArray(), VERTICAL_UP_DIRECTION.asArray())

    min_vertical = min(v1, v2)
    max_vertical = max(v1, v2)
    log("Spanwise range = {} to {}".format(min_spanwise, max_spanwise))
    log("Vertical range = {} to {}".format(min_vertical, max_vertical))

    # create sketch and draw circles in a row spanwise
    spar_face_sketch = component.sketches.add(center_plane)
    circle_locs = []
    loc = min_spanwise + settings.SPAR_CIRCLE_SPACING_CM
    while loc + settings.SPAR_CIRCLE_DIAMETER_CM / 2 < max_spanwise:
        circle_locs.append(loc)
        loc = loc + settings.SPAR_CIRCLE_SPACING_CM

    log('Circle locs:', circle_locs)
    vertical_center = 0

    # sketch origin seems to be centered on the body. x seems to be spanwise
    # TODO: formalise this, for different part orientations. Need a way to deduce sketch orientation
    x_offset = (max_spanwise - min_spanwise) / 2
    # create circles
    for loc in circle_locs:
        spar_face_sketch.sketchCurves.sketchCircles.addByCenterRadius(
            Point3D.create(loc - x_offset, vertical_center, 0),
            settings.SPAR_CIRCLE_DIAMETER_CM / 2)

    profiles = ObjectCollection.create()
    for c in spar_face_sketch.profiles:
        profiles.add(c)
    # now extrude the circles to cut the spar
    extrudes = component.features.extrudeFeatures
    extrude_input = extrudes.createInput(profiles, FeatureOperations.CutFeatureOperation)
    # extrude_input.setAllExtent(ExtentDirections.SymmetricExtentDirection)
    distanceForCut = ValueInput.createByString('2 cm')  # some distance > we need.
    extrude_input.setSymmetricExtent(distanceForCut, True)
    extrude_input.participantBodies = [spar]

    extrudes.add(extrude_input)

    return spar


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

        # locate the spars sketch & wing body
        sketch = item_by_name(root.sketches, settings.SPARS_SKETCH)
        wing_body = item_by_name(root.bRepBodies, settings.WING_BODY)

        # create new component
        component_occurrence = root.occurrences.addNewComponent(Matrix3D.create())
        component = Component.cast(component_occurrence.component)
        component.name = settings.SPARS_COMPONENT_NAME

        # now create the spars, one for each line on the sketch.
        lines = sketch.sketchCurves.sketchLines
        log('num lines:', lines.count)
        for i, line in enumerate(lines):
            spar = create_spar_from_line(component, component_occurrence, wing_body, sketch, line,
                                         settings.SPAR_THICKNESS_CM)
            spar.name = "spar_{}".format(i + 1)
            log('Created spar', spar.name)

        ui.messageBox('done')

    except Exception as ex:
        msg = 'Failed:\n{}'.format(traceback.format_exc())
        log(msg)
        if ui:
            ui.messageBox(str(ex))


if __name__ == '__main__':
    import doctest

    doctest.testmod()
