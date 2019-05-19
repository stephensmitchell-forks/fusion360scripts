# Author-
# Description-

import datetime as dt
import traceback
from functools import partial

from adsk.core import Application, Matrix3D, ValueInput, ObjectCollection
from adsk.fusion import Design, Component, FeatureOperations
from .orientation import VERTICAL_UP_DIRECTION, SPANWISE_DIRECTION
from .orientation import vert_spanwise_plane, point, chordwise_coord
from .utils import boundary_fill_between_planes, relative_location
from .utils import load_settings
from .utils import log_func
from .utils import project_coord, centroid_of_bounding_box, find_coplanar_face


app = Application.get()
des = Design.cast(app.activeProduct)
ui = app.userInterface
settings = load_settings('ribSettings', ui)


ROOT_SKETCH = 'root-profile'
TIP_SKETCH = 'tip-profile'
WING_BODY = 'skin'

CREATE_COMPONENT_NAME = 'ribs'

# rib locations in cm spanwise from root.
# RIB_STATIONS_CM = [0, 2, 4 - 0.12]
RIB_STATIONS_CM = [0, 2, 4, 6, 8, 10 - 0.12]
# spanwise thickness of rib
RIB_THICKNESS = "1.2 mm"
# inset of rib from surface
RIB_INSET = "0.8 mm"
# chordwise positions of rib verticals posts in cm (at root position. locations proportional elsewhere)
# RIB_POST_ROOT_LOCS_CM = [2.5, 5.0, 7.5 , 10, 12.5]
RIB_POST_ROOT_LOCS_CM = [1.0, 2, 3, 4, 5]
# chordwise width of rib posts
RIB_POST_WIDTH_CM = 0.1
TRIANGLE_LEN_CM = 0.2

LOGFILE = '/Users/andy/logs/create-ribs.log'
log = partial(log_func, LOGFILE)

ui = None



def create_rib_vertical_post(component, comp_occurrence, wing_body, rib_body, rib_post_loc, rib_post_width):
    """
    create a vertical rib post
    """
    # log('creating rib post of width', rib_post_width, ' at ', rib_post_loc)
    # create 2 planes, rib_post_width apart, centered on rib_post_loc

    p1loc = rib_post_loc - (rib_post_width / 2)
    p2loc = rib_post_loc + (rib_post_width / 2)
    planes = component.constructionPlanes

    plane1_input = planes.createInput()
    plane1_input.setByOffset(vert_spanwise_plane(component), ValueInput.createByReal(p1loc))
    plane1 = planes.add(plane1_input)

    plane2_input = planes.createInput()
    plane2_input.setByOffset(vert_spanwise_plane(component), ValueInput.createByReal(p2loc))
    plane2 = planes.add(plane2_input)

    post = boundary_fill_between_planes(component, comp_occurrence, rib_body, plane1, plane2)

    # hide the construction planes
    plane1.isLightBulbOn = False
    plane2.isLightBulbOn = False

    # get dimensions of post
    bounding_box = post.boundingBox
    top = project_coord(bounding_box.maxPoint.asArray(), VERTICAL_UP_DIRECTION.asArray())
    bottom = project_coord(bounding_box.minPoint.asArray(), VERTICAL_UP_DIRECTION.asArray())
    spanwise_mid = project_coord(centroid_of_bounding_box(bounding_box).asArray(), SPANWISE_DIRECTION.asArray())

    log('top:', top, 'bottom:', bottom)
    assert plane1.isValid

    sketch = component.sketches.add(plane2, comp_occurrence)
    lines = sketch.sketchCurves.sketchLines

    tri_side = TRIANGLE_LEN_CM

    # only create triangles if the section is tall enough
    if top - bottom > 2 * tri_side:
        p1 = sketch.modelToSketchSpace(point(chordwise=p1loc, spanwise=spanwise_mid, vertical=top - tri_side))
        p2 = sketch.modelToSketchSpace(point(chordwise=p1loc, spanwise=spanwise_mid + tri_side, vertical=top))
        p3 = sketch.modelToSketchSpace(point(chordwise=p1loc, spanwise=spanwise_mid - tri_side, vertical=top))

        lines.addByTwoPoints(p1, p2)
        lines.addByTwoPoints(p2, p3)
        lines.addByTwoPoints(p3, p1)

        p1 = sketch.modelToSketchSpace(point(chordwise=p1loc, spanwise=spanwise_mid, vertical=bottom + tri_side))
        p2 = sketch.modelToSketchSpace(point(chordwise=p1loc, spanwise=spanwise_mid + tri_side, vertical=bottom))
        p3 = sketch.modelToSketchSpace(point(chordwise=p1loc, spanwise=spanwise_mid - tri_side, vertical=bottom))

        lines.addByTwoPoints(p1, p2)
        lines.addByTwoPoints(p2, p3)
        lines.addByTwoPoints(p3, p1)

        # extrude the 2 triangular profiles just created
        assert sketch.profiles.count == 2, "expected 2 triangle profiles in the sketch just created"
        profile = sketch.profiles.item(0)
        extrudes = component.features.extrudeFeatures
        top_triangle_extrusion = extrudes.addSimple(profile, ValueInput.createByReal(rib_post_width),
                                                    FeatureOperations.NewBodyFeatureOperation)
        top_triangle = top_triangle_extrusion.bodies.item(0)
        top_triangle.name = 'top_triangle'

        profile = sketch.profiles.item(1)
        extrudes = component.features.extrudeFeatures
        bottom_triangle_extrusion = extrudes.addSimple(profile, ValueInput.createByReal(rib_post_width),
                                                       FeatureOperations.NewBodyFeatureOperation)
        bottom_triangle = bottom_triangle_extrusion.bodies.item(0)
        bottom_triangle.name = 'bottom_triangle'

        # now trim the triangles to the intersection with the wing body
        tool_bodies = ObjectCollection.create()
        tool_bodies.add(wing_body)
        combines = component.features.combineFeatures

        combine_input = combines.createInput(top_triangle, tool_bodies)
        combine_input.isKeepToolBodies = True
        combine_input.isNewComponent = False
        combine_input.operation = FeatureOperations.IntersectFeatureOperation
        combines.add(combine_input)

        combine_input = combines.createInput(bottom_triangle, tool_bodies)
        combine_input.isKeepToolBodies = True
        combine_input.isNewComponent = False
        combine_input.operation = FeatureOperations.IntersectFeatureOperation
        combines.add(combine_input)

    return post


def create_rib(wing_body, root_sketch, component, comp_occurrence, dist_from_root, rib_thickness, rib_inset, rib_name,
               rib_post_relative_positions, rib_post_width):
    root_plane = root_sketch.referencePlane

    # Create rib body and return it and the 2 construction planes along each fact
    rib_body, plane1, plane2 = create_rib_body(component, comp_occurrence, wing_body, root_plane, dist_from_root,
                                               rib_thickness)
    rib_body.name = rib_name

    # find the chordwise extremities of the wing body
    start_coord = chordwise_coord(rib_body.boundingBox.minPoint)
    end_coord = chordwise_coord(rib_body.boundingBox.maxPoint)
    rib_post_locs = [relative_location(start_coord, end_coord, frac) for frac in rib_post_relative_positions]

    for i, rib_post_loc in enumerate(rib_post_locs):
        post = create_rib_vertical_post(component, comp_occurrence, wing_body, rib_body, rib_post_loc, rib_post_width)
        post.name = '{}_post_{}'.format(rib_name, i + 1)

    # find the faces aligned with the construction planes
    plane1_face = find_coplanar_face(rib_body, plane1)
    plane2_face = find_coplanar_face(rib_body, plane2)
    assert plane1_face is not None, 'plane1'
    assert plane2_face is not None, 'plane2'

    # use shell tool to remove center of rib, and the 2 faces
    # Create a collection of entities for shell
    entities1 = ObjectCollection.create()
    entities1.add(plane1_face)
    entities1.add(plane2_face)

    # Create a shell feature
    shell_feats = component.features.shellFeatures
    is_tangent_chain = False
    shell_feature_input = shell_feats.createInput(entities1, is_tangent_chain)
    rib_thickness = ValueInput.createByString(rib_inset)
    shell_feature_input.insideThickness = rib_thickness
    shell_feats.add(shell_feature_input)


def create_rib_body(component, comp_occurrence, wing_body, root_plane, dist_from_root, thickness):
    """
    Creates 2 construction planes and a solid rib body using a boundary fill between the 2 planes and the wing body
    """
    # Create 2 construction planes:
    #   1) offset from root sketch plane
    #   2) offset from the first
    planes = component.constructionPlanes

    plane1_input = planes.createInput()
    plane1_input.setByOffset(root_plane, ValueInput.createByString(dist_from_root))
    plane1 = planes.add(plane1_input)

    plane2_input = planes.createInput()
    plane2_input.setByOffset(plane1, ValueInput.createByString(thickness))
    plane2 = planes.add(plane2_input)

    rib_body = boundary_fill_between_planes(component, comp_occurrence, wing_body, plane1, plane2)

    # hide the construction planes
    plane1.isLightBulbOn = False
    plane2.isLightBulbOn = False

    return rib_body, plane1, plane2


def run(context):
    """
    Main entrypoint
    """
    global ui
    try:
        log('--------------------------------')
        log(dt.datetime.now(), __file__)
        log()
        app = Application.get()
        des = Design.cast(app.activeProduct)
        ui = app.userInterface
        root = Component.cast(des.rootComponent)


        # locate the root and tip sketches
        root_sketch = root.sketches.itemByName(ROOT_SKETCH)
        if root_sketch is None:
            raise ValueError('Root sketch "{}" not found'.format(ROOT_SKETCH))

        tip_sketch = root.sketches.itemByName(TIP_SKETCH)
        if tip_sketch is None:
            raise ValueError('Tip sketch "{}" not found'.format(TIP_SKETCH))

        # locate the wing body
        wing_body = root.bRepBodies.itemByName(WING_BODY)
        if wing_body is None:
            raise ValueError('Wing body "{}" not found'.format(WING_BODY))

        # find the chordwise extremities of the wing body
        start_coord = chordwise_coord(wing_body.boundingBox.minPoint)
        end_coord = chordwise_coord(wing_body.boundingBox.maxPoint)
        chord_length = end_coord - start_coord
        log('start, end, chord length', start_coord, end_coord, chord_length)

        rib_vertical_fracs = [r / chord_length for r in RIB_POST_ROOT_LOCS_CM]
        log('rib vertical positions (mm)', RIB_POST_ROOT_LOCS_CM)
        log('rib vertical relative positions', rib_vertical_fracs)

        # create new component
        component_occurrence = root.occurrences.addNewComponent(Matrix3D.create())
        component = Component.cast(component_occurrence.component)
        component.name = CREATE_COMPONENT_NAME

        # now create the ribs
        for rib_id, rs in enumerate(RIB_STATIONS_CM):
            rib_name = "rib_{}".format(rib_id + 1)
            create_rib(wing_body, root_sketch, component, component_occurrence,
                       '{} cm'.format(rs), RIB_THICKNESS, RIB_INSET, rib_name,
                       rib_vertical_fracs, RIB_POST_WIDTH_CM)

    except Exception as ex:
        msg = 'Failed:\n{}'.format(traceback.format_exc())
        log(msg)
        if ui:
            ui.messageBox(str(ex))


if __name__ == '__main__':
    import doctest

    doctest.testmod()
