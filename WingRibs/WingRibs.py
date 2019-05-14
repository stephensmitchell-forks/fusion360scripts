# Author-
# Description-

import datetime as dt
import traceback

from adsk.core import Application, Matrix3D, Point3D, Vector3D, ValueInput, ObjectCollection, SurfaceTypes, Plane
from adsk.fusion import Design, Component, FeatureOperations
from .utils import boundary_fill_between_planes
from .utils import log_func
from .utils import project_coord, centroid_of_bounding_box, find_coplanar_face, cell_between_planes
from functools import partial

ROOT_SKETCH = 'root-profile'
TIP_SKETCH = 'tip-profile'
WING_BODY = 'skin'

CREATE_COMPONENT_NAME = 'ribs'

# rib locations in cm spanwise from root.
RIB_STATIONS_CM = [0,2,4-0.12]
# spanwise thickness of rib
RIB_THICKNESS = "1.2 mm"
# inset of rib from surface
RIB_INSET = "0.8 mm"
# chordwise positions of rib verticals posts in cm (at root position. locations proportional elsewhere)
RIB_POST_ROOT_LOCS_CM = [2.5, 5.0, 7.5, 10, 12.5]
# chordwise width of rib posts
RIB_POST_WIDTH_CM = 0.1
TRIANGLE_LEN_CM = 0.5

LOGFILE = '/Users/andy/logs/create-ribs.log'
log = partial(log_func, LOGFILE)

VERT_DIRECTION = Vector3D.create(0., 0., 1.0)
SPANWISE_DIRECTION = Vector3D.create(0., 1., 0.)

ui = None


def vert_spanwise_plane(comp):
    return comp.yZConstructionPlane


def chordwise_coord(point):
    return point.x


def point(chordwise, spanwise, vertical):
    return Point3D.create(chordwise, spanwise, vertical)


def relative_location(from_loc, to_loc, frac):
    """
    returns location that a given fraction of the distance between the from and to points
    >>> relative_location(0,1,0.5)
    0.5
    >>> relative_location(-5,6,1)
    6
    >>> relative_location(-5,6,0)
    -5
    >>> relative_location(-5,15,0.75)
    10.0
    """
    full_dist = to_loc - from_loc
    dist = full_dist * frac
    loc = from_loc + dist
    return loc


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
    top = project_coord(bounding_box.maxPoint.asArray(), VERT_DIRECTION.asArray())
    bottom = project_coord(bounding_box.minPoint.asArray(), VERT_DIRECTION.asArray())
    spanwise_mid = project_coord(centroid_of_bounding_box(bounding_box).asArray(), SPANWISE_DIRECTION.asArray())

    log('top:', top, 'bottom:', bottom)
    assert plane1.isValid

    sketch = component.sketches.add(plane2, comp_occurrence)
    lines = sketch.sketchCurves.sketchLines

    p1 = sketch.modelToSketchSpace(point(chordwise=p1loc, spanwise=spanwise_mid, vertical=top - TRIANGLE_LEN_CM))
    p2 = sketch.modelToSketchSpace(point(chordwise=p1loc, spanwise=spanwise_mid + TRIANGLE_LEN_CM, vertical=top))
    p3 = sketch.modelToSketchSpace(point(chordwise=p1loc, spanwise=spanwise_mid - TRIANGLE_LEN_CM, vertical=top))

    lines.addByTwoPoints(p1, p2)
    lines.addByTwoPoints(p2, p3)
    lines.addByTwoPoints(p3, p1)

    p1 = sketch.modelToSketchSpace(point(chordwise=p1loc, spanwise=spanwise_mid, vertical=bottom + TRIANGLE_LEN_CM))
    p2 = sketch.modelToSketchSpace(point(chordwise=p1loc, spanwise=spanwise_mid + TRIANGLE_LEN_CM, vertical=bottom))
    p3 = sketch.modelToSketchSpace(point(chordwise=p1loc, spanwise=spanwise_mid - TRIANGLE_LEN_CM, vertical=bottom))

    lines.addByTwoPoints(p1, p2)
    lines.addByTwoPoints(p2, p3)
    lines.addByTwoPoints(p3, p1)

    # extrude the 2 triangular profiles just created
    assert sketch.profiles.count ==2, "expected 2 profiles in the sketch"
    profile = sketch.profiles.item(0)
    extrudes = component.features.extrudeFeatures
    top_triangle_extrusion = extrudes.addSimple(profile, ValueInput.createByReal(rib_post_width), FeatureOperations.NewBodyFeatureOperation)
    top_triangle = top_triangle_extrusion.bodies.item(0)
    top_triangle.name = 'top_triangle'

    profile = sketch.profiles.item(1)
    extrudes = component.features.extrudeFeatures
    bottom_triangle_extrusion = extrudes.addSimple(profile, ValueInput.createByReal(rib_post_width), FeatureOperations.NewBodyFeatureOperation)
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

    except:
        msg = 'Failed:\n{}'.format(traceback.format_exc())
        log(msg)
        if ui:
            ui.messageBox(msg)





# def cell_in_the_middle(cells):
#     """ from a list of 3 cells, return the one in the middle.
#     """
#     assert len(cells) == 3, "expected 3 cells - ensure the main body is solid and not shelled"
#     # find centroids of bounding boxes
#     bounding_boxes = [c.cellBody.boundingBox for c in cells]
#     centroids = [centroid_of_bounding_box(bb) for bb in bounding_boxes]
#     idx = index_of_point_in_middle(centroids)
#     return cells[idx]




# def index_of_point_in_middle(points):
#     """ determine which point is in the middle of 3 approximately colinear points in space.
#         This is done by taking each point in turn, and looking at the directions of the vectors
#         to the other 2 points. The point in the middle should have these vectors in approx
#         opposite directions. This can be checked by looking at the sign of the dot product
#         between the vectors.
#     """
#     assert len(points) == 3, "expected3 points"
#     dots = []
#     for i in range(3):
#         a = points[i]
#         b = points[(i + 1) % 3]
#         c = points[(i + 2) % 3]
#         ab = b.copy()
#         ab.subtract(a)
#         ab.normalize()
#         ac = c.copy()
#         ac.subtract(a)
#         ac.normalize()
#         dot = ab.dotProduct(ac)
#         dots.append(dot)
#
#     # log("dots: {}".format(dots))
#
#     negative_dots = [d for d in dots if d < 0]
#     assert len(negative_dots) == 1, "expected exactly 1 negative dot product."
#
#     # return index of the negative dot product
#     for i in range(len(dots)):
#         if dots[i] < 0:
#             return i
#

if __name__ == '__main__':
    import doctest

    doctest.testmod()
