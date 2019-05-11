# Author-
# Description-

import datetime as dt
import traceback

from adsk.core import Application, Matrix3D, Point3D, Vector3D, ValueInput, ObjectCollection, SurfaceTypes, Plane
from adsk.fusion import Design, Component, FeatureOperations

ui = None

ROOT_SKETCH = 'root-profile'
TIP_SKETCH = 'tip-profile'
WING_BODY = 'skin'

# rib locations in cm spanwise from root.
RIB_STATIONS = [0, 20]
# spanwise thickness of rib
RIB_THICKNESS = "1.5 mm"
# inset of rib from surface
RIB_INSET = "0.8 mm"
# chordwise positions of rib verticals in cm (at root position. locations proportional elsewhere)
RIB_VERTICAL_ROOT_LOCS_CM = [2.5, 5.0, 7.5, 10, 12.5]
RIB_POST_WIDTH_CM = 0.1

LOGFILE = '/Users/andy/logs/create-ribs.log'


def convert(x):
    if type(x) in (Vector3D, Point3D):
        return "{:.3f},{:.3f},{:.3f}".format(x.x, x.y, x.z)
    return str(x)


def log(*msgs):
    line = " ".join([convert(msg) for msg in msgs])
    line = "{}\n".format(line)

    with open(LOGFILE, 'a') as f:
        f.write(line)


def plane(comp):
    return comp.xZConstructionPlane


def vert_spanwise_plane(comp):
    return comp.yZConstructionPlane


def chordwise_coord(point):
    return point.x


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


def create_rib_vertical_post(component, comp_occurrence, rib_body, rib_post_loc, rib_post_width):
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
        post = create_rib_vertical_post(component, comp_occurrence, rib_body, rib_post_loc, rib_post_width)
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


def find_coplanar_face(body, plane):
    for face in body.faces:
        if face.geometry.surfaceType == SurfaceTypes.PlaneSurfaceType:
            face_plane = Plane.cast(face.geometry)
            if face_plane.isCoPlanarTo(plane.geometry):
                return face


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


def cell_between_planes(cells, plane1, plane2):

    log('cell between planes: plane1', plane1)
    return cells[0]


def boundary_fill_between_planes(component, comp_occurrence, body, plane1, plane2):
    boundary_fills = component.features.boundaryFillFeatures
    tools = ObjectCollection.create()
    tools.add(body)
    tools.add(plane1)
    tools.add(plane2)

    boundary_fill_input = boundary_fills.createInput(tools, FeatureOperations.NewBodyFeatureOperation)
    try:
        # Boundary fill will be created in sub component
        boundary_fill_input.creationOccurrence = comp_occurrence

        # Specify which cell is kept
        cells = boundary_fill_input.bRepCells
        cell_count = cells.count
        if cell_count == 3:
            cell = cell_in_the_middle(cells)
        elif cell_count == 2:
            cell = cell_between_planes(cells, plane1, plane2)
        else:
            raise Exception("Expected exactly 2 or 3 cells for boundary fill!")
        cell.isSelected = True

        # Create the boundary fill, based on the input data object
        boundary_fill_feature = boundary_fills.add(boundary_fill_input)
        assert 1 == boundary_fill_feature.bodies.count, 'expected a single rib body to be created'
        rib_body = boundary_fill_feature.bodies.item(0)
        return rib_body
    except:
        # rollback the boundary fill transaction
        boundary_fill_input.cancel()
        raise


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
        rootSketch = root.sketches.itemByName(ROOT_SKETCH)
        if rootSketch is None:
            raise ValueError('Root sketch "{}" not found'.format(ROOT_SKETCH))

        tipSketch = root.sketches.itemByName(TIP_SKETCH)
        if tipSketch is None:
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

        rib_vertical_fracs = [r / chord_length for r in RIB_VERTICAL_ROOT_LOCS_CM]
        log('rib vertical positions (mm)', RIB_VERTICAL_ROOT_LOCS_CM)
        log('rib vertical relative positions', rib_vertical_fracs)

        # create new component = 'ribs'
        ribsOcc = root.occurrences.addNewComponent(Matrix3D.create())
        ribs = Component.cast(ribsOcc.component)
        ribs.name = 'ribs'

        # now create the ribs
        for rib_id, rs in enumerate(RIB_STATIONS):
            rib_name = "rib_{}".format(rib_id + 1)
            create_rib(wing_body, rootSketch, ribs, ribsOcc, '{} mm'.format(rs), RIB_THICKNESS, RIB_INSET, rib_name,
                       rib_vertical_fracs, RIB_POST_WIDTH_CM)

    except:
        msg = 'Failed:\n{}'.format(traceback.format_exc())
        log(msg)
        if ui:
            ui.messageBox(msg)


def cell_in_the_middle(cells):
    """ from a list of 3 cells, return the one in the middle.
    """
    assert len(cells) == 3, "expected 3 cells - ensure the main body is solid and not shelled"
    # find centroids of bounding boxes
    bounding_boxes = [c.cellBody.boundingBox for c in cells]
    centroids = [centroid_of_bounding_box(bb) for bb in bounding_boxes]
    idx = index_of_point_in_middle(centroids)
    return cells[idx]


def centroid_of_bounding_box(bb):
    """ returns a Vector3D representing the centroid of the bounding box """
    min_point = bb.minPoint
    max_point = bb.maxPoint
    min_to_max = min_point.vectorTo(max_point)
    min_to_max.scaleBy(0.5)
    centre_vec = min_point.asVector()
    centre_vec.add(min_to_max)
    return centre_vec


def index_of_point_in_middle(points):
    """ determine which point is in the middle of 3 approximately colinear points in space.
        This is done by taking each point in turn, and looking at the directions of the vectors
        to the other 2 points. The point in the middle should have these vectors in approx 
        opposite directions. This can be checked by looking at the sign of the dot product
        between the vectors.
    """
    assert len(points) == 3, "expected3 points"
    dots = []
    for i in range(3):
        a = points[i]
        b = points[(i + 1) % 3]
        c = points[(i + 2) % 3]
        ab = b.copy()
        ab.subtract(a)
        ab.normalize()
        ac = c.copy()
        ac.subtract(a)
        ac.normalize()
        dot = ab.dotProduct(ac)
        dots.append(dot)

    # log("dots: {}".format(dots))

    negative_dots = [d for d in dots if d < 0]
    assert len(negative_dots) == 1, "expected exactly 1 negative dot product."

    # return index of the negative dot product
    for i in range(len(dots)):
        if dots[i] < 0:
            return i


if __name__ == '__main__':
    import doctest

    doctest.testmod()
