# Author-
# Description-

import datetime as dt
import traceback

from adsk.core import Application, Matrix3D, Point3D, Vector3D, ValueInput, ObjectCollection
from adsk.fusion import Design, Component, FeatureOperations

ui = None

ROOT_SKETCH = 'root-profile'
TIP_SKETCH = 'tip-profile'
WING_BODY = 'skin'

# rib locations in mm
RIB_STATIONS = [25] #[x + 5 for x in range(0, 40, 10)]
RIB_THICKNESS = "1 mm"

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


def create_rib(wing_body, root_sketch, component, comp_occurrence, dist_from_root, thickness, rib_name):
    root_plane = root_sketch.referencePlane

    # Create rib as using boundary fill, between the 2 construction planes, and the wing body
    rib_body = create_rib_body(component, comp_occurrence, wing_body, root_plane, dist_from_root, thickness)
    rib_body.name = rib_name


def create_rib_body(component, comp_occurrence, wing_body, root_plane, dist_from_root, thickness):
    """
    Creates a solid rib body using a boundary fill between the 2 planes
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

    boundary_fills = component.features.boundaryFillFeatures
    tools = ObjectCollection.create()
    tools.add(wing_body)
    tools.add(plane1)
    tools.add(plane2)
    boundary_fill_input = boundary_fills.createInput(tools, FeatureOperations.NewBodyFeatureOperation)
    try:
        # Boundary fill will be created in sub component
        boundary_fill_input.creationOccurrence = comp_occurrence

        # Specify which cell is kept
        assert boundary_fill_input.bRepCells.count == 3, "expected 3 cells"

        cell = cell_in_the_middle(boundary_fill_input.bRepCells)
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
        wingBody = root.bRepBodies.itemByName(WING_BODY)
        if wingBody is None:
            raise ValueError('Wing body "{}" not found'.format(WING_BODY))

        # create new component = 'ribs'
        ribsOcc = root.occurrences.addNewComponent(Matrix3D.create())
        ribs = Component.cast(ribsOcc.component)
        ribs.name = 'ribs'

        # now create the ribs
        for rib_id, rs in enumerate(RIB_STATIONS):
            rib_name = "rib_{}".format(rib_id+1)
            create_rib(wingBody, rootSketch, ribs, ribsOcc, '{} mm'.format(rs), RIB_THICKNESS, rib_name)

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
    log("index of point in middle", idx)
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
        # log()
        # log('i=', i)
        # log('a', a)
        # log('b', b)
        # log('c', c)
        # log('ab', ab)
        # log('ac', ac)
        # log('dot', dot)

        dots.append(dot)

    log("dots: {}".format(dots))

    negative_dots = [d for d in dots if d < 0]
    assert len(negative_dots) == 1, "expected exactly 1 negative dot product."

    # return index of the negative dot product
    for i in range(len(dots)):
        if dots[i] < 0:
            return i
