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
RIB_STATIONS = [x+5 for x in range (0,40,10)]
RIB_THICKNESS = "1 mm"

LOGFILE = '/Users/andy/logs/create-ribs.log'


def convert(x):
    if type(x)  in (Vector3D, Point3D):
        return "{:.3f},{:.3f},{:.3f}".format(x.x, x.y, x.z)
    return str(x)


def log(*msgs):
    line = " ".join([convert(msg) for msg in msgs])
    line = "{}\n".format(line)

    with open(LOGFILE, 'a') as f:
        f.write(line)


def plane(comp):
    return comp.xZConstructionPlane


def run(context):
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

        def create_rib(dist_from_root, thickness):
            rootPlane = rootSketch.referencePlane
            tipPlane = tipSketch.referencePlane

            # Create 2 construction planes: 
            #   1) offset from root sketch plane
            #   2) offset from the first
            planes = ribs.constructionPlanes

            plane1Input = planes.createInput()
            plane1Input.setByOffset(rootPlane, ValueInput.createByString(dist_from_root))
            plane1 = planes.add(plane1Input)

            plane2Input = planes.createInput()
            plane2Input.setByOffset(plane1, ValueInput.createByString(thickness))
            plane2 = planes.add(plane2Input)

            # Create rib as using boundary fill, between the 2 construction planes, and the wing body
            boundaryFills = ribs.features.boundaryFillFeatures
            tools = ObjectCollection.create()
            tools.add(wingBody)
            tools.add(plane1)
            tools.add(plane2)


            boundaryFillInput = boundaryFills.createInput(tools, FeatureOperations.NewBodyFeatureOperation)
            try:

                # Boundary fill will be created in sub component
                boundaryFillInput.creationOccurrence = ribsOcc

                # Specify which cell is kept
                assert boundaryFillInput.bRepCells.count == 3, "expected 3 cells"

                # volumes = [cell.cellBody.volume for cell in boundaryFillInput.bRepCells]
                # log('Volumes: {}'.format(volumes))


                cell = cell_in_the_middle(boundaryFillInput.bRepCells)
                cell.isSelected = True

                # Create the boundary fill, based on the input data object
                boundaryFills.add(boundaryFillInput)
            except:
                # rollback the boundary fill transaction
                boundaryFillInput.cancel()
                raise
            # end of create_rib
            # ----------------------------------

        # now create the ribs
        for rs in RIB_STATIONS:
            create_rib('{} mm'.format(rs), RIB_THICKNESS)

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
