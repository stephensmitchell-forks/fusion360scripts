import doctest
import math
import os.path
import unittest

from adsk.core import Vector3D, Point3D, SurfaceTypes, Plane, ObjectCollection, Application
from adsk.fusion import FeatureOperations


def load_settings(user_parameters, user_parameter_name, ui):
    """
    Reads a filename from the comment field of the given parameter
    and loads settings from the file.
    :param the collection of user parameters
    :param user_parameter_name: name of the user parameter to read
    :param reference to the UI, used to report errors. (Since logfile typically not yet opened)
    :return: map of settings
    """
    try:
        param = user_parameters.itemByName(user_parameter_name)
        if param is None:
            raise Exception("No user parameter '{}' defined.".format(user_parameter_name))

        filename = param.comment
        if filename is None or not (os.path.exists(filename) and os.path.isfile(filename)):
            raise Exception(
                "Expected comment field of parameter {} to contain the pathame of a settings file but was '{}'"
                    .format(user_parameter_name, filename))

        return load_settings_from_file(filename)

    except Exception as ex:
        if ui:
            ui.messageBox(str(ex))
        raise ex


def load_settings_from_file(filename):
    """
    loads settings from the specified file, which is assumed to be in python.
    """
    assert os.path.exists(filename), "settings file: '{}' does not exist".format(filename)
    assert os.path.isfile(filename), "settings file: '{}' is not a file".format(filename)

    import importlib.util
    spec = importlib.util.spec_from_file_location("settings", filename)
    settings = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(settings)
    return settings


def to_string(x):
    """
    provides string representations of common types
    """
    if type(x) in (Vector3D, Point3D):
        return "{:.3f},{:.3f},{:.3f}".format(x.x, x.y, x.z)
    return str(x)


def log_func(file, *msgs):
    line = " ".join([to_string(msg) for msg in msgs])
    line = "{}\n".format(line)

    with open(file, 'a') as f:
        f.write(line)


def centroid_of_bounding_box(bb):
    """ returns a Vector3D representing the centroid of the bounding box """
    min_point = bb.minPoint
    max_point = bb.maxPoint
    min_to_max = min_point.vectorTo(max_point)
    min_to_max.scaleBy(0.5)
    centre_vec = min_point.asVector()
    centre_vec.add(min_to_max)
    return centre_vec


def is_point_between_planes(point, plane1, plane2):
    assert plane1.geometry.normal.isEqualTo(plane2.geometry.normal), "plane normals expected the same"
    normal = plane1.geometry.normal

    plane1_coord = project_coord(plane1.geometry.origin.asArray(), normal.asArray())
    point_coord = project_coord(point.asArray(), normal.asArray())
    plane2_coord = project_coord(plane2.geometry.origin.asArray(), normal.asArray())
    between = min(plane1_coord, plane2_coord) <= point_coord <= max(plane1_coord, plane2_coord)
    return between


def cell_between_planes(cells, plane1, plane2):
    # find centroids of bounding boxes
    bounding_boxes = [c.cellBody.boundingBox for c in cells]
    centroids = [centroid_of_bounding_box(bb) for bb in bounding_boxes]

    # see which centroids are between the construction planes
    in_betweens = [is_point_between_planes(centroid.asPoint(), plane1, plane2) for centroid in centroids]
    assert len([b for b in in_betweens if b is True]) == 1, \
        "Expected 1 cell in between planes. Are you sure that the wing body is not shelled?"

    for i in range(len(in_betweens)):
        if in_betweens[i] is True:
            return cells[i]


def find_coplanar_face(body, plane):
    """
    returns the face of the body which is coplanar with the given plane
    """
    for face in body.faces:
        if face.geometry.surfaceType == SurfaceTypes.PlaneSurfaceType:
            face_plane = Plane.cast(face.geometry)
            if face_plane.isCoPlanarTo(plane.geometry):
                return face


def project_coord(point, direction):
    """
    returns the projection of a point in a direction

    >>> project_coord([1,2,3], [0,1,0])
    2.0

    >>> project_coord([1,2,3], [1,0,0])
    1.0

    >>> project_coord([1,2,3], [0,0,1])
    3.0

    >>> project_coord([1,2,3], [0,0,2])
    3.0

    """
    a, b, c = point
    x, y, z = direction

    return a * x + b * y + c * z / math.sqrt(x * x + y * y + z * z)


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
        if cell_count in [2, 3]:
            cell = cell_between_planes(cells, plane1, plane2)
        else:
            raise Exception("Expected exactly 2 or 3 cells for boundary fill. Got {}!".format(cell_count))
        cell.isSelected = True

        # Create the boundary fill, based on the input data object
        boundary_fill_feature = boundary_fills.add(boundary_fill_input)
        assert 1 == boundary_fill_feature.bodies.count, 'expected a single rib body to be created'
        rib_body = boundary_fill_feature.bodies.item(0)
        return rib_body
    except:
        # rollback the boundary fill transaction
        try:
            boundary_fill_input.cancel()
        except Exception as ignored:
            pass

        raise


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


class Tests(unittest.TestCase):
    def test_load_settings_from_file(self):
        """
        loads some example settings from a file and checks that the contents get set in the current environment
        """
        script_dir = os.path.dirname(os.path.abspath(__file__))
        test_file = os.path.join(script_dir, 'test/test_settings.py')

        settings = load_settings_from_file(test_file)
        self.assertEqual(settings.TEST_SETTING, 42)
        self.assertEqual(settings.TEST_LIST, [0, 2, 4])


def load_tests(loader, tests, ignore):
    """ test loader to include doctest into unit test suite"""
    tests.addTests(doctest.DocTestSuite())
    return tests


if __name__ == '__main__':
    # #doctest.testmod()
    #
    # test_suite = unittest.TestSuite()
    # #test_suite.addTests(unittest.makeSuite(Tests))
    # test_suite.addTest(doctest.DocTestSuite())
    # unittest.TextTestRunner(verbosity = 2).run(test_suite)

    unittest.main()
