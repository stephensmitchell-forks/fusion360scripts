import math

from adsk.core import Vector3D, Point3D, SurfaceTypes, Plane, ObjectCollection
from adsk.fusion import FeatureOperations


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


if __name__ == '__main__':
    import doctest

    doctest.testmod()
