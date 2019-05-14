from adsk.core import Vector3D, Point3D, SurfaceTypes, Plane


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

    plane1_coord = project_coord(plane1.geometry.origin, normal)
    point_coord = project_coord(point, normal)
    plane2_coord = project_coord(plane2.geometry.origin, normal)
    between = min(plane1_coord, plane2_coord) <= point_coord <= max(plane1_coord, plane2_coord)
    return between


def cell_between_planes(cells, plane1, plane2):
    # find centroids of bounding boxes
    bounding_boxes = [c.cellBody.boundingBox for c in cells]
    centroids = [centroid_of_bounding_box(bb) for bb in bounding_boxes]

    # see which centroids are between the construction planes
    in_betweens = [is_point_between_planes(centroid.asPoint(), plane1, plane2) for centroid in centroids]
    assert len([b for b in in_betweens if b is True]) == 1, "Expected 1 cell in between planes. Are you sure that the wing body is not shelled?"

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
    returns the coordinate of the plane in it's normal direction
    """
    point = point.asArray()
    direction = direction.asArray()

    assert direction[0] + direction[1] + direction[2] == 1.0, "expected plane aligned with axes"

    result = 0
    for i in range(3):
        result += direction[i] * point[i]
    return result



if __name__ == '__main__':
    import doctest
    doctest.testmod()
