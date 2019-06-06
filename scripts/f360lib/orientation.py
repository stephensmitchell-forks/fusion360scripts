from adsk.core import Vector3D, Point3D

"""
Defines the orientation of the model.
"""
VERTICAL_UP_DIRECTION = Vector3D.create(0., 0., 1.0)
SPANWISE_DIRECTION = Vector3D.create(0., 1., 0.)
CHORDWISE_DIRECTION = Vector3D.create(1., 0., 0.)

# -----------------------------------


def vert_spanwise_plane(comp):
    return comp.yZConstructionPlane


def chordwise_coord(point):
    return point.x


def point(chordwise, spanwise, vertical):
    """
    creates a point, given the chordwise, spanwise and vertical components
    """
    return Point3D.create(chordwise, spanwise, vertical)
