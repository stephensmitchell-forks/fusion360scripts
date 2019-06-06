from adsk.core import ValueInput, ObjectCollection
from adsk.fusion import FeatureOperations
from .orientation import VERTICAL_UP_DIRECTION, SPANWISE_DIRECTION
from .orientation import chordwise_coord, point
from .orientation import vert_spanwise_plane
from .utils import boundary_fill_between_planes
from .utils import centroid_of_bounding_box
from .utils import find_coplanar_face
from .utils import project_coord
from .utils import relative_location


def create_rib_body(component, comp_occurrence, wing_body, root_plane, dist_from_root, thickness):
    """
    Creates 2 construction planes and a solid rib body using a boundary fill between the 2 planes and the wing body
    """
    # Create 2 construction planes:
    #   1) offset from root sketch plane
    #   2) offset from the first
    planes = component.constructionPlanes

    plane1_input = planes.createInput()
    plane1_input.setByOffset(root_plane, ValueInput.createByReal(dist_from_root))
    plane1 = planes.add(plane1_input)

    plane2_input = planes.createInput()
    plane2_input.setByOffset(plane1, ValueInput.createByReal(thickness))
    plane2 = planes.add(plane2_input)

    rib_body = boundary_fill_between_planes(component, comp_occurrence, wing_body, plane1, plane2)

    # hide the construction planes
    plane1.isLightBulbOn = False
    plane2.isLightBulbOn = False

    return rib_body, plane1, plane2


def create_rib_vertical_post(component, comp_occurrence, wing_body, rib_body, rib_post_loc, rib_post_width,
                             rib_post_triangle_len):
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

    assert plane1.isValid

    sketch = component.sketches.add(plane2, comp_occurrence)
    lines = sketch.sketchCurves.sketchLines

    tri_side = rib_post_triangle_len

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
               rib_post_relative_positions, rib_post_width, rib_post_triangle_len):
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
        post = create_rib_vertical_post(component, comp_occurrence, wing_body, rib_body, rib_post_loc, rib_post_width,
                                        rib_post_triangle_len)
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
    rib_thickness = ValueInput.createByReal(rib_inset)
    shell_feature_input.insideThickness = rib_thickness
    shell_feats.add(shell_feature_input)
