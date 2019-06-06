# Author-
# Description-

import datetime as dt
import traceback
from functools import partial

from adsk.core import Application, Matrix3D
from adsk.fusion import Design, Component
from .f360lib.orientation import chordwise_coord
from .f360lib.utils import load_settings
from .f360lib.utils import log_func
from .f360lib.wing import create_rib

app = Application.get()
design = Design.cast(app.activeProduct)
ui = app.userInterface
settings = load_settings(design.userParameters, 'ribSettings', ui)
log = partial(log_func, settings.LOGFILE)


def run(context):
    """
    Main entrypoint
    """
    global ui
    try:
        log('--------------------------------')
        log(dt.datetime.now(), __file__)
        log()

        root = Component.cast(design.rootComponent)

        # locate the root sketch
        root_sketch = root.sketches.itemByName(settings.RIB_ROOT_SKETCH)
        if root_sketch is None:
            raise ValueError('Root sketch "{}" not found'.format(settings.RIB_ROOT_SKETCH))

        # locate the wing body
        wing_body = root.bRepBodies.itemByName(settings.WING_BODY)
        if wing_body is None:
            raise ValueError('Wing body "{}" not found'.format(settings.WING_BODY))

        # find the chordwise extremities of the wing body
        start_coord = chordwise_coord(wing_body.boundingBox.minPoint)
        end_coord = chordwise_coord(wing_body.boundingBox.maxPoint)
        chord_length = end_coord - start_coord
        log('start, end, chord length', start_coord, end_coord, chord_length)

        rib_post_loc_type = settings.RIB_POST_LOC_TYPE

        rib_vertical_fracs = [r / chord_length for r in settings.RIB_POST_ROOT_LOCS_CM]
        log('rib vertical positions', settings.RIB_POST_ROOT_LOCS_CM)
        log('rib vertical relative positions', rib_vertical_fracs)

        # create new component
        component_occurrence = root.occurrences.addNewComponent(Matrix3D.create())
        component = Component.cast(component_occurrence.component)
        component.name = settings.RIB_COMPONENT_NAME

        # now create the ribs
        for rib_id, rib_dist_from_root in enumerate(settings.RIB_STATIONS_CM):
            rib_name = "rib_{}".format(rib_id + 1)
            create_rib(wing_body, root_sketch, component, component_occurrence,
                       rib_dist_from_root, settings.RIB_THICKNESS_CM, settings.RIB_INSET_CM, rib_name,
                       rib_vertical_fracs, settings.RIB_POST_WIDTH_CM, settings.RIB_POST_TRIANGLE_LEN_CM)

    except Exception as ex:
        msg = 'Failed:\n{}'.format(traceback.format_exc())
        log(msg)
        if ui:
            ui.messageBox(str(ex))


if __name__ == '__main__':
    import doctest

    doctest.testmod()
