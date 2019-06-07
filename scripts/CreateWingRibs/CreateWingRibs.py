# Author-
# Description-

import datetime as dt
import traceback
from functools import partial

from adsk.core import Application
from adsk.fusion import Design, Component
from .f360lib.orientation import chordwise_coord
from .f360lib.utils import load_settings
from .f360lib.utils import log_func
from .f360lib.wing import RibCreator

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

        maker = RibCreator(root, settings.RIB_COMPONENT_NAME, settings.RIB_ROOT_SKETCH, settings.WING_BODY)

        wing_body = maker.wing_body

        # find the chordwise extremities of the wing body
        start_coord = chordwise_coord(wing_body.boundingBox.minPoint)
        end_coord = chordwise_coord(wing_body.boundingBox.maxPoint)
        chord_length = end_coord - start_coord
        log('start, end, chord length', start_coord, end_coord, chord_length)

        rib_vertical_fracs = [r / chord_length for r in settings.RIB_POST_ROOT_LOCS_CM]
        log('rib vertical positions', settings.RIB_POST_ROOT_LOCS_CM)
        log('rib vertical relative positions', rib_vertical_fracs)

        # now create the ribs
        for rib_id, rib_dist_from_root in enumerate(settings.RIB_STATIONS_CM):
            rib_name = "rib_{}".format(rib_id + 1)
            maker.create_rib(rib_dist_from_root, settings.RIB_THICKNESS_CM, settings.RIB_INSET_CM, rib_name,
                             rib_vertical_fracs, settings.RIB_POST_WIDTH_CM, settings.RIB_POST_TRIANGLE_LEN_CM)

    except Exception as ex:
        msg = 'Failed:\n{}'.format(traceback.format_exc())
        log(msg)
        if ui:
            ui.messageBox(str(ex))


if __name__ == '__main__':
    import doctest

    doctest.testmod()
