# Author-
# Description-

import re
import datetime as dt
import traceback
from functools import partial

from adsk.core import Application
from adsk.fusion import Design, Component
from .f360lib.utils import log_func, current_document_name

app = Application.get()
design = Design.cast(app.activeProduct)
ui = app.userInterface
#settings = load_settings(design.userParameters, 'ribSettings', ui)

# name of a file to log diagnostics and errors
LOGFILE = '/Users/andy/logs/script_output.log'

log = partial(log_func, LOGFILE)


def run(context):
    """
    Main entrypoint
    """
    global ui
    try:
        log('--------------------------------')
        log(dt.datetime.now(), __file__)
        log()

        log("doc", app.activeDocument)
        log("doc name:", current_document_name(app))


    except Exception as ex:
        msg = 'Failed:\n{}'.format(traceback.format_exc())
        log(msg)
        if ui:
            ui.messageBox(str(ex))


if __name__ == '__main__':
    import doctest

    doctest.testmod()
