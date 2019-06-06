# Author-
# Description-

import re
import datetime as dt
import traceback
from functools import partial

from adsk.core import Application
from adsk.fusion import Design, Component
from .f360lib.utils import log_func

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
        log("doc name:", parse_document_name(app.activeDocument.name))


    except Exception as ex:
        msg = 'Failed:\n{}'.format(traceback.format_exc())
        log(msg)
        if ui:
            ui.messageBox(str(ex))


def parse_document_name(n):
    """
    Parses an f360 document (drawing) name, stripping out any version information
    >>> parse_document_name('example-undercambered-wing v9')
    'example-undercambered-wing'
    >>> parse_document_name('another test v3 wing v0')
    'another test v3 wing'
    """
    return re.sub(' v[0-9]+$', '', n)

if __name__ == '__main__':
    import doctest

    doctest.testmod()
