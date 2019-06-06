# Author-
# Description-create a 3d printable wing

import adsk.core, adsk.fusion, adsk.cam, traceback

PROFILE_SKETCH = 'wing-profile'


app = adsk.core.Application.get()
ui = app.userInterface


def run(context):
    try:
        design = app.activeProduct
        root = design.rootComponent

        ui.selectEntity
        (sketch_name, cancelled) = ui.inputBox('Enter name of profile sketch')
        if cancelled:
            return
            
        sketch = root.sketches.itemByName(sketch_name)
        if sketch is None:
            raise Exception("No sketch named '{}' in component '{}'".format(sketch_name, root.name))

        num_profiles = sketch.profiles.count
        if num_profiles != 1:
            raise Exception('Expected exactly 1 profile in sketch: {}, but had {}'.format(sketch_name, num_profiles))
            
        ui.messageBox('sketch has %d profiles' % num_profiles)
        profile = sketch.profiles[0]
        ui.messageBox('area props: %s' % profile.areaProperties())


    except Exception as ex:
        if ui:
            ui.messageBox('Error: {}:\n{}'.format(ex, traceback.format_exc()))

