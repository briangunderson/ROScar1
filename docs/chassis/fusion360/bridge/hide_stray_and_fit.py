"""Hide the stray RPi5 + Mast_TPlate occurrences (whose multi-body
STEP imports landed at wrong world positions in rev28), then fit the
viewport. Lets us see the actual chassis at a reasonable zoom level
while we iterate on fixing the remaining 2/22 placements.

Run via the ROScar bridge:
    python fctl.py run hide_stray_and_fit.py
"""

import adsk.core
import adsk.fusion


def run(context):
    app = adsk.core.Application.get()
    des = adsk.fusion.Design.cast(app.activeProduct)
    if des is None:
        return
    rc = des.rootComponent
    names_to_hide = {'RPi5', 'Mast_TPlate'}
    hidden = []
    for i in range(rc.occurrences.count):
        occ = rc.occurrences.item(i)
        if occ.component and occ.component.name in names_to_hide:
            try:
                occ.isLightBulbOn = False
                hidden.append(occ.component.name)
            except Exception:
                pass
    print(f'hidden: {hidden}')
    try:
        app.activeViewport.fit()
    except Exception:
        pass
    print('viewport fitted')
