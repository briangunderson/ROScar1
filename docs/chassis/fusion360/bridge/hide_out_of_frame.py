"""Walk the design and hide any body whose bounding box is OUTSIDE the
expected chassis frame volume. Useful for cleaning up stray specks left
by multi-body STEP imports whose nested sub-occurrences don't translate
rigidly (typically grub screws inside bracket assemblies).

Run via the bridge:
    python fctl.py run hide_out_of_frame.py
"""

import adsk.core
import adsk.fusion


# Chassis frame volume in cm (cut sheet dimensions; small margin).
X_MIN, X_MAX = -5.0, 30.0
Y_MIN, Y_MAX = -5.0, 30.0
Z_MIN, Z_MAX = -1.0, 45.0


def _hide_if_out(occ, depth=0):
    hidden = 0
    try:
        comp = occ.component
        for i in range(comp.bRepBodies.count):
            body = comp.bRepBodies.item(i)
            try:
                bb = body.boundingBox
                cx = (bb.minPoint.x + bb.maxPoint.x) / 2
                cy = (bb.minPoint.y + bb.maxPoint.y) / 2
                cz = (bb.minPoint.z + bb.maxPoint.z) / 2
                out = (cx < X_MIN or cx > X_MAX
                       or cy < Y_MIN or cy > Y_MAX
                       or cz < Z_MIN or cz > Z_MAX)
                if out:
                    try:
                        body.isLightBulbOn = False
                        hidden += 1
                    except Exception:
                        pass
            except Exception:
                pass
        # Recurse into sub-occurrences
        for i in range(comp.occurrences.count):
            hidden += _hide_if_out(comp.occurrences.item(i), depth + 1)
    except Exception:
        pass
    return hidden


def run(context):
    app = adsk.core.Application.get()
    des = adsk.fusion.Design.cast(app.activeProduct)
    if des is None:
        return
    rc = des.rootComponent
    total = 0
    for i in range(rc.occurrences.count):
        total += _hide_if_out(rc.occurrences.item(i))
    # Also bodies directly in root
    for i in range(rc.bRepBodies.count):
        body = rc.bRepBodies.item(i)
        try:
            bb = body.boundingBox
            cx = (bb.minPoint.x + bb.maxPoint.x) / 2
            cy = (bb.minPoint.y + bb.maxPoint.y) / 2
            cz = (bb.minPoint.z + bb.maxPoint.z) / 2
            out = (cx < X_MIN or cx > X_MAX
                   or cy < Y_MIN or cy > Y_MAX
                   or cz < Z_MIN or cz > Z_MAX)
            if out:
                body.isLightBulbOn = False
                total += 1
        except Exception:
            pass

    print(f'hidden {total} stray bodies outside chassis volume')
    try:
        app.activeViewport.fit()
        print('viewport fitted')
    except Exception as e:
        print(f'fit failed: {e}')
