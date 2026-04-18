"""Re-fit the Fusion viewport to the current design bbox."""
import adsk.core


def run(context):
    try:
        adsk.core.Application.get().activeViewport.fit()
        print('viewport fitted')
    except Exception as e:
        print(f'fit failed: {e}')
