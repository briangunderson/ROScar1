"""Tiny probe to verify the bridge captures stdout from scripts it runs."""

def run(context):
    import sys
    print('probe: hello from stdout')
    print('probe: sys.stdout type =', type(sys.stdout).__name__)
    sys.stderr.write('probe: hello from stderr\n')
    return
