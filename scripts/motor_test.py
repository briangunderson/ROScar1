#!/usr/bin/env python3
"""Motor diagnostic: test each wheel and report encoder response.

Usage:
  1. Set robot to idle mode (web UI or ros2 service call)
  2. LIFT the robot off the ground!
  3. Run: python3 ~/ROScar1/scripts/motor_test.py

Motor mapping (from STM32 firmware):
  M1 = front-left   (TIM2)
  M2 = rear-left    (TIM4)
  M3 = front-right  (TIM5)
  M4 = rear-right   (TIM3)
"""

import time
import sys

sys.path.insert(0, '/home/brian/.local/lib/python3.12/dist-packages')
from Rosmaster_Lib import Rosmaster

SERIAL_PORT = '/dev/roscar_board'
TEST_SPEED = 0.15  # m/s — gentle test speed
TEST_DURATION = 1.5  # seconds per test


def read_encoders(bot):
    """Read encoder counts (m1, m2, m3, m4)."""
    return bot.get_motor_encoder()


def test_motion(bot, label, vx, vy, wz):
    """Command a motion, measure encoder deltas, report."""
    print(f"\n--- {label} (vx={vx}, vy={vy}, wz={wz}) ---")

    # Read before
    before = read_encoders(bot)
    time.sleep(0.1)  # let encoder reading settle

    # Command motion
    # Board is 180-deg rotated + L/R ports swapped:
    # linear axes cancel (two reversals), only negate wz (same as driver_node.py)
    bot.set_car_motion(vx, vy, wz)
    time.sleep(TEST_DURATION)

    # Stop
    bot.set_car_motion(0, 0, 0)
    time.sleep(0.2)

    # Read after
    after = read_encoders(bot)

    # Compute deltas
    names = ['M1 (front-left)', 'M2 (rear-left)', 'M3 (front-right)', 'M4 (rear-right)']
    deltas = []
    for i in range(4):
        delta = after[i] - before[i]
        deltas.append(delta)
        status = 'OK' if abs(delta) > 5 else '** NO RESPONSE **'
        print(f"  {names[i]:20s}: {before[i]:6d} -> {after[i]:6d}  delta={delta:+6d}  {status}")

    return deltas


def main():
    print("=" * 60)
    print("ROScar1 Motor Diagnostic")
    print("=" * 60)
    print()
    print("WARNING: Make sure the robot is LIFTED off the ground!")
    print("         Wheels will spin during this test.")
    print()
    input("Press Enter to start (Ctrl+C to abort)...")

    print(f"\nConnecting to {SERIAL_PORT}...")
    bot = Rosmaster(car_type=1, com=SERIAL_PORT, delay=0.002)
    bot.create_receive_threading()
    time.sleep(1)  # let serial sync

    # Stop any residual motion
    bot.set_car_motion(0, 0, 0)
    time.sleep(0.5)

    results = {}

    # Test 1: Forward — all 4 wheels should spin
    results['forward'] = test_motion(bot, 'FORWARD', TEST_SPEED, 0, 0)

    time.sleep(1)

    # Test 2: Strafe right — all 4 wheels, different pattern
    results['strafe'] = test_motion(bot, 'STRAFE RIGHT', 0, -TEST_SPEED, 0)

    time.sleep(1)

    # Test 3: Rotate CW — all 4 wheels, yet another pattern
    results['rotate'] = test_motion(bot, 'ROTATE CW', 0, 0, 0.5)

    # Stop everything
    bot.set_car_motion(0, 0, 0)
    time.sleep(0.5)

    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    names = ['M1 (front-left)', 'M2 (rear-left)', 'M3 (front-right)', 'M4 (rear-right)']
    for i in range(4):
        fwd = abs(results['forward'][i])
        stf = abs(results['strafe'][i])
        rot = abs(results['rotate'][i])
        total = fwd + stf + rot
        if total < 15:
            verdict = 'DEAD — check wiring!'
        elif fwd < 5 or stf < 5 or rot < 5:
            verdict = 'PARTIAL — may have intermittent connection'
        else:
            verdict = 'HEALTHY'
        print(f"  {names[i]:20s}: fwd={fwd:4d}  stf={stf:4d}  rot={rot:4d}  => {verdict}")

    # Check for consistent direction issues
    print()
    fwd_deltas = results['forward']
    # For mecanum forward: ALL four wheels should turn the same direction
    # (FL, RL same sign AND FR, RR same sign AND left/right pairs same sign)
    signs = [1 if d > 0 else (-1 if d < 0 else 0) for d in fwd_deltas]
    if any(s == 0 for s in signs):
        print("  Direction consistency: CANNOT CHECK (some motors didn't respond)")
    elif all(s == signs[0] for s in signs):
        print("  Direction consistency: OK (all four wheels same direction)")
    elif signs[0] == signs[1] and signs[2] == signs[3] and signs[0] != signs[2]:
        print("  Direction consistency: FAIL — left/right sides oppose each other (tug-of-war!)")
        print(f"    Signs: FL={signs[0]:+d} RL={signs[1]:+d} FR={signs[2]:+d} RR={signs[3]:+d}")
        print("    Check motor command negation in driver_node.py")
    else:
        print("  Direction consistency: WARNING — unexpected encoder sign pattern")
        print(f"    Signs: FL={signs[0]:+d} RL={signs[1]:+d} FR={signs[2]:+d} RR={signs[3]:+d}")

    print()
    bot.reset_car_state()


if __name__ == '__main__':
    main()
