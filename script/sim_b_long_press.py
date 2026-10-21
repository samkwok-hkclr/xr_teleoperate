#!/usr/bin/env python3
"""
Simulate a B-button LONG press.

Publishes pressed=true on one or both /xr/<side>_b_button topics at a
fixed rate for HOLD seconds, then publishes pressed=false. After HOLD
exceeds b_button_hold_duration (default 3.0 s), xr_teleop fires
/start_robot or /shutdown_robot depending on the current
/robot_hardware_status.

Usage:
    ./script/sim_b_long_press.py                       # both, 3.5 s
    ./script/sim_b_long_press.py left                  # left only, 3.5 s
    ./script/sim_b_long_press.py right 5.0             # right, 5 s
    ./script/sim_b_long_press.py both 3.2 --rate 20

Exit code is always 0; the service result appears in the xr_teleop log.
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from xr_teleop_msgs.msg import Button


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("side", nargs="?", default="both",
                    choices=["left", "right", "both"],
                    help="which B button(s) to press (default: both)")
    ap.add_argument("hold", nargs="?", type=float, default=3.5,
                    help="hold duration in seconds (default: 3.5, "
                         "must exceed b_button_hold_duration=3.0)")
    ap.add_argument("--rate", type=float, default=20.0,
                    help="publish rate in Hz (default: 20)")
    return ap.parse_args()


def main():
    args = parse_args()

    topics = {
        "left":  ["/xr/left_b_button"],
        "right": ["/xr/right_b_button"],
        "both":  ["/xr/left_b_button", "/xr/right_b_button"],
    }[args.side]

    rclpy.init()
    node = Node("sim_b_long_press")
    log = node.get_logger()

    pubs = {t: node.create_publisher(Button, t, 10) for t in topics}
    press = Button()
    press.pressed = True
    release = Button()
    release.pressed = False

    # Wait for every topic to have at least one subscriber (xr_teleop)
    # before we start timing. Otherwise the first few press messages go
    # into the void and the node sees a shorter hold than we intend.
    log.info(f"waiting for subscribers on {topics}...")
    t0 = time.time()
    while time.time() - t0 < 3.0:
        if all(p.get_subscription_count() > 0 for p in pubs.values()):
            break
        time.sleep(0.05)
    missing = [t for t, p in pubs.items() if p.get_subscription_count() == 0]
    if missing:
        log.warn(f"no subscriber on {missing}; publishing anyway")
    else:
        log.info("subscribers connected")

    log.info(f"pressing {args.side} for {args.hold} s @ {args.rate} Hz")

    period = 1.0 / args.rate
    deadline = time.monotonic() + args.hold
    n = 0
    while time.monotonic() < deadline:
        for p in pubs.values():
            p.publish(press)
        n += 1
        time.sleep(period)

    # Immediate release from the same process — no inter-process gap.
    for p in pubs.values():
        p.publish(release)

    log.info(f"released after {args.hold} s ({n} press msgs per topic)")
    log.info("check xr_teleop log for '/start_robot OK' or '/shutdown_robot OK'")

    # Give DDS a moment to flush the release message before shutdown.
    time.sleep(0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()