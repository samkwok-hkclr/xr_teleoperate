#!/usr/bin/env python3
# script/sim_b_short_press.py
import argparse, time
import rclpy
from rclpy.node import Node
from xr_teleop_msgs.msg import Button

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("side", choices=["left", "right"])
    ap.add_argument("hold", type=float, help="seconds, e.g. 0.3")
    ap.add_argument("--rate", type=float, default=50.0)
    args = ap.parse_args()

    rclpy.init()
    node = Node("sim_b_short_press")
    topic = f"/xr/{args.side}_b_button"
    pub = node.create_publisher(Button, topic, 10)

    # Wait for a subscriber so timing starts after discovery
    t0 = time.time()
    while time.time() - t0 < 2.0 and pub.get_subscription_count() == 0:
        time.sleep(0.02)
    if pub.get_subscription_count() == 0:
        node.get_logger().warn(f"no subscriber on {topic}, publishing anyway")

    press = Button();   press.pressed = True
    release = Button(); release.pressed = False

    period = 1.0 / args.rate
    deadline = time.monotonic() + args.hold
    n = 0
    while time.monotonic() < deadline:
        pub.publish(press)
        n += 1
        time.sleep(period)

    # Immediate release — same process, no spawn gap
    pub.publish(release)

    node.get_logger().info(f"{args.side}: {n} press msgs over {args.hold}s, released")
    time.sleep(0.1)
    rclpy.shutdown()

if __name__ == "__main__":
    main()