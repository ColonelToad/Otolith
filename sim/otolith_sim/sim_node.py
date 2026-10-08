"""rclpy publisher node: runs the kinematic puppet and streams sensors.

Scaffolding-quality Python (documented in ADR 0002): the artifact is the
C++ fusion node; this node exists to generate honest sensor streams.

Topics (rmw_zenoh):
  /otolith/imu            sensor_msgs/Imu            @ sim rate
  /otolith/joint_states   sensor_msgs/JointState     @ sim rate
  /otolith/foot_contacts  std_msgs/Float32MultiArray @ sim rate (FL FR RL RR)
  /otolith/ground_truth   nav_msgs/Odometry          @ gt_rate (exact puppet state)

Run:  pixi run python -m otolith_sim.sim_node   (from sim/)
"""

from __future__ import annotations

import argparse
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray, Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Vector3, Point, Pose, Twist, Vector3Stamped

import mujoco

from otolith_sim.puppet import _quat_to_mat, GRAVITY
from otolith_sim.robot_adapters import ROBOT_CHOICES, make_adapter
from otolith_sim.sensors import ImuNoise, EncoderNoise, contacts_exact

SCENE = "third_party/menagerie/unitree_go2/scene.xml"
# Joint NAMES now come from the descriptor, not from string formatting.
# v0.5 established that a biped's chain order is the tree topology and that
# mirroring rules differ per robot, so rebuilding names here would quietly
# reintroduce exactly those assumptions. See robot_adapters.py.


class SimClock:
    """Wall-clock paced loop: sleeps until the next tick deadline. Reports
    achieved rate so Python pacing jitter is visible, not hidden."""

    def __init__(self, rate_hz: float):
        self.dt = 1.0 / rate_hz
        self.next_deadline = time.monotonic()
        self.ticks = 0
        self.late = 0

    def sleep(self):
        self.next_deadline += self.dt
        now = time.monotonic()
        if self.next_deadline > now:
            time.sleep(self.next_deadline - now)
        else:
            self.late += 1
            # fell behind: resync instead of spiraling
            self.next_deadline = now
        self.ticks += 1


class OtolithSimNode(Node):
    def __init__(self, rate_hz: float = 500.0, gt_rate_hz: float = 100.0,
                 robot: str = "go2",
                 scene: str = SCENE, replay: str | None = None):
        super().__init__("otolith_sim")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub_imu = self.create_publisher(Imu, "/otolith/imu", qos)
        self.pub_joints = self.create_publisher(JointState, "/otolith/joint_states", qos)
        self.pub_contacts = self.create_publisher(Float32MultiArray,
                                                  "/otolith/foot_contacts", qos)
        self.pub_gt = self.create_publisher(Odometry, "/otolith/ground_truth", qos)

        # The adapter owns the model. Loading it here as well would give the node and
        # the puppet two different MjModel instances of the same file, and every
        # `data.qpos` write would be against a model the puppet never reads.
        self.adapter = make_adapter(robot, scene, replay)
        self.model = self.adapter.model
        self.data = mujoco.MjData(self.model)
        self.imu = ImuNoise()
        self.enc = EncoderNoise()
        self.clock = SimClock(rate_hz)
        self.gt_every = max(1, int(round(rate_hz / gt_rate_hz)))
        self.rate_hz = rate_hz
        self.t_sim = 0.0

        self.get_logger().info(
            # adapter.scene, not the argument: for a biped the argument is None and
            # the adapter resolves the robot's own default (OP3's patched scene,
            # G1's .work scene), so logging the argument printed "scene=None".
            f"sim up: robot={robot} rate={rate_hz}Hz "
            f"scene={self.adapter.scene} nq={self.model.nq} "
            f"legs={self.adapter.n_legs} joints={len(self.adapter.joint_names)} "
            f"gait={'replay:' + replay if replay else 'live IK'} "
            f"(ctrl-c to stop)")

    def _header(self, frame: str = "base") -> Header:
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = frame
        return h

    def spin(self):
        gt_tick = 0
        t_wall_start = time.monotonic()
        while rclpy.ok():
            sample = self.adapter.sample(self.data, self.t_sim, self.clock.dt)

            # IMU: body-frame rates/accel; accel = R^T (a_world - g)
            R = _quat_to_mat(sample.base_quat)
            gyro = sample.base_rpy_rate.copy()  # small-angle: rpy rate ~ body rates
            accel_body = R.T @ (sample.base_accel + np.array([0.0, 0.0, GRAVITY]))
            gyro_m, accel_m = self.imu.step(self.clock.dt, gyro, accel_body)

            imu = Imu()
            imu.header = self._header("base")
            imu.orientation = Quaternion(x=float(sample.base_quat[1]),
                                         y=float(sample.base_quat[2]),
                                         z=float(sample.base_quat[3]),
                                         w=float(sample.base_quat[0]))
            # true orientation is published: the EKF may use or ignore it
            imu.angular_velocity = Vector3(x=float(gyro_m[0]),
                                           y=float(gyro_m[1]),
                                           z=float(gyro_m[2]))
            imu.linear_acceleration = Vector3(x=float(accel_m[0]),
                                              y=float(accel_m[1]),
                                              z=float(accel_m[2]))
            self.pub_imu.publish(imu)

            js = JointState()
            js.header = self._header()
            js.name = list(self.adapter.joint_names)
            js.position = self.enc.step(self.adapter.joint_q(sample)).tolist()
            self.pub_joints.publish(js)

            fc = Float32MultiArray()
            fc.data = [float(c) for c in self.adapter.contacts(sample)]
            self.pub_contacts.publish(fc)

            gt_tick += 1
            if gt_tick >= self.gt_every:
                gt_tick = 0
                odom = Odometry()
                odom.header = self._header("world")
                odom.pose.pose = Pose(
                    position=Point(x=float(sample.base_pos[0]),
                                   y=float(sample.base_pos[1]),
                                   z=float(sample.base_pos[2])),
                    orientation=imu.orientation)
                odom.twist.twist = Twist()  # rates live in the IMU message
                self.pub_gt.publish(odom)

            self.t_sim += self.clock.dt
            self.clock.sleep()

            if self.clock.ticks % (self.rate_hz * 5) == 0:
                elapsed = time.monotonic() - t_wall_start
                self.get_logger().info(
                    f"t={self.t_sim:6.2f}s ticks={self.clock.ticks} "
                    f"late={self.clock.late} "
                    f"achieved={self.clock.ticks / elapsed:.1f}Hz")


def main():
    ap = argparse.ArgumentParser(description="Otolith sensor simulator")
    ap.add_argument("--robot", default="go2", choices=list(ROBOT_CHOICES),
                    help="go2 (quadruped) or a 6-DoF biped: g1, apollo, op3")
    ap.add_argument("--rate", type=float, default=500.0, help="sensor rate, Hz")
    ap.add_argument("--gt-rate", type=float, default=100.0, help="ground-truth rate, Hz")
    ap.add_argument("--replay", default=None,
                    help="baked joint trajectory (.npz); needed for a biped to reach "
                         "500 Hz, since the numpy DLS IK is the bottleneck "
                         "(21.95 ms/sample). See otolith_sim.bake_replay")
    ap.add_argument("--scene", default=None,
                    help="MJCF override; defaults to the robot's own scene")
    args = ap.parse_args()
    rclpy.init()
    # scene=None lets the adapter pick the robot's default (OP3 needs the patched
    # scene for its named contact geoms; passing Go2's would be wrong, not merely
    # unusual).
    node = OtolithSimNode(rate_hz=args.rate, gt_rate_hz=args.gt_rate,
                         robot=args.robot, replay=args.replay,
                         scene=args.scene or SCENE if args.robot == "go2"
                         else args.scene)
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
