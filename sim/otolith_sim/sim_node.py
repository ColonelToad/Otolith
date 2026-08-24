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

from otolith_sim.puppet import Go2Puppet, GaitConfig, _quat_to_mat, GRAVITY
from otolith_sim.sensors import ImuNoise, EncoderNoise, contacts_exact

SCENE = "third_party/menagerie/unitree_go2/scene.xml"
FOOT_ORDER = ("FL", "FR", "RL", "RR")


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
                 scene: str = SCENE):
        super().__init__("otolith_sim")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub_imu = self.create_publisher(Imu, "/otolith/imu", qos)
        self.pub_joints = self.create_publisher(JointState, "/otolith/joint_states", qos)
        self.pub_contacts = self.create_publisher(Float32MultiArray,
                                                  "/otolith/foot_contacts", qos)
        self.pub_gt = self.create_publisher(Odometry, "/otolith/ground_truth", qos)

        self.model = mujoco.MjModel.from_xml_path(scene)
        self.data = mujoco.MjData(self.model)
        self.puppet = Go2Puppet(self.model, GaitConfig())
        self.imu = ImuNoise()
        self.enc = EncoderNoise()
        self.clock = SimClock(rate_hz)
        self.gt_every = max(1, int(round(rate_hz / gt_rate_hz)))
        self.rate_hz = rate_hz
        self.t_sim = 0.0

        self.get_logger().info(
            f"sim up: rate={rate_hz}Hz scene={scene} "
            f"nq={self.model.nq} (ctrl-c to stop)")

    def _header(self, frame: str = "base") -> Header:
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = frame
        return h

    def spin(self):
        gt_tick = 0
        t_wall_start = time.monotonic()
        while rclpy.ok():
            sample = self.puppet.sample(self.model, self.data, self.t_sim,
                                        self.clock.dt)

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
            js.name = [f"{leg}_{part}_joint" for leg in FOOT_ORDER
                       for part in ("hip", "thigh", "calf")]
            q_joints = np.array([sample.qpos[adr]
                                 for leg in FOOT_ORDER
                                 for adr in self.puppet.legs[leg].qpos_adr])
            js.position = self.enc.step(q_joints).tolist()
            self.pub_joints.publish(js)

            fc = Float32MultiArray()
            fc.data = [float(c) for c in contacts_exact(sample.contacts)]
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
    rclpy.init()
    node = OtolithSimNode()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
