import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import MarkerArray, Marker
from tf2_ros import TransformBroadcaster
import math

class VizNode(Node):
    def __init__(self):
        super().__init__("otolith_viz")
        self.tf = TransformBroadcaster(self)
        # Path for GT vs est (throttled to 10 Hz for Foxglove)
        self.pub_gt_path = self.create_publisher(Path, "/otolith/gt_path", 10)
        self.pub_est_path = self.create_publisher(Path, "/otolith/est_path", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/otolith/markers", 10)
        self.gt_path = Path(); self.gt_path.header.frame_id = "world"
        self.est_path = Path(); self.est_path.header.frame_id = "world"
        self.create_subscription(Odometry, "/otolith/ground_truth", self.on_gt, 10)
        self.create_subscription(Odometry, "/otolith/state_estimate", self.on_est, 10)
        self.get_logger().info("viz_node up: world->base tf + paths + markers")

    def on_gt(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "world"
        t.child_frame_id = "base_gt"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(t)
        # path
        ps = PoseStamped(); ps.header = msg.header; ps.pose = msg.pose.pose
        self.gt_path.header.stamp = msg.header.stamp
        self.gt_path.poses.append(ps)
        if len(self.gt_path.poses) > 500:
            self.gt_path.poses.pop(0)
        self.pub_gt_path.publish(self.gt_path)

    def on_est(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "world"
        t.child_frame_id = "base"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(t)
        ps = PoseStamped(); ps.header = msg.header; ps.pose = msg.pose.pose
        self.est_path.header.stamp = msg.header.stamp
        self.est_path.poses.append(ps)
        if len(self.est_path.poses) > 500:
            self.est_path.poses.pop(0)
        self.pub_est_path.publish(self.est_path)
        # covariance ellipsoid (position) at 5 Hz throttled
        if len(self.est_path.poses) % 20 != 0:
            return
        # 3x3 P_pos from covariance[0:3,0:3] (pose cov)
        cov = msg.pose.covariance
        # extract 3x3
        sx = math.sqrt(max(cov[0], 1e-6))
        sy = math.sqrt(max(cov[7], 1e-6))
        sz = math.sqrt(max(cov[14], 1e-6))
        ma = MarkerArray()
        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = msg.header.stamp
        m.ns = "cov"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = msg.pose.pose.position
        m.pose.orientation.w = 1.0
        m.scale.x = 2*sx*2  # 2 sigma
        m.scale.y = 2*sy*2
        m.scale.z = 2*sz*2
        m.color.r = 1.0; m.color.g = 0.5; m.color.b = 0.0; m.color.a = 0.3
        ma.markers.append(m)
        # GT sphere
        m2 = Marker()
        m2.header.frame_id = "world"
        m2.header.stamp = msg.header.stamp
        m2.ns = "gt"
        m2.id = 1
        m2.type = Marker.SPHERE
        m2.action = Marker.ADD
        # we don't have GT here, but we can publish a small sphere at est for now
        # instead publish at last GT path tip if available
        if self.gt_path.poses:
            m2.pose.position = self.gt_path.poses[-1].pose.position
            m2.pose.orientation.w = 1.0
            m2.scale.x = m2.scale.y = m2.scale.z = 0.04
            m2.color.r = 0.0; m2.color.g = 1.0; m2.color.b = 0.0; m2.color.a = 0.9
            ma.markers.append(m2)
        self.pub_markers.publish(ma)

def main():
    rclpy.init()
    node = VizNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
