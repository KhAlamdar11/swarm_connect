import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import PoseStamped
import tf2_py as tf2

class TransformToPosePublisher(Node):
    def __init__(self):
        super().__init__('transform_to_pose_publisher')
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        self.pose_publisher = self.create_publisher(PoseStamped, '/cf1/pose', 10)
        self.target_transform = "cf1"
        self.reference_frame = "world"
        self.timer = self.create_timer(0.1, self.publish_pose)

    def publish_pose(self):
        try:
            now = self.get_clock().now()
            # Using a small duration to allow some leeway in time synchronization
            past = now - rclpy.time.Duration(seconds=0.1)
            trans = self.tf_buffer.lookup_transform(self.reference_frame, 
                                                    self.target_transform, 
                                                    past)
            pose_msg = PoseStamped()
            pose_msg.header.stamp = now.to_msg()
            pose_msg.header.frame_id = self.reference_frame
            pose_msg.pose.position.x = trans.transform.translation.x
            pose_msg.pose.position.y = trans.transform.translation.y
            pose_msg.pose.position.z = trans.transform.translation.z
            pose_msg.pose.orientation = trans.transform.rotation
            self.pose_publisher.publish(pose_msg)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('Could not transform: ' + str(e))

def main(args=None):
    rclpy.init(args=args)
    node = TransformToPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
