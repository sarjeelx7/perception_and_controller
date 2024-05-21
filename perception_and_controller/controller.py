import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Twist
import math


class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.subscription_pose_msg = self.create_subscription(
            PoseStamped,
            '/pose_msg',
            self.pose_msg_callback,
            10
        )
        self.subscription_pose_info = self.create_subscription(
            PoseArray,
            '/pose_info',
            self.pose_info_callback,
            10
        )
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.current_pose = None
        self.car_pose = None

    def pose_msg_callback(self, msg):
        self.current_pose = msg.pose
        self.control_car()

    def pose_info_callback(self, msg):
        if len(msg.poses) > 0:
            self.car_pose = msg.poses[0]
            self.control_car()

    def control_car(self):
        if self.current_pose is None or self.car_pose is None:
            return

        twist_msg = Twist()

        # Calculate control commands
        x_error = self.current_pose.position.x - self.car_pose.position.x
        y_error = self.current_pose.position.y - self.car_pose.position.y

        # Proportional control gains
        Kp_linear = 0.5
        Kp_angular = 2.0

        # Linear velocity
        twist_msg.linear.x = Kp_linear * x_error

        # Angular velocity (simplified control law for demonstration)
        angle_to_goal = math.atan2(y_error, x_error)
        car_orientation = self.car_pose.orientation
        car_yaw = self.quaternion_to_yaw(car_orientation)
        angular_error = angle_to_goal - car_yaw
        twist_msg.angular.z = Kp_angular * angular_error

        # Publish the command velocity
        self.cmd_vel_publisher.publish(twist_msg)

    def quaternion_to_yaw(self, q):
        # Convert quaternion to yaw angle
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
