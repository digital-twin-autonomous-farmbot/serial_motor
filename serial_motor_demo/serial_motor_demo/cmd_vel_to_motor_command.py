from geometry_msgs.msg import Twist
from serial_motor_demo_msgs.msg import MotorCommand
import rclpy
from rclpy.node import Node

class CmdVelToMotorCommandNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_motor_command')

        # Subscriber for /cmd_vel
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        # Publisher for /motor_command
        self.motor_command_publisher = self.create_publisher(
            MotorCommand,
            '/motor_command',
            10)

        self.get_logger().info("Node started: Relaying /cmd_vel to /motor_command")

    def cmd_vel_callback(self, twist_msg):
        # Create MotorCommand message
        motor_command = MotorCommand()
        motor_command.is_pwm = True  # Use PWM mode

        linear_speed = twist_msg.linear.x  # Forward/Backward
        angular_speed = twist_msg.angular.z  # Turning

        if linear_speed != 0 and angular_speed == 0:
            # Forward or Backward: Both motors move at the same speed
            motor_command.mot_1_req_rad_sec = linear_speed * 150
            motor_command.mot_2_req_rad_sec = linear_speed * 150
        elif linear_speed == 0 and angular_speed != 0:
            # Turning in place: Adjust motor directions for correct turning
            motor_command.mot_1_req_rad_sec = -angular_speed * 255  # Reverse sign
            motor_command.mot_2_req_rad_sec = angular_speed * 255   # Reverse sign
        elif linear_speed != 0 and angular_speed != 0:
            # Curving: One motor moves faster than the other
            motor_command.mot_1_req_rad_sec = (linear_speed + angular_speed) * 150
            motor_command.mot_2_req_rad_sec = (linear_speed - angular_speed) * 255
        else:
            # Stop: Both motors are stationary
            motor_command.mot_1_req_rad_sec = 0.0
            motor_command.mot_2_req_rad_sec = 0.0

        # Publish the motor command
        self.motor_command_publisher.publish(motor_command)

        self.get_logger().info(
            f"Relayed: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}, "
            f"mot_1_req_rad_sec={motor_command.mot_1_req_rad_sec}, "
            f"mot_2_req_rad_sec={motor_command.mot_2_req_rad_sec}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToMotorCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
