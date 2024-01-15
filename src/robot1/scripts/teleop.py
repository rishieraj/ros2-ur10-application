#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import select
import tty
import termios
from pynput import keyboard

# Define key codes
# LIN_VEL_STEP_SIZE = 1
ANG_STEP_SIZE = 0.1

class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        # self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_keyboard_control(self):
        self.msg = """
        Controls for the robot!
        ---------------------------
        Manipulator operation:
        as  df  gh  jk
            zx  cv

        Esc to quit
        """

        self.get_logger().info(self.msg)
        joint_positions = Float64MultiArray()
        # wheel_velocities = Float64MultiArray()
        man_angle1=0.0
        man_angle2=0.0
        man_angle3=0.0
        man_angle4=0.0
        man_angle5=0.0
        man_angle6=0.0


        while True:
            key = self.getKey()
            if key is not None:
                if key == '\x1b':  # Escape key
                    break
                elif key == 'a':  # Right
                    man_angle1 += ANG_STEP_SIZE
                elif key == 's':  # Left
                    man_angle1 -= ANG_STEP_SIZE
                elif key == 'd':  # Left
                    man_angle2 += ANG_STEP_SIZE
                elif key == 'f':  # Left
                    man_angle2 -= ANG_STEP_SIZE
                elif key == 'g':  # Left
                    man_angle3 += ANG_STEP_SIZE
                elif key == 'h':  # Left
                    man_angle3 -= ANG_STEP_SIZE
                elif key == 'j':  # Left
                    man_angle4 += ANG_STEP_SIZE
                elif key == 'k':  # Left
                    man_angle4 -= ANG_STEP_SIZE
                elif key == 'z':  # Left
                    man_angle5 += ANG_STEP_SIZE
                elif key == 'x':  # Left
                    man_angle5 -= ANG_STEP_SIZE
                elif key == 'c':  # Left
                    man_angle6 += ANG_STEP_SIZE
                elif key == 'v':  # Left
                    man_angle6 -= ANG_STEP_SIZE


                print("Joint Angle 1: ", man_angle1, "Joint Angle 2: ", man_angle2, "Joint Angle 3: ", man_angle3, "Joint Angle 4: ", man_angle4,
                      "Joint Angle 5: ", man_angle5, "Joint Angle 6: ", man_angle6)
                
                # Publish the twist message
                joint_positions.data = [man_angle1, man_angle2, man_angle3, man_angle4, man_angle5, man_angle6]

                self.joint_position_pub.publish(joint_positions)
                # self.wheel_velocities_pub.publish(wheel_velocities)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run_keyboard_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()