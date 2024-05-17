import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import JointState

from gpiozero.pins.lgpio import LGPIOFactory
import ament_index_python
import gpiozero
import ctypes
import time
import os


class GPIONode(Node):
    def __init__(self, ppr, encoder_l, encoder_r, motor_l, motor_r):
        super().__init__("gpio")

        self.ppr, self.encoder_l, self.encoder_r = ppr, encoder_l, encoder_r

        self.motor_l, self.motor_r = motor_l, motor_r

        self.encoder_time = time.perf_counter()
        self.encoder_position_l = 0
        self.encoder_position_r = 0

        self.publisher_ = self.create_publisher(
            JointState, "/diff_drive_controller/joint_states", 10
        )
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.subscription = self.create_subscription(
            JointState,
            "/diff_drive_controller/joint_commands",
            self.subscription_callback,
            10,
        )

    def timer_callback(self):
        """
        encoder_lookup_l = self.encoder_l.steps
        encoder_lookup_r = self.encoder_r.steps

        encoder_time = time.perf_counter()
        self.get_logger().info(f'Lookup: "{encoder_lookup_l} {encoder_lookup_r}"')

        encoder_position_l = 360 / self.ppr * encoder_lookup_l
        encoder_position_r = 360 / self.ppr * encoder_lookup_r

        encoder_velocity_l = (encoder_position_l - self.encoder_position_l) / (
            encoder_time - self.encoder_time
        )
        encoder_velocity_r = (encoder_position_r - self.encoder_position_r) / (
            encoder_time - self.encoder_time
        )

        self.encoder_time = encoder_time
        self.encoder_position_l = encoder_position_l
        self.encoder_position_r = encoder_position_r
        """
        encoder_position_l, encoder_position_r = 0.0, 0.0
        encoder_velocity_l, encoder_velocity_r = 0.0, 0.0

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["l_wheel_joint", "r_wheel_joint"]
        msg.position = [encoder_position_l, encoder_position_r]
        msg.velocity = [encoder_velocity_l, encoder_velocity_r]
        msg.effort = [0.0, 0.0]
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Publishing: "{encoder_position_l} {encoder_position_r} {encoder_velocity_l} {encoder_velocity_r}"'
        )

    def subscription_callback(self, msg):
        sign = lambda x: bool(x > 1e-5) - bool(x < -1e-5)

        self.get_logger().info(
            f'I heard: "{list(msg.name)}: {list(msg.position)} {list(msg.velocity)} {list(msg.effort)}"'
        )
        """
        l_index = msg.name.index("l_wheel_joint")
        r_index = msg.name.index("r_wheel_joint")
        if (0 <= l_index and l_index < len(msg.velocity)) and (
            0 <= r_index and r_index < len(msg.velocity)
        ):
            self.get_logger().info(
                f'I sent: "0,0,{msg.velocity[l_index]},{msg.velocity[r_index]}'
            )

            delta = 1e-5
            delta_l, delta_r = msg.velocity[l_index], msg.velocity[r_index]

            match delta_l:
                case _ if delta_l > delta:
                    self.motor_l.forward(0.1)
                case _ if delta_l < -delta:
                    self.motor_l.backward(0.1)
                case _:
                    self.motor_l.stop()

            match delta_r:
                case _ if delta_r > delta:
                    self.motor_r.forward(0.1)
                case _ if delta_r < -delta:
                    self.motor_r.backward(0.1)
                case _:
                    self.motor_r.stop()
        """


def main(args=None):
    rclpy.init(args=args)

    pin_factory = LGPIOFactory()
    encoder_l = (
        None  # gpiozero.RotaryEncoder(22, 23, max_steps=0, pin_factory=pin_factory)
    )
    encoder_r = (
        None  # gpiozero.RotaryEncoder(24, 25, max_steps=0, pin_factory=pin_factory)
    )
    motor_l = None  # gpiozero.Motor(12, 18, pin_factory=pin_factory)
    motor_r = None  # gpiozero.Motor(13, 19, pin_factory=pin_factory)

    node = GPIONode(
        ppr=537.7 / 4,
        encoder_l=encoder_l,
        encoder_r=encoder_r,
        motor_l=motor_l,
        motor_r=motor_r,
    )

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
