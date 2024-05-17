import rclpy
from rclpy.node import Node

from sensor_msgs.msg import BatteryState

import time
from DFRobot_INA219 import INA219


class BatteryNode(Node):
    def __init__(self, ina):
        super().__init__("battery")

        self.ina = ina

        self.publisher_ = self.create_publisher(BatteryState, "battery", 10)
        self.timer = self.create_timer(5.0, self.callback)

    def callback(self):
        try:
            # https://electronics.stackexchange.com/a/551667
            cell = 4.0
            voltage = self.ina.get_bus_voltage_V()
            voltage = abs(voltage)
            percentage = (
                123.0 - 123.0 / (1.0 + ((voltage / cell) / 3.7) ** 80.0) ** 0.165
            )
            percentage = max(min(percentage, 100.0), 0.0)

            msg = BatteryState()
            msg.voltage = voltage
            msg.percentage = percentage
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.voltage} - {msg.percentage}"')
        except Exception as e:
            self.get_logger().info(f'Exception: "{e}"')


def main(args=None):
    # Change I2C address by dialing DIP switch
    ina = INA219(1, INA219.INA219_I2C_ADDRESS4)

    index = 0
    while index < 10 and not ina.begin():
        time.sleep(1)
        index += 1
    assert index < 10, "INA not begin"

    ina219_reading_mA = 1000
    ext_meter_reading_mA = 1000
    ina.linear_cal(ina219_reading_mA, ext_meter_reading_mA)

    rclpy.init(args=args)

    node = BatteryNode(ina)

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
