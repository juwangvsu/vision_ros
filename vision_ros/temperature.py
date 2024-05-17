import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Temperature

import psutil


class TemperatureNode(Node):
    def __init__(self):
        super().__init__("temperature")
        self.publisher_ = self.create_publisher(Temperature, "temperature", 10)
        self.timer = self.create_timer(5.0, self.callback)

    def callback(self):
        try:
            if psutil.sensors_temperatures().get("coretemp"):
                entries = psutil.sensors_temperatures().get("coretemp")
                entries = list(filter(lambda e: (e.label == "Package id 0"), entries))
            elif psutil.sensors_temperatures().get("k10temp"):
                entries = psutil.sensors_temperatures().get("k10temp")
                entries = list(filter(lambda e: (e.label == "Tdie"), entries))
            elif psutil.sensors_temperatures().get("cpu_thermal"):
                entries = psutil.sensors_temperatures().get("cpu_thermal")
            elif psutil.sensors_temperatures().get("nvme"):
                entries = psutil.sensors_temperatures().get("nvme")
            entries = list(map(lambda e: e.current, entries))
            temperature = sum(entries) / len(entries)
            msg = Temperature()
            msg.temperature = temperature
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.temperature}"')
        except Exception as e:
            self.get_logger().info(f'Exception: "{e}"')


def main(args=None):
    rclpy.init(args=args)

    node = TemperatureNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
