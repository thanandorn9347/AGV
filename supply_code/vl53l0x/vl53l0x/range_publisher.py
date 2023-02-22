import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import time
import board
import busio
import adafruit_vl53l0x

class RangePublisher(Node):
    def __init__(self):
        super().__init__('range_publisher')
        i2c = busio.I2C(board.SCL, board.SDA)
        self.publisher_ = self.create_publisher(Range, 'range_topic', 10)
        self.timer = self.create_timer(1, self.publish_range)
        self.sensor = adafruit_vl53l0x.VL53L0X(i2c)

        # sensor configuration
        self.sensor.start_continuous()
        self.sensor.measurement_timing_budge = 33000

    def publish_range(self):
        range_msg = Range()
        range_msg.header.frame_id = 'range_sensor'
        range_msg.field_of_view = 0.436332
        range_msg.min_range = 0.0
        range_msg.max_range = 2.0
        range_msg.range = self.sensor.range / 1000.0  # convert mm to m
        self.publisher_.publish(range_msg)

def main(args=None):
    rclpy.init(args=args)
    range_publisher = RangePublisher()
    rclpy.spin(range_publisher)
    range_publisher.sensor.stop_ranging()  # stop the sensor
    range_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
