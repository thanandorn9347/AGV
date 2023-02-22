from gpiozero import Button
from gpiozero import LED
from gpiozero import Buzzer
from gpiozero import OutputDevice
from sensor_msgs.msg import Range
import rclpy
from rclpy.node import Node
import time
from std_srvs.srv import Empty

class GpioControl(Node):
    def __init__(self):
        super().__init__('gpio_control')
        self.get_logger().info('Setting up GPIO...')

        # Configuration
        self.start_button_pin =7;
        self.operation_lamp =1;
        self.stop_button_pin =2;
        self.buzzer_pin = 3;
        self.linear_actuator_forward_relay_pin = 4;
        self.linear_actuator_reverse_relay_pin = 5;
        self.relay_forward = OutputDevice(self.linear_actuator_forward_relay_pin,active_high=True,initial_value=False)
        self.relay_reverse = OutputDevice(self.linear_actuator_reverse_relay_pin,active_high=True,initial_value=False)


        # ROS Interactions

        self.range_sub = self.create_subscription(
            Range,
            'range_publisher',
            self.range_callback,
            10)
        self.range_sub  # prevent unused variable warning

        self.motor_start = self.create_client(Empty, 'start_motor')
        if not self.motor_start.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('WARNING: start_motor service not available')

        self.motor_stop = self.create_client(Empty, 'stop_motor')
        if not self.motor_stop.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('WARNING: stop_motor service not available')
        
        self.motor_req = Empty.Request()

        self.client_futures = []
    def send_start_motor_req(self):
        if self.motor_start.service_is_ready():
            print("Starting lidar...")
            self.client_futures.append(self.motor_start.call_async(self.motor_req))
        else:
            print("start_motor not ready!")

    def send_stop_motor_req(self):
        if self.motor_stop.service_is_ready():
            print("Stopping lidar...")
            self.client_futures.append(self.motor_stop.call_async(self.motor_req))
        else:
            print("stop_motor not ready!")

    def range_callback(self, range_vals):
        self.get_logger().info(range_vals.range)
        return
    def check_for_finished_calls(self):
        incomplete_futures = []
        for f in self.client_futures:
            if f.done():
                res = f.result()
            else:
                incomplete_futures.append(f)  
def main(args=None):
    
    rclpy.init(args=args)

    gpio_control = GpioControl()

    gpio_control.send_stop_motor_req()

    # rate = face_player.create_rate(2)
    while rclpy.ok():
        rclpy.spin_once(gpio_control)
        gpio_control.check_for_finished_calls()
        time.sleep(0.01)
        
        

    gpio_control.destroy_node()
    rclpy.shutdown()