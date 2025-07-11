import rclpy
from rclpy.node import Node

import time
import math

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int16, Float64
from rf_sim_interfaces.srv import GetRxPower

from .my_ros_module import PubSubManager

UPDATE_RATE = 0.1
VEL_ALIVE = 10
MAX_TRANS = 1.0
MAX_ROTATE = 1.0


class FakeRover(Node):

    def __init__(self):
        super().__init__('fake_rover')
        self.pubsub = PubSubManager(self)
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id', "p0"),
                ('x', 0.0),
                ('y', 0.0),
                ('t', 0.0),
                ('prefix', "")
            ]
        )
        params = self._parameters
        for name, param in params.items():
            self.get_logger().info(f"PARAMS/ {name}: {param.value}")

        self.robot_id = self.get_parameter('robot_id').value
        self.prefix = self.get_parameter('prefix').value
        
        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        t = self.get_parameter('t').value

        self.position = {'x': x, 'y': y, 'theta': t}
        self.vel = {'transform': 0.0, 'rotate': 0.0, 'alive': 0}

        self.rssi = None

        self.pubsub.create_subscription(Twist, f'{self.prefix}/{self.robot_id}/cmd_vel', self.teleop_callback, 10)
        self.pubsub.create_publisher(Pose2D, f'{self.prefix}/{self.robot_id}/pose2D', 10)
        self.pubsub.create_publisher(Float64, f'{self.prefix}/{self.robot_id}/rssi', 5)
        
        timer_period = UPDATE_RATE
        self.pub_timer = self.create_timer(timer_period, self.timer_callback)

        self.client = self.create_client(GetRxPower, 'get_rx_power')
        if self.client.wait_for_service(timeout_sec=1.0):
            self.rssi_timer = self.create_timer(timer_period, self.check_rssi)

    def update_position(self):
        if self.vel['alive'] <= 0:
            self.pubsub.publish(f'{self.prefix}/{self.robot_id}/pose2D', Pose2D(x=self.position['x'], y=self.position['y'], theta=self.position['theta']))
            return
        else:
            _theta_avg = self.position['theta'] - self.vel['rotate'] * UPDATE_RATE / 2
            self.position['theta'] -= UPDATE_RATE  * self.vel['rotate']
            self.position['theta'] = self.wrap_to_pi(self.position['theta'])
            self.position['x'] += UPDATE_RATE  * self.vel['transform'] * math.sin(_theta_avg)
            self.position['y'] += UPDATE_RATE  * self.vel['transform'] * math.cos(_theta_avg)
            self.vel['alive'] -= 1
        self.pubsub.publish(f'{self.prefix}/{self.robot_id}/pose2D', Pose2D(x=self.position['x'], y=self.position['y'], theta=self.position['theta']))
    
    def wrap_to_pi(self, t):
        return (t + math.pi) % (2 * math.pi) - math.pi
        
    def timer_callback(self):
        self.update_position()
        if self.rssi is not None:
            self.publish_rssi()
    
    def publish_rssi(self):
        rssi_msg = Float64()
        rssi_msg.data = self.rssi
        self.pubsub.publish(f'{self.prefix}/{self.robot_id}/rssi', rssi_msg)
        #self.get_logger().info(f'{self.prefix}/{self.robot_id}/rssi {rssi_msg.data}')
    
    def check_rssi(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        request = GetRxPower.Request()
        request.x = self.position['x']
        request.y = self.position['y']
        request.z = 0.5

        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)
    
    def response_callback(self, future):
        try:
            response = future.result()
            self.rssi = response.rx_db
            #self.get_logger().info(f"Received RSSI: {self.rssi}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def teleop_callback(self, msg):
        self.vel.update({'transform': max(-MAX_TRANS, min(msg.linear.x, MAX_TRANS)), 'rotate': max(-MAX_ROTATE, min(msg.angular.z, MAX_ROTATE)), 'alive': VEL_ALIVE})



def main(args=None):
    rclpy.init(args=args)

    fake_rover = FakeRover()

    rclpy.spin(fake_rover)

    fake_rover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()