import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Vector3, Twist, Accel
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32MultiArray, Int16
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue

import time
import board
import adafruit_bno055
import os
import threading
import numpy as np


class ExtendedImuNode(Node):
    def __init__(self):
        # Get robot ID from parameter or environment
        robot_id = os.getenv("ROBOT_ID", "pX")
        username = os.getenv("USER", "pioneer")
        
        super().__init__(f'{robot_id}_imu_extended')
        
        # Declare parameters with defaults from system_config.yaml structure
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id', robot_id),
                ('publish_rate', 20.0),  # Hz
                ('connection_timeout', 10.0),
                ('retry_delay', 2.0),
                ('calibration_file', f"/home/{username}/imu_calib/bno055_offsets.json")
            ]
        )
        
        # Get parameters
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.connection_timeout = self.get_parameter('connection_timeout').get_parameter_value().double_value
        self.retry_delay = self.get_parameter('retry_delay').get_parameter_value().double_value
        self.calibration_file = self.get_parameter('calibration_file').get_parameter_value().string_value
        
        # Log loaded configuration
        self.get_logger().info(f'Extended IMU Node Configuration:')
        self.get_logger().info(f'  Robot ID: {self.robot_id}')
        self.get_logger().info(f'  Publish Rate: {self.publish_rate} Hz')
        self.get_logger().info(f'  Connection Timeout: {self.connection_timeout}s')
        self.get_logger().info(f'  Retry Delay: {self.retry_delay}s')
        
        # Connection state
        self.i2c = None
        self.sensor = None
        self.connection_active = False
        
        # Thread safety
        self.data_lock = threading.Lock()
        
        # Create publishers for extended IMU data
        self.publisher_imu = self.create_publisher(
            Imu, 
            f'{self.robot_id}/imu/data', 
            10
        )
        
        self.publisher_mag = self.create_publisher(
            MagneticField, 
            f'{self.robot_id}/imu/magnetic_field', 
            10
        )
        
        self.publisher_linear_accel = self.create_publisher(
            Vector3, 
            f'{self.robot_id}/imu/linear_acceleration', 
            10
        )
        
        self.publisher_angular_vel = self.create_publisher(
            Vector3, 
            f'{self.robot_id}/imu/angular_velocity', 
            10
        )
        
        self.publisher_gravity = self.create_publisher(
            Vector3, 
            f'{self.robot_id}/imu/gravity_vector', 
            10
        )
        
        self.publisher_temperature = self.create_publisher(
            Int16, 
            f'{self.robot_id}/imu/temperature', 
            10
        )
        
        self.publisher_diagnostics = self.create_publisher(
            DiagnosticArray, 
            f'{self.robot_id}/imu/diagnostics', 
            10
        )
        
        # Initialize IMU connection
        self.initialize_imu_connection()
        
        # Main timer
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Diagnostics timer
        self.diag_timer = self.create_timer(1.0, self.diagnostics_callback)
        
        self.get_logger().info('Extended IMU node initialized')

    def initialize_imu_connection(self):
        """Initialize IMU connection with retry mechanism"""
        retry_count = 0
        max_retries = 5
        
        while retry_count < max_retries:
            if self.connect_to_imu():
                self.get_logger().info('IMU connected successfully')
                return
            
            retry_count += 1
            self.get_logger().warn(f'IMU connection attempt {retry_count} failed, retrying in {self.retry_delay}s')
            time.sleep(self.retry_delay)
        
        self.get_logger().error('Failed to connect to IMU after maximum retries')

    def connect_to_imu(self):
        """Establish connection to BNO055 IMU"""
        try:
            with self.data_lock:
                self.i2c = board.I2C()
                self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
                
                # Wait for sensor to be ready
                start_time = time.time()
                while time.time() - start_time < self.connection_timeout:
                    if self.sensor.calibration_status[0] >= 0:  # Valid status
                        self.connection_active = True
                        self.load_calibration()
                        return True
                    time.sleep(0.1)
                
                return False
                
        except Exception as e:
            self.get_logger().error(f'IMU connection error: {e}')
            return False

    def load_calibration(self):
        """Load calibration from file if available"""
        try:
            if os.path.exists(self.calibration_file):
                import yaml
                with open(self.calibration_file, 'r') as f:
                    data = yaml.safe_load(f)
                
                if all(key in data for key in ['offsets_accelerometer', 'radius_accelerometer',
                                               'offsets_magnetometer', 'radius_magnetometer',
                                               'offsets_gyroscope']):
                    
                    # Apply calibration
                    self.sensor.offsets_accelerometer = tuple(data['offsets_accelerometer'])
                    self.sensor.radius_accelerometer = data['radius_accelerometer']
                    self.sensor.offsets_magnetometer = tuple(data['offsets_magnetometer'])
                    self.sensor.radius_magnetometer = data['radius_magnetometer']
                    self.sensor.offsets_gyroscope = tuple(data['offsets_gyroscope'])
                    
                    self.get_logger().info('Calibration loaded successfully')
                    
        except Exception as e:
            self.get_logger().warn(f'Failed to load calibration: {e}')

    def timer_callback(self):
        """Main callback to publish all IMU data"""
        try:
            with self.data_lock:
                if not self.connection_active or not self.sensor:
                    return
                
                # Get current timestamp
                stamp = self.get_clock().now().to_msg()
                
                # Publish combined IMU message
                imu_msg = Imu()
                imu_msg.header.stamp = stamp
                imu_msg.header.frame_id = f"{self.robot_id}_imu_link"
                
                # Orientation (quaternion)
                quat = self.sensor.quaternion
                if quat and None not in quat:
                    imu_msg.orientation.w = quat[0]
                    imu_msg.orientation.x = quat[1]
                    imu_msg.orientation.y = quat[2]
                    imu_msg.orientation.z = quat[3]
                    imu_msg.orientation_covariance[0] = 0.01  # Variance
                    imu_msg.orientation_covariance[4] = 0.01
                    imu_msg.orientation_covariance[8] = 0.01
                
                # Angular velocity (rad/s)
                gyro = self.sensor.gyro
                if gyro and None not in gyro:
                    imu_msg.angular_velocity.x = gyro[0]
                    imu_msg.angular_velocity.y = gyro[1]
                    imu_msg.angular_velocity.z = gyro[2]
                    imu_msg.angular_velocity_covariance[0] = 0.01
                    imu_msg.angular_velocity_covariance[4] = 0.01
                    imu_msg.angular_velocity_covariance[8] = 0.01
                
                # Linear acceleration (m/s^2)
                linear_accel = self.sensor.linear_acceleration
                if linear_accel and None not in linear_accel:
                    imu_msg.linear_acceleration.x = linear_accel[0]
                    imu_msg.linear_acceleration.y = linear_accel[1]
                    imu_msg.linear_acceleration.z = linear_accel[2]
                    imu_msg.linear_acceleration_covariance[0] = 0.1
                    imu_msg.linear_acceleration_covariance[4] = 0.1
                    imu_msg.linear_acceleration_covariance[8] = 0.1
                
                self.publisher_imu.publish(imu_msg)
                
                # Publish individual messages
                # Linear acceleration
                if linear_accel and None not in linear_accel:
                    linear_msg = Vector3()
                    linear_msg.x = linear_accel[0]
                    linear_msg.y = linear_accel[1]
                    linear_msg.z = linear_accel[2]
                    self.publisher_linear_accel.publish(linear_msg)
                
                # Angular velocity
                if gyro and None not in gyro:
                    angular_msg = Vector3()
                    angular_msg.x = gyro[0]
                    angular_msg.y = gyro[1]
                    angular_msg.z = gyro[2]
                    self.publisher_angular_vel.publish(angular_msg)
                
                # Gravity vector
                gravity = self.sensor.gravity
                if gravity and None not in gravity:
                    gravity_msg = Vector3()
                    gravity_msg.x = gravity[0]
                    gravity_msg.y = gravity[1]
                    gravity_msg.z = gravity[2]
                    self.publisher_gravity.publish(gravity_msg)
                
                # Magnetic field
                mag = self.sensor.magnetic
                if mag and None not in mag:
                    mag_msg = MagneticField()
                    mag_msg.header.stamp = stamp
                    mag_msg.header.frame_id = f"{self.robot_id}_imu_link"
                    mag_msg.magnetic_field.x = mag[0] * 1e-6  # Convert to Tesla
                    mag_msg.magnetic_field.y = mag[1] * 1e-6
                    mag_msg.magnetic_field.z = mag[2] * 1e-6
                    mag_msg.magnetic_field_covariance[0] = 1e-7
                    mag_msg.magnetic_field_covariance[4] = 1e-7
                    mag_msg.magnetic_field_covariance[8] = 1e-7
                    self.publisher_mag.publish(mag_msg)
                
                # Temperature
                temp = self.sensor.temperature
                if temp is not None:
                    temp_msg = Int16()
                    temp_msg.data = int(temp)
                    self.publisher_temperature.publish(temp_msg)
                    
        except Exception as e:
            self.get_logger().error(f'Timer callback error: {e}')
            self.handle_sensor_error()

    def diagnostics_callback(self):
        """Publish diagnostic information"""
        try:
            diag_array = DiagnosticArray()
            diag_array.header.stamp = self.get_clock().now().to_msg()
            
            status = DiagnosticStatus()
            status.name = f"{self.robot_id}_imu_extended"
            status.hardware_id = f"{self.robot_id}_bno055"
            
            with self.data_lock:
                if self.connection_active and self.sensor:
                    # Get calibration status
                    cal_status = self.sensor.calibration_status
                    
                    # Overall status
                    if all(s >= 2 for s in cal_status):
                        status.level = DiagnosticStatus.OK
                        status.message = "IMU fully calibrated and operational"
                    elif any(s >= 1 for s in cal_status):
                        status.level = DiagnosticStatus.WARN
                        status.message = "IMU partially calibrated"
                    else:
                        status.level = DiagnosticStatus.ERROR
                        status.message = "IMU not calibrated"
                    
                    # Add calibration details
                    status.values.append(KeyValue(key="System Calibration", value=str(cal_status[0])))
                    status.values.append(KeyValue(key="Gyro Calibration", value=str(cal_status[1])))
                    status.values.append(KeyValue(key="Accel Calibration", value=str(cal_status[2])))
                    status.values.append(KeyValue(key="Mag Calibration", value=str(cal_status[3])))
                    
                    # Add temperature
                    temp = self.sensor.temperature
                    if temp is not None:
                        status.values.append(KeyValue(key="Temperature", value=f"{temp}°C"))
                    
                else:
                    status.level = DiagnosticStatus.ERROR
                    status.message = "IMU not connected"
                
            diag_array.status.append(status)
            self.publisher_diagnostics.publish(diag_array)
            
        except Exception as e:
            self.get_logger().error(f'Diagnostics error: {e}')

    def handle_sensor_error(self):
        """Handle sensor errors by attempting reconnection"""
        with self.data_lock:
            self.connection_active = False
        
        # Attempt reconnection in a separate thread
        threading.Thread(target=self.reconnect_imu, daemon=True).start()

    def reconnect_imu(self):
        """Reconnect to IMU after error"""
        self.get_logger().warn('Attempting to reconnect to IMU...')
        time.sleep(self.retry_delay)
        self.initialize_imu_connection()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ExtendedImuNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"Extended IMU node failed: {e}")
    finally:
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()