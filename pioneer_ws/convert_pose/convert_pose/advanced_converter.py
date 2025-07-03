import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Quaternion, Pose2D, PoseWithCovarianceStamped, Twist, Vector3
from std_msgs.msg import Float32MultiArray, Int16MultiArray, Int16, Bool
from diagnostic_msgs.msg import DiagnosticArray

import math
import threading
import time
import numpy as np

from pioneer_interfaces.srv import RefGPS
from .ekf_fusion import ExtendedKalmanFilter, SensorFusionManager

import datetime
import os
from math import sin, cos, asin, atan2, sqrt, degrees, pi, radians


class AdvancedPoseConverter(Node):
    def __init__(self, n_rover=6):
        # Get robot ID from parameter or environment
        robot_id = os.getenv("ROBOT_ID", "pX")
        super().__init__(f'{robot_id}_advanced_pose_converter')
        
        # Declare parameters with defaults
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id', robot_id),
                ('timer_period', 0.05),  # 20Hz for EKF
                ('health_check_period', 1.0),
                ('reset_timeout', 5.0),
                ('use_ekf', True),
                ('publish_diagnostics', True)
            ]
        )
        
        # Get parameters
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        self.health_check_period = self.get_parameter('health_check_period').get_parameter_value().double_value
        self.reset_timeout = self.get_parameter('reset_timeout').get_parameter_value().double_value
        self.use_ekf = self.get_parameter('use_ekf').get_parameter_value().bool_value
        self.publish_diagnostics = self.get_parameter('publish_diagnostics').get_parameter_value().bool_value
        
        # Log loaded configuration
        self.get_logger().info(f'Advanced Pose Converter Configuration:')
        self.get_logger().info(f'  Robot ID: {self.robot_id}')
        self.get_logger().info(f'  Timer Period: {self.timer_period}s')
        self.get_logger().info(f'  EKF Enabled: {self.use_ekf}')
        
        # Initialize EKF
        self.ekf = ExtendedKalmanFilter(dt=self.timer_period)
        self.fusion_manager = SensorFusionManager(self.ekf)
        
        # EKF initialization flags
        self.ekf_initialized = False
        self.waiting_for_ref_gps = True
        self.initial_samples = []
        self.initial_sample_count = 10  # Number of samples to average for initialization
        
        # Initialize all data variables
        self.ref_lat = None
        self.ref_lon = None
        self.lat = None
        self.lon = None
        self.gps_status = None
        self.gps_satellites = 0
        self.gps_hdop = 1.0
        
        # IMU data
        self.imu_orientation = None
        self.imu_angular_velocity = None
        self.imu_linear_acceleration = None
        self.imu_calibration_status = (0, 0, 0, 0)
        
        # Legacy compatibility
        self.quaternion = None
        self.euler_x = None
        self.euler_y = None
        self.euler_z = None
        self.calibration = None
        self.lat_offset = 0.0
        self.lon_offset = 0.0
        
        # Flags for data availability
        self.gps_data_available = False
        self.imu_data_available = False
        self.last_gps_time = time.time()
        self.last_imu_time = time.time()
        
        # Thread safety lock
        self.data_lock = threading.Lock()

        # Create subscriptions
        self.create_robust_subscriptions()
        
        # Create publishers
        self.pose_publisher = self.create_publisher(
            Pose2D,
            f'/{self.robot_id}/pose2D',
            5)
        
        # Advanced pose with covariance
        self.pose_cov_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            f'/{self.robot_id}/pose_with_covariance',
            5)
        
        # Velocity estimate
        self.velocity_publisher = self.create_publisher(
            Twist,
            f'/{self.robot_id}/velocity_estimate',
            5)
        
        # Service client for reference GPS
        self.cli = self.create_client(RefGPS, 'reference_gps')
        
        # Timer for main processing loop
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Timer for data health monitoring
        self.health_timer = self.create_timer(self.health_check_period, self.health_check_callback)
        
        # Initialize reference GPS
        self.initialize_reference_gps()

    def create_robust_subscriptions(self):
        """Create subscriptions with error handling"""
        # Original subscriptions for backward compatibility
        try:
            self.gps_subscription = self.create_subscription(
                NavSatFix,
                f"/{self.robot_id}/gps1",
                self.gps_callback,
                1)
        except Exception as e:
            self.get_logger().error(f"Failed to create GPS subscription: {e}")
            
        try:
            self.quaternion_subscription = self.create_subscription(
                Quaternion,
                f"/{self.robot_id}/imu/quaternion",
                self.quaternion_callback,
                1)
        except Exception as e:
            self.get_logger().error(f"Failed to create quaternion subscription: {e}")
            
        try:
            self.euler_subscription = self.create_subscription(
                Float32MultiArray,
                f"/{self.robot_id}/imu/eulerAngle",
                self.euler_callback,
                1)
        except Exception as e:
            self.get_logger().error(f"Failed to create euler subscription: {e}")
        
        # New subscriptions for advanced IMU data
        try:
            self.imu_subscription = self.create_subscription(
                Imu,
                f"/{self.robot_id}/imu/data",
                self.imu_callback,
                1)
        except Exception as e:
            self.get_logger().error(f"Failed to create IMU subscription: {e}")
            
        try:
            self.diagnostics_subscription = self.create_subscription(
                DiagnosticArray,
                f"/{self.robot_id}/imu/diagnostics",
                self.diagnostics_callback,
                1)
        except Exception as e:
            self.get_logger().error(f"Failed to create diagnostics subscription: {e}")
            
        # Reset subscriptions
        try:
            self.reset_gps_subscription = self.create_subscription(
                Bool,
                f"/{self.robot_id}/reset_gps",
                self.reset_gps_callback,
                1)
        except Exception as e:
            self.get_logger().error(f"Failed to create reset GPS subscription: {e}")
            
        try:
            self.reset_imu_subscription = self.create_subscription(
                Bool,
                f"/{self.robot_id}/reset_imu",
                self.reset_imu_callback,
                1)
        except Exception as e:
            self.get_logger().error(f"Failed to create reset IMU subscription: {e}")

    def initialize_reference_gps(self):
        """Initialize reference GPS with retry mechanism"""
        def wait_for_service():
            retry_count = 0
            max_retries = 10
            while not self.cli.wait_for_service(timeout_sec=2.0) and retry_count < max_retries:
                self.get_logger().warn(f'Reference GPS service not available, waiting... (attempt {retry_count + 1})')
                retry_count += 1
                time.sleep(1.0)
            
            if retry_count < max_retries:
                self.request_reference_gps()
            else:
                self.get_logger().error('Reference GPS service failed to become available')
        
        # Run service waiting in a separate thread to avoid blocking
        threading.Thread(target=wait_for_service, daemon=True).start()
    
    def request_reference_gps(self):
        """Request reference GPS with improved error handling"""
        try:
            self.req = RefGPS.Request()
            self.req.robot_id = self.robot_id
            self.future = self.cli.call_async(self.req)
            self.future.add_done_callback(self.srv_callback)
            self.get_logger().info(f'[{self.req.robot_id}]Sent request for reference GPS')
        except Exception as e:
            self.get_logger().error(f'Failed to request reference GPS: {e}')
    
    def srv_callback(self, future):
        """Handle reference GPS service response"""
        try:
            result = future.result()
            with self.data_lock:
                self.ref_lat = result.gps.latitude
                self.ref_lon = result.gps.longitude
                # Clear any previous initialization samples when reference GPS is updated
                if self.waiting_for_ref_gps:
                    self.initial_samples = []
                    self.get_logger().info('Reference GPS updated, clearing initialization samples')
            self.get_logger().info(f'Reference GPS received: {self.ref_lat:.6f}, {self.ref_lon:.6f}')
        except Exception as e:
            self.get_logger().error(f'Failed to get reference GPS: {e}')
            # Retry after delay
            threading.Timer(5.0, self.request_reference_gps).start()
    
    def gps_callback(self, msg):
        """Handle GPS data with validation"""
        try:
            with self.data_lock:
                # Validate GPS data
                if msg.latitude != 0.0 and msg.longitude != 0.0:
                    self.lat = msg.latitude
                    self.lon = msg.longitude
                    self.gps_status = msg.status.status
                    
                    # Extract additional GPS quality info if available
                    if hasattr(msg.status, 'satellites_used'):
                        self.gps_satellites = msg.status.satellites_used
                    if hasattr(msg, 'position_covariance') and msg.position_covariance[0] > 0:
                        # Estimate HDOP from covariance
                        self.gps_hdop = math.sqrt(msg.position_covariance[0])
                    
                    self.gps_data_available = True
                    self.last_gps_time = time.time()
                    
                    # Update GPS quality in fusion manager
                    self.fusion_manager.update_gps_quality(self.gps_satellites, self.gps_hdop)
                else:
                    self.get_logger().debug('Invalid GPS data received')
        except Exception as e:
            self.get_logger().error(f'GPS callback error: {e}')
    
    def imu_callback(self, msg):
        """Handle comprehensive IMU data"""
        try:
            with self.data_lock:
                self.imu_orientation = msg.orientation
                self.imu_angular_velocity = msg.angular_velocity
                self.imu_linear_acceleration = msg.linear_acceleration
                
                # Convert quaternion to euler for legacy compatibility
                if self.imu_orientation:
                    self.quaternion = self.imu_orientation
                    euler = self.quaternion_to_euler(
                        msg.orientation.w,
                        msg.orientation.x,
                        msg.orientation.y,
                        msg.orientation.z
                    )
                    self.euler_x = math.degrees(euler[0])
                    self.euler_y = math.degrees(euler[1])
                    self.euler_z = math.degrees(euler[2])
                    
                    if self.calibration is None:
                        self.calibration = self.euler_x
                
                self.imu_data_available = True
                self.last_imu_time = time.time()
        except Exception as e:
            self.get_logger().error(f'IMU callback error: {e}')
    
    def quaternion_callback(self, msg):
        """Handle quaternion data (legacy support)"""
        try:
            with self.data_lock:
                self.quaternion = msg
                self.last_imu_time = time.time()
        except Exception as e:
            self.get_logger().error(f'Quaternion callback error: {e}')
    
    def euler_callback(self, msg):
        """Handle Euler angle data (legacy support)"""
        try:
            if len(msg.data) >= 3:
                with self.data_lock:
                    self.euler_x = msg.data[0]
                    self.euler_y = msg.data[1]
                    self.euler_z = msg.data[2]
                    if self.calibration is None:
                        self.calibration = self.euler_x
                    self.imu_data_available = True
                    self.last_imu_time = time.time()
        except Exception as e:
            self.get_logger().error(f'Euler callback error: {e}')
    
    def diagnostics_callback(self, msg):
        """Handle IMU diagnostics for calibration status"""
        try:
            for status in msg.status:
                if 'imu' in status.name.lower():
                    # Extract calibration values
                    cal_values = [0, 0, 0, 0]
                    for kv in status.values:
                        if 'System Calibration' in kv.key:
                            cal_values[0] = int(kv.value)
                        elif 'Gyro Calibration' in kv.key:
                            cal_values[1] = int(kv.value)
                        elif 'Accel Calibration' in kv.key:
                            cal_values[2] = int(kv.value)
                        elif 'Mag Calibration' in kv.key:
                            cal_values[3] = int(kv.value)
                    
                    with self.data_lock:
                        self.imu_calibration_status = tuple(cal_values)
                        self.fusion_manager.update_imu_quality(self.imu_calibration_status)
                        
        except Exception as e:
            self.get_logger().error(f'Diagnostics callback error: {e}')

    def reset_gps_callback(self, msg):
        """Handle GPS reset"""
        with self.data_lock:
            if self.lat is not None and self.ref_lat is not None:
                self.lat_offset = self.lat - self.ref_lat
                self.lon_offset = self.lon - self.ref_lon
                
                # Clear any initialization samples and reset EKF
                self.initial_samples = []
                self.waiting_for_ref_gps = False
                
                # Reset EKF position with reference GPS as origin
                x, y = self.convert_gps_to_pose(self.lat, self.lon, self.ref_lat, self.ref_lon)
                theta = self.degree_to_radian_pi_range(self.euler_x - self.calibration) if self.euler_x is not None else 0.0
                
                self.ekf.reset(x=x, y=y, theta=theta)
                self.ekf_initialized = True
                
                self.get_logger().info(f'[{self.robot_id}]GPS reset completed: offsets {self.lat_offset:.6f}, {self.lon_offset:.6f}')
                self.get_logger().info(f'EKF reset at: x={x:.2f}m, y={y:.2f}m from reference GPS')

    def reset_imu_callback(self, msg):
        """Handle IMU reset"""
        with self.data_lock:
            if self.euler_x is not None:
                self.calibration = self.euler_x
                
                # Clear initialization samples and trigger re-initialization if needed
                if self.ekf_initialized:
                    # If EKF is already initialized, just update the heading in the current state
                    if self.ref_lat is not None and self.lat is not None:
                        # Get current position
                        x, y = self.convert_gps_to_pose(
                            self.lat - self.lat_offset,
                            self.lon - self.lon_offset,
                            self.ref_lat,
                            self.ref_lon
                        )
                        theta = self.degree_to_radian_pi_range(self.euler_x - self.calibration)
                        
                        # Update EKF state with new heading
                        self.ekf.state[4] = theta  # Update heading in state vector
                        self.get_logger().info(f'[{self.robot_id}]EKF heading updated: {np.degrees(theta):.1f}°')
                else:
                    # If EKF not initialized yet, clear samples to trigger fresh initialization
                    self.initial_samples = []
                    self.waiting_for_ref_gps = True
                    self.get_logger().info(f'[{self.robot_id}]IMU reset during initialization, clearing samples')
                
                self.get_logger().info(f'[{self.robot_id}]IMU reset completed: calibration {self.calibration:.2f}')

    def health_check_callback(self):
        """Monitor sensor health and data availability"""
        current_time = time.time()
        
        # Check GPS health
        if current_time - self.last_gps_time > 5.0:
            self.gps_data_available = False
            self.fusion_manager.gps_quality = 0.1
            self.get_logger().warn('GPS data timeout detected')
        
        # Check IMU health
        if current_time - self.last_imu_time > 5.0:
            self.imu_data_available = False
            self.fusion_manager.imu_quality = 0.1
            self.get_logger().warn('IMU data timeout detected')
        
        # Check reference GPS
        if self.ref_lat is None or self.ref_lon is None:
            self.get_logger().warn('Reference GPS not available, retrying...')
            self.initialize_reference_gps()
    
    def timer_callback(self):
        """Main processing loop with EKF fusion"""
        try:
            with self.data_lock:
                # Check if reference GPS is available
                if self.ref_lat is None or self.ref_lon is None:
                    return
                
                # Handle EKF initialization if not yet initialized
                if not self.ekf_initialized and self.waiting_for_ref_gps:
                    if self.validate_initial_data():
                        if self.initialize_ekf_from_current_position():
                            self.waiting_for_ref_gps = False
                            self.get_logger().info('EKF initialization complete, starting normal operation')
                        else:
                            # Still collecting samples or initialization failed
                            return
                    else:
                        # Required data not yet available
                        return
                
                # Prepare sensor data for fusion
                gps_data = None
                imu_data = None
                
                if self.gps_data_available and self.lat is not None:
                    x, y = self.convert_gps_to_pose(
                        self.lat - self.lat_offset, 
                        self.lon - self.lon_offset, 
                        self.ref_lat, 
                        self.ref_lon
                    )
                    gps_data = {
                        'x': x,
                        'y': y,
                        'satellites': self.gps_satellites,
                        'hdop': self.gps_hdop
                    }
                
                if self.imu_data_available:
                    # Get heading and angular velocity
                    theta = self.degree_to_radian_pi_range(self.euler_x - self.calibration) if self.euler_x is not None else 0
                    omega = self.imu_angular_velocity.z if self.imu_angular_velocity else 0
                    
                    # Get linear accelerations (body frame)
                    ax = self.imu_linear_acceleration.x if self.imu_linear_acceleration else 0
                    ay = self.imu_linear_acceleration.y if self.imu_linear_acceleration else 0
                    
                    imu_data = {
                        'theta': theta,
                        'omega': omega,
                        'ax': ax,
                        'ay': ay,
                        'calibration': self.imu_calibration_status
                    }
                
                if self.use_ekf and self.ekf_initialized and (gps_data or imu_data):
                    # Perform sensor fusion
                    state = self.fusion_manager.fuse_sensors(gps_data, imu_data)
                    
                    # Publish fused pose
                    msg = Pose2D()
                    msg.x = state['x']
                    msg.y = state['y']
                    msg.theta = state['theta']
                    self.pose_publisher.publish(msg)
                    
                    # Publish pose with covariance
                    self.publish_pose_with_covariance(state)
                    
                    # Publish velocity estimate
                    self.publish_velocity(state)
                    
                elif self.validate_data():
                    # Fallback to original simple fusion
                    msg = Pose2D()
                    msg.x, msg.y = self.convert_gps_to_pose(
                        self.lat - self.lat_offset, 
                        self.lon - self.lon_offset, 
                        self.ref_lat, 
                        self.ref_lon
                    )
                    msg.theta = self.degree_to_radian_pi_range(self.euler_x - self.calibration)
                    
                    self.pose_publisher.publish(msg)
                    
        except Exception as e:
            self.get_logger().error(f'Timer callback error: {e}')

    def publish_pose_with_covariance(self, state):
        """Publish pose with covariance matrix from EKF"""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"{self.robot_id}_odom"
        
        # Position
        msg.pose.pose.position.x = state['x']
        msg.pose.pose.position.y = state['y']
        msg.pose.pose.position.z = 0.0
        
        # Orientation (convert theta to quaternion)
        msg.pose.pose.orientation = self.euler_to_quaternion(0, 0, state['theta'])
        
        # Covariance (6x6 matrix as 36-element array)
        # Extract relevant parts from EKF covariance
        P = self.ekf.get_covariance()
        cov = np.zeros(36)
        cov[0] = P[0, 0]  # x variance
        cov[1] = P[0, 1]  # x-y covariance
        cov[6] = P[1, 0]  # y-x covariance  
        cov[7] = P[1, 1]  # y variance
        cov[35] = P[4, 4]  # theta variance
        
        msg.pose.covariance = cov.tolist()
        
        self.pose_cov_publisher.publish(msg)

    def publish_velocity(self, state):
        """Publish velocity estimate from EKF"""
        msg = Twist()
        
        # Linear velocity in robot frame
        vx_world = state['vx']
        vy_world = state['vy']
        theta = state['theta']
        
        # Transform to robot frame
        msg.linear.x = vx_world * cos(theta) + vy_world * sin(theta)
        msg.linear.y = -vx_world * sin(theta) + vy_world * cos(theta)
        msg.linear.z = 0.0
        
        # Angular velocity
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = state['omega']
        
        self.velocity_publisher.publish(msg)

    def validate_data(self):
        """Validate that all required data is available (legacy)"""
        if self.ref_lat is None or self.ref_lon is None:
            return False
        
        if not self.gps_data_available or self.lat is None or self.lon is None:
            return False
        
        if not self.imu_data_available or self.euler_x is None or self.calibration is None:
            return False
        
        return True
    
    def validate_initial_data(self):
        """Validate that all required data is available for initialization"""
        # Reference GPS must be available
        if self.ref_lat is None or self.ref_lon is None:
            self.get_logger().debug("Reference GPS not available for initialization")
            return False
        
        # Current GPS must be available
        if not self.gps_data_available or self.lat is None or self.lon is None:
            self.get_logger().debug("GPS data not available for initialization")
            return False
        
        # IMU data is optional but recommended
        if not self.imu_data_available:
            self.get_logger().debug("IMU data not available for initialization (will use GPS only)")
        
        return True
    
    def initialize_ekf_from_current_position(self):
        """Initialize EKF from current robot position relative to reference GPS"""
        try:
            # Calculate relative position from reference GPS
            x, y = self.convert_gps_to_pose(
                self.lat - self.lat_offset,
                self.lon - self.lon_offset,
                self.ref_lat,
                self.ref_lon
            )
            
            # Get initial heading from IMU if available
            theta = 0.0
            if self.euler_x is not None:
                theta = self.degree_to_radian_pi_range(self.euler_x - self.calibration)
            
            # Collect initial samples for averaging
            self.initial_samples.append([x, y, theta])
            
            # Wait for enough samples
            if len(self.initial_samples) < self.initial_sample_count:
                self.get_logger().debug(f'Collecting initialization samples: {len(self.initial_samples)}/{self.initial_sample_count}')
                return False
            
            # Calculate average position and heading
            samples_array = np.array(self.initial_samples)
            avg_x = np.mean(samples_array[:, 0])
            avg_y = np.mean(samples_array[:, 1])
            
            # Handle circular mean for angles
            sin_sum = np.sum(np.sin(samples_array[:, 2]))
            cos_sum = np.sum(np.cos(samples_array[:, 2]))
            avg_theta = np.arctan2(sin_sum, cos_sum)
            
            # Initialize EKF state
            self.ekf.state = np.array([avg_x, avg_y, 0.0, 0.0, avg_theta, 0.0])
            
            # Set initial covariance based on GPS quality
            initial_position_variance = 2.0 if self.gps_satellites >= 6 else 5.0
            self.ekf.P = np.diag([
                initial_position_variance,  # x position variance (m²)
                initial_position_variance,  # y position variance (m²)
                0.5,                       # x velocity variance
                0.5,                       # y velocity variance
                0.1,                       # heading variance
                0.05                       # angular velocity variance
            ])
            
            self.ekf_initialized = True
            self.get_logger().info(
                f'EKF initialized successfully:\n'
                f'  Position: x={avg_x:.2f}m, y={avg_y:.2f}m (relative to reference GPS)\n'
                f'  Heading: {avg_theta:.2f}rad ({np.degrees(avg_theta):.1f}°)\n'
                f'  GPS satellites: {self.gps_satellites}\n'
                f'  Reference GPS: lat={self.ref_lat:.6f}, lon={self.ref_lon:.6f}'
            )
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'EKF initialization error: {e}')
            return False
        
    def degree_to_radian_pi_range(self, degree):
        """Convert degree to radian with range normalization"""
        try:
            rad = math.radians(degree)
            rad = (rad + math.pi) % (2 * math.pi) - math.pi
            return rad
        except Exception as e:
            self.get_logger().error(f'Degree to radian conversion error: {e}')
            return 0.0
    
    def convert_gps_to_pose(self, cur_lat, cur_lon, ref_lat, ref_lon):
        """Convert GPS coordinates to local pose"""
        try:
            R = 6371000  # Earth radius in meters
            
            delta_lat = radians(cur_lat - ref_lat)
            delta_lon = radians(cur_lon - ref_lon)
            
            ref_lat_rad = radians(ref_lat)
            
            x = R * delta_lon * cos(ref_lat_rad)
            y = R * delta_lat
            
            return x, y
        except Exception as e:
            self.get_logger().error(f'GPS to pose conversion error: {e}')
            return 0.0, 0.0
    
    def quaternion_to_euler(self, w, x, y, z):
        """Convert quaternion to euler angles"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        pitch = asin(sinp) if abs(sinp) <= 1 else math.copysign(pi / 2, sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert euler angles to quaternion"""
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        
        return q


def main(args=None):
    rclpy.init(args=args)
    try:
        converter = AdvancedPoseConverter()
        rclpy.spin(converter)
    except Exception as e:
        print(f"Advanced converter failed: {e}")
    finally:
        try:
            converter.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()