import numpy as np
from typing import Tuple, Optional
import math


class ExtendedKalmanFilter:
    """
    Extended Kalman Filter for GPS/IMU sensor fusion
    State vector: [x, y, vx, vy, theta, omega]
    - x, y: position (m)
    - vx, vy: velocity (m/s)
    - theta: heading (rad)
    - omega: angular velocity (rad/s)
    """
    
    def __init__(self, dt: float = 0.1, accel_threshold: float = 0.1):
        self.dt = dt
        self.accel_threshold = accel_threshold  # Threshold for acceleration noise (m/s²)
        
        # State vector initialization
        self.state = np.zeros(6)  # [x, y, vx, vy, theta, omega]
        
        # State covariance matrix
        self.P = np.eye(6) * 100  # Initial uncertainty
        
        # Process noise covariance
        self.Q = np.diag([
            0.1,    # x position noise
            0.1,    # y position noise  
            0.5,    # x velocity noise
            0.5,    # y velocity noise
            0.01,   # heading noise
            0.1     # angular velocity noise
        ])
        
        # GPS measurement noise covariance
        self.R_gps = np.diag([
            2.0,    # x position measurement noise (m)
            2.0     # y position measurement noise (m)
        ])
        
        # IMU measurement noise covariance
        self.R_imu = np.diag([
            0.01,   # heading measurement noise (rad)
            0.05,   # angular velocity measurement noise (rad/s)
            0.1,    # x acceleration measurement noise (m/s²)
            0.1     # y acceleration measurement noise (m/s²)
        ])
        
        # Last update time for dead reckoning
        self.last_prediction_time = None
        
    def predict(self, ax: float = 0.0, ay: float = 0.0, omega: Optional[float] = None):
        """
        Prediction step using motion model
        ax, ay: linear accelerations in body frame (m/s²)
        omega: angular velocity (rad/s), if available
        """
        # Apply threshold to accelerations to remove noise offset
        if abs(ax) < self.accel_threshold:
            ax = 0.0
        if abs(ay) < self.accel_threshold:
            ay = 0.0
        
        # Extract current state
        x, y, vx, vy, theta, w = self.state
        
        # Use measured omega if available, otherwise use predicted
        if omega is not None:
            w = omega
            self.state[5] = w
        
        # Convert body frame accelerations to world frame
        ax_world = ax * np.cos(theta) - ay * np.sin(theta)
        ay_world = ax * np.sin(theta) + ay * np.cos(theta)
        
        # State transition matrix (Jacobian of motion model)
        F = np.eye(6)
        F[0, 2] = self.dt  # x depends on vx
        F[1, 3] = self.dt  # y depends on vy
        F[2, 4] = -self.dt * (ax * np.sin(theta) + ay * np.cos(theta))  # vx depends on theta
        F[3, 4] = self.dt * (ax * np.cos(theta) - ay * np.sin(theta))   # vy depends on theta
        F[4, 5] = self.dt  # theta depends on omega
        
        # Update state prediction
        self.state[0] += vx * self.dt  # x
        self.state[1] += vy * self.dt  # y
        self.state[2] += ax_world * self.dt  # vx
        self.state[3] += ay_world * self.dt  # vy
        self.state[4] += w * self.dt  # theta
        
        # Normalize theta to [-pi, pi]
        self.state[4] = self._normalize_angle(self.state[4])
        
        # Update covariance
        self.P = F @ self.P @ F.T + self.Q
        
    def update_gps(self, x_gps: float, y_gps: float, gps_accuracy: float = 1.0):
        """
        Update step with GPS measurement
        gps_accuracy: GPS accuracy factor (higher = less accurate)
        """
        # Measurement model for GPS (observes x, y)
        H = np.zeros((2, 6))
        H[0, 0] = 1  # GPS measures x
        H[1, 1] = 1  # GPS measures y
        
        # Adjust measurement noise based on GPS accuracy
        R = self.R_gps * gps_accuracy
        
        # Innovation (measurement residual)
        z = np.array([x_gps, y_gps])
        y = z - H @ self.state
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # State update
        self.state += K @ y
        
        # Covariance update
        I = np.eye(6)
        self.P = (I - K @ H) @ self.P
        
    def update_imu(self, theta_imu: float, omega_imu: float, 
                   ax_imu: float, ay_imu: float,
                   calibration_status: Tuple[int, int, int, int]):
        """
        Update step with IMU measurements
        calibration_status: (system, gyro, accel, mag) calibration levels
        """
        # Adjust measurement noise based on calibration status
        calib_factor = 1.0 / (1.0 + min(calibration_status))
        
        # Measurement model for IMU
        H = np.zeros((4, 6))
        H[0, 4] = 1  # IMU measures theta
        H[1, 5] = 1  # IMU measures omega
        H[2, 2] = 1  # IMU indirectly measures vx through acceleration
        H[3, 3] = 1  # IMU indirectly measures vy through acceleration
        
        # Adjust measurement noise based on calibration
        R = self.R_imu * calib_factor
        
        # Convert IMU accelerations to velocity measurements
        # This is approximate - assumes constant acceleration over dt
        vx_meas = self.state[2] + ax_imu * self.dt
        vy_meas = self.state[3] + ay_imu * self.dt
        
        # Innovation
        z = np.array([theta_imu, omega_imu, vx_meas, vy_meas])
        z_pred = H @ self.state
        
        # Handle angle wrapping for theta
        y = z - z_pred
        y[0] = self._normalize_angle(y[0])
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # State update
        self.state += K @ y
        self.state[4] = self._normalize_angle(self.state[4])
        
        # Covariance update
        I = np.eye(6)
        self.P = (I - K @ H) @ self.P
        
    def get_state(self) -> dict:
        """Get current state estimate"""
        return {
            'x': self.state[0],
            'y': self.state[1],
            'vx': self.state[2],
            'vy': self.state[3],
            'theta': self.state[4],
            'omega': self.state[5],
            'speed': np.sqrt(self.state[2]**2 + self.state[3]**2)
        }
        
    def get_covariance(self) -> np.ndarray:
        """Get current state covariance matrix"""
        return self.P.copy()
        
    def reset(self, x: float = 0, y: float = 0, theta: float = 0):
        """Reset filter state"""
        self.state = np.array([x, y, 0, 0, theta, 0])
        self.P = np.eye(6) * 100
        
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle


class SensorFusionManager:
    """
    Manages sensor fusion with adaptive weighting based on sensor quality
    """
    
    def __init__(self, ekf: ExtendedKalmanFilter):
        self.ekf = ekf
        
        # Sensor quality metrics
        self.gps_quality = 1.0
        self.imu_quality = 1.0
        
        # GPS quality factors
        self.min_satellites = 4
        self.good_satellites = 8
        self.max_hdop = 5.0
        
        # IMU quality thresholds
        self.min_calibration = 2  # Minimum calibration level
        
    def update_gps_quality(self, num_satellites: int, hdop: float = 1.0):
        """Update GPS quality based on satellite count and HDOP"""
        if num_satellites < self.min_satellites:
            self.gps_quality = 0.1
        elif num_satellites >= self.good_satellites and hdop < 2.0:
            self.gps_quality = 1.0
        else:
            # Linear interpolation
            sat_factor = (num_satellites - self.min_satellites) / (self.good_satellites - self.min_satellites)
            hdop_factor = max(0, 1 - (hdop - 1) / (self.max_hdop - 1))
            self.gps_quality = 0.5 * (sat_factor + hdop_factor)
            
    def update_imu_quality(self, calibration_status: Tuple[int, int, int, int]):
        """Update IMU quality based on calibration status"""
        # Average calibration level (0-3)
        avg_calib = sum(calibration_status) / 4.0
        self.imu_quality = avg_calib / 3.0
        
    def fuse_sensors(self, gps_data: Optional[dict], imu_data: Optional[dict]):
        """
        Perform sensor fusion with adaptive weighting
        gps_data: {'x': float, 'y': float, 'satellites': int, 'hdop': float}
        imu_data: {'theta': float, 'omega': float, 'ax': float, 'ay': float, 'calibration': tuple}
        """
        # Always perform prediction with IMU if available
        if imu_data:
            self.ekf.predict(
                ax=imu_data.get('ax', 0),
                ay=imu_data.get('ay', 0),
                omega=imu_data.get('omega')
            )
            
            # Update with IMU measurements if quality is sufficient
            if self.imu_quality > 0.3:
                self.ekf.update_imu(
                    theta_imu=imu_data['theta'],
                    omega_imu=imu_data['omega'],
                    ax_imu=imu_data.get('ax', 0),
                    ay_imu=imu_data.get('ay', 0),
                    calibration_status=imu_data.get('calibration', (0, 0, 0, 0))
                )
        
        # Update with GPS if available and quality is sufficient
        if gps_data and self.gps_quality > 0.2:
            # Adjust GPS accuracy based on quality
            gps_accuracy = 1.0 / self.gps_quality
            self.ekf.update_gps(
                x_gps=gps_data['x'],
                y_gps=gps_data['y'],
                gps_accuracy=gps_accuracy
            )
            
        return self.ekf.get_state()