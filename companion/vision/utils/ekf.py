import numpy as np
import math
import matplotlib.pyplot as plt
from filterpy.kalman import KalmanFilter


class DroneStateEstimator:
    def __init__(self, dt=0.1):
        self.dt = dt  # Time step

        # Initialize Kalman Filter
        self.kf = KalmanFilter(dim_x=4, dim_z=2)

        # State Transition Matrix (F)
        self.kf.F = np.array([[1, 0, self.dt, 0],
                              [0, 1, 0, self.dt],
                              [0, 0, 1,    0],
                              [0, 0, 0,    1]])

        # Control Input Matrix (B)
        self.kf.B = np.array([[0.5*self.dt**2, 0],
                              [0, 0.5*self.dt**2],
                              [self.dt, 0],
                              [0, self.dt]])

        # Measurement Matrix (H)
        self.kf.H = np.array([[0, 0, 1, 0],
                              [0, 0, 0, 1]])

        # Initial State Covariance Matrix (P)
        self.kf.P *= 1000.

        # Process Noise Covariance Matrix (Q)
        self.kf.Q = np.eye(4) * 0.1

        # Measurement Noise Covariance Matrix (R)
        self.kf.R = np.array([[0.5, 0],
                              [0, 0.5]])

        # Initial State Estimate
        self.kf.x = np.array([0., 0., 0., 0.])

        # Lists to store estimates
        self.estimated_positions = []
        self.estimated_velocities = []

    def deg2rad(self, degrees):
        return degrees * np.pi / 180

    def update(self, ax_body, ay_body, roll_deg, pitch_deg, vel_x_body, vel_y_body):
        # Convert roll and pitch to radians
        roll = self.deg2rad(roll_deg)
        pitch = self.deg2rad(pitch_deg)

        # Rotation matrix from body frame to inertial (world) frame
        R = np.array([
            [math.cos(pitch), math.sin(roll)*math.sin(pitch),
             math.cos(roll)*math.sin(pitch)],
            [0,               math.cos(roll),                -math.sin(roll)],
            [-math.sin(pitch), math.sin(roll)*math.cos(pitch),
             math.cos(roll)*math.cos(pitch)]
        ])

        # Simplify to 2D (assuming small angles and neglecting Z components)
        a_body = np.array([ax_body, ay_body, 0])
        a_world = R @ a_body  # Transform acceleration to world frame
        u = np.array([a_world[0], a_world[1]])  # Control input

        # Predict step
        self.kf.predict(u=u)

        # Transform velocities to world frame
        v_body = np.array([vel_x_body, vel_y_body, 0])
        v_world = R @ v_body  # Transform velocity to world frame
        z = np.array([v_world[0], v_world[1]])  # Measurement

        # Update step
        self.kf.update(z)

        # Store the estimates
        self.estimated_positions.append(self.kf.x[:2].copy())
        self.estimated_velocities.append(self.kf.x[2:].copy())

    def get_estimates(self):
        return np.array(self.estimated_positions), np.array(self.estimated_velocities)


if __name__ == '__main__':
    # Time settings
    dt = 0.1  # Time step
    t = np.arange(0, 100, dt)  # Total time 100 seconds

    # Generate dummy IMU data (accelerations and angles)
    # Simulate a drone moving in a circle with radius r
    r = 10  # Radius
    omega = 0.1  # Angular velocity

    np.random.seed(42)
    imu_accel_x_body = np.zeros_like(t)  # -r * omega**2 * np.cos(omega * t)
    imu_accel_y_body = np.zeros_like(t)  # -r * omega**2 * np.sin(omega * t)
    imu_roll = np.random.normal(0, 20, size=t.shape)
    imu_pitch = np.random.normal(0, 20, size=t.shape)

    # Generate dummy optical flow data (velocities)
    optical_flow_vel_x_body = -r * omega * np.sin(omega * t)
    optical_flow_vel_y_body = r * omega * np.cos(omega * t)

    # imu_accel_x_body += np.random.normal(0, 0.5, size=imu_accel_x_body.shape)
    # imu_accel_y_body += np.random.normal(0, 0.5, size=imu_accel_y_body.shape)
    optical_flow_vel_x_body += np.random.normal(
        0, 0.8, size=optical_flow_vel_x_body.shape)
    optical_flow_vel_y_body += np.random.normal(
        0, 0.8, size=optical_flow_vel_y_body.shape)
    # Initialize the estimator
    estimator = DroneStateEstimator(dt=dt)

    # Run the estimator over the data
    for i in range(len(t)):
        estimator.update(
            ax_body=imu_accel_x_body[i],
            ay_body=imu_accel_y_body[i],
            roll_deg=imu_roll[i],
            pitch_deg=imu_pitch[i],
            vel_x_body=optical_flow_vel_x_body[i],
            vel_y_body=optical_flow_vel_y_body[i]
        )

    # Get the estimates
    estimated_positions, estimated_velocities = estimator.get_estimates()

    # Plot the estimated positions
    plt.figure(figsize=(12, 6))
    plt.subplot(1, 2, 1)
    plt.plot(estimated_positions[:, 0],
             estimated_positions[:, 1], label='Estimated Path')
    plt.title('Estimated Position')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.legend()
    plt.axis('equal')

    # Plot the estimated velocities
    plt.subplot(1, 2, 2)
    plt.plot(t, estimated_velocities[:, 0], label='Estimated Vx')
    plt.plot(t, estimated_velocities[:, 1], label='Estimated Vy')
    plt.title('Estimated Velocities')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.legend()

    plt.tight_layout()
    plt.show()
