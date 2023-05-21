import numpy as np

class KalmanFilter1D:
    def __init__(self, initial_state, initial_covariance, process_noise, measurement_noise):
        self.state = initial_state
        self.covariance = initial_covariance
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise

    def predict(self):
        self.covariance += self.process_noise
        return self.state

    def update(self, measurement):
        kalman_gain = self.covariance / (self.covariance + self.measurement_noise)
        self.state = self.state + kalman_gain * (measurement - self.state)
        self.covariance = (1 - kalman_gain) * self.covariance

# Initialize the Kalman filter
initial_state = 0.0
initial_covariance = 1.0
process_noise = 0.001
measurement_noise = 0.1
kalman_filter = KalmanFilter1D(initial_state, initial_covariance, process_noise, measurement_noise)

# theoretical velocities and IMU measurements
theoretical_velocities = [2, 2.1, 1.9, 2.2, 1.8]
imu_measurements = [1.9, 2.3, 2.0, 2.1, 1.7]

#theoretical velocities calculate for each time step
#extracting velocities from the car each time step according to the frequency and list it.

# Apply the Kalman filter
for i in range(len(theoretical_velocities)):
    # Prediction step
    kalman_filter.predict()

    # Update step
    measurement = imu_measurements[i]
    kalman_filter.update(measurement)

    # Print the updated state (fused velocity)
    print("Fused velocity at step {}: {:.2f}".format(i, kalman_filter.state))
