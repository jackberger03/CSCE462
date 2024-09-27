import board
import time
import busio
import adafruit_mpu6050
import math
from collections import deque

# Create I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Create an instance of the MPU6050 sensor
sensor = adafruit_mpu6050.MPU6050(i2c)

def moving_average(data):
    return sum(data) / len(data)

def detect_steps(acc_magnitude, threshold, step_cooldown):
    steps = 0
    step_detected = False
    last_step_time = 0
    current_time = time.time()
    
    for i, acc in enumerate(acc_magnitude):
        if acc > threshold and not step_detected and (current_time - last_step_time) > step_cooldown:
            steps += 1
            step_detected = True
            last_step_time = current_time
        elif acc < threshold * 0.9:  # Reset step detection when acceleration drops significantly below threshold
            step_detected = False
        
        current_time += 0.1  # Assuming 10Hz sampling rate
    
    return steps

# Parameters
window_size = 50  # Increased window size for better averaging
acc_window = deque(maxlen=window_size)
threshold_window = deque(maxlen=100)  # Window for calculating moving average threshold
steps = 0
start_time = time.time()
last_step_time = 0
step_cooldown = 0.4  # Minimum time between steps (in seconds)

print("Calibrating sensor. Please keep the sensor still...")
calibration_samples = 100
calibration_sum = [0, 0, 0]
for _ in range(calibration_samples):
    accel = sensor.acceleration
    calibration_sum[0] += accel[0]
    calibration_sum[1] += accel[1]
    calibration_sum[2] += accel[2]
    time.sleep(0.01)

calibration_offset = [
    calibration_sum[0] / calibration_samples,
    calibration_sum[1] / calibration_samples,
    (calibration_sum[2] / calibration_samples) - 9.81  # Subtract gravity
]
print("Calibration complete.")

print("Start walking. Press Ctrl+C to stop.")

try:
    while True:
        try:
            accelerometer_data = sensor.acceleration
        except Exception as e:
            print(f"Error reading sensor data: {e}")
            print("Make sure the sensor is connected properly.")
            break

        # Apply calibration offset
        calibrated_accel = (
            accelerometer_data[0] - calibration_offset[0],
            accelerometer_data[1] - calibration_offset[1],
            accelerometer_data[2] - calibration_offset[2]
        )

        # Calculate acceleration magnitude
        acc_magnitude = math.sqrt(
            calibrated_accel[0]**2 + 
            calibrated_accel[1]**2 + 
            calibrated_accel[2]**2
        )

        # Add to acceleration window
        acc_window.append(acc_magnitude)
        
        # Calculate moving average and update threshold window
        if len(acc_window) == window_size:
            avg_magnitude = moving_average(acc_window)
            threshold_window.append(avg_magnitude)
        
        # Process data when both windows are full
        if len(acc_window) == window_size and len(threshold_window) == threshold_window.maxlen:
            moving_avg_threshold = moving_average(threshold_window) * 1.1  # 10% above moving average
            current_time = time.time()
            if (current_time - last_step_time) > step_cooldown:
                new_steps = detect_steps(acc_window, threshold=moving_avg_threshold, step_cooldown=step_cooldown)
                if new_steps > 0:
                    steps += new_steps
                    last_step_time = current_time
                    print(f"Steps: {steps}, Threshold: {moving_avg_threshold:.2f}")
        
        # Uncomment for debugging
        # print(f"Acc Magnitude: {acc_magnitude:.2f}, Moving Avg: {moving_avg_threshold:.2f}")
        
        time.sleep(0.1)  # 10Hz sampling rate

except KeyboardInterrupt:
    end_time = time.time()
    duration = end_time - start_time
    print(f"\nFinal step count: {steps}")
    print(f"Duration: {duration:.2f} seconds")
    print(f"Average steps per minute: {steps / (duration / 60):.2f}")