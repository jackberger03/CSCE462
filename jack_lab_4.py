import board
import time
import busio
import adafruit_mpu6050
import math

# Create I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Create an instance of the MPU6050 sensor
sensor = adafruit_mpu6050.MPU6050(i2c)

def low_pass_filter(data, alpha=0.1):
    filtered_data = [0] * len(data)
    filtered_data[0] = data[0]
    for i in range(1, len(data)):
        filtered_data[i] = alpha * data[i] + (1 - alpha) * filtered_data[i-1]
    return filtered_data

def detect_steps(acc_magnitude, threshold=11.0, step_cooldown=0.5):
    steps = 0
    step_detected = False
    last_step_time = 0
    current_time = time.time()
    
    for i, acc in enumerate(acc_magnitude):
        if acc > threshold and not step_detected and (current_time - last_step_time) > step_cooldown:
            steps += 1
            step_detected = True
            last_step_time = current_time
        elif acc < threshold - 1.0:  # Add some hysteresis
            step_detected = False
        
        current_time += 0.1  # Assuming 10Hz sampling rate
    
    return steps

# Parameters
window_size = 20  # Increased window size
acc_window = []
steps = 0
start_time = time.time()
last_step_time = 0
step_cooldown = 0.5  # Minimum time between steps (in seconds)

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

        # Add to window
        acc_window.append(acc_magnitude)
        if len(acc_window) > window_size:
            acc_window.pop(0)
        
        # Process data when window is full
        if len(acc_window) == window_size:
            filtered_acc = low_pass_filter(acc_window)
            current_time = time.time()
            if (current_time - last_step_time) > step_cooldown:
                new_steps = detect_steps(filtered_acc, threshold=11.0, step_cooldown=step_cooldown)
                if new_steps > 0:
                    steps += new_steps
                    last_step_time = current_time
                    print(f"Steps: {steps}")
        
        # Print calibrated sensor data and acceleration magnitude (optional)
        # print("Calibrated Accelerometer:", calibrated_accel)
        # print("Acceleration Magnitude:", acc_magnitude)
        
        time.sleep(0.1)  # 10Hz sampling rate

except KeyboardInterrupt:
    end_time = time.time()
    duration = end_time - start_time
    print(f"\nFinal step count: {steps}")
    print(f"Duration: {duration:.2f} seconds")
    print(f"Average steps per minute: {steps / (duration / 60):.2f}")