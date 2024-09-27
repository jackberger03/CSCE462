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

def detect_steps(acc_magnitude, threshold=10.5):
    steps = 0
    step_detected = False
    for acc in acc_magnitude:
        if acc > threshold and not step_detected:
            steps += 1
            step_detected = True
        elif acc < threshold:
            step_detected = False
    return steps

# Parameters
window_size = 10
acc_window = []
steps = 0
start_time = time.time()

# Calibration
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
        # Read sensor data
        try:
            accelerometer_data = sensor.acceleration
            gyroscope_data = sensor.gyro
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
            new_steps = detect_steps(filtered_acc)
            if new_steps > 0:
                steps += new_steps
                print(f"Steps: {steps}")
        
        # Print raw and calibrated sensor data (optional, comment out if not needed)
        print("\nRaw Sensor Data:")
        print("Accelerometer:", accelerometer_data)
        print("Gyroscope:", gyroscope_data)
        print("Calibrated Accelerometer:", calibrated_accel)
        print("Acceleration Magnitude:", acc_magnitude)
        
        time.sleep(0.1)  # Adjust sampling rate if needed

except KeyboardInterrupt:
    end_time = time.time()
    duration = end_time - start_time
    print(f"\nFinal step count: {steps}")
    print(f"Duration: {duration:.2f} seconds")
    print(f"Average steps per minute: {steps / (duration / 60):.2f}")