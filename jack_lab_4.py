import board
import time
import busio
import adafruit_mpu6050

# Create I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Create an instance of the MPU6050 sensor
sensor = adafruit_mpu6050.MPU6050(i2c)

def detect_step(acc_z, threshold, last_step_time, step_cooldown):
    current_time = time.time()
    if acc_z > threshold and (current_time - last_step_time) > step_cooldown:
        return True, current_time
    return False, last_step_time

# Parameters
steps = 0
start_time = time.time()
last_step_time = 0
step_cooldown = 1  # Adjust this value to change sensitivity
threshold = 12  # Manually set threshold, adjust as needed

print("Calibrating sensor. Please keep the sensor still...")
calibration_samples = 100
calibration_sum = 0
for _ in range(calibration_samples):
    accel = sensor.acceleration
    calibration_sum += accel[2]
    time.sleep(0.01)

calibration_offset = (calibration_sum / calibration_samples) - 9.81  # Subtract gravity
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

        # We're only using the z-axis data
        acc_z = accelerometer_data[2] - calibration_offset

        step_detected, last_step_time = detect_step(acc_z, threshold, last_step_time, step_cooldown)
        
        if step_detected:
            steps += 1
            print(f"Steps: {steps}")
        
        # Debug print
        print(f"Acc Z: {acc_z:.2f}, Threshold: {threshold:.2f}")
        
        time.sleep(0.1)  # 10Hz sampling rate

except KeyboardInterrupt:
    end_time = time.time()
    duration = end_time - start_time
    print(f"\nFinal step count: {steps}")
    print(f"Duration: {duration:.2f} seconds")
    print(f"Average steps per minute: {steps / (duration / 60):.2f}")