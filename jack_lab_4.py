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

print("Start walking. Press Ctrl+C to stop.")

try:
    while True:
        # Read sensor data
        try:
            accelerometer_data = sensor.acceleration
            gyroscope_data = sensor.gyro
        except AttributeError:
            print("Error: Unable to read sensor data. Make sure the sensor is connected properly.")
            break

        # Print raw sensor data for debugging
        print("\nRaw Sensor Data:")
        print("Accelerometer:", accelerometer_data)
        print("Gyroscope:", gyroscope_data)

        # Check if accelerometer_data is in the expected format
        if isinstance(accelerometer_data, tuple) and len(accelerometer_data) == 3:
            # Calculate acceleration magnitude
            acc_magnitude = math.sqrt(
                accelerometer_data[0]**2 + 
                accelerometer_data[1]**2 + 
                accelerometer_data[2]**2
            )
        else:
            print("Error: Unexpected accelerometer data format")
            break

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
        
        time.sleep(0.1)  # Adjust sampling rate if needed

except KeyboardInterrupt:
    end_time = time.time()
    duration = end_time - start_time
    print(f"\nFinal step count: {steps}")
    print(f"Duration: {duration:.2f} seconds")
    print(f"Average steps per minute: {steps / (duration / 60):.2f}")