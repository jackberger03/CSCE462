import board
import busio
import adafruit_mpu6050

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
        # Read raw sensor data
        accelerometer_data = sensor.get_accel_data()
        gyroscope_data = sensor.get_gyro_data()
        
        # Calculate acceleration magnitude
        acc_magnitude = math.sqrt(
            accelerometer_data['x']**2 + 
            accelerometer_data['y']**2 + 
            accelerometer_data['z']**2
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
        
        # Print raw sensor data (optional, comment out if not needed)
        print("\nRaw Sensor Data:")
        print("Accelerometer:", accelerometer_data)
        print("Gyroscope:", gyroscope_data)
        
        time.sleep(0.1)  # Adjust sampling rate if needed

except KeyboardInterrupt:
    end_time = time.time()
    duration = end_time - start_time
    print(f"\nFinal step count: {steps}")
    print(f"Duration: {duration:.2f} seconds")
    print(f"Average steps per minute: {steps / (duration / 60):.2f}")