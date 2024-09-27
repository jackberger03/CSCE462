import time
import numpy as np
from mpu6050 import mpu6050
from scipy.signal import butter, filtfilt, find_peaks
from filterpy.kalman import KalmanFilter
from sklearn.decomposition import PCA
import math
import matplotlib.pyplot as plt

# ============================ #
#       Configuration          #
# ============================ #

# IMU configuration
IMU_ADDRESS = 0x68  # MPU6050 default I2C address

# Sampling parameters
SAMPLING_RATE = 50  # Hz
DT = 1.0 / SAMPLING_RATE

# Step detection parameters
INITIAL_THRESHOLD = 1.0  # Initial acceleration magnitude threshold
MIN_TIME_BETWEEN_STEPS = 0.3  # Minimum time between steps in seconds

# Butterworth filter parameters
LOWPASS_CUTOFF = 5  # Hz
FILTER_ORDER = 4

# Dynamic threshold parameters
MOVING_AVG_WINDOW = SAMPLING_RATE * 2  # 2-second window
STD_MULTIPLIER = 0.5  # Multiplier for standard deviation

# ============================ #
#       Helper Functions       #
# ============================ #

def butter_lowpass(cutoff, fs, order=4):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def apply_lowpass_filter(data, cutoff=LOWPASS_CUTOFF, fs=SAMPLING_RATE, order=FILTER_ORDER):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = filtfilt(b, a, data)
    return y

def compute_pca_magnitude(ax, ay, az):
    pca = PCA(n_components=1)
    data = np.array([[ax, ay, az]])
    pca.fit(data)
    return pca.transform(data)[0][0]

def initialize_kalman():
    kf = KalmanFilter(dim_x=2, dim_z=1)
    kf.F = np.array([[1, DT],
                     [0, 1]])  # State transition matrix
    kf.H = np.array([[1, 0]])  # Measurement function
    kf.P *= 1000.  # Initial uncertainty
    kf.R = 5  # Measurement noise
    kf.Q = np.array([[1, 0],
                     [0, 3]])  # Process noise
    return kf

def calibrate_sensor(sensor, num_samples=100):
    ax_total = 0
    ay_total = 0
    az_total = 0
    gx_total = 0
    gy_total = 0
    gz_total = 0
    print("Calibrating sensor. Please keep it stationary.")
    for _ in range(num_samples):
        accel = sensor.get_accel_data()
        gyro = sensor.get_gyro_data()
        ax_total += accel['x']
        ay_total += accel['y']
        az_total += accel['z']
        gx_total += gyro['x']
        gy_total += gyro['y']
        gz_total += gyro['z']
        time.sleep(DT)
    ax_offset = ax_total / num_samples
    ay_offset = ay_total / num_samples
    az_offset = (az_total / num_samples) - 9.81  # Assuming z-axis points upwards
    gx_offset = gx_total / num_samples
    gy_offset = gy_total / num_samples
    gz_offset = gz_total / num_samples
    print(f"Calibration offsets:\n"
          f"Accelerometer - ax: {ax_offset:.3f}, ay: {ay_offset:.3f}, az: {az_offset:.3f}\n"
          f"Gyroscope     - gx: {gx_offset:.3f}, gy: {gy_offset:.3f}, gz: {gz_offset:.3f}")
    return ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset

# ============================ #
#        Main Script           #
# ============================ #

def main():
    # Initialize the IMU
    sensor = mpu6050(IMU_ADDRESS)
    
    # Calibrate sensor
    ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset = calibrate_sensor(sensor)
    
    # Initialize Kalman Filters for each axis
    kf_x = initialize_kalman()
    kf_y = initialize_kalman()
    kf_z = initialize_kalman()
    
    # Initialize variables
    accel_data = []
    timestamps = []
    step_count = 0
    last_step_time = time.time()
    dynamic_threshold = INITIAL_THRESHOLD
    log_time = []
    log_a_mag = []
    log_filtered = []
    log_steps = []
    
    # For visualization (optional)
    plt.ion()
    fig, ax_plot = plt.subplots()
    line1, = ax_plot.plot([], [], label='Raw Acceleration Magnitude')
    line2, = ax_plot.plot([], [], label='Filtered Acceleration')
    peak_scatter = ax_plot.scatter([], [], color='red', label='Detected Steps')
    ax_plot.set_xlabel('Time (s)')
    ax_plot.set_ylabel('Acceleration (m/s²)')
    ax_plot.legend()
    
    try:
        start_time_total = time.time()
        while step_count < 50:
            current_time = time.time()
            
            # Read raw accelerometer and gyroscope data
            accel = sensor.get_accel_data()
            gyro = sensor.get_gyro_data()
            ax = accel['x'] - ax_offset
            ay = accel['y'] - ay_offset
            az = accel['z'] - az_offset
            gx = gyro['x'] - gx_offset
            gy = gyro['y'] - gy_offset
            gz = gyro['z'] - gz_offset
            
            # Apply Kalman Filter to accelerometer data
            kf_x.predict()
            kf_x.update(ax)
            filtered_ax = kf_x.x[0]
            
            kf_y.predict()
            kf_y.update(ay)
            filtered_ay = kf_y.x[0]
            
            kf_z.predict()
            kf_z.update(az)
            filtered_az = kf_z.x[0]
            
            # Multi-Axis Analysis using PCA
            a_mag = compute_pca_magnitude(filtered_ax, filtered_ay, filtered_az)
            
            # Append data for filtering and thresholding
            accel_data.append(a_mag)
            timestamps.append(current_time)
            
            # Maintain a window of recent data for filtering
            window_size = SAMPLING_RATE * 5  # 5-second window
            if len(accel_data) > window_size:
                accel_data.pop(0)
                timestamps.pop(0)
            
            # Apply low-pass filter to smooth the signal
            if len(accel_data) >= SAMPLING_RATE * 2:  # Ensure enough data
                filtered = apply_lowpass_filter(np.array(accel_data))
                
                # Dynamic threshold adjustment
                recent_data = filtered[-int(MOVING_AVG_WINDOW):]
                moving_avg = np.mean(recent_data)
                moving_std = np.std(recent_data)
                dynamic_threshold = moving_avg + (STD_MULTIPLIER * moving_std)
                
                # Peak detection using scipy's find_peaks
                peaks, properties = find_peaks(filtered, height=dynamic_threshold, distance=SAMPLING_RATE*MIN_TIME_BETWEEN_STEPS)
                
                # Count new steps
                for peak in peaks:
                    peak_time = timestamps[peak]
                    if (peak_time - last_step_time) > MIN_TIME_BETWEEN_STEPS:
                        step_count += 1
                        last_step_time = peak_time
                        print(f"Step {step_count} detected at {time.strftime('%H:%M:%S', time.localtime(peak_time))}")
                        log_steps.append(peak_time - start_time_total)
                        if step_count >= 50:
                            break
                
                # Logging for visualization
                log_time.append(current_time - start_time_total)
                log_a_mag.append(a_mag)
                log_filtered.append(filtered[-1])
                
                # Update plot
                line1.set_xdata(log_time)
                line1.set_ydata(log_a_mag)
                line2.set_xdata(log_time)
                line2.set_ydata(log_filtered)
                peak_scatter.set_offsets(np.column_stack((log_steps, [filtered[int(step / DT)] for step in log_steps])))
                ax_plot.relim()
                ax_plot.autoscale_view()
                plt.pause(0.001)
            
            time.sleep(DT)
        
        print(f"Total steps counted: {step_count}")
        plt.ioff()
        plt.show()
    
    except KeyboardInterrupt:
        print("Step counting interrupted.")
        plt.ioff()
        plt.show()

if __name__ == "__main__":
    main()
