import time
import math
import busio
import digitalio
import board
import numpy as np
import matplotlib.pyplot as plt
import threading
import queue
from collections import deque
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

# Create the SPI bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# Create the CS (chip select)
cs = digitalio.DigitalInOut(board.D22)

# Create the MCP object
mcp = MCP.MCP3008(spi, cs)

# Create an analog input channel on pin 2 (change as needed)
chan = AnalogIn(mcp, MCP.P2)

# Sampling parameters
sampling_rate = 2000  # Increase sampling rate for better resolution
sampling_interval = 1.0 / sampling_rate
measurement_duration = 1.0  # Seconds
samples_needed = int(sampling_rate * measurement_duration)

# Queue to communicate between threads
data_queue = queue.Queue()

def collect_samples():
    samples = []
    for _ in range(samples_needed):
        samples.append(chan.voltage)
        time.sleep(sampling_interval)
    return samples

def ema_filter(data, alpha=0.7):
    filtered_data = [data[0]]  # Initialize with the first data point
    for i in range(1, len(data)):
        filtered_value = alpha * data[i] + (1 - alpha) * filtered_data[i - 1]
        filtered_data.append(filtered_value)
    return filtered_data

# Frequency calculation using peak detection
def calculate_frequency(samples):
    # Find peaks in the waveform
    threshold = max(samples) * 0.7  # Adjust threshold as needed
    peaks = []
    for i in range(1, len(samples) - 1):
        if samples[i] > threshold and samples[i] > samples[i - 1] and samples[i] > samples[i + 1]:
            peaks.append(i)
    # Calculate periods between peaks
    if len(peaks) < 2:
        return 0  # Frequency cannot be determined
    periods = [ (peaks[i+1] - peaks[i]) * sampling_interval for i in range(len(peaks) - 1)]
    avg_period = sum(periods) / len(periods)
    frequency = 1 / avg_period
    return frequency

def calculate_amplitude(samples):
    max_value = max(samples)
    amplitude = max_value  # Since the negative peaks are clipped to zero
    return amplitude

def is_square_wave(samples):
    tol = max(samples) * 0.1
    high_counts = sum(1 for s in samples if s > max(samples) - tol)
    low_counts = sum(1 for s in samples if s < tol)
    high_ratio = high_counts / len(samples)
    low_ratio = low_counts / len(samples)
    if high_ratio > 0.4 and low_ratio > 0.4:
        return True
    return False

def is_triangle_wave(samples):
    deriv = np.diff(samples)
    positive_deriv = deriv[deriv > 0]
    if len(positive_deriv) == 0:
        return False
    variance = np.var(positive_deriv)
    if variance < 0.01:  # Adjust threshold as needed
        return True
    return False

def classify_waveform(samples):
    if is_square_wave(samples):
        return "Square Wave"
    elif is_triangle_wave(samples):
        return "Triangle Wave"
    else:
        return "Sine Wave"

def data_acquisition_thread():
    try:
        while True:
            # Collect raw samples
            raw_samples = collect_samples()

            # Apply filtering
            filtered_samples = ema_filter(raw_samples, alpha=0.7)

            # Put data into queue for plotting
            data_queue.put(filtered_samples)

            # Calculate frequency and amplitude
            frequency = calculate_frequency(filtered_samples)
            amplitude = calculate_amplitude(filtered_samples)

            # Classify waveform
            waveform_type = classify_waveform(filtered_samples)

            # Output results
            print(f"Frequency: {frequency:.2f} Hz")
            print(f"Amplitude (Max Voltage): {amplitude:.2f} V")
            print(f"Waveform Type: {waveform_type}\n")

            # Pause before next measurement
            time.sleep(1)

    except KeyboardInterrupt:
        print("Data acquisition thread terminated by user.")

def plot_thread():
    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots()
    line, = ax.plot([], [])
    ax.set_xlabel('Sample Number')
    ax.set_ylabel('Voltage (V)')
    ax.set_title('Waveform')

    while True:
        try:
            # Get data from queue
            data = data_queue.get(timeout=1)
            # Update plot
            line.set_xdata(np.arange(len(data)))
            line.set_ydata(data)
            ax.relim()
            ax.autoscale_view()
            plt.draw()
            plt.pause(0.01)
        except queue.Empty:
            continue
        except KeyboardInterrupt:
            print("Plot thread terminated by user.")
            break

# Create and start threads
acquisition_thread = threading.Thread(target=data_acquisition_thread)
plotting_thread = threading.Thread(target=plot_thread)

acquisition_thread.start()
plotting_thread.start()

# Wait for threads to finish
acquisition_thread.join()
plotting_thread.join()
