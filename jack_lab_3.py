import time
import math
import busio
import digitalio
import board
import numpy as np
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

# Data collection function
def collect_samples():
    samples = []
    for _ in range(samples_needed):
        samples.append(chan.voltage)
        time.sleep(sampling_interval)
    return samples

# Exponential moving average filter
def ema_filter(data, alpha):
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

# Amplitude calculation
def calculate_amplitude(samples):
    max_value = max(samples)
    amplitude = max_value  # Since the negative peaks are clipped to zero
    return amplitude

# Square wave detection function
def is_square_wave(samples):
    # Since negative values are clipped, square waves will have long periods at max value followed by zeros
    tol = max(samples) * 0.1
    high_counts = sum(1 for s in samples if s > max(samples) - tol)
    low_counts = sum(1 for s in samples if s < tol)
    high_ratio = high_counts / len(samples)
    low_ratio = low_counts / len(samples)
    # Square waves will have a significant portion of samples at high and low levels
    if high_ratio > 0.4 and low_ratio > 0.4:
        return True
    return False

# Triangle wave detection function
def is_triangle_wave(samples):
    # Triangle waves will have a linear increase in the positive portion
    # Calculate the first derivative
    deriv = np.diff(samples)
    # Since negative values are clipped, we only consider positive derivatives
    positive_deriv = deriv[deriv > 0]
    if len(positive_deriv) == 0:
        return False
    # Calculate variance of the positive derivatives
    variance = np.var(positive_deriv)
    # Triangle waves will have lower variance in the positive derivatives compared to sine waves
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

# Main script
try:
    while True:
        # Collect raw samples
        raw_samples = collect_samples()

        # Apply filtering
        filtered_samples = ema_filter(raw_samples, alpha=0.9)

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
    print("Script terminated by user.")
