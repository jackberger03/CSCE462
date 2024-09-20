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
sampling_rate = 1000  # Samples per second
sampling_interval = 1.0 / sampling_rate
measurement_duration = 1.0  # Seconds
samples_needed = int(sampling_rate * measurement_duration)
vt = 0.06  # Voltage tolerance

# Tolerance check function
def tol_check(x, y, t):
    if abs(x) < t or abs(y) < t:
        return abs(x - y) < t
    return abs(x - y) / abs(x) < t

# Data collection function
def collect_samples():
    samples = []
    for _ in range(samples_needed):
        samples.append(chan.voltage)
        time.sleep(sampling_interval)
    return samples

# Noise reduction using moving average filter
def moving_average_filter(data, window_size=5):
    return np.convolve(data, np.ones(window_size)/window_size, mode='valid')

# Frequency calculation using zero-crossing method
def calculate_frequency(samples):
    # Use the mean voltage as the threshold for zero-crossing
    threshold = sum(samples) / len(samples)
    zero_crossings = 0
    for i in range(1, len(samples)):
        if (samples[i - 1] < threshold and samples[i] >= threshold):
            zero_crossings += 1
    # Calculate frequency
    time_period = len(samples) / sampling_rate
    frequency = (zero_crossings / 2) / time_period
    return frequency*1.333

# Amplitude calculation
def calculate_amplitude(samples):
    max_value = max(samples)
    min_value = min(samples)
    peak_to_peak = max_value - min_value
    amplitude = peak_to_peak / 2  # Peak amplitude
    return amplitude

# Waveform recognition functions
def is_square_wave(samples):
    tol = max(samples) * 0.1
    ref_high = max(samples)
    ref_low = min(samples)
    high_counts = 0
    low_counts = 0
    for s in samples:
        if tol_check(s, ref_high, tol):
            high_counts += 1
        elif tol_check(s, ref_low, tol):
            low_counts += 1
    high_ratio = high_counts / len(samples)
    low_ratio = low_counts / len(samples)
    # Square wave spends roughly equal time at high and low levels
    if abs(high_ratio - 0.5) < 0.2 and abs(low_ratio - 0.5) < 0.2:
        return True
    return False

def is_triangle_wave(samples):
    deriv = np.diff(samples)
    # The derivative should alternate between positive and negative at a constant rate
    sign_changes = np.diff(np.sign(deriv))
    num_sign_changes = np.count_nonzero(sign_changes)
    # Triangle wave has regular sign changes in the derivative
    expected_changes = (len(samples) / (sampling_rate / calculate_frequency(samples))) * 2
    if abs(num_sign_changes - expected_changes) < expected_changes * 0.3:
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

        # Apply noise reduction
        filtered_samples = moving_average_filter(raw_samples)

        # Recalculate samples_needed after filtering
        adjusted_samples_needed = len(filtered_samples)

        # Calculate frequency and amplitude
        frequency = calculate_frequency(filtered_samples)
        amplitude = calculate_amplitude(filtered_samples)
        waveform_type = classify_waveform(filtered_samples)

        # Output results
        print(f"Frequency: {frequency:.2f} Hz")
        print(f"Amplitude (Peak): {amplitude:.2f} V")
        print(f"Waveform Type: {waveform_type}\n")

        # Optional: Pause before next measurement
        time.sleep(1)

except KeyboardInterrupt:
    print("Script terminated by user.")
