import time
import math
import busio
import digitalio
import board
import numpy as np
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
sampling_rate = 2000  # Increase sampling rate for higher frequencies
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

# Frequency calculation using zero-crossing method with correction factor
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
    # Apply correction factor to improve accuracy
    return frequency * 1.333

# Amplitude calculation
def calculate_amplitude(samples):
    max_value = max(samples)
    min_value = min(samples)
    peak_to_peak = max_value - min_value
    amplitude = peak_to_peak / 2  # Peak amplitude
    return amplitude

# Square wave detection function
def is_square_wave(samples):
    tol = max(samples) * 0.1
    ref = max(samples)
    cnt = 0
    for s in samples:
        if tol_check(s, ref, tol):
            cnt += 1
    if cnt > len(samples) * 0.47:
        return True
    return False

# Triangle wave detection function
def is_triangle_wave(samples, period):
    # Calculate the first derivative
    deriv = [samples[i] - samples[i - 1] for i in range(1, len(samples))]

    # Initialize counters
    max_cnt = cnt = 1
    delta = deriv[0]
    sign = math.copysign(1, delta)

    for i in range(1, len(deriv)):
        d = deriv[i]
        curr_sign = math.copysign(1, d)
        # Check if the derivative is approximately constant and has the same sign
        if tol_check(d, delta, vt * 2) and curr_sign == sign:
            cnt += 1
            max_cnt = max(max_cnt, cnt)
        else:
            cnt = 1
            delta = d
            sign = curr_sign

    # Require the derivative to be constant for at least 40% of the period
    if max_cnt * sampling_interval < period * 0.4:
        return False
    return True

# Main script
try:
    while True:
        # Collect raw samples
        raw_samples = collect_samples()

        # Apply noise reduction
        filtered_samples = moving_average_filter(raw_samples)

        # Calculate frequency and amplitude
        frequency = calculate_frequency(filtered_samples)
        amplitude = calculate_amplitude(filtered_samples)

        # Estimate period
        period = 1 / frequency

        # Check if waveform is a square wave
        if is_square_wave(filtered_samples):
            waveform_type = "Square Wave"
        else:
            # Check if waveform is a triangle wave
            if is_triangle_wave(filtered_samples, period):
                waveform_type = "Triangle Wave"
            else:
                waveform_type = "Sine Wave"

        # Output results
        print(f"Frequency: {frequency:.2f} Hz")
        print(f"Amplitude (Peak): {amplitude:.2f} V")
        print(f"Waveform Type: {waveform_type}\n")

        # Pause before next measurement
        time.sleep(1)

except KeyboardInterrupt:
    print("Script terminated by user.")
