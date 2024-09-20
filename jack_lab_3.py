import time
import math
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import numpy as np

# Create the SPI bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# Create the CS (chip select)
cs = digitalio.DigitalInOut(board.D22)

# Create the MCP object
mcp = MCP.MCP3008(spi, cs)

# Create an analog input channel on pin 2 (change as needed)
chan = AnalogIn(mcp, MCP.P2)

# Sampling parameters
sampling_rate = 2000  # Samples per second
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

# Exponential moving average filter
def ema_filter(data, alpha):
    filtered_data = [data[0]]  # Initialize with the first data point
    for i in range(1, len(data)):
        filtered_value = alpha * data[i] + (1 - alpha) * filtered_data[i - 1]
        filtered_data.append(filtered_value)
    return filtered_data

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
    return frequency * 1.333*0.8

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

def is_triangle_wave(samples):
    # Calculate the first derivative
    deriv = np.diff(samples)

    # Normalize derivative to account for amplitude differences
    deriv_norm = deriv / np.max(np.abs(deriv))

    # Find where the derivative changes sign
    sign_changes = np.where(np.diff(np.sign(deriv_norm)))[0]

    # Calculate the lengths of segments with constant derivative sign
    segment_lengths = np.diff(np.concatenate(([0], sign_changes, [len(deriv_norm)])))

    # Check if segments alternate between positive and negative derivatives
    signs = np.sign(deriv_norm[sign_changes])

    # Ensure we have at least 4 segments (2 positive and 2 negative)
    if len(segment_lengths) < 4:
        return False

    # Check for alternating signs
    alternating = np.all(signs[:-1] * signs[1:] < 0)

    # Check if the segment lengths are approximately equal
    avg_length = np.mean(segment_lengths)
    length_tolerance = avg_length * 0.5  # Adjust as needed
    lengths_similar = np.all(np.abs(segment_lengths - avg_length) < length_tolerance)

    if alternating and lengths_similar:
        return True
    else:
        return False


def is_triangle_wave_enhanced(samples):
    # Normalize the samples
    samples = np.array(samples)
    samples = (samples - np.mean(samples)) / np.std(samples)
    
    # Calculate the first derivative
    deriv = np.diff(samples)
    
    # Identify zero crossings in the derivative to find peaks and troughs
    zero_crossings = np.where(np.diff(np.sign(deriv)))[0] + 1
    
    # If we don't have enough zero crossings, we can't proceed
    if len(zero_crossings) < 2:
        return False
    
    # Segment indices for rising and falling edges
    segments = []
    start_idx = 0
    for idx in zero_crossings:
        segments.append((start_idx, idx))
        start_idx = idx
    segments.append((start_idx, len(samples) - 1))
    
    # Analyze each segment
    variance_threshold = 0.1  # Adjust as needed
    low_variance_segments = 0
    for seg in segments:
        y = deriv[seg[0]:seg[1]]
        if len(y) < 2:
            continue
        variance = np.var(y)
        if variance < variance_threshold:
            low_variance_segments += 1
    
    # Check if the majority of segments have low variance
    if low_variance_segments >= len(segments) * 0.8:
        return True
    else:
        return False

# Main script
try:
    while True:
        # Collect raw samples
        raw_samples = collect_samples()

        # Apply filtering (adjust alpha for less heavy filtering)
        filtered_samples = ema_filter(raw_samples, alpha=0.9)

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
