import time
import math
import busio
import digitalio
import board
import numpy as np
from scipy import stats
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
cs = digitalio.DigitalInOut(board.D22)
mcp = MCP.MCP3008(spi, cs)
chan = AnalogIn(mcp, MCP.P2) # Pin 2

# Sampling parameters
sampling_rate = 2000  # Samples per second
sampling_interval = 1.0 / sampling_rate
measurement_duration = 1.0  # Seconds
samples_needed = int(sampling_rate * measurement_duration)

# Data collection function
def collect_samples():
    samples = []
    start_time = time.time()
    while len(samples) < samples_needed:
        samples.append(chan.voltage)
        while time.time() - start_time < len(samples) * sampling_interval:
            pass
    return samples

# Exponential moving average filter
def ema_filter(data, alpha=0.7):
    filtered_data = [data[0]]
    for i in range(1, len(data)):
        filtered_value = alpha * data[i] + (1 - alpha) * filtered_data[i - 1]
        filtered_data.append(filtered_value)
    return filtered_data

# Square wave detection
def is_square_wave(samples):
    tol = max(samples) * 0.1
    ref = max(samples)
    cnt = sum(1 for s in samples if abs(s - ref) < tol)
    if cnt > len(samples) * 0.47:
        return True
    return False

# Frequency calculation using FFT
def calculate_frequency(samples):
    fft_result = np.fft.fft(samples)
    frequencies = np.fft.fftfreq(len(samples), d=sampling_interval)

    positive_frequencies = frequencies[:len(frequencies)//2]
    positive_amplitudes = np.abs(fft_result)[:len(frequencies)//2]

    peak_index = np.argmax(positive_amplitudes[1:]) + 1

    return abs(positive_frequencies[peak_index])

# Amplitude calculation
def calculate_amplitude(samples):
    return (max(samples) - min(samples)) / 2

# Triangle wave detection function based on your approach
def is_triangle_wave(samples, frequency):
    N_period = sampling_rate / frequency
    N_period = int(N_period)
    if N_period == 0:
        return False

    # Number of periods in data
    num_periods = len(samples) // N_period

    if num_periods == 0:
        return False

    # For each period
    for period_idx in range(num_periods):
        period_start = period_idx * N_period
        period_end = period_start + N_period
        period_samples = samples[period_start:period_end]

        # Divide period into 10 segments
        N_segment = N_period // 10
        if N_segment == 0:
            continue

        # For each segment
        for segment_idx in range(10):
            segment_start = segment_idx * N_segment
            segment_end = segment_start + N_segment
            if segment_end > len(period_samples):
                break
            segment_samples = period_samples[segment_start:segment_end]
            # Compute derivative
            derivative = np.diff(segment_samples)
            # Check if derivative is consistently positive or negative
            if len(derivative) == 0:
                continue
            derivative_signs = np.sign(derivative)
            all_positive = np.all(derivative_signs > 0)
            all_negative = np.all(derivative_signs < 0)
            if all_positive or all_negative:
                # Found a segment where derivative is consistently positive or negative
                # Now check if standard deviation of derivative is low
                std_derivative = np.std(derivative)
                if std_derivative < 0.1:  # Adjust based on need
                    return True
    return False

def classify_waveform(samples):
    # First, calculate frequency
    frequency = calculate_frequency(samples)
    if frequency == 0:
        return "Unknown Waveform"
    if is_square_wave(samples):
        return "Square Wave"
    elif is_triangle_wave(samples, frequency):
        return "Triangle Wave"
    else:
        return "Sine Wave"

# Main execution
try:
    while True:
        # Collect raw samples
        raw_samples = collect_samples()

        # Apply filtering
        filtered_samples = ema_filter(raw_samples, alpha=0.7)

        # Calculate frequency and amplitude
        frequency = calculate_frequency(filtered_samples)
        amplitude = calculate_amplitude(filtered_samples)

        # Classify waveform
        waveform_type = classify_waveform(filtered_samples)

        # Output results
        print(f"Frequency: {frequency:.2f} Hz")
        print(f"Amplitude (Peak): {amplitude:.2f} V")
        print(f"Waveform Type: {waveform_type}")
        print()

        # Pause before next measurement
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Script terminated by user.")
