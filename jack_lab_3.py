import time
import math
import busio
import digitalio
import board
import numpy as np
from scipy import stats
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
sampling_rate = 2000  # Samples per second
sampling_interval = 1.0 / sampling_rate
measurement_duration = 1.0  # Seconds
samples_needed = int(sampling_rate * measurement_duration)

# Tolerance check function
def tol_check(x, y, t):
    if abs(x) < t or abs(y) < t:
        return abs(x - y) < t
    return abs(x - y) / abs(x) < t

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
def ema_filter(data, alpha=0.5):
    filtered_data = [data[0]]
    for i in range(1, len(data)):
        filtered_value = alpha * data[i] + (1 - alpha) * filtered_data[i - 1]
        filtered_data.append(filtered_value)
    return filtered_data

# Square wave detection
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

# Improved frequency calculation using FFT
def calculate_frequency(samples):
    fft_result = np.fft.fft(samples)
    frequencies = np.fft.fftfreq(len(samples), d=sampling_interval)
    
    positive_frequencies = frequencies[:len(frequencies)//2]
    positive_amplitudes = np.abs(fft_result)[:len(frequencies)//2]
    
    # Skip the DC component (index 0)
    peak_index = np.argmax(positive_amplitudes[1:]) + 1
    
    return positive_frequencies[peak_index]

# Amplitude calculation
def calculate_amplitude(samples):
    return (max(samples) - min(samples)) / 2

# New triangle wave detection based on linearity
def is_triangle_wave(samples):
    # Normalize the samples
    normalized = (samples - np.min(samples)) / (np.max(samples) - np.min(samples))
    
    # Find peaks and troughs
    peaks = []
    troughs = []
    for i in range(1, len(normalized) - 1):
        if normalized[i-1] < normalized[i] > normalized[i+1]:
            peaks.append(i)
        elif normalized[i-1] > normalized[i] < normalized[i+1]:
            troughs.append(i)
    
    # Ensure we have at least one complete cycle
    if len(peaks) < 2 or len(troughs) < 2:
        return False
    
    # Calculate linearity between peaks and troughs
    linearity_scores = []
    for start, end in zip(peaks[:-1] + troughs[:-1], peaks[1:] + troughs[1:]):
        x = np.arange(start, end)
        y = normalized[start:end]
        _, _, r_value, _, _ = stats.linregress(x, y)
        linearity_scores.append(abs(r_value))
    
    # Calculate the average linearity score
    avg_linearity = np.mean(linearity_scores)
    
    # Triangle waves should have high linearity (close to 1)
    # Adjust this threshold as needed
    return avg_linearity > 0.99

# Main script
try:
    previous_waveform = None
    while True:
        # Collect raw samples
        raw_samples = collect_samples()

        # Apply filtering
        filtered_samples = ema_filter(raw_samples, alpha=0.7)

        # Calculate frequency and amplitude
        frequency = calculate_frequency(filtered_samples)
        amplitude = calculate_amplitude(filtered_samples)

        # Determine waveform type
        if is_square_wave(filtered_samples):
            waveform_type = "Square Wave"
        elif is_triangle_wave(filtered_samples):
            waveform_type = "Triangle Wave"
        else:
            waveform_type = "Sine Wave"

        # Output results
        print(f"Frequency: {frequency:.2f} Hz")
        print(f"Amplitude (Peak): {amplitude:.2f} V")
        print(f"Waveform Type: {waveform_type}")
        
        print()  # Empty line for readability

        # Pause before next measurement
        time.sleep(0.1)  # Reduced delay for more frequent updates

except KeyboardInterrupt:
    print("Script terminated by user.")