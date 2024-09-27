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
def ema_filter(data, alpha):
    filtered_data = [data[0]]
    for i in range(1, len(data)):
        filtered_value = alpha * data[i] + (1 - alpha) * filtered_data[i - 1]
        filtered_data.append(filtered_value)
    return filtered_data

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

# RMS calculation
def calculate_rms(samples):
    return np.sqrt(np.mean(np.array(samples)**2))

# Waveform detection using RMS
def detect_waveform(samples):
    peak_to_peak = max(samples) - min(samples)
    rms = calculate_rms(samples)
    
    # Calculate crest factor (peak-to-RMS ratio)
    crest_factor = peak_to_peak / (2 * rms)
    
    # Calculate form factor (RMS to mean ratio)
    mean = np.mean(np.abs(samples))
    form_factor = rms / mean
    
    # Classify based on crest factor and form factor
    if 0.7 < crest_factor < 0.75 and 1.39 < form_factor < 1.42:
        return "Square Wave"
    elif 0.95 < crest_factor < 1.05 and 1.30 < form_factor < 1.35:
        return "Triangle Wave"
    elif 0.85 < crest_factor < 0.92 and 1.35 < form_factor < 1.40:
        return "Sine Wave"
    else:
        return "Unknown Waveform"

# Main execution
try:
    while True:
        # Collect raw samples
        raw_samples = collect_samples()

        # Apply filtering
        filtered_samples = ema_filter(raw_samples, alpha=0.9)

        # Calculate frequency and amplitude
        frequency = calculate_frequency(filtered_samples)
        amplitude = calculate_amplitude(filtered_samples)

        # Classify waveform using RMS method
        waveform_type = detect_waveform(filtered_samples)

        # Output results
        print(f"Frequency: {frequency:.2f} Hz")
        print(f"Amplitude (Peak): {amplitude:.2f} V")
        print(f"Waveform Type: {waveform_type}")
        print()

        # Pause before next measurement
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Script terminated by user.")
