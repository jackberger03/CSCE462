import time
import busio
import digitalio
import board
import numpy as np
import scipy.fft
import matplotlib.pyplot as plt
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
from collections import deque
import threading

# Initialize SPI and MCP3008
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
cs = digitalio.DigitalInOut(board.D22)  # Change as per your connection
mcp = MCP.MCP3008(spi, cs)
wave_channel = AnalogIn(mcp, MCP.P2)

# Sampling configuration
SAMPLE_RATE = 1000  # Hz
WINDOW_SIZE = 1024  # Number of samples per window
DT = 1.0 / SAMPLE_RATE  # Sampling interval

# Buffer to store samples
samples_buffer = deque(maxlen=WINDOW_SIZE)

def collect_sample():
    """Collect a single sample from the ADC."""
    return wave_channel.voltage

def update_buffer():
    """Continuously collect samples and update the buffer."""
    while True:
        sample = collect_sample()
        samples_buffer.append(sample)
        time.sleep(DT)

def compute_fft(samples, sample_rate):
    """Compute FFT and return frequency and amplitude arrays."""
    # Remove DC component
    samples = samples - np.mean(samples)
    # Apply window to reduce spectral leakage
    window = np.hanning(len(samples))
    samples_windowed = samples * window
    # Compute FFT
    fft_result = scipy.fft.fft(samples_windowed)
    fft_freq = scipy.fft.fftfreq(len(samples), d=1/sample_rate)
    # Take only the positive frequencies
    idx = np.where(fft_freq >= 0)
    fft_freq = fft_freq[idx]
    fft_ampl = np.abs(fft_result[idx]) * 2 / np.sum(window)
    return fft_freq, fft_ampl

def detect_frequency(fft_freq, fft_ampl):
    """Detect the dominant frequency in the FFT result."""
    peak_idx = np.argmax(fft_ampl[1:]) + 1  # Exclude the DC component
    dominant_freq = fft_freq[peak_idx]
    return dominant_freq

def classify_waveform(fft_freq, fft_ampl, dominant_freq):
    """Classify waveform based on harmonic amplitude ratios."""
    # Fundamental amplitude
    fundamental_ampl = fft_ampl[np.argmax(fft_ampl[1:]) + 1]  # Exclude DC

    # Expected harmonics (up to 5th)
    harmonics = [dominant_freq * n for n in range(1, 6)]
    tolerance = dominant_freq * 0.05  # 5% tolerance

    harmonic_ampls = []
    for harmonic in harmonics[1:]:  # Exclude fundamental
        idx = np.where((fft_freq >= harmonic - tolerance) & (fft_freq <= harmonic + tolerance))
        if len(idx[0]) > 0:
            harmonic_ampls.append(np.max(fft_ampl[idx]))
        else:
            harmonic_ampls.append(0)

    # Compute amplitude ratios
    harmonic_ratios = [amp / fundamental_ampl for amp in harmonic_ampls]

    # Classification thresholds (empirical and may require tuning)
    square_threshold = 0.2
    triangle_threshold = 0.05

    # Check for Square Wave: Strong harmonics
    if all(r > square_threshold for r in harmonic_ratios):
        return "Square"

    # Check for Triangle Wave: Weaker harmonics
    if all(r > triangle_threshold for r in harmonic_ratios):
        return "Triangle"

    # Otherwise, assume Sine Wave
    return "Sine"

def visualize(samples, fft_freq, fft_ampl, dominant_freq, waveform):
    """Update the plots with current waveform and frequency spectrum."""
    # Time axis
    t = np.linspace(0, len(samples)*DT, num=len(samples))

    # Update waveform plot
    line1.set_data(t, samples)
    ax1.set_xlim(0, t[-1])
    ax1.set_ylim(-3.5, 3.5)  # Adjust based on your signal amplitude

    # Update frequency spectrum plot
    line2.set_data(fft_freq, fft_ampl)
    ax2.set_xlim(0, SAMPLE_RATE / 2)
    ax2.set_ylim(0, np.max(fft_ampl)*1.1)

    # Update titles
    ax1.set_title(f"Waveform: {waveform}, Dominant Frequency: {dominant_freq:.2f} Hz")
    ax2.set_title("Frequency Spectrum")

    plt.pause(0.001)

def oscilloscope():
    """Main oscilloscope loop."""
    print("Starting Oscilloscope. Press Ctrl+C to exit.")
    previous_wave = None
    while True:
        if len(samples_buffer) == WINDOW_SIZE:
            samples = np.array(samples_buffer)
            fft_freq, fft_ampl = compute_fft(samples, SAMPLE_RATE)
            dominant_freq = detect_frequency(fft_freq, fft_ampl)
            waveform = classify_waveform(fft_freq, fft_ampl, dominant_freq)
            
            if waveform != previous_wave:
                if dominant_freq > 0:
                    print(f"Detected Waveform: {waveform}, Frequency: {dominant_freq:.2f} Hz")
                else:
                    print(f"Detected Waveform: {waveform}, Frequency: Unable to determine")
                previous_wave = waveform

            # Clear buffer for next window
            samples_buffer.clear()

def main():
    """Run the oscilloscope."""
    try:
        # Start collecting samples in the background
        sampling_thread = threading.Thread(target=update_buffer, daemon=True)
        sampling_thread.start()

        # Start the oscilloscope
        oscilloscope()

    except KeyboardInterrupt:
        print("\nExiting...")

if __name__ == "__main__":
    main()