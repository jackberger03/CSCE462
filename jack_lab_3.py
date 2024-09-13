import time
import busio
import digitalio
import board
import numpy as np
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
cs = digitalio.DigitalInOut(board.D22)
mcp = MCP.MCP3008(spi, cs)

wave_channel = AnalogIn(mcp, MCP.P2)

WINDOW = 2
TI = 0.001
S_SIZE = int(WINDOW / TI)
VT = 0.06

def tol_check(x, y, t):
    """Check if two voltage readings are within a specified tolerance."""
    return abs(x - y) < t

def collect_samples(channel):
    """Collect samples from the specified ADC channel."""
    samples = []
    start_time = time.time()
    while len(samples) < S_SIZE:
        samples.append(channel.voltage)
        time.sleep(TI)
    return samples

def detect_waveform(samples):
    """Detect the type of waveform from the sampled data."""
    samples = np.array(samples) - np.mean(samples)
    
    fft = np.fft.fft(samples)
    freqs = np.fft.fftfreq(len(samples), TI)
    magnitudes = np.abs(fft)
    
    peak_freq = abs(freqs[np.argmax(magnitudes[1:]) + 1])
    
    threshold = 0.1 * np.max(magnitudes)
    harmonics = np.where(magnitudes > threshold)[0]
    
    if len(harmonics) == 1:
        return "Sine", peak_freq
    elif all((harmonic % int(peak_freq) == 0) for harmonic in harmonics):
        if len(harmonics) > 3:
            return "Square", peak_freq
        else:
            return "Triangle", peak_freq
    else:
        return "Unknown", peak_freq

def oscilloscope():
    """Continuous oscilloscope functionality."""
    print("Starting Oscilloscope. Press Ctrl+C to exit.")
    previous_wave = None
    while True:
        samples = collect_samples(wave_channel)
        waveform, frequency = detect_waveform(samples)
        
        if waveform != previous_wave:
            print(f"Detected Waveform: {waveform}, Frequency: {frequency:.2f} Hz")
            previous_wave = waveform
        
        time.sleep(0.1)

if __name__ == "__main__":
    try:
        oscilloscope()
    except KeyboardInterrupt:
        print("\nExiting...")
