import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

# Initialize SPI and MCP3008
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
cs = digitalio.DigitalInOut(board.D22)
mcp = MCP.MCP3008(spi, cs)

wave_channel = AnalogIn(mcp, MCP.P2)

# Sampling configuration
WINDOW = 1  # seconds
TI = 0.001  # seconds (1 ms sampling interval)
S_SIZE = int(WINDOW / TI)
VT = 0.06  # Voltage tolerance

def tol_check(x, y, t):
    """Check if two voltage readings are within a specified absolute tolerance."""
    return abs(x - y) < t

def moving_average(samples, window_size=5):
    """Apply a simple moving average filter to the samples."""
    return [sum(samples[i:i+window_size])/window_size for i in range(len(samples)-window_size+1)]

def collect_samples():
    """Collect samples from the ADC channel with precise timing."""
    samples = []
    start_time = time.perf_counter()
    for i in range(S_SIZE):
        current_time = time.perf_counter()
        samples.append(wave_channel.voltage)
        # Calculate the next expected sample time
        expected = start_time + (i + 1) * TI
        sleep_time = expected - time.perf_counter()
        if sleep_time > 0:
            time.sleep(sleep_time)
    return samples

def detect_square(samples, tolerance=VT):
    """Detect if the waveform is square based on duty cycle."""
    peak = max(samples)
    trough = min(samples)
    high_threshold = peak - tolerance
    low_threshold = trough + tolerance
    
    high_time = sum(1 for s in samples if s > high_threshold)
    low_time = sum(1 for s in samples if s < low_threshold)
    
    # Square wave should spend significant time in high and low states
    return high_time > 0.4 * len(samples) and low_time > 0.4 * len(samples)

def detect_triangle(samples, tolerance=VT):
    """Detect if the waveform is triangle based on slope linearity."""
    smoothed = moving_average(samples)
    derivatives = [smoothed[i+1] - smoothed[i] for i in range(len(smoothed)-1)]
    mean_derivative = sum(derivatives) / len(derivatives)
    variance = sum((d - mean_derivative) ** 2 for d in derivatives) / len(derivatives)
    # Triangle waves have low variance in derivative changes
    return variance < (tolerance * 10)

def calculate_period(samples):
    """Calculate the period of the waveform using peak detection on a smoothed signal."""
    smoothed = moving_average(samples)
    peak = max(smoothed)
    trough = min(smoothed)
    threshold = (peak + trough) / 2

    peaks = []
    for i in range(1, len(smoothed)-1):
        if smoothed[i] > smoothed[i-1] and smoothed[i] > smoothed[i+1] and smoothed[i] > threshold:
            peaks.append(i)
    
    if len(peaks) < 2:
        return 0  # Unable to determine period

    # Calculate periods between consecutive peaks
    periods = [peaks[i] - peaks[i-1] for i in range(1, len(peaks))]
    avg_period_samples = sum(periods) / len(periods)
    
    return avg_period_samples * TI  # Time per sample

def detect_waveform(samples):
    """Detect the type of waveform from the sampled data."""
    if detect_square(samples):
        waveform = "Square"
    elif detect_triangle(samples):
        waveform = "Triangle"
    else:
        waveform = "Sine"
    
    period = calculate_period(samples)
    frequency = 1 / period if period > 0 else 0
    return waveform, frequency

def oscilloscope():
    """Continuous oscilloscope functionality."""
    print("Starting Oscilloscope. Press Ctrl+C to exit.")
    previous_wave = None
    while True:
        samples = collect_samples()
        waveform, frequency = detect_waveform(samples)
        
        if waveform != previous_wave:
            if frequency > 0:
                print(f"Detected Waveform: {waveform}, Frequency: {frequency:.2f} Hz")
            else:
                print(f"Detected Waveform: {waveform}, Frequency: Unable to determine")
            previous_wave = waveform

if __name__ == "__main__":
    try:
        oscilloscope()
    except KeyboardInterrupt:
        print("\nExiting...")