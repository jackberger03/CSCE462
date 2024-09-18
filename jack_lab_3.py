import time
import busio
import digitalio
import board
import numpy as np  # Import NumPy for efficient numerical operations
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
    if len(samples) < window_size:
        return samples
    return np.convolve(samples, np.ones(window_size)/window_size, mode='valid')

def collect_samples():
    """Collect samples from the ADC channel with precise timing."""
    samples = []
    start_time = time.perf_counter()
    for i in range(S_SIZE):
        samples.append(wave_channel.voltage)
        # Calculate the next expected sample time
        expected = start_time + (i + 1) * TI
        sleep_time = expected - time.perf_counter()
        if sleep_time > 0:
            time.sleep(sleep_time)
    return samples

def detect_square(samples, tolerance=VT):
    """Detect if the waveform is square based on duty cycle and sharp transitions."""
    samples_np = np.array(samples)
    peak = np.max(samples_np)
    trough = np.min(samples_np)
    high_threshold = peak - tolerance
    low_threshold = trough + tolerance
    
    high_time = np.sum(samples_np > high_threshold)
    low_time = np.sum(samples_np < low_threshold)
    
    # Square wave should spend significant time in high and low states
    is_square = high_time > 0.4 * len(samples_np) and low_time > 0.4 * len(samples_np)
    
    # Additionally, check for sharp transitions by analyzing derivative spikes
    derivatives = np.diff(samples_np)
    derivative_threshold = 0.5  # Adjust based on expected signal
    sharp_transitions = np.sum(np.abs(derivatives) > derivative_threshold)
    
    return is_square and sharp_transitions > (len(samples_np) * 0.05)  # At least 5% sharp transitions

def detect_triangle(samples, tolerance=VT):
    """Detect if the waveform is triangle based on slope linearity and symmetry."""
    smoothed = moving_average(samples)
    derivatives = np.diff(smoothed)
    
    # Calculate the difference between consecutive derivatives
    second_derivatives = np.diff(derivatives)
    
    # Triangle waves have second derivatives close to zero except at peaks and troughs
    std_second_derivative = np.std(second_derivatives)
    
    # Check for symmetry by comparing the first half to the second half
    half = len(smoothed) // 2
    first_half = smoothed[:half]
    second_half = smoothed[-half:]
    correlation = np.corrcoef(first_half, second_half[::-1])[0, 1]  # Reverse second half for symmetry
    
    is_triangle = std_second_derivative < (tolerance * 5) and correlation > 0.8  # Adjust thresholds as needed
    
    return is_triangle

def detect_sine(samples, tolerance=VT):
    """Detect if the waveform is sine based on smoothness."""
    smoothed = moving_average(samples)
    derivatives = np.diff(smoothed)
    
    # Calculate the standard deviation of derivatives
    std_derivatives = np.std(derivatives)
    
    # Check for smoothness: sine waves have smooth, continuous derivatives
    is_smooth = std_derivatives < (tolerance * 2)  # Adjust threshold as needed
    
    return is_smooth

def calculate_period(samples):
    """Calculate the period of the waveform using peak detection with minimum peak distance."""
    smoothed = moving_average(samples)
    peak = np.max(smoothed)
    trough = np.min(smoothed)
    threshold = (peak + trough) / 2

    peaks = []
    minimum_peak_distance = int(0.001 / TI)  # Example: 1 ms minimum distance

    last_peak = -minimum_peak_distance
    for i in range(1, len(smoothed)-1):
        if (smoothed[i] > smoothed[i-1] and
            smoothed[i] > smoothed[i+1] and
            smoothed[i] > threshold):
            if i - last_peak >= minimum_peak_distance:
                peaks.append(i)
                last_peak = i

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
    elif detect_sine(samples):
        waveform = "Sine"
    else:
        waveform = "Unknown"
    
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