import time
import busio
import digitalio
import board
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
    if abs(x) < t or abs(y) < t:
        return abs(x - y) < t
    return abs(x - y) / abs(x) < t

def collect_samples():
    """Collect samples from the ADC channel."""
    samples = []
    for _ in range(S_SIZE):
        samples.append(wave_channel.voltage)
        time.sleep(TI)
    return samples

def detect_square(samples):
    """Detect if the waveform is square."""
    first = samples[0]
    cnt = 1
    for s in samples[1:]:
        if tol_check(s, first, VT):
            cnt += 1
        if cnt > len(samples) * 0.4:
            return True
    return False

def detect_triangle(samples):
    """Detect if the waveform is triangle."""
    derivatives = [samples[i+1] - samples[i] for i in range(len(samples)-1)]
    second_derivatives = [derivatives[i+1] - derivatives[i] for i in range(len(derivatives)-1)]
    
    max_second_derivative = max(abs(d) for d in second_derivatives)
    mean_abs_second_derivative = sum(abs(d) for d in second_derivatives) / len(second_derivatives)
    
    # Triangle waves have a more consistent second derivative than sine waves
    return max_second_derivative / mean_abs_second_derivative < 5

def calculate_period(samples):
    """Calculate the period of the waveform using peak-to-peak method."""
    peak = max(samples)
    trough = min(samples)
    threshold = (peak + trough) / 2

    crossings = [i for i in range(1, len(samples)) if (samples[i-1] - threshold) * (samples[i] - threshold) <= 0]

    if len(crossings) < 2:
        return 0  # Unable to determine period

    periods = [crossings[i] - crossings[i-1] for i in range(1, len(crossings))]
    avg_period = sum(periods) / len(periods)
    
    return avg_period * TI * 2  # Multiply by 2 for full period and by TI for time

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
            print(f"Detected Waveform: {waveform}, Frequency: {frequency:.2f} Hz")
            previous_wave = waveform
        
        time.sleep(0.1)

if __name__ == "__main__":
    try:
        oscilloscope()
    except KeyboardInterrupt:
        print("\nExiting...")
