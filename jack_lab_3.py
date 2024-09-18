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
    # Calculate the first derivative
    derivatives = [samples[i+1] - samples[i] for i in range(len(samples)-1)]
    
    # Count sign changes in the derivative
    sign_changes = sum(1 for i in range(len(derivatives)-1) if derivatives[i] * derivatives[i+1] < 0)
    
    # Calculate the ratio of sign changes to total samples
    sign_change_ratio = sign_changes / len(samples)
    
    # Triangle waves should have a higher sign change ratio than sine waves
    # but lower than square waves. Adjust these thresholds as needed.
    return 0.01 < sign_change_ratio < 0.1

def calculate_period(samples):
    """Calculate the period of the waveform."""
    peak = max(samples)
    periods = []
    for _ in range(50):
        while not tol_check(wave_channel.voltage, peak, VT):
            time.sleep(TI)
        start = time.monotonic()
        time.sleep(TI)
        while not tol_check(wave_channel.voltage, peak, VT):
            time.sleep(TI)
        elapsed = time.monotonic() - start
        periods.append(elapsed)
    
    periods.sort()
    ret = periods[0]
    cnt = 1
    max_cnt = 1
    for i in range(1, len(periods)):
        if tol_check(periods[i], periods[i-1], 0.1):
            cnt += 1
            if cnt > max_cnt:
                ret = periods[i]
                max_cnt = cnt
        else:
            cnt = 1
    return ret * 10/9.5  # calibration

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
