import os
import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

# create the spi bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# create the cs (chip select)
cs = digitalio.DigitalInOut(board.D22)

# create the mcp object
mcp = MCP.MCP3008(spi, cs)

# create an analog input channel on pin 1
chan = AnalogIn(mcp, MCP.P2)

#sample time in seconds
window = 2
#time increments
ti = 0.001
#sample size
sSize = round(window / ti)
#voltage tolerance
vt = 0.06
#checks if two voltages are near equal
def tolCheck(x,y, t):
    if (abs(x) < t or abs(y) < t):
        return abs(x-y) < t
    return abs(x-y) / abs(x) < t
#reads data from adc
def collection():
    samples = []
    for i in range(sSize):
        samples.append(chan.voltage)
        time.sleep(ti)
    return samples
#checks data set for square wave
def square(samples):
    first = samples[0]
    cnt = 1
    for s in samples[1:]:
        if tolCheck(s,first,vt):
            cnt += 1
        if cnt > len(samples) * 0.4:
            return True
    return False
#checks data set for triangle wave
def triangle(samples, amp, per):
    delta = samples[1] - samples[0]
    slopes = []
    slopes.append(delta)
    for i in range(1,sSize-1):
        d = samples[i]-samples[i-1]
        if (abs(d) < 0.04): #works up to 50Hz
            continue
        match = False
        for s in slopes:
            if (tolCheck(s,d,0.1)):
                match = True
        if not match:
            slopes.append(d)
    if len(slopes) > 2:
        print("slopes:",str(len(slopes)))
        return False
    return True
#Peak of a square function
def square_period(samples):
    spc = 1/ti #samples per second
    volt = samples[0]
    idx = 0
    while (tolCheck(volt,samples[idx],0.1)):
        idx += 1
    cnt = 0
    volt = samples[idx]
    while (tolCheck(volt,samples[idx],0.1)):
        cnt += 1
        idx += 1
    calibration = 10/7
    return cnt / spc * 2 *calibration
#Finds time in seconds between peak values
def period(peak):
    # Get several samples of period Lengths
    periods = []
    for i in range(50):
        #Wait til peak voltage hit
        while (not tolCheck(chan.voltage, peak, vt)):
            time.sleep(ti)
        start = time.monotonic()
        time.sleep(ti)
        elapsed = 0
        #Wait for peak voltage again
        while (not tolCheck(chan.voltage, peak, vt)):
            time.sleep(ti)
        elapsed = time.monotonic() - start
        periods.append(elapsed)
    #Sort samples
    ascending = sorted(periods)
    #Get period length with most frequency
    ret = ascending[0]
    cnt = 1
    maxCnt = 1
    for i in range(1,len(ascending)):
        if (tolCheck(ascending[i],ascending[i-1],0.1)):
            cnt += 1
            if (cnt > maxCnt):
                ret = ascending[i]
                maxCnt = cnt
        else:
            cnt = 1 
    per = ret * 10/9.5 #calibration
    print("Period:",str(per))
    return per

samples = collection()
amp = max(samples)
print("Amplitude:",str(amp))

if (square(samples)):
    print("Square")
    print("Period:",str(square_period(samples)))
else:
    print("Not a Square")
    per = period(amp)
    if (triangle(samples,amp,per)):
        print("Triangle")
    else:
        print("Not a Triangle")
    