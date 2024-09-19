import os
import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import math

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
    tol = max(samples) * 0.1
    ref = max(samples)
    cnt = 1
    for s in samples[1:]:
        if tolCheck(s,ref,tol):
            cnt += 1
        if cnt > len(samples) * 0.47:
            return True
    return False
#Period of a square function
def square_period(samples):
    ref = samples[0]
    cnt = 1
    maxCnt = 1
    for s in samples[1:]:
        if (tolCheck(ref,s,0.1)):
            cnt += 1
            maxCnt = max(maxCnt,cnt)
        else:
            cnt = 1
            ref = s
    return maxCnt * ti * 2 * (10/7) # 10/7 is calibration value
#checks data set for triangle wave
def triangle(samples, per):
    # Calculate the first derivative (difference between consecutive samples)
    deriv = [samples[i] - samples[i - 1] for i in range(1, len(samples))]

    # Initialize counters
    maxCnt = cnt = 1
    delta = deriv[0]
    sign = math.copysign(1, delta)

    for i in range(1, len(deriv)):
        d = deriv[i]
        curr_sign = math.copysign(1, d)
        # Check if the derivative is approximately constant and has the same sign
        if tolCheck(d, delta, vt * 2) and curr_sign == sign:
            cnt += 1
            maxCnt = max(maxCnt, cnt)
        else:
            cnt = 1
            delta = d
            sign = curr_sign

    # Require the derivative to be constant for at least 70% of the period
    if maxCnt * ti < per * 0.7:
        return False
    return True

#Analyze
def shape(samples, per):
    # first derivative
    deriv = []
    for i in range(1,len(samples)):
        deriv.append(samples[i]-samples[i-1])
    # second derivative
    deriv2 = []
    for i in range(1,len(deriv)):
        deriv2.append(deriv[i]-deriv[i-1])
   # if second derivative is 0: triangle
    cntT = 0
    for d in deriv2:
        if abs(d) < vt:
            cntT += 1
    if (cntT * ti / window > per * 0.7):
        return "Triangle"
   # if second derivative alternates often: sin
    sign = math.copysign(1,deriv2[0])
    cntS = 0
    for d in deriv2:
        curr_sign = math.copysign(1,d)
        if curr_sign != sign:
            cntS += 1
            sign = curr_sign
    if (cntS*ti > per*0.4):
        return "Sine"        
    
    return "Unknown Shape"
    
#Finds time in seconds between peak values
def period(peak,trough):
    # Get several samples of period Lengths
    periods = []
    for i in range(1):
        #Wait til peak voltage hit
        while (not tolCheck(chan.voltage, peak, vt)):
            time.sleep(ti)
        start = time.monotonic()
        #wait til minimum voltage hit
        while (not tolCheck(chan.voltage, trough, vt*2)):
            time.sleep(ti)
        #Wait for peak voltage again
        while (not tolCheck(chan.voltage, peak, vt*2)):
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
        if (tolCheck(ascending[i],ascending[i-1],0.2)):
            cnt += 1
            if (cnt > maxCnt):
                ret = ascending[i]
                maxCnt = cnt
        else:
            cnt = 1
    return ret

samples = collection()
amp = (max(samples) + min(samples)) / 2

if (square(samples)):
    print("Square")
    per = square_period(samples)
    print("Period:",str(per))
    print("Frequency:",str(1/per))
else:
    per = period(max(samples),min(samples))
    if (triangle(samples,per)):
        print("Triangle")
    else:
        print("Sine")
    print("Shape Function:" , shape(samples,per))
    print("Frequency:",str(1/per))
    