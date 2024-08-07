import time
import board
import busio
import adafruit_mcp4725
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import analogio
import usb_cdc
#import supervisor

serial = usb_cdc.data

# Initialize analog input from your existing setup
analogin = analogio.AnalogIn(board.IO1)

# Initialize I2C bus
i2c = busio.I2C(board.IO6, board.IO5)

# Initialize MCP4725 DAC
dac = adafruit_mcp4725.MCP4725(i2c)
dac.value = 0  # 65535 for 3.3v (100%) / 32768 for 1.65v (50%)

# Initialize ADS1115 ADC
ads = ADS.ADS1115(i2c)

# Create a differential input between channels 0 and 1
diff_channel = AnalogIn(ads, ADS.P0, ADS.P1)

shunt_resistor = 100000

# Possible red values and their initial counts
red_counts = {0: 0, 60: 0, 120: 0, 150: 0, 250: 0}
voltage_map = {1: 0.3, 2: 0.9, 3: 1.5, 4: 2.1, 5: 2.7}  # Rank to voltage

batch_complete = False  # To control when to process data

current_maxima = []

def map_to_red_value(current, min_current, max_current):
    # Map the current value to a red value between 0-255
    return int((current - min_current) / (max_current - min_current) * 255) if max_current != min_current else 0

def read_serial_count_red():
    global batch_complete

    if serial is None:
        print("Serial port is not initialized.")
        return

    while serial.in_waiting > 0:
    #if serial.in_waiting > 0:

        rgbdata = serial.readline().strip().decode()  # Read the incoming data

        if rgbdata == "END":  # You can send this from p5.js to indicate end of batch
            batch_complete = True
            break

        try:
            # Expecting data in 'R,G,B' format
            rgb_values = list(map(int, rgbdata.split(',')))
            if len(rgb_values) == 3:
                red_value = rgb_values[0]
                #serial.write((str(red_value)+"\n").encode())
                if red_value in red_counts:
                    red_counts[red_value] += 1
        except ValueError:
            # In case of an error, just ignore the current data
            pass
    #return None


def getDifferentialVoltage(pin):  # helper function for differential voltage
    return pin.voltage

def output_voltage():
    global current_maxima

    """Output voltage corresponding to the frequency of red values."""
    if any(count > 0 for count in red_counts.values()):  # Check if there are any non-zero counts
        sorted_reds = sorted((red for red in red_counts if red_counts[red] > 0), key=red_counts.get, reverse=True)

        for i, red in enumerate(sorted_reds):
            rank = i + 1
            if rank in voltage_map:
                dac.normalized_value = voltage_map[rank] / 3.3  # Scale to DAC range
                #print(dac.normalized_value * 3.3)
                #serial.write((str(dac.normalized_value * 3.3)+"\n").encode())

                max_current = 0

                # Wait for 2 seconds, during this period send differential voltage readings
                start_time = time.monotonic()
                while time.monotonic() - start_time < 2:
                    #diff_voltage = getDifferentialVoltage(diff_channel)
                    current = getCurrent()
                    max_current = max(max_current, current)

                    #serial.write((str(getDifferentialVoltage(diff_channel))+"\n").encode())
                    time.sleep(0.1)

                    current_maxima.append(max_current)
                    #serial.write((f"Max current after applying {dac.normalized_value * 3.3:.2f}V: {max_current:.10f}A\n").encode())


                #time.sleep(2)  # Hold the voltage for 2 seconds

        # Reset red_counts after processing
        red_counts.update((key, 0) for key in red_counts.keys())

        # Find min and max current values
        if current_maxima:
            min_current = min(current_maxima)
            max_current = max(current_maxima)
            # Map each maximum current to a red value and send it
            for current in current_maxima:
                red_value = map_to_red_value(current, min_current, max_current)
                serial.write((f"{red_value}\n").encode())
            current_maxima = []  # Reset for the next batch
    #else:
        #print("no red value")
        #serial.write("No valid red value counts received, skipping output.\n".encode())



        #if i < len(voltage_map):
            #dac.normalized_value = voltage_map[i+1] / 3.3  # Convert to a range from 0.0 to 1.0
            #serial.write((str(dac.normalized_value * 3.3)+"\n").encode())
            #time.sleep(2)  # Hold the voltage for 2 seconds
            #break  # Update the DAC with the voltage of the most frequent red value
    #print(f"Output voltage set for most frequent red: {dac.normalized_value * 3.3}V")

def getVoltage(pin):  # helper function to get voltage
    return (pin.value * 3.3) / 65535



def getCurrent():
    voltage_across_shunt = getDifferentialVoltage(diff_channel)
    current = voltage_across_shunt / shunt_resistor
    return current

scaling_factor = 1e6

while True:
    read_serial_count_red()
    if batch_complete:
        output_voltage()
        batch_complete = False  # Reset for next batch
    #output_voltage()
    #print("Current: {:.10f} A".format(getCurrent()))
    scaled_current = getCurrent()*scaling_factor
    #serial.write((str(scaled_current)+"\n").encode())
    #print("MCP4725 Analog Voltage: %f" % getVoltage(analogin))
    #print("ADS1115 Differential Voltage between A0 and A1: %f V" % getDifferentialVoltage(diff_channel))
    #print((getDifferentialVoltage(diff_channel)*10,scaled_current))
    #serial.write((str(getDifferentialVoltage(diff_channel))+"\n").encode())
    time.sleep(0.1)# Write your code here :-)
