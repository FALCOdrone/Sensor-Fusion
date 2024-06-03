import argparse
import serial
import datetime
import os
import numpy as np
import time

import scipy.io
import serial.tools.list_ports

parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument(
    "-d", "--device", help="device to read from", default="COM6")
parser.add_argument("-s", "--speed", help="speed in bps",
                    default=115200, type=int)
args = parser.parse_args()

outputFilePath = os.path.join(os.path.dirname(__file__),
                              datetime.datetime.now().strftime("%Y-%m-%dT%H.%M.%S") + ".mat")

# data = {"Acc": np.array([]), "Gyro": np.array([]), "EKF Pos": np.array([]), "EKF_Speed": np.array([]), "EKF Quat": np.array([]), "EKF YPR": np.array([]), "GPS Pos": np.array([]), "GPS Speed": np.array([]), "Mag": np.array([]), "Bar": np.array([])}
data = {}
started = False

# Set the logging duration to 4 hours (in seconds)
logging_duration = 120

while len(serial.tools.list_ports.comports()) == 0:
    time.sleep(1)
    

with serial.Serial(args.device, args.speed) as ser, open(outputFilePath, mode='wb') as outputFile:
    print("Logging started. Ctrl-C to stop.")
    try:
        while True :
            while not started:
                d = ser.readline().strip().decode('ascii')
                print(d)
                if d == "Initialization done":
                    started = True
                    # Get the start time
                    start_time = time.time()

            # Check if the logging duration has exceeded the limit
            if time.time() - start_time > logging_duration:
                print("Logging stopped due to time limit.")
                break

            d = ser.readline().strip().decode('ascii')

            # print(d)  # uncomment this line to debug the coming data

            type_ = d.split(':')[0].strip()
            val = d.strip().split('\t')[1:]
            # print(val)
            val_array = np.array([float(v.split(':')[1].strip())
                                 for v in val if ':' in v])
            # print(val_array)

            if type_ not in data.keys():
                # If it doesn't exist, initialize it as an empty array
                # Assuming 3 values per entry
                data[type_] = val_array.reshape(-1, 1)

            # Concatenate the new values to the existing array
            data[type_] = np.concatenate(
                (data[type_], val_array.reshape(-1, 1)), axis=1)
            #print(type_, data[type_])

    except KeyboardInterrupt:
        print("Logging stopped, saving to file")
        print(time.time() - start_time)

#print(data)
with open(outputFilePath, "wb") as f:
    for type_ in data.keys():
        scipy.io.savemat(f, {'data': data})

print("Data saved to", outputFilePath)
