import argparse
import serial
import datetime
import os
import scipy.io
import numpy as np
import serial.tools.list_ports
import time

parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument(
    "-d", "--device", help="device to read from", default="COM9")
parser.add_argument("-s", "--speed", help="speed in bps",
                    default=115200, type=int)
args = parser.parse_args()

outputFilePath = os.path.join(os.path.dirname(__file__),
                              datetime.datetime.now().strftime("%Y-%m-%dT%H.%M.%S") + ".mat")

#data = {"Acc": np.array([]), "Gyro": np.array([]), "EKF Pos": np.array([]), "EKF_Speed": np.array([]), "EKF Quat": np.array([]), "EKF YPR": np.array([]), "GPS Pos": np.array([]), "GPS Speed": np.array([]), "Mag": np.array([]), "Bar": np.array([])}
data = {}
started = False

while len(serial.tools.list_ports.comports()) == 0:
    time.sleep(1)


with serial.Serial(args.device, args.speed) as ser, open(outputFilePath, mode='wb') as outputFile:
    print("Logging started. Ctrl-C to stop.")
    try:
        while True:
            while not started:
                d = ser.readline().strip().decode('ascii')
                print(d)
                if d == "Initialization done":
                    started = True
            d = ser.readline().strip().decode('ascii')
            #print(d)  # uncomment this line to debug the coming data
            type_ = d.split(':')[0].strip()
            val = d.split(':')[1].split(',')
            val_array = np.array([float(x) for x in val])
            if type_ not in data.keys():
                # If it doesn't exist, initialize it as an empty array
                data[type_] = val_array.reshape(-1,1)  # Assuming 3 values per entry
                #print(type_, data[type_])
                # Concatenate the new values to the existing array
            data[type_] = np.concatenate((data[type_], val_array.reshape(-1,1)), axis=1)
            
            

    except KeyboardInterrupt:
        print("Logging stopped, saving to file")

print("Data saved to", outputFilePath)
#print(data)
with open(outputFilePath, "wb") as f:
    for type_ in data.keys():
        scipy.io.savemat(f, {'data':data})