# Import libraries
import time
import board
import busio

# Import the APDS9930 library
from apds9930.apds9930 import APDS9930

# Initialize I2C - this config is for a custom board
i2c = busio.I2C(board.GPIO18, board.GPIO8)

# Create instance of APDS9930
sensor = APDS9930(i2c)

# Print ALS at startup
print(f"{sensor.als=}")
time.sleep(2)

# Print the proximity continuously
while True:
    time.sleep(0.1)
    print(f"{sensor.proximity=}")
