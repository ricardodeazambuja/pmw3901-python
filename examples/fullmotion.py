#!/usr/bin/env python
import time
import argparse
from pmw3901 import PMW3901, BG_CS_FRONT_BCM, BG_CS_BACK_BCM

print("""motion.py - Detect flow/motion in front of the PMW3901 sensor.
Press Ctrl+C to exit!
""")

parser = argparse.ArgumentParser()
parser.add_argument('--rotation', type=int,
                    default=180, choices=[0, 90, 180, 270],
                    help='Rotation of sensor in degrees.')
parser.add_argument('--spi-slot', type=str,
                    default='front', choices=['front', 'back'],
                    help='Breakout Garden SPI slot.')

args = parser.parse_args()

flo = PMW3901(spi_port=0, spi_cs=1, spi_cs_gpio=BG_CS_FRONT_BCM if args.spi_slot == 'front' else BG_CS_BACK_BCM, secret_sauce=3)
flo.set_rotation(args.rotation)

last_sample = time.time()
try:
    while True:
        try:
            print(flo.get_full_motion_data())
        except RuntimeError:
            continue
        dt = time.time()-last_sample

        if dt < flo.SAMPLE_INTERVAL:
            time.sleep(flo.SAMPLE_INTERVAL-dt)
        last_sample = time.time()
finally:
    flo.reset()