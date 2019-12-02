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

parser.add_argument('--height', type=float,
                    default=1,
                    help='Distance to the surface, in mm.')

args = parser.parse_args()

flo = PMW3901(spi_port=0, spi_cs=1, spi_cs_gpio=BG_CS_FRONT_BCM if args.spi_slot == 'front' else BG_CS_BACK_BCM, secret_sauce=3)
flo.set_rotation(args.rotation)

# https://forum.bitcraze.io/viewtopic.php?t=2876
# http://lup.lub.lu.se/luur/download?func=downloadFile&recordOId=8905295&fileOId=8905299
NofPixels = 30.0
Apperture = (4.2*3.14/180.0) # in degrees
app_npix = Apperture/NofPixels

LP_CONSTANT = 0.8

h = args.height # distance to the surface, in mm

last_sample = time.time()
x = 0
y = 0
prev_dx = 0
prev_dy = 0
try:
    while True:
        dx, dy, q = flo.get_motion_with_quality()

        dt = time.time()-last_sample

        dx = LP_CONSTANT * prev_dx + (1.0 - LP_CONSTANT) * dx
        prev_dx = dx

        dy = LP_CONSTANT * prev_dy + (1.0 - LP_CONSTANT) * dy
        prev_dy = dy

        vx = h*dx*app_npix/dt
        vy = h*dy*app_npix/dt
        x += dt*vx
        y += dt*vy
        print("Sensor: dx {:03f} dy {:03f} | Velocity (mm/s): vx {:03f} vy {:03f} | Distance (mm): x {:03f} y {:03f} | Quality: {}".format(dx,dy,vx,vy,x,y,q))

        if dt < flo.SAMPLE_INTERVAL:
            time.sleep(flo.SAMPLE_INTERVAL-dt)

        last_sample = time.time()
finally:
    flo.reset()
