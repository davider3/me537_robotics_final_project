# %% Setup

# general imports
import time
import inspect
import numpy as np

# if pyfirmata is not installed, install it
try:
    from pyfirmata import Arduino, util, SERVO
except:
    import pip
    pip.main(['install', 'pyfirmata'])
    from pyfirmata import Arduino, util, SERVO

# Fix the deprecated part of the pyfirmata library
if not hasattr(inspect, 'getargspec'):
    inspect.getargspec = inspect.getfullargspec

# %% Create arduino and servo objects

servo_pins=[10, 9, 11]
controller = Arduino('COM11')
joints = [controller.get_pin(f'd:{pin}:p') for pin in servo_pins]
for joint in joints:
    joint.mode = SERVO

# %% Smoothly move the 3 servos
q_curr = [0, 0, 0]
qs = [90, 90, 90]
steps = 50
q_steps = [np.linspace(q_curr[i], qs[i], steps) for i in range(len(qs))]
for i in range(steps):
    for j in range(len(joints)):
        joints[0].write(q_steps[j][i])
    if i == 0:
        time.sleep(.5)
    time.sleep(.01)


controller.exit()
# %%

