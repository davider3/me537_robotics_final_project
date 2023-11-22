# %% Setup

# general imports
import time
import inspect

# if pyfirmata is not installed, install it
try:
    from pyfirmata import Arduino, util, SERVO
except:
    import pip
    pip.main(['install', 'pyfirmata'])

# Fix the deprecated part of the pyfirmata library
if not hasattr(inspect, 'getargspec'):
    inspect.getargspec = inspect.getfullargspec

# %% Functions

def set_servo_pos(servo_pin, angle):
    if servo_pin.mode == SERVO:
        servo_pin.write(angle)
        time.sleep(.02)
    else:
        print('There\'s no servo attached to that pin')

# %% Control Code

controller = Arduino('COM11')

servo = controller.get_pin('d:9:p')

servo.mode = SERVO

for i in range(0, 180):
    set_servo_pos(servo, i)
for i in range(180, 0, -1):
    set_servo_pos(servo, i)

controller.exit()

# %%
