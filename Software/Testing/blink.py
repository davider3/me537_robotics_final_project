# %% Setup

# general imports
import time
import inspect

# if pyfirmata is not installed, install it
try:
    from pyfirmata import Arduino, util
except:
    import pip
    pip.main(['install', 'pyfirmata'])

# Fix the deprecated part of the pyfirmata library
if not hasattr(inspect, 'getargspec'):
    inspect.getargspec = inspect.getfullargspec

# %%

# check that you are connected to the correct com port before each run
controller = Arduino('COM11')

on = True
for _ in range(20):
    controller.digital[13].write(on)
    on = not on
    time.sleep(.5)

controller.exit()
