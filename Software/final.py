# %% Setup

# general imports
import time
import inspect
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QLineEdit, QPushButton, QVBoxLayout
import numpy as np

# Custom imports
import kinematics as kin
from visualization import VizScene

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

# %% Robot class

rad_to_deg = 180 / np.pi

class RobotArmGUI(QWidget):
    def __init__(self, com=None, dh=[],
                 joint_limits=None, 
                 servo_pins=[9, 10, 11], 
                 led=13, init_viz=False):
        super().__init__()
        
        # Setup Arduino
        if not com == None:
            self.controller = Arduino(com)

            # Setup LED
            self.led = self.controller.get_pin(f'd:{led}:o')

            # Setup servo motor joints and move them to zero
            self.joints = [self.controller.get_pin(f'd:{pin}:p') for pin in servo_pins]
            for joint in self.joints:
                joint.mode = SERVO
                joint.write(0)
                time.sleep(.5)
            self.blink()

        # Setup Serial arm
        if len(servo_pins) > 0:
            # Create arm object
            self.arm = kin.SerialArm(dh, joint_limits=joint_limits)

            # Set position to zeros
            self.q_curr = [0.0, 0.0, 0.0]

            # Show the initial orientation
            if init_viz:
                viz = VizScene()
                viz.add_arm(self.arm)
                viz.hold()
                viz.close_viz()

        # Create the GUI
        self.init_ui()

    def init_ui(self):
        # Create labels and input fields for x, y, and z coordinates
        self.labels = [QLabel('X Coordinate:'),
                       QLabel('Y Coordinate:'),
                       QLabel('Z Coordinate:'),
                       QLabel('Obstacle X:'),
                       QLabel('Obstacle Y:'),
                       QLabel('Obstacle Z:'),
                       QLabel('Calculated Angles:')]
        self.inputs = [QLineEdit(self),
                       QLineEdit(self),
                       QLineEdit(self),
                       QLineEdit(self),
                       QLineEdit(self),
                       QLineEdit(self)]

        # Create buttons for inverse kinematics and moving the robot arm
        self.inverse_kinematics_button = QPushButton('Inverse Kinematics', self)
        self.move_arm_button = QPushButton('Move Robot Arm', self)

        # Connect buttons to functions
        self.inverse_kinematics_button.clicked.connect(self.ik)
        self.move_arm_button.clicked.connect(self.move)

        # Set up the layout
        input_layout = QVBoxLayout()
        for i in range(len(self.inputs)):
            input_layout.addWidget(self.labels[i])
            input_layout.addWidget(self.inputs[i])

        button_layout = QVBoxLayout()
        button_layout.addWidget(self.inverse_kinematics_button)
        button_layout.addWidget(self.move_arm_button)

        result_layout = QVBoxLayout()
        result_layout.addWidget(self.labels[3])

        main_layout = QVBoxLayout()
        main_layout.addLayout(input_layout)
        main_layout.addLayout(button_layout)
        main_layout.addLayout(result_layout)

        self.setLayout(main_layout)

        # Set window properties
        self.setGeometry(300, 300, 400, 200)
        self.setWindowTitle('Robot Arm Control')

    def ik(self):
        # Get goal location from GUI
        points = [float(input.text()) for input in self.inputs]
        goal = points[:3]
        obst = points[3:]

        gain = .008 * np.eye(3) 
        k_obst = 0.2
        k_perp = 0.01

        # Hardcoded Values
        # obst = [4,.5,4.35]
        radius = 1.0

        
        safety_factor = 1.5

        # self.qs, _, _, _, _ = self.arm.ik_position(goal, plot=True, 
        #                                            method='p_inv', K=gain, 
        #                                            q0=self.q_curr, max_iter=2000)

        self.qs = self.arm.ik_position_w_obstacle(goal,obst,radius, plot=False, kd = 0.000003,
                                                   method='p_inv', K=gain,k_obst=k_obst,
                                                   k_perp=k_perp,safety_factor=safety_factor, 
                                                   q0=self.q_curr, max_iter=2000, tol=.1)

        viz = VizScene()
        viz.add_arm(self.arm)
        viz.add_marker(goal)
        viz.add_obstacle(obst, rad=radius)

        for q in self.qs:
            viz.update(qs=q)
            time.sleep(.05)
        
        viz.hold()

        # Display the calculated angles in the label
        self.labels[3].setText(f'q\u2081 = {round(self.qs[-1][0]*rad_to_deg, 3)}, ' 
                               f'q\u2082 = {round(self.qs[-1][1]*rad_to_deg, 3)}, '
                               f'q\u2083 = {round(self.qs[-1][2]*rad_to_deg, 3)}')

    

    def move(self):
        
        # run the inverse kinematics to get to that point
        self.ik()

        for q_s in self.qs:
            # Smoothly-ish move the servos
            for joint, q in zip(self.joints, q_s):
                joint.write(np.abs(q*rad_to_deg))
                time.sleep(.5)
            self.q_curr = self.qs
        
        viz = VizScene()
        viz.add_arm(self.arm)
        viz.update(qs=self.q_curr)
        viz.hold()

    def valid_qs(self):
        limits = self.arm.qlim
        for i in range(len(self.qs)):
            if self.qs[i] < limits[i][0] or self.qs[i] > limits[i][1]:
                return False
            
        return True
    
    def blink(self, flashes=2):
        on = True
        for _ in range(flashes):
            self.led.write(on)
            on = not on
            time.sleep(.1)

    def __del__(self):
        self.controller.exit()

# %% Main
if __name__ == '__main__':
    app = QApplication(sys.argv)

    dh = [[0, 2.5, 0, np.pi/2],
          [0, 0, 2.5, 0],
          [np.pi/3, 0, 2.5, 0]]
    limits = [[0, np.pi],
             [0, np.pi],
             [-np.pi, 0]]
    window = RobotArmGUI(dh=dh, joint_limits=limits, led=12, init_viz=False)
    window.show()

    sys.exit(app.exec_())
    del window

# %%
