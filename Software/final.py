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
    from pyfirmata import Arduino, util
except:
    import pip
    pip.main(['install', 'pyfirmata'])

# Fix the deprecated part of the pyfirmata library
if not hasattr(inspect, 'getargspec'):
    inspect.getargspec = inspect.getfullargspec

# %% Robot class

class RobotArmGUI(QWidget):
    def __init__(self, com, servo_pins=[9, 10, 11]):
        super().__init__()
        
        # Setup Arduino
        self.controller = Arduino(com)
        joints = [self.controller.get_pin(f'd:{pin}:p') for pin in servo_pins]

        # Setup Serial arm
        # TODO update the link lengths with the actual values
        dh = [[0, 3, 0, -np.pi/2],
              [0, 0, 3, 0],
              [np.pi/2, 0, 3, 0]]
        # TODO define joint limits
        limits = None
        self.arm = kin.SerialArm(dh, joint_limits=limits)
        self.q_curr = [0, 0, 0]
        viz = VizScene()
        viz.add_arm(self.arm)
        viz.hold()
        viz.close_viz()

        self.init_ui()

    def init_ui(self):
        # Create labels and input fields for x, y, and z coordinates
        self.labels = [QLabel('X Coordinate:'),
                       QLabel('Y Coordinate:'),
                       QLabel('Z Coordinate:'),
                       QLabel('Calculated Angles:')]
        self.inputs = [QLineEdit(self),
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
        goal = [float(input.text()) for input in self.inputs]

        # TODO Fine tune the gain matrix
        gain = .3 * np.eye(6)

        self.qs = self.arm.ik_position(goal, plot=True, method='p_inv', K=gain, q0=self.q_curr)

        # TODO Display the calculated angles in the label

    def move(self):
        
        # run the inverse kinematics to get to that point
        self.ik()

        # TODO define moving the robot arm

        self.q_curr = self.qs

# %% Main
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = RobotArmGUI('COM11')
    window.show()
    sys.exit(app.exec_())

# %%
