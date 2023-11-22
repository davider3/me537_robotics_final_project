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

# %%

class RobotArmGUI(QWidget):
    def __init__(self):
        super().__init__()

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
        self.x_label = QLabel('X Coordinate:')
        self.x_input = QLineEdit(self)

        self.y_label = QLabel('Y Coordinate:')
        self.y_input = QLineEdit(self)

        self.z_label = QLabel('Z Coordinate:')
        self.z_input = QLineEdit(self)

        # Create buttons for inverse kinematics and moving the robot arm
        self.inverse_kinematics_button = QPushButton('Inverse Kinematics', self)
        self.move_arm_button = QPushButton('Move Robot Arm', self)

        # Connect buttons to functions
        self.inverse_kinematics_button.clicked.connect(self.ik)
        self.move_arm_button.clicked.connect(self.move)

        # Create a label to display the calculated angles
        self.angles_label = QLabel('Calculated Angles:')

        # Set up the layout
        input_layout = QVBoxLayout()
        input_layout.addWidget(self.x_label)
        input_layout.addWidget(self.x_input)
        input_layout.addWidget(self.y_label)
        input_layout.addWidget(self.y_input)
        input_layout.addWidget(self.z_label)
        input_layout.addWidget(self.z_input)

        button_layout = QVBoxLayout()
        button_layout.addWidget(self.inverse_kinematics_button)
        button_layout.addWidget(self.move_arm_button)

        result_layout = QVBoxLayout()
        result_layout.addWidget(self.angles_label)

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
        goal = np.zeros(3)
        goal[0] = float(self.x_input.text())
        goal[1] = float(self.y_input.text())
        goal[2] = float(self.z_input.text())

        # TODO Fine tune the gain matrix
        gain = .3 * np.eye(6)

        self.qs = self.arm.ik_position(goal, plot=True, K=gain, q0=self.q_curr)

        # TODO Display the calculated angles in the label

        print(self.qs)
        
        # Show the orientation of the arm
        viz = VizScene()
        viz.add_marker(goal)
        viz.add_arm(self.arm)
        viz.update(qs=self.qs)
        viz.hold()
        viz.close_viz()

    def move(self):
        
        # run the inverse kinematics to get to that point
        self.ik()

        # TODO define moving the robot arm

        self.q_curr = self.qs

# %%
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = RobotArmGUI()
    window.show()
    sys.exit(app.exec_())


# %%
