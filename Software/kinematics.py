"""
Kinematics Module - Contains code for:
- Forward Kinematics, from a set of DH parameters to a serial linkage arm with callable forward kinematics
- Inverse Kinematics
- Jacobian

John Morrell, Jan 26 2022
Tarnarmour@gmail.com

modified by: 
Marc Killpack, Sept 21, 2022 and Sept 21, 2023
"""

from transforms import *
from visualization import VizScene
import time
from utility import skew


eye = np.eye(4)
pi = np.pi

class dh2AFunc:
    """
    A = dh2AFunc(dh, joint_type="r")
    Description:
    Accepts a list of 4 dh parameters corresponding to the transformation for one link 
    and returns a function "f" that will generate a homogeneous transform "A" given 
    "q" as an input. A represents the transform from link i-1 to link i. This follows
    the "standard" DH convention. 

    Parameters:
    dh - 1 x 4 list from dh parameter table for one transform from link i-1 to link i,
    in the order [theta d a alpha] - THIS IS NOT THE CONVENTION IN THE BOOK!!! But it is the order of operations. 

    Returns:
    f(q) - a function that can be used to generate a 4x4 numpy matrix representing the homogeneous transform 
        from one link to the next
    """
    def __init__(self, dh, jt):

        # if joint is revolute implement correct equations here:
        if jt == 'r':
            # although A(q) is only a function of "q", the dh parameters are available to these next functions 
            # because they are passed into the "init" function above. 

            def A(q): 
                # See eq. (2.52), pg. 64

                # Extract DH parameters
                theta, d, a, alpha = dh

                # Construct individual DH transformation matrix
                T = np.array([[np.cos(q + theta), -np.sin(q + theta) * np.cos(alpha), np.sin(q + theta) * np.sin(alpha), a * np.cos(q + theta)],
                                [np.sin(q + theta), np.cos(q + theta) * np.cos(alpha), -np.cos(q + theta) * np.sin(alpha), a * np.sin(q + theta)],
                                [0, np.sin(alpha), np.cos(alpha), d],
                                [0, 0, 0, 1]])


                return T


        # if joint is prismatic implement correct equations here:
        else:
            def A(q):
                # See eq. (2.52), pg. 64

                # Extract DH parameters
                theta, d, a, alpha = dh

                # Construct individual DH transformation matrix
                T = np.array([[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
                                [np.sin(theta), np.cos(alpha) * np.cos(theta), -np.sin(alpha) * np.cos(theta), a * np.sin(theta)],
                                [0, np.sin(alpha), np.cos(alpha), d + q],
                                [0, 0, 0, 1]])

                return T


        self.A = A


class SerialArm:
    """
    SerialArm - A class designed to represent a serial link robot arm

    SerialArms have frames 0 to n defined, with frame 0 located at the first joint and aligned with the robot body
    frame, and frame n located at the end of link n.

    """

    def __init__(self, dh_params, jt=None, base=eye, tip=eye, joint_limits=None):
        """
        arm = SerialArm(dh_params, joint_type, base=I, tip=I, radians=True, joint_limits=None)
        :param dh: n length list where each entry in list is another list of length 4, representing dh parameters, [theta d a alpha]
        :param jt: n length list of strings, 'r' for revolute joint and 'p' for prismatic joint
        :param base: 4x4 numpy array representing SE3 transform from world or inertial frame to frame 0
        :param tip: 4x4 numpy array representing SE3 transform from frame n to tool frame or tip of robot
        :param joint_limits: 2 length list of n length lists, holding first negative joint limit then positive, none for
        not implemented
        """
        self.dh = dh_params
        self.n = len(dh_params)

        # we will use this list to store the A matrices for each set/row of DH parameters. 
        self.transforms = []

        # assigning a joint type
        if jt is None:
            self.jt = ['r'] * self.n
        else:
            self.jt = jt
            if len(self.jt) != self.n:
                print("WARNING! Joint Type list does not have the same size as dh param list!")
                return None

        # using the code we wrote above to generate the function A(q) for each set of DH parameters
        for i in range(self.n):
            # Create a transformation function for the current link using dh2AFunc
            dh = dh_params[i]
            joint_type = jt[i] if jt is not None else 'r'
            f = dh2AFunc(dh, joint_type)

            # Append the transformation function to the transforms list
            self.transforms.append(f.A)


        # assigning the base, and tip transforms that will be added to the default DH transformations.
        self.base = base
        self.tip = tip
        self.qlim = joint_limits

        self.reach = 0
        for i in range(self.n):
            self.reach += np.sqrt(self.dh[i][0]**2 + self.dh[i][2]**2)

        self.max_reach = 0.0
        for dh in self.dh:
            self.max_reach += norm(np.array([dh[0], dh[2]]))

    def fk(self, q, index=None, base=False, tip=False):
        """
            T = arm.fk(q, index=None, base=False, tip=False)
            Description: 
                Returns the transform from a specified frame to another given a 
                set of joint inputs q and the index of joints

            Parameters:
                q - list or iterable of floats which represent the joint positions
                index - integer or list of two integers. If a list of two integers, the first integer represents the starting JOINT 
                    (with 0 as the first joint and n as the last joint) and the second integer represents the ending FRAME
                    If one integer is given only, then the integer represents the ending Frame and the FK is calculated as starting from 
                    the first joint
                base - bool, if True then if index starts from 0 the base transform will also be included
                tip - bool, if true and if the index ends at the nth frame then the tool transform will be included
            
            Returns:
                T - the 4 x 4 homogeneous transform from frames determined from "index" variable
        """

        # Data Type and error checking
        if not hasattr(q, '__getitem__'):
            q = [q]

        if len(q) != self.n:
            print("WARNING: q (input angle) not the same size as number of links!")
            return None

        if isinstance(index, (list, tuple)):
            start_frame = index[0]
            end_frame = index[1]
        elif index == None:
            start_frame = 0
            end_frame = self.n
        else:
            start_frame = 0
            if index < 0:
                print("WARNING: Index less than 0!")
                print(f"Index: {index}")
                return None
            end_frame = index

        if end_frame > self.n:
            print("WARNING: Ending index greater than number of joints!")
            print(f"Starting frame: {start_frame}  Ending frame: {end_frame}")
            return None
        if start_frame < 0:
            print("WARNING: Starting index less than 0!")
            print(f"Starting frame: {start_frame}  Ending frame: {end_frame}")
            return None
        if start_frame > end_frame:
            print("WARNING: starting frame must be less than ending frame!")
            print(f"Starting frame: {start_frame}  Ending frame: {end_frame}")
            return None

        # Initialize the transformation matrix as the base if base=True
        T = self.base if base else np.eye(4)

        # Calculate the cumulative transformation
        for i in range(start_frame, end_frame):
            # Get the transformation function for the current link
            transform_func = self.transforms[i]

            # Calculate the transformation for the current joint position q[i]
            T_i = transform_func(q[i])

            # Multiply the cumulative transformation by T_i
            T = T @ T_i

        # Post-multiply with the tip transformation if tip=True
        if tip and end_frame == self.n:
            T = T @ self.tip

        return T

    def jacob(self, q, index=None, base=False, tip=False):
        """
        J = arm.jacob(q)
        Description: 
        Returns the geometric jacobian for the frame defined by "index", which corresponds
        to a frame on the arm, with the arm in a given configuration defined by "q"

        Parameters:
        q - list or numpy array of joint positions
        index - integer, which joint frame at which to calculate the Jacobian

        Returns:
        J - numpy matrix 6xN, geometric jacobian of the robot arm
        """


        if index is None:
            index = self.n
        elif index > self.n:
            print("WARNING: Index greater than number of joints!")
            print(f"Index: {index}")

        # Initialize the Jacobian with zeros
        J = np.zeros([6, index])
        # Get the translation of the desired frame in frame 0
        pe = self.fk(q, index, base, tip)[:3, 3]

        for i in range(index):
            # check if joint is revolute
            if self.jt[i] == 'r':
                # Get homogeneous transformation to current joint
                a = self.fk(q, i, base)
                # Calculate the rotational axis of the revolute joint
                z_axis = a[:3, 2]
                # Calculate the linear velocity part of the Jacobian
                J[:3, i] = np.cross(z_axis, pe - self.fk(q, i, base)[:3, 3])
                # Calculate the angular velocity part of the Jacobian
                J[3:, i] = z_axis
                
            # if not assume joint is prismatic
            else:
                J[:3, i] = self.fk(q, i, base)[:3, 2]

        return J

    def __str__(self):
        """
            This function just provides a nice interface for printing information about the arm. 
            If we call "print(arm)" on an SerialArm object "arm", then this function gets called.
            See example in "main" below. 
        """
        dh_string = """DH PARAMS\n"""
        dh_string += """theta\t|\td\t|\ta\t|\talpha\t|\ttype\n"""
        dh_string += """---------------------------------------\n"""
        for i in range(self.n):
            dh_string += f"{self.dh[i][0]}\t|\t{self.dh[i][1]}\t|\t{self.dh[i][2]}\t|\t{self.dh[i][3]}\t|\t{self.jt[i]}\n"
        return "Serial Arm\n" + dh_string
    
    def ik_position(self, target, plot=False, q0=None, method='J_T', force=True, tol=1e-4, K=None, kd=0.001, max_iter=100):
        """
        (qf, ef, iter, reached_max_iter, status_msg) = arm.ik2(target, q0=None, method='jt', force=False, tol=1e-6, K=None)
        Description:
            Returns a solution to the inverse kinematics problem finding
            joint angles corresponding to the position (x y z coords) of target

        Args:
            target: 3x1 numpy array that defines the target location. 

            q0: length of initial joint coordinates, defaults to q=0 (which is
            often a singularity - other starting positions are recommended)

            method: String describing which IK algorithm to use. Options include:
                - 'pinv': damped pseudo-inverse solution, qdot = J_dag * e * dt, where
                J_dag = J.T * (J * J.T + kd**2)^-1
                - 'J_T': jacobian transpose method, qdot = J.T * K * e

            force: Boolean, if True will attempt to solve even if a naive reach check
            determines the target to be outside the reach of the arm

            tol: float, tolerance in the norm of the error in pose used as termination criteria for while loop

            K: 3x3 numpy matrix. For both pinv and J_T, K is the positive definite gain matrix used for both. 

            kd: is a scalar used in the pinv method to make sure the matrix is invertible. 

            max_iter: maximum attempts before giving up.

        Returns:
            qf: 6x1 numpy matrix of final joint values. If IK fails to converge the last set
            of joint angles is still returned

            ef: 3x1 numpy vector of the final error

            count: int, number of iterations

            flag: bool, "true" indicates successful IK solution and "false" unsuccessful

            status_msg: A string that may be useful to understanding why it failed. 
        """
        # Fill in q if none given, and convert to numpy array 
        if isinstance(q0, np.ndarray):
            q = q0
        elif q0 == None:
            q = np.array([0.0]*self.n)
        else:
            q = np.array(q0)

        # initializing some variables in case checks below don't work
        error = None
        count = 0

        maximum_reach = 0
        for i in range(self.n):  # Add max length of each link
            maximum_reach = maximum_reach + np.sqrt(self.dh[i][1] ** 2 + self.dh[i][2] ** 2)

        pt = target  # Find distance to target
        target_distance = np.sqrt(pt[0] ** 2 + pt[1] ** 2 + pt[2] ** 2)

        if target_distance > maximum_reach and not force:
            print("WARNING: Target outside of reachable workspace!")
            return q, error, count, False, "Failed: Out of workspace"

        if not isinstance(K, np.ndarray):
            return q, error, count, False,  "No gain matrix 'K' provided"

        def get_error(p_des, ht_cur):
            e = np.zeros(6)
            e[:3] = p_des - ht_cur[:3, 3]
            return e

        def jt_q_dot(q, e):
            jt = np.transpose(self.jacob(q))
            return jt @ (K @ e)

        def pinv_q_dot(q, e):
            j = self.jacob(q)
            jt = np.transpose(j)
            return jt @ np.linalg.inv(j @ jt + (kd**2) * np.eye(6)) @ (K @ e)

        # Get initial error
        pose = self.fk(q)
        error = get_error(target, pose)

        # Initialize visualization if necessary
        viz = None
        if plot:
            viz = VizScene()
            viz.add_arm(self)
            viz.update(qs=[q]) 
            viz.add_marker(target, radius=.6)
        
        # Iteratively get closer to desired location
        while np.linalg.norm(error) > tol and count < max_iter:
            pose = self.fk(q)

            error = get_error(target, pose)
            
            if method == 'J_T':
                q_dot = jt_q_dot(q, error)
            elif method == 'p_inv':
                q_dot = pinv_q_dot(q, error)
            else:
                print("Error, invalid method")
                break
            
            count += 1
            q = q_dot + q
            
            if plot:
                viz.update(qs=[q])

        # Show the visualization for a bit longer when finished and then close viz
        if plot:
            viz.hold()

        return (q, error, count, count < max_iter, 'No errors noted')

    def Z_shift(self, R=np.eye(3), p=np.zeros(3), p_frame='i'):

        """
        Z = Z_shift(R, p, p_frame_order)
        Description: 
            Generates a shifting operator (rotates and translates) to move twists and Jacobians 
            from one point to a new point defined by the relative transform R and the translation p. 

        Parameters:
            R - 3x3 numpy array, expresses frame "i" in frame "j" (e.g. R^j_i)
            p - 3x1 numpy array length 3 iterable, the translation from the initial Jacobian point to the final point, expressed in the frame as described by the next variable.
            p_frame - is either 'i', or 'j'. Allows us to define if "p" is expressed in frame "i" or "j", and where the skew symmetrics matrix should show up. 

        Returns:
            Z - 6x6 numpy array, can be used to shift a Jacobian, or a twist
        """

        # generate our skew matrix
        S = skew(p)

        r_big = np.zeros((6, 6))
        r_big[:3, :3] = R
        r_big[3:, 3:] = R

        skew_thingy = np.eye(6)
        skew_thingy[:3, 3:] = -S

        if p_frame == 'i':
            Z = r_big @ skew_thingy
            pass
        elif p_frame == 'j':
            Z = skew_thingy @ r_big
            pass
        else:
            Z = None

        return Z
