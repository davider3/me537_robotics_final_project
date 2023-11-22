"""
Transforms Module - Contains code to learn about rotations
and eventually homogenous transforms. 

Empty outline derived from code written by John Morrell. 
"""

import numpy as np
import sympy as sp
from numpy import sin, cos, sqrt
from numpy.linalg import norm

## 2D Rotations
def rot2(th):
    """
    R = rot2(theta)
    Parameters
        theta: float or int, angle of rotation
    Returns
        R: 2 x 2 numpy array representing rotation in 2D by theta
    """

    R = np.array([[np.cos(th), -1*np.sin(th)],
                 [np.sin(th), np.cos(th)]])
    return R

## 3D Transformations
def rotx(th):
    """
    R = rotx(th)
    Parameters
        th: float or int, angle of rotation
    Returns
        R: 3 x 3 numpy array representing rotation about x-axis by amount theta
    """
    
    R = np.array([[1, 0, 0],
         [0, np.cos(th), -1*np.sin(th)],
         [0, np.sin(th), np.cos(th)]])

    return R

def roty(th):
    """
    R = rotx(th)
    Parameters
        th: float or int, angle of rotation
    Returns
        R: 3 x 3 numpy array representing rotation about y-axis by amount theta
    """

    R = np.array([[np.cos(th), 0, np.sin(th)],
         [0, 1, 0],
         [-1*np.sin(th), 0, np.cos(th)]])

    return R

def rotz(th):
    """
    R = rotx(th)
    Parameters
        th: float or int, angle of rotation
    Returns
        R: 3 x 3 numpy array representing rotation about z-axis by amount theta
    """

    R = np.array([[np.cos(th), -1*np.sin(th), 0],
                 [np.sin(th), np.cos(th), 0],
                 [0, 0, 1]])

    return R

def rotx_sp(th):
    """
    R = rotx(th)
    Parameters
        th: float or int, angle of rotation
    Returns
        R: 3 x 3 numpy array representing rotation about x-axis by amount theta
    """
    
    R = sp.Matrix([[1, 0, 0],
         [0, sp.cos(th), -1*sp.sin(th)],
         [0, sp.sin(th), sp.cos(th)]])

    return R

def roty_sp(th):
    """
    R = rotx(th)
    Parameters
        th: float or int, angle of rotation
    Returns
        R: 3 x 3 numpy array representing rotation about y-axis by amount theta
    """

    R = sp.Matrix([[sp.cos(th), 0, sp.sin(th)],
         [0, 1, 0],
         [-1*sp.sin(th), 0, sp.cos(th)]])

    return R

def rotz_sp(th):
    """
    R = rotx(th)
    Parameters
        th: float or int, angle of rotation
    Returns
        R: 3 x 3 numpy array representing rotation about z-axis by amount theta
    """

    R = sp.Matrix([[sp.cos(th), -1*sp.sin(th), 0],
                 [sp.sin(th), sp.cos(th), 0],
                 [0, 0, 1]])

    return R

# inverse of rotation matrix 
def rot_inv(R):
    '''
    R = rot_inv(R)
    Parameters
        R: 2x2 or 3x3 numpy array representing a proper rotation matrix
    Returns
        R: 2x2 or 3x3 inverse of the input rotation matrix
    '''

    R2 = np.transpose(R)
    return R2

def se3(R=np.eye(3), p=np.array([0, 0, 0])):
    """
        T = se3(R, p)
        Description:
            Given a numpy 3x3 array for R, and a 1x3 or 3x1 array for p, 
            this function constructs a 4x4 homogeneous transformation 
            matrix "T". 

        Parameters:
        R - 3x3 numpy array representing orientation, defaults to identity
        p = 3x1 numpy array representing position, defaults to [0, 0, 0]

        Returns:
        T - 4x4 numpy array
    """
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p

    return T

# inverse for Homogeneous Transformation Matrix
def inv(T):
    """
        Tinv = inv(T)
        Description:
        Returns the inverse transform to T

        Parameters:
        T

        Returns:
        Tinv - 4x4 numpy array that is the inverse to T so that T @ Tinv = I
    """

    R = T[:3, :3]
    p = T[:3, 3]
    R_inv = np.transpose(R)
    p_inv = -1* R_inv @ p
    T_inv = np.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = p_inv

    return T_inv

# DH Parameter Builder
def dh_param(theta, d, a, alpha):
    DH = np.array([[np.cos(theta), -1*np.sin(theta)*np.cos(alpha), np.sin(alpha)*np.sin(theta), a*np.cos(theta)],
                   [np.sin(theta), np.cos(alpha)*np.cos(theta), -1*np.sin(alpha)*np.cos(theta), a*np.sin(theta)],
                   [0, np.sin(alpha), np.cos(alpha), d],
                   [0, 0, 0, 1]])
    return DH

def dh_param_sp(theta, d, a, alpha):
    DH = sp.Matrix([[sp.cos(theta), -1*sp.sin(theta)*sp.cos(alpha), sp.sin(alpha)*sp.sin(theta), a*sp.cos(theta)],
                   [sp.sin(theta), sp.cos(alpha)*sp.cos(theta), -1*sp.sin(alpha)*sp.cos(theta), a*sp.sin(theta)],
                   [0, sp.sin(alpha), sp.cos(alpha), d],
                   [0, 0, 0, 1]])
    return DH

def R2rpy(R):
    """
    rpy = R2rpy(R)
    Description:
    Returns the roll-pitch-yaw representation of the SO3 rotation matrix

    Parameters:
    R - 3 x 3 Numpy array for any rotation

    Returns:
    rpy - 1 x 3 Numpy Matrix, containing <roll pitch yaw> coordinates (in radians)
    """
    
    # follow formula in book, use functions like "np.atan2" 
    # for the arctangent and "**2" for squared terms. 

    roll = np.arctan2(R[1,0], R[0,0])
    pitch = np.arctan2(-1*R[2,0], np.sqrt((R[2,1])**2 + (R[2,2])**2))
    yaw = np.arctan2(R[2,1], R[2,2])

    return np.array([roll, pitch, yaw])

def R2axis(R):
    """
    axis_angle = R2axis(R)
    Description:
    Returns an axis angle representation of a SO(3) rotation matrix

    Parameters:
    R

    Returns:
    axis_angle - 1 x 4 numpy matrix, containing  the axis angle representation
    in the form: <angle, rx, ry, rz>
    """

    # see equation (2.27) and (2.28) on pg. 54, using functions like "np.arccos," "np.sin," etc. 
    ang = np.arccos((R[0,0] + R[1,1] + R[2,2] - 1) * 0.5) 
    axis = (1 / (2 * np.sin(ang))) * np.array([R[2,1] - R[1,2],
                                                     R[0,2] - R[2,0],
                                                     R[1,0] - R[0,1]])
    axis_angle = np.array([ang,
                           axis[0],
                           axis[1],
                           axis[2]])
    return axis_angle

def axis2R(ang, v):
    """
    R = axis2R(angle, rx, ry, rz, radians=True)
    Description:
    Returns an SO3 object of the rotation specified by the axis-angle

    Parameters:
    angle - float, the angle to rotate about the axis in radians
    v = [rx, ry, rz] - components of the unit axis about which to rotate as 3x1 numpy array
    
    Returns:
    R - 3x3 numpy array
    """
    # Page 54 Eq 2.25

    rx = v[0]
    ry = v[1]
    rz = v[2]
    c = np.cos(ang)
    s = np.sin(ang)
    R = np.array([[(rx**2)*(1-c)+c, rx*ry*(1-c)-rz*s, rx*rz*(1-c)+ry*s],
                  [rx*ry*(1-c)+rz*s, (ry**2)*(1-c)+c, ry*rz*(1-c)-rx*s],
                  [rx*rz*(1-c)-ry*s, ry*rz*(1-c)+rx*s, (rz**2)*(1-c)+c]])
    return R.round(3)

def R2quat(R):
    """
    quaternion = R2quat(R)
    Description:
    Returns a quaternion representation of pose

    Parameters:
    R

    Returns:
    quaternion - 1 x 4 numpy matrix, quaternion representation of pose in the 
    format [nu, ex, ey, ez]
    """

    nu = 0.5 * sp.sqrt(R[0,0] + R[1,1] + R[2,2] + 1)
    epi = 0.5 * sp.Matrix([sp.sign(R[2,1] - R[1,2]) * sp.sqrt(R[0,0] - R[1,1] - R[2,2] + 1),
                           sp.sign(R[0,2] - R[2,0]) * sp.sqrt(R[1,1] - R[0,0] - R[2,2] + 1),
                           sp.sign(R[1,0] - R[0,1]) * sp.sqrt(R[2,2] - R[1,1] - R[0,0] + 1)])
    return np.array([nu,
                     epi[0],
                     epi[1],
                     epi[2]])
                    
def quat2R(q):
    """
    R = quat2R(q)
    Description:
    Returns a 3x3 rotation matrix

    Parameters:
    q - 4x1 numpy array, [nu, ex, ey, ez ] - defining the quaternion
    
    Returns:
    R - a 3x3 numpy array 
    """

    nu = q[0]
    ex = q[1]
    ey = q[2]
    ez = q[3]
    R =  np.array([[2*(nu**2 + ex**2)-1, 2*(ex*ey - nu*ez), 2*(ex*ez + nu*ey)],
                   [2*(ex*ey + nu*ez), 2*(nu**2 + ey**2)-1, 2*(ez*ey - nu*ex)],
                   [2*(ex*ez - nu*ey), 2*(ez*ey + nu*ex), 2*(nu**2 + ez**2)-1]])
    return R

def euler2R(th1, th2, th3, order='xyz'):
    """
    R = euler2R(th1, th2, th3, order='xyz')
    Description:
    Returns a 3x3 rotation matrix as specified by the euler angles, we assume in all cases
    that these are defined about the "current axis," which is why there are only 12 versions 
    (instead of the 24 possiblities noted in the course slides). 

    Parameters:
    th1, th2, th3 - float, angles of rotation
    order - string, specifies the euler rotation to use, for example 'xyx', 'zyz', etc.
    
    Returns:
    R - 3x3 numpy matrix
    """
    if order == 'xyx':
        R = rotx(th1) @ roty(th2) @ rotx(th3)
    elif order == 'xyz':
        R = rotx(th1) @ roty(th2) @ rotz(th3)
    elif order == 'xzx':
        R = rotx(th1) @ rotz(th2) @ rotx(th3)
    elif order == 'xzy':
        R = rotx(th1) @ rotz(th2) @ roty(th3)
    elif order == 'yxy':
        R = roty(th1) @ rotx(th2) @ roty(th3)
    elif order == 'yxz':
        R = roty(th1) @ rotx(th2) @ rotz(th3)
    elif order == 'yzx':
        R = roty(th1) @ rotz(th2) @ rotx(th3)
    elif order == 'yzy':
        R = roty(th1) @ rotz(th2) @ roty(th3)
    elif order == 'zxy':
        R = rotz(th1) @ rotx(th2) @ roty(th3)
    elif order == 'zxz':
        R = rotz(th1) @ rotx(th2) @ rotz(th3)
    elif order == 'zyx':
        R = rotz(th1) @ roty(th2) @ rotx(th3)
    elif order == 'zyz':
        R = rotz(th1) @ roty(th2) @ rotz(th3)
    else:
        print("Invalid Order!")
        return

    return R