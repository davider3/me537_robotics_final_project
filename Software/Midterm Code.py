# %% [markdown]
# # Midterm 2023
# * Copy this file to your homework workspace to have access to your other kinematic functions

# %%
# To test your setup, after defining the robot arm as described below, (but nothing else)
# you can run this file directly to make sure it is plotting the arm, obstacle, and goal 
# as expected. 

import kinematics as kin  #this is your kinematics file that you've been developing all along
from visualization import VizScene # this is the newest visualization file updated on Oct 12
import time
import numpy as np


# Define your kinematics and an "arm" variable here using DH parameters so they
# are global variables that are available in your function below:

dh = [[np.pi/2.0, 4.0, 0.0, np.pi/2],
      [np.pi/6.0, 0.0, 0.0, np.pi/2],
      [0, 4.0, 0.0, np.pi/2],
      [np.pi/6.0, 0.0, 2.0, np.pi/2]]
arm = kin.SerialArm(dh)


# let's also plot robot to make sure it matches what we think it should
# (this will look mostly like the pictures on part 1 if your DH parameters
# are correct
# viz_check = VizScene()
# viz_check.add_arm(arm, joint_colors=[np.array([0.95, 0.13, 0.13, 1])]*arm.n)
# viz_check.update(qs = [[0, 0, 0, 0]])
# viz_check.hold()


def compute_robot_path(q_init, goal, obst_location, obst_radius):
      # this can be similar to your IK solution for HW 6, but will require modifications
      # to make sure you avoid the obstacle as well.

      goal_arr = np.array(goal).reshape(3,)
      obst_location_arr = np.array(obst_location).reshape(3,)

      # this is an empty list that we'll use to return our solutions so they can be plotted
      q_s = []

      # this helper function will give distance vectors between goals and obstacles
      def get_distance_vec(q, position, index):
            T_cur = arm.fk(q, index)
            cur_position = T_cur[0:3,3]

            distance_vec = (position-cur_position)

            return distance_vec

      # setting gains for the algorithm. K moves us towards the goal, k_obst moves joints and tip away
      # from the obstacle, and k_perp is used for a fake virtual force that moves us around the obstacle. 
      # There are trade-offs in this tuning such that the resulting motion for these gains is less smooth
      # but a little quicker to converge. 
      K = np.eye(3) * 0.03
      k_obst = 0.2
      k_perp = 0.01 # only used to move the tip of the robot around the obstacle. 

      # initializing counter, error, and q 
      counter = 0
      error = 100.0
      q = np.array(q_init)

      # safety factor that can be tuned for when the spring forces to push away from the obstacle starts
      # acting. 
      safety_factor = 2.0  

      # will stop when reach within 1 cm or hit 500 iterations. 
      while error > 0.01 and counter < 500:
            qd_total = np.zeros(arm.n, )

            # for each coordinate frame (or joint) from frame 2 to the tip, do the following. 
            for i in range(2,arm.n):

                  # get the jacobian for that frame (joint or tip)
                  Ji = arm.jacob(q, index=i+1)

                  # get the obstacle distance from the coordinate frame. If we wanted to be more
                  # careful, we could find the distance at intermediate points along each link 
                  # as well. 
                  obst_dist = get_distance_vec(q, obst_location_arr, i+1)
                  dist_mag = np.sqrt(obst_dist.T @ obst_dist)

                  # if frame is the tip of the robot do this
                  if i+1 == arm.n:
                        # find distance vector from the tip to the goal
                        goal_dist = get_distance_vec(q, goal_arr, i+1)

                        # find joint velocities that should move the tip of the robot towards the goal using a 
                        # virtual spring force (K*goal_dist)
                        qd = Ji[0:3,:].T @ K @ goal_dist
                        qd_total = qd_total + qd

                        # check obstacle distance for the tip as well to see if we should move the tip 
                        # perpendicular to the obstacle
                        obst_dist = get_distance_vec(q, obst_location_arr, i+1)
                        if dist_mag < obst_radius*safety_factor:
                              # calculate the perpendicular direction around the obstacle
                              perpendicular = np.cross(obst_dist, goal_arr-obst_location_arr)
                              dir_perpendicular  = perpendicular / np.sqrt(perpendicular.T@perpendicular)

                              # find a joint velocity that also moves the tip around or perpendicular to  the object
                              # but in the general direction of the goal 
                              qd = k_perp * Ji[0:3,:].T @ dir_perpendicular
                              qd_total = qd_total + qd

                  # if frame is too close to the obstacle, find a joint velocity that moves it away. 
                  if dist_mag < obst_radius*safety_factor:
                  
                        # calculates how much our virtual spring is compressed 
                        compressed = obst_radius*safety_factor-dist_mag

                        # finds the direction we should move the frame away 
                        dir_away = -obst_dist/np.sqrt(obst_dist.T@obst_dist)

                        # using J.T method to calculate a pseudo torque, that we assume is correlated
                        # with the joint velocity that torque would cause. We just tune the gain to
                        # get reasonable motion. 
                        qd = k_obst*compressed * Ji[0:3,:].T @ dir_away
                        qd_total = qd_total + qd

            # update q using qd_total, assuming the time step has been folded in the gains in
            # the calculation of qd in each step
            q = q+qd_total

            # update error to check termination criteria
            error_vec = get_distance_vec(q, goal_arr, arm.n)
            error = error_vec.T@error_vec

            q_s.append(q)
            counter = counter + 1
            print("counter:", counter)
            print("error:", error)

      return q_s

if __name__ == "__main__":

      # if your function works, this code should show the goal, the obstacle, and your robot moving towards the goal.
      q_0 = [0, 0, 0, 0]
      goal = [0, 2, 4]
      obst_position = [0, 3, 2]
      obst_rad = 1.0

      q_ik_slns = compute_robot_path(q_0, goal, obst_position, obst_rad)


      # if you just want to check if you have your code set up correctly, you can uncomment the next three lines and run this file
      # using either vs code or the terminal (and running "python3 midterm_2022.py"). None of the next three lines are needed
      # for your solution, they may just help you check your visualization before you get going. It will just display 100 different
      # random sets of joint angles as well as the goal and obstacle.

      # import numpy as np
      # q_ik_slns = np.random.uniform(size=(100,4))
      # q_ik_slns = q_ik_slns.tolist()




      # depending on how you store q_ik_slns inside your function, you may need to change this for loop
      # definition. However if you store q as I've done above, this should work directly.
      viz = VizScene()
      viz.add_arm(arm, joint_colors=[np.array([0.95, 0.13, 0.13, 1])]*arm.n)
      viz.add_marker(goal, radius=0.1)
      viz.add_obstacle(obst_position, rad=obst_rad)
      for q in q_ik_slns:
            viz.update(qs=[q])

            # if your step in q is very small, you can shrink this time, or remove it completely to speed up your animation
            time.sleep(0.05)