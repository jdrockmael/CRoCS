import numpy as np
import math
import sympy
from sympy import symbols, Matrix
from math import sqrt, tan, cos, sin, atan2
import matplotlib.pyplot as plt
from numpy.random import randn
import rospy
# from IPython.display import display
sympy.init_printing(use_latex='mathjax')

VAR_ENC = 1e-4
VAR_RANGE = 0 
VAR_BEARING = 0
VAR_POSE1 = 1e-4
VAR_POSE2 = 1e-4

class RobotEKF():
    def __init__(self, wheelbase=0.10082, init_x=0.0, init_y=0.0, init_heading=0.0):        
        # EKF.__init__(self, 3, 2, 2)
        # Dimensions of pose, measurement, and control vectors
        self.dim_x = 3
        self.pose = np.zeros((self.dim_x, 1))   # state
        self.pose[0,0] = init_x
        self.pose[1,0] = init_y
        self.pose[2,0] = init_heading
        self.wheel_offset = 0.039624
        self.dim_z = 2
        self.dim_u = 2
        self.wheelbase = wheelbase

        self.P = np.eye(self.dim_x)                           # uncertainty covariance
        self.P[0,0] = 1e-2
        self.P[1,1] = 1e-2
        self.P[2,2] = 1e-3
        self.Q = np.diag([VAR_ENC, VAR_ENC])                  # process uncertainty
        self.R = np.diag([VAR_POSE1, VAR_POSE1, VAR_POSE1, VAR_POSE2, VAR_POSE2])

        # Initializing variables
        x, y, theta, Vl, Vr, dt, x2, y2, b = symbols('x, y, theta, Vl, Vr, dt, x2, y2, b')
        wr, wl, wx, wy, wtheta = symbols('wr, wl, wx, wy, wtheta')
        vxk, vx2, vyk, vy2, vthetak = symbols('vxk, vx2, vyk, vy2, vthetak')

        # f matrix estimate the next state
        self.f = Matrix(
            [[x + dt*(Vr + wr + Vl + wl)/2 * sympy.cos(theta) + wx],
            [y + dt*(Vr + wr + Vl + wl)/2 * sympy.sin(theta) + wy],
            [theta + dt*(Vr + wr - (Vl + wl))/b + wtheta]]
        )

        # h matrix convert pose to measurement
        self.h = Matrix(
            [[sympy.sqrt((y2+vy2-y-vyk)**2+(x2+vx2-x-vxk)**2)],
            # [theta + vthetak - sympy.atan2(x2+vx2-x-vxk, y2+vyk-y-vyk)]]

             [np.pi/2 - (theta + vthetak) - sympy.atan2(x2+vx2 - x-vxk, y2+vyk - y-vyk)]]
        )

        pose = Matrix([x, y, theta])
        process_noise = Matrix([wl, wr])
        measurement_noise = Matrix([vxk, vyk, vthetak, vx2, vy2])

        self.H_j = self.h.jacobian(pose)
        self.V_j = self.h.jacobian(measurement_noise)
        self.A_j = self.f.jacobian(pose)
        self.W_j= self.f.jacobian(process_noise)

        self.subs = {x:init_x, y: init_y, theta:init_heading, Vl:0, Vr:0, dt:0, x2:0, y2:0, b:self.wheelbase, 
                     wl: 0, wr:0, wx: 0, wy: 0, wtheta: 0,
                     vxk: 0, vx2: 0, vyk: 0, vy2: 0, vthetak: 0}

        self.dt = dt
        self.x_x, self.x_y, self.x_theta = x, y, theta
        self.x2_x, self.x2_y = x2, y2
        self.vl, self.vr = Vl, Vr
        self.w_Wl, self.w_Wr, self.w_Wx, self.w_Wy, self.w_Wtheta = wl, wr, wx, wy, wtheta
        self.V_vxk, self.V_vx2, self.V_vyk, self.V_vy2, self.V_vthetak = vxk, vx2, vyk, vy2, vthetak

    def predict(self, u, dt):
        """ Prediction step of EKF.
        x = f(x,u,w=0)
        P = A @ P @ A' + W @ Q @ W'
        :u: 2x1 vector of left and right wheel speed in mm/s
        :dt: time elapsed
        """
    
        self.subs[self.dt] = dt

        self.subs[self.vl] = u[0]
        self.subs[self.vr] = u[1]

        self.subs[self.w_Wl] = 0
        self.subs[self.w_Wr] = 0

        # Update pose
        self.pose = np.array(self.f.evalf(subs=self.subs)).astype(float)  

        # self.pose[2,0] = self.pose[2,0] % (2*np.pi)
        # if self.pose[2,0] > np.pi:             
        #     self.pose[2,0] -= 2*np.pi
        

        # rospy.loginfo("POSEEEEEEEEEEEEE PREDICT%s", self.pose.T)
        
        self.subs[self.x_x] = self.pose[0,0]
        self.subs[self.x_y] = self.pose[1,0]
        self.subs[self.x_theta] = self.pose[2,0]

        # Set the process noise
        w = np.random.normal(0, self.Q)
        self.subs[self.w_Wl] = w[0,0]
        self.subs[self.w_Wr] = w[1,1]

        A = np.array(self.A_j.evalf(subs=self.subs)).astype(float)        
        W = np.array(self.W_j.evalf(subs=self.subs)).astype(float)

        # Update covariance
        self.P = A @ self.P @ A.T + W @ self.Q @ W.T

        real_pose = self.pose
        real_pose[0,0] += self.wheel_offset - self.wheel_offset * np.cos(self.pose[-1])
        real_pose[1,0] -= self.wheel_offset * np.sin(self.pose[-1])

        real_pose[2,0] = real_pose[2,0] % (2*np.pi)
        if real_pose[2,0] > np.pi:             
            real_pose[2,0] -= 2*np.pi

        return real_pose
    
    def update(self, z, pose1, pose2, dt):
        """ Update/Correction step of EKF.
        K = PH'(HPH' + VRV')^-1
        x = x + K(z - h(x,0))
        P = (I - KH)P
        :z: 2x1 measurement vector of range and bearing
        :pose2: 2x1 vector consisting of x2 and y2
        :dt: elapsed time
        """
        self.subs[self.x2_x] = pose2[0]
        self.subs[self.x2_y] = pose2[1]

        self.subs[self.dt] = dt

        # Set the process noise
        v = np.random.normal(0, self.R)
        self.subs[self.V_vxk] = v[0,0]
        self.subs[self.V_vyk] = v[1,1]
        self.subs[self.V_vthetak] = v[2,2]
        self.subs[self.V_vx2] = v[3,3]
        self.subs[self.V_vy2] = v[4,4]
        # rospy.loginfo("TEST1 %s",self.pose[2,0])

        H = np.array(self.H_j.evalf(subs=self.subs)).astype(float) 

        V = np.array(self.V_j.evalf(subs=self.subs)).astype(float) 

        K = self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + V @ self.R @ V.T)

        res = self.residual(z, pose1, pose2)
        rospy.loginfo("RESIDUALLLLLLLLL %s", res.T)

        self.pose += K @ res#self.residual(z, pose2)

        # self.pose[2,0] = self.pose[2,0] % (2*np.pi)
        # if self.pose[2,0] > np.pi:             
        #     self.pose[2,0] -= 2*np.pi
        # rospy.loginfo("POSEEEEEEEEEEEEE UPDATE%s", self.pose.T)

        # Set poses
        self.subs[self.x_x] = self.pose[0,0]
        self.subs[self.x_y] = self.pose[1,0]
        self.subs[self.x_theta] = self.pose[2,0]
        
        # Update covariance, equivalent to P = (I - KH)P
        self.P -= K @ H @ self.P

        real_pose = self.pose
        real_pose[0] += self.wheel_offset - self.wheel_offset * np.cos(self.pose[2,0])
        real_pose[1] -= self.wheel_offset * np.sin(self.pose[2,0])

        # x = x + wheeloffst - wheeloffset*cos
        # y = y - wheeloffset*

        # x = - wheeloffset + wheeloffset*

        return real_pose

    def residual(self, z, pose1, pose2):
        """ Helper function to calculate the residual
        """
        # Set pose
        # self.subs[self.x_x] = self.pose[0,0]
        # self.subs[self.x_y] = self.pose[1,0]
        self.subs[self.x_x] = pose1[0,0] - self.wheel_offset + self.wheel_offset * np.cos(pose1[2,0])
        self.subs[self.x_y] = pose1[1,0] + self.wheel_offset * np.sin(pose1[2,0])
        self.subs[self.x_theta] = pose1[2,0] #self.pose[2,0]

        self.subs[self.x2_x] = pose2[0]
        self.subs[self.x2_y] = pose2[1]

        # Set the process noise
        self.subs[self.V_vxk] = 0
        self.subs[self.V_vyk] = 0
        self.subs[self.V_vthetak] = 0
        self.subs[self.V_vx2] = 0
        self.subs[self.V_vy2] = 0


        # sam = math.atan2(pose2[0] - self.pose[0,0], pose2[1] - self.pose[1,0])
        rospy.loginfo("PRE INNOVATION POSE %s", self.pose.T)
        # rospy.loginfo("DELTAX DELTAY (%s, %s)",pose2[0] - self.pose[0,0], pose2[1] - self.pose[1,0])

        # sam = sam % (2*np.pi)
        # if sam > np.pi:             
        #     sam -= 2*np.pi
        # rospy.loginfo("ATAN2 %s", sam)

        innovation = np.array(self.h.evalf(subs=self.subs)).astype(float) 
        # rospy.loginfo("INNOVATION %s", innovation.T)
        innovation[1] = innovation[1] % (2*np.pi)
        if innovation[1] > np.pi:             
            innovation[1] -= 2*np.pi

        rospy.loginfo("INNOVATION AFTER %s", innovation.T)
        # rospy.loginfo("Z %s", z.T)
        zx = z - innovation

        # rospy.loginfo("ZX BEFORE %s", zx.T)
        # # Normalize bearing to [-pi/2, pi/2]
        # zx[-1] = zx[-1] % np.pi
        # # rospy.loginfo("ZX DURING %s", zx.T)
        # if zx[-1] > np.pi/2:             
        #     zx[-1] -= 2 * np.pi

        # rospy.loginfo("ZX AFTER %s", zx.T)

        return zx
    
    def z_landmark(self, pose2):
        """ Helper function to convert pose to measurement with h matrix
        """
        # Set pose
        self.subs[self.x_x] = self.pose[0,0]
        self.subs[self.x_y] = self.pose[1,0]
        self.subs[self.x_theta] = self.pose[2,0]

        self.subs[self.x2_x] = pose2[0]
        self.subs[self.x2_y] = pose2[1]

        # Set the process noise
        self.subs[self.V_vxk] = 0
        self.subs[self.V_vyk] = 0
        self.subs[self.V_vthetak] = 0
        self.subs[self.V_vx2] = 0
        self.subs[self.V_vy2] = 0

        return np.array(self.h.evalf(subs=self.subs)).astype(float) 