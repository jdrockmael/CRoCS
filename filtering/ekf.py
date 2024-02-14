from filterpy.kalman import ExtendedKalmanFilter as EKF
import numpy as np
import math
import sympy
from copy import deepcopy
from sympy import symbols, Matrix
from filterpy.stats import plot_covariance_ellipse
from math import sqrt, tan, cos, sin, atan2
import matplotlib.pyplot as plt
from numpy.random import randn
from IPython.display import display
sympy.init_printing(use_latex='mathjax')

VAR_ENC = 1e-1
VAR_RANGE = 0 
VAR_BEARING = 0
VAR_POSE1 = 1e-2
VAR_POSE2 = 1e-2

class RobotEKF():
    def __init__(self, wheelbase):        
        # EKF.__init__(self, 3, 2, 2)
        # Dimensions of pose, measurement, and control vectors
        self.dim_x = 3
        self.pose = np.zeros((self.dim_x, 1))   # state
        self.dim_z = 2
        self.dim_u = 2
        self.wheelbase = wheelbase

        self.P = np.eye(self.dim_x)                           # uncertainty covariance
        self.P[0,0] = 1e-2
        self.P[1,1] = 1e-2
        self.Q = np.diag([VAR_ENC, VAR_ENC])                  # process uncertainty
        self.R = np.diag([VAR_POSE1, VAR_POSE1, VAR_POSE1, VAR_POSE2, VAR_POSE2])

        # Initializing variables
        x, y, theta, Vl, Vr, dt, x2, y2, b = symbols('x, y, theta, Vl, Vr, dt, x2, y2, b')
        wr, wl = symbols('wr, wl')
        vxk, vx2, vyk, vy2, vthetak = symbols('vxk, vx2, vyk, vy2, vthetak')

        # f matrix estimate the next state
        self.f = Matrix(
            [[x + dt*(Vr + wr + Vl + wl)/2 * sympy.cos(theta)],
            [y + dt*(Vr + wr + Vl + wl)/2 * sympy.sin(theta)],
            [theta + dt*(Vr + wr - (Vl + wl))/b]]
        )

        # h matrix convert pose to measurement
        self.h = Matrix(
            [[sympy.sqrt((y2+vy2-y-vyk)**2+(x2+vx2-x-vxk)**2)],
             [theta + vthetak - sympy.atan2(y2+vyk-y-vyk,x2+vx2-x-vxk)]]
        )

        pose = Matrix([x, y, theta])
        process_noise = Matrix([wl, wr])
        measurement_noise = Matrix([vxk, vyk, vthetak, vx2, vy2])

        self.H_j = self.h.jacobian(pose)
        self.V_j = self.h.jacobian(measurement_noise)
        self.A_j = self.f.jacobian(pose)
        self.W_j= self.f.jacobian(process_noise)

        self.subs = {x:0, y: 0, theta:0, Vl:0, Vr:0, dt:0, x2:0, y2:0, b:wheelbase, 
                     wl: 0, wr:0, 
                     vxk: 0, vx2: 0, vyk: 0, vy2: 0, vthetak: 0}

        self.dt = dt
        self.x_x, self.x_y, self.x_theta = x, y, theta
        self.x2_x, self.x2_y = x2, y2
        self.vl, self.vr = Vl, Vr
        self.w_Wl, self.w_Wr = wl, wr
        self.V_vxk, self.V_vx2, self.V_vyk, self.V_vy2, self.V_vthetak = vxk, vx2, vyk, vy2, vthetak

    def predict(self, u, dt):
        """ Prediction step of EKF.
        x = f(x,u,w=0)
        P = A @ P @ A' + W @ Q @ W'
        :u: 2x1 vector of left and right wheel speed in mm/s
        :dt: time elapsed
        """
    
        self.subs[self.dt] = dt

        self.subs[self.x_x] = self.pose[0,0]
        self.subs[self.x_y] = self.pose[1,0]
        self.subs[self.x_theta] = self.pose[2,0]

        self.subs[self.vl] = u[0]
        self.subs[self.vr] = u[1]

        self.subs[self.w_Wl] = 0
        self.subs[self.w_Wr] = 0

        # Update pose
        self.pose = np.array(self.f.evalf(subs=self.subs)).astype(float)  

        # Set the process noise
        w = np.random.normal(0, self.Q)
        self.subs[self.w_Wl] = w[0,0]
        self.subs[self.w_Wr] = w[1,1]

        A = np.array(self.A_j.evalf(subs=self.subs)).astype(float)        
        W = np.array(self.W_j.evalf(subs=self.subs)).astype(float)

        # Update covariance
        self.P = A @ self.P @ A.T + W @ self.Q @ W.T

    def update(self, z, pose2, dt):
        """ Update/Correction step of EKF.
        K = PH'(HPH' + VRV')^-1
        x = x + K(z - h(x,0))
        P = (I - KH)P
        :z: 2x1 measurement vector of range and bearing
        :pose2: 2x1 vector consisting of x2 and y2
        :dt: elapsed time
        """
        # Set poses
        self.subs[self.x_x] = self.pose[0,0]
        self.subs[self.x_y] = self.pose[1,0]
        self.subs[self.x_theta] = self.pose[2,0]

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

        H = np.array(self.H_j.evalf(subs=self.subs)).astype(float) 
        V = np.array(self.V_j.evalf(subs=self.subs)).astype(float) 
        K = self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + V @ self.R @ V.T)
        # print("K: ", K)
        self.pose += K @ self.residual(z, pose2)
        
        # Update covariance, equivalent to P = (I - KH)P
        self.P -= K @ H @ self.P

    def residual(self, z, pose2):
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

        # print("Pose2: ", pose2)
        # print("Pose: ", self.pose)

        innovation = np.array(self.h.evalf(subs=self.subs)).astype(float) 
        
        # print("Innovation: ", innovation)
        zx = z - innovation

        # Normalize bearing to [-pi, pi]
        zx[-1] = zx[-1] % (2 * np.pi)
        if zx[-1] > np.pi:             
            zx[-1] -= 2 * np.pi

        # print("Zx: ", zx)

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

ekf = RobotEKF(95.25)

landmarks = np.array([[60, -20], [20, 30], [40, 20]])
dt = 0.1

u = np.array([5, 12])

plt.figure()
if landmarks.size != 0:
    plt.scatter(landmarks[:, 0], landmarks[:, 1], marker='s', s=60)

track = []
for i in range(100):
    ekf.predict(u, dt)
    sim_pos = ekf.pose.copy()
    track.append(sim_pos)

    if i % 10 == 0:
        plot_covariance_ellipse(
            (ekf.pose[0,0], ekf.pose[1,0]), ekf.P[0:2, 0:2], 
                std=15, facecolor='k', alpha=0.3)

    for lmark in landmarks:
        z = ekf.z_landmark(lmark)
        ekf.update(z, lmark, dt)

    if i % 10 == 0:
        plot_covariance_ellipse(
            (ekf.pose[0,0], ekf.pose[1,0]), ekf.P[0:2, 0:2],
            std=15, facecolor='g', alpha=0.8)
            
track = np.array(track)
plt.plot(track[:, 0], track[:,1], color='k', lw=2)
plt.axis('equal')
plt.title("EKF Robot localization")
# plt.ylim([-100, 100])
# plt.xlim([-5, 80])
plt.show()