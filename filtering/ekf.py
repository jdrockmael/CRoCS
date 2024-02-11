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

# Function for h 
def Hx(x, april_pose, v):
    """ takes a state variable and returns the measurement
    that would correspond to that state.
    """
    xk, yk, thetak = x[0, 0], x[1, 0], x[2, 0]
    x2, y2= april_pose[0], april_pose[1]

    vxk, vx2, vyk, vy2, vtheta = v[0], v[1], v[2], v[3], v[4]
    xk = xk + vxk
    yk = yk + vyk
    x2 = x2 + vx2
    y2 = y2 + vy2
    thetak = thetak + vtheta
    
    Hx = np.array([[math.sqrt((y2 + vy2 - yk - vyk)**2 + (x2 + vx2 -xk - vxk)**2)],
                [thetak + vtheta - math.atan2((y2 + vy2 -yk - vyk),(x2 + vx2-xk - vxk))]])
    return Hx

def z_landmark(lmark, sim_pos, std_range, std_bearing, v):
    xk, yk, thetak = sim_pos[0,0], sim_pos[1, 0], sim_pos[2, 0]
    x2, y2= lmark[0], lmark[1]

    vxk, vx2, vyk, vy2, vtheta = v[0], v[1], v[2], v[3], v[4]
    xk = xk + vxk
    yk = yk + vyk
    x2 = x2 + vx2
    y2 = y2 + vy2
    thetak = thetak + vtheta

    rng = math.sqrt((y2 + vy2 - yk - vyk)**2 + (x2 + vx2 -xk - vxk)**2)
    bearing = thetak + vtheta - math.atan2((y2 + vy2 -yk - vyk),(x2 + vx2-xk - vxk))
    z = np.array([[rng +  randn() * std_range],
                [bearing + randn() * std_bearing]])
    return z

# Function for H matrix, jacobian of h w.r.t pose
def H_of(x, april_pose, v):
    """ compute Jacobian of H matrix where h(x) computes 
    the range and bearing to a landmark for state x """

    xk, yk, thetak = x[0, 0], x[1, 0], x[2, 0]
    x2, y2 = april_pose[0], april_pose[1]

    vxk, vx2, vyk, vy2, vtheta = v[0], v[1], v[2], v[3], v[4]
    xk = xk + vxk
    yk = yk + vyk
    x2 = x2 + vx2
    y2 = y2 + vy2
    thetak = thetak + vtheta

    H = np.array([[-((x2-xk)/math.sqrt((y2-yk)**2 + (x2-xk)**2)), -((y2-yk)/math.sqrt((y2-yk)**2 + (x2-xk)**2)), 0],
                [-(y2-yk)/((y2-yk)**2 + (x2-xk)**2), -(x2-xk)/((y2-yk)**2 + (x2-xk)**2), 1]])
    return H

def residual(a, b):
    """ compute residual (a-b) between measurements containing 
    [xc, yc, phi]. Yaw is normalized to [-pi, pi)"""
    y = a - b
    y[1] = y[1] % (2 * np.pi)    # force in range [0, 2 pi)
    if y[1] > np.pi:             # move to [-pi, pi)
        y[1] -= 2 * np.pi
    return y

class RobotEKF(EKF):
    def __init__(self, dt, wheelbase, std_WL, std_WR):
        EKF.__init__(self, 3, 2, 2)
        self.dim_z = 2
        self.dim_x = 3
        self.dt = dt
        self.wheelbase = wheelbase
        self.std_WL = std_WL
        self.std_WR = std_WR

        x, y, theta, Vl, Vr, t, x2, y2, b = sympy.symbols('x, y, theta, Vl, Vr, t, x2, y2, b')
        wr, wl = sympy.symbols('wr, wl')
        vxk, vx2, vyk, vy2, vtheta = sympy.symbols('vxk, vx2, vyk, vy2, vtheta')

        self.f = Matrix(
            [[x + t*(Vr + wr + Vl + wl)/2 * sympy.cos(theta)],
            [y + t*(Vr + wr + Vl + wl)/2 * sympy.sin(theta)],
            [theta + t*(Vr + wr - (Vl + wl))/b]]
        )

        self.h = Matrix(
            [[sympy.sqrt((y2+vy2-y-vyk)**2+(x2+vx2-x-vxk)**2)],
             [theta - sympy.atan2(y2+vyk-y-vyk,x2+vx2-x-vxk)]]
        )

        process_noise = Matrix([wl, wr])
        measurement_noise = ([vxk, vx2, vyk, vy2, vtheta])
        pose = Matrix([x, y, theta])

        self.H_j = self.h.jacobian(pose)
        self.V_j = self.h.jacobian(measurement_noise)
        self.A_j = self.f.jacobian(pose)
        self.W_j= self.f.jacobian(process_noise)

        self.subs = {x:0, y: 0, theta:0, t:self.dt, Vl:0, Vr:0, b:self.wheelbase, 
                     wl: 0, wr:0, vxk: 0, vx2: 0, vyk: 0, vy2: 0, vtheta: 0, x2:0, y2:0}
        self.x_x, self.x_y, self.x_theta = x, y, theta
        self.x2_x, self.x2_y = x2, y2
        self.Vl, self.Vr = Vl, Vr
        self.Wl, self.Wr = wl, wr
        self.vxk, self.vx2, self.vyk, self.vy2, self.vtheta = vxk, vx2, vyk, vy2, vtheta
    
    def predict(self, u, w):
        self.x = self.move(self.x, u, self.dt, w)
        self.subs[self.x_x] = self.x[0,0]
        self.subs[self.x_y] = self.x[1,0]
        self.subs[self.x_theta] = self.x[2,0]

        self.subs[self.Vl] = u[0]
        self.subs[self.Vr] = u[1]

        self.subs[self.Wl] = w[0]
        self.subs[self.Wr] = w[1]

        A = np.array(self.A_j.evalf(subs=self.subs)).astype(float)        
        W = np.array(self.W_j.evalf(subs=self.subs)).astype(float)

        Q = np.array([[self.std_WL**2, 0],
                      [0, self.std_WR**2]])
        
        self.P = A @ self.P @ A.T + W @ Q @ W.T

    def move(self, x, u, dt, w):
        vl, vr = u[0], u[1]
        wl, wr = w[0], w[1]
        theta = x[2,0]

        dx = np.array([[dt*(vr + wr + vl + wl)/2 * cos(theta)],
                       [dt*(vr + wr + vl + wl)/2 * sin(theta)],
                       [dt*(vr + wr - (vl + wl))/self.wheelbase]])
        return x + dx 
     
    
def ekf_update(ekf, z, landmark, v):
    ekf.update(z, HJacobian=H_of, Hx=Hx,residual=residual,args=(landmark, v), hx_args=(landmark, v))

dt = 0.1

def run_localization(landmarks, std_WL, std_WR, std_range, std_bearing, step=1, ellipse_step=10, v_m=[0,0,0,0,0], w_m=[0,0], ylim=None):
    ekf = RobotEKF(dt, wheelbase=0.5, std_WL=std_WL, std_WR=std_WR)
    ekf.x = np.array([[2, 6, 0]]).T
    ekf.P = np.diag([0, 0.1, 0.1])
    ekf.R = np.diag([std_range**2, std_bearing**2])
    
    v = v_m
    
    sim_pos = ekf.x.copy()
    u = np.array([1, 1.01])

    plt.figure()
    plt.scatter(landmarks[:, 0], landmarks[:, 1], marker='s', s=60)

    track = []
    for i in range(200):
        sim_pos = ekf.move(sim_pos, u, dt, w=w_m) # simulate robot
        track.append(sim_pos)

        if i % step == 0:
            ekf.predict(u=u, w=w_m)

            if i % ellipse_step == 0:
                plot_covariance_ellipse(
                    (ekf.x[0,0], ekf.x[1,0]), ekf.P[0:2, 0:2], 
                     std=6, facecolor='k', alpha=0.3)

            for lmark in landmarks:
                z = z_landmark(lmark, sim_pos, std_range, std_bearing, v)
                ekf_update(ekf, z, lmark, v)

            if i % ellipse_step == 0:
                plot_covariance_ellipse(
                    (ekf.x[0,0], ekf.x[1,0]), ekf.P[0:2, 0:2],
                    std=6, facecolor='g', alpha=0.8)
                
    track = np.array(track)
    plt.plot(track[:, 0], track[:,1], color='k', lw=2)
    plt.axis('equal')
    plt.title("EKF Robot localization")
    # plt.ylim([0, 30])
    if ylim is not None: plt.ylim(*ylim)
    plt.show()
    return ekf

# landmarks = np.array([[4, 10], [10, 17], [15, 15]])
# landmarks = np.array([[4, 10], [10, 17]])

landmarks = np.array([[4, 10]])
# landmarks = np.array([])

# Noise/weight for Vl and Vr in update step
w = [0.1, 0.1]
# Noise/weight for xk, x2, yk, y2, thetak in correction step
v = [0.1, 0.1, 0.1, 0.1, 0.1] 


ekf = run_localization(landmarks, std_WL=0.1, std_WR=0.1, std_range=0.2, std_bearing=0.1, v_m=v, w_m=w)
print(ekf.x)