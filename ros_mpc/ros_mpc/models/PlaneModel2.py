import numpy as np

import casadi as ca

from matplotlib import pyplot as plt

"""
Model based off of 
Deep reinforcement learning‑based air combat maneuver decision‑making: 
literature review, implementation tutorial and future direction


Xinwei Wang1 · Yihui Wang1 · Xichao Su2 · Lei Wang3 · Chen Lu4,5,6 · Haijun Peng1 · Jie Liu7
"""

class PlaneModel2():
    def __init__(self, 
                 include_time:bool=False,
                 dt_val:float=0.05) -> None:
        self.include_time = include_time
        self.dt_val = dt_val
        self.define_states()
        self.define_controls() 
        
    def define_states(self):
        """define the states of your system"""
        #positions ofrom world
        self.x_f = ca.SX.sym('x_f')
        self.y_f = ca.SX.sym('y_f')
        self.z_f = ca.SX.sym('z_f')

        #attitude
        self.phi_f = ca.SX.sym('phi_f')
        self.theta_f = ca.SX.sym('theta_f')
        self.psi_f = ca.SX.sym('psi_f')
        self.v = ca.SX.sym('t')

        if self.include_time:
            self.states = ca.vertcat(
                self.x_f,
                self.y_f,
                self.z_f,
                self.phi_f,
                self.theta_f,
                self.psi_f, 
                self.v)
        else:
            self.states = ca.vertcat(
                self.x_f,
                self.y_f,
                self.z_f,
                self.phi_f,
                self.theta_f,
                self.psi_f,
                self.v 
            )

        self.n_states = self.states.size()[0] #is a column vector 

    def define_controls(self):
        """controls for your system"""
        self.load_x = ca.SX.sym('load_x')
        self.load_z = ca.SX.sym('load_z')
        self.u_phi = ca.SX.sym('u_phi')
        self.v_cmd = ca.SX.sym('v_cmd')

        self.controls = ca.vertcat(
            self.load_x,
            self.load_z,
            self.u_phi, #
            self.v_cmd
        )
        self.n_controls = self.controls.size()[0] 

    def set_state_space(self):
        """define the state space of your system"""
        self.g = 9.81 #m/s^2
        #body to inertia frame 
        self.x_fdot = self.v_cmd * ca.cos(self.theta_f) * ca.cos(self.psi_f) 
        self.y_fdot = self.v_cmd * ca.cos(self.theta_f) * ca.sin(self.psi_f)
        self.z_fdot = -self.v_cmd * ca.sin(self.theta_f)
        
        self.phi_fdot   = self.load_x 
        self.theta_fdot = (self.g/self.v)*(self.load_z*ca.cos(self.u_phi) - ca.cos(self.theta_f))
        
        #check if the denominator is zero
        # self.v_cmd = ca.if_else(self.v_cmd == 0, 1e-6, self.v_cmd)
        self.psi_fdot = self.g*self.load_z*ca.sin(self.u_phi)/(self.v*ca.cos(self.theta_f))
        self.v_dot = self.g*(self.load_x - ca.sin(self.theta_f))        

        
        if self.include_time:
            self.z_dot = ca.vertcat(
                self.x_fdot,
                self.y_fdot,
                self.z_fdot,
                self.phi_fdot,
                self.theta_fdot,
                self.psi_fdot,
                self.v_dot
            )
        else:
            self.z_dot = ca.vertcat(
                self.x_fdot,
                self.y_fdot,
                self.z_fdot,
                self.phi_fdot,
                self.theta_fdot,
                self.psi_fdot,
                self.v_dot
            )

        #ODE function
        self.function = ca.Function('f', 
            [self.states, self.controls], 
            [self.z_dot])
        
        
    def rk45(self, x, u, dt, use_numeric:bool=True):
        """
        Runge-Kutta 4th order integration
        x is the current state
        u is the current control input
        dt is the time step
        use_numeric is a boolean to return the result as a numpy array
        """
        k1 = self.function(x, u)
        k2 = self.function(x + dt/2 * k1, u)
        k3 = self.function(x + dt/2 * k2, u)
        k4 = self.function(x + dt * k3, u)
        
        next_step = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
        
        #return as numpy row vector
        if use_numeric:
            next_step = np.array(next_step).flatten()
            return next_step
        else:
            return x + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
