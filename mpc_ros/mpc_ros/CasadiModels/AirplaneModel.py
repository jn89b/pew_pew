#!/usr/bin/env python3
import casadi as ca

"""
Path-following control for small fixed-wing unmanned aerial vehicles under wind disturbances
"""

class AirplaneSimpleModel():
    def __init__(self):
        self.define_states()
        self.define_controls()

    def define_states(self):
        """define the states of your system"""
        #positions ofrom world
        self.x_f = ca.SX.sym('x_f')
        self.y_f = ca.SX.sym('y_f')
        self.z_f = ca.SX.sym('z_f')

        #attitude
        self.theta_f = ca.SX.sym('theta_f')
        self.psi_f = ca.SX.sym('psi_f')

        self.states = ca.vertcat(
            self.x_f,
            self.y_f,
            self.z_f,
            self.theta_f,
            self.psi_f
        )

        self.n_states = self.states.size()[0] #is a column vector 

