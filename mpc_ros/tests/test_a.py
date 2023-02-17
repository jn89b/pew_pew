from mpc_ros import MPC
from mpc_ros.CasadiModels import FlatQuadModel 
import numpy as np
import pytest
#https://realpython.com/pytest-python-testing/


@pytest.fixture
def quad_model():
    model = FlatQuadModel.FlatQuadcopterModel()
    return model

@pytest.fixture
def good_mpc_params(quad_model):
    mpc_params = {
        'model': quad_model,
        'N': 10,
        'dt_val': 0.1,
        'Q': np.diag([1, 1, 1, 1, 1, 1, 1, 1]),
        'R': np.diag([1, 1, 1, 1])
    }
    return mpc_params


@pytest.fixture
def setup_quad_mpc_constraints(good_mpc_params):

    mpc_ref = MPC.MPC(good_mpc_params)
    mpc_ref.__initDecisionVariables()
    mpc_ref.defineBoundaryConstraints()
    good_quad_constraints = {
        'vx_max': 15.0, #m/s
        'vx_min': -15.0, #m/s

        'vy_max': 15.0, #m/s
        'vy_min': -15.0, #m/s

        'vz_max': 5.0, #m/s
        'vz_min': -5.0, #m/s

        'psi_dot_max': np.rad2deg(5.0),#rad/s
        'psi_dot_min': np.rad2deg(-5.0),#rad/s

        'z_min': 5.0, #m
        'z_max': 20.0, #m

        'thrust_max': 10.0, #N
        'thrust_min': 0.0, #N
    }

    mpc_ref.lbx['U'][0,:] = good_quad_constraints['thrust_min']
    mpc_ref.ubx['U'][0,:] = good_quad_constraints['thrust_max']

    mpc_ref.lbx['U'][1,:] = good_quad_constraints['thrust_min']
    mpc_ref.ubx['U'][1,:] = good_quad_constraints['thrust_max']

    mpc_ref.lbx['U'][2,:] = good_quad_constraints['thrust_min']
    mpc_ref.ubx['U'][2,:] = good_quad_constraints['thrust_max']

    mpc_ref.lbx['U'][3,:] = good_quad_constraints['thrust_min']
    mpc_ref.ubx['U'][3,:] = good_quad_constraints['thrust_max']

    mpc_ref.lbx['X'][4,:] = good_quad_constraints['vx_min']
    mpc_ref.ubx['X'][4,:] = good_quad_constraints['vx_max']

    mpc_ref.lbx['X'][5,:] = good_quad_constraints['vy_min']
    mpc_ref.ubx['X'][5,:] = good_quad_constraints['vy_max']

    mpc_ref.lbx['X'][6,:] = good_quad_constraints['vz_min']
    mpc_ref.ubx['X'][6,:] = good_quad_constraints['vz_max']

    mpc_ref.lbx['X'][7,:] = good_quad_constraints['psi_dot_min']
    mpc_ref.ubx['X'][7,:] = good_quad_constraints['psi_dot_max']

    return mpc_ref

def test_good_mpc_param_inputs(good_mpc_params):
    mpc_params = good_mpc_params
    mpc = MPC.MPC(good_mpc_params)
    assert mpc.N == mpc_params['N']
    assert mpc.dt_val == mpc_params['dt_val']
    #check if Q and R are the same
    assert np.array_equal(mpc.Q, mpc_params['Q'])
    assert np.array_equal(mpc.R, mpc_params['R'])


def test_bad_mpc_param_inputs():
    #test if bad inputs are caught
    bad_mpc_params = {
        'model': None,
        'N': 10,
        'dt_val': 0.1,
        'Q': np.diag([1, 1, 1, 1, 1, 1, 1, 1]),
        'R': np.diag([1, 1, 1, 1])
    }

    with pytest.raises(AssertionError):
        mpc = MPC.MPC(bad_mpc_params)


