default_environment_params = {
    # Generic Parameters
    "rho": 1.1120,  # Air density in kg/m^3
    # Parameters for the atmoshpere model
    "groundTemp": 25,  # in Celsius
    "groundPres": 1013.25,  # in mBar
    # Parameters for the wind model
    "windRef": 10.0,  # in m/s
    "windRefAlt": 60.0,  # in m
    "windDir": 90.0,  # in degrees
    "surfSmooth": 0.28,  # rural environment
    "Dryden": {
        "Lu": 533.0,
        "Lw": 533.0,
        "sigmau": 155.0,
        "sigmaw": 78.0,  # Less powerful than XY because it causes very strong up/downdrafts
        "use": False,  # produce turbulence
        "randomizeSeed": False,  # Randomize Dryden noise with current time seed
    },
}

default_simulation_params = {
    "kinematics_engine": 0,  # 0 for internal kinematics, 1 for external
}

default_aerodynamics_params = {
    "name": "airfoil_1_link",
    "aerodynamicsType": 3,
    "s": 0.92,
    "b": 2.40,
    "c": 0.40,
    "c_L_0": 0.1296,
    "c_L_deltae": 0,  # -0.502,
    "c_L_alpha": 3.80,
    "c_L_qn": 0,
    "mcoeff": 50,
    "oswald": 0.9,
    "alpha_stall": 0.38,
    "c_D_0": 0.0359,
    "c_D_qn": 0,
    "c_D_deltae": 0.0,
    "c_D_p": 0.04,  # Probably parasitic drag not used
    "c_D_alpha": 0.5,  # Linearization from quadratic to operational point of 4deg
    "c_Y_0": 0,
    "c_Y_beta": -0.224,
    "c_Y_pn": -0.137,
    "c_Y_rn": -0.0839,
    "c_Y_deltaa": 0,
    "c_Y_deltar": 0,  # opposite sign than c_n_deltar
    "c_l_0": 0,
    "c_l_pn": -0.404,
    "c_l_beta": -0.0849,
    "c_l_rn": -0.0555,
    "c_l_deltaa": 0.131,
    "c_l_deltar": 0.0,
    "c_m_0": 0.004,
    "c_m_alpha": -0.2,
    "c_m_qn": -1.3,
    "c_m_deltae": 0.283,
    "c_n_0": 0,
    "c_n_beta": 0.0283,
    "c_n_pn": 0.022,
    "c_n_rn": -0.012,
    "c_n_deltaa": -0.00339,  # Adverse yaw
    "c_n_deltar": 0.0,  # opposite sign than c_y_deltar
    "deltaa_max": 0.3491,
    "deltae_max": 0.3491,
    "deltar_max": 0.3491,
    "deltaa_max_nominal": 0.3491,
    "deltae_max_nominal": 0.3491,
    "deltar_max_nominal": 0.3491,
    "chanAileron": 0,
    "chanElevator": 1,
    "chanRudder": 3,
}

default_thruster_params = {
    "chanMotor": -1,
    "rotationDir": 1.0,
    "motorType": 5,
    "propDiam": 16 * 2.54 / 100,  # 16 inch props
    "engInertia": 200e-6,
    "Kv": 8.17,
    "Rm": 0.1,
    "Rs": 0.022,
    "Cells": 6,
    "I0": 0.5,
    "RadPSLimits": [0.01, 1000],
    "propThrustPoly": {
        "polyType": 0,
        "polyNo": 5,
        "coeffs": [0.0737, -0.4778, 2.2161, -5.5296, 6.2749, -2.6331],
    },
    "propThrustMultiplier": 1.10,
    "propPowerPoly": {
        "polyType": 0,
        "polyNo": 1,
        "coeffs": [0.0289, -0.0405],
    },
    "momentumDragCoeff": 0.0047,
}

# TODO deprecate this
default_ground_reaction_params = {
    "groundReactionType": 0,
}

# TODO deprecate this, it's not used
default_inertial_params = {
    "m": 2,
    "j_x": 0.8,
    "j_y": 1.1,
    "j_z": 1.76,
    "j_xz": 0.1,
}

# TODO deprecate this
default_randomizer_params = {
    "std_dev": 0,
    "aerodynamics": [],
    "ground": [],
    "inertial": [],
    "propulsion": [],
    "init": [],
}

# TODO deprecate this
default_world_params = {
    "simRate": 400,
    "deltaT": 0.0025,
    "timeControls": 0,
    "integratorType": 0,
}

# TODO deprecate this
default_initialization_params = {
    # Initial State
    "coordinates": [
        37.889063,
        23.731863,
        0.0,
    ],  # latitude, longitude, altitude coordinates. Altitude for now should be ground elevation
    "position": [0.0, 0.0, -0.3],  # NED frame
    "orientation": [
        -0.0000,
        0.0039,
        0.0000,
        1.0000,
    ],  # NED frame Northwards, quaternion written as [x, y, z, w],
    "velLin": [10.0, 0.0, 0.0],  # body frame
    "velAng": [0.000, 0.000, 0.000],  # body frame
    "ctrlInput": [0.0, 0.0, 0.0, 0.0],  # Starting control input, normalized range
    "chanReset": 9,
    "chanPause": 8,
}
