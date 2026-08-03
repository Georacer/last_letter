#!/usr/bin/python
import ctypes as ct
import os
import time
from math import pi

import numpy as np


print(f"Currently working on {os.getcwd()}")


class TrimmerInput:
    dll = None
    obj = None
    trim_func = None
    input_type = ct.c_double * 6
    output_type = ct.POINTER(ct.c_double)
    trim_input = None

    def __init__(self, uav_name):
        # Load the lib_trimmer dll
        dll_path = os.path.expanduser(
            "~/ros_workspaces/uav_ftc/devel/lib/liblib_trimmer.so"
        )
        self.dll = ct.cdll.LoadLibrary(dll_path)

        # Set the exposed functions data types
        self.dll.trimmer_input_new.argtypes = [ct.c_char_p]
        self.dll.trimmer_input_new.restype = ct.c_void_p
        self.trim_func = self.dll.find_input_trim
        self.dll.find_input_trim.argtypes = [ct.c_void_p, self.input_type]
        self.dll.find_input_trim.restype = self.output_type
        self.print_optimal_result = self.dll.print_optimal_input_result
        self.dll.print_optimal_input_result.argtypes = [ct.c_void_p]
        self.dll.print_optimal_input_result.restype = ct.c_double

        self.uav_name = uav_name
        self.obj = self.dll.trimmer_input_new(uav_name)

    # Convert trim states from a numpy array of [phi, theta, Va, alpha, beta ,r]
    # to a suitable input type for find_trim()
    def convert_trim_states(self, trim_states_np):
        return self.input_type(*trim_states_np)

    def convert_trim_controls(self, trim_ctrls_ct):
        trim_ctrls = np.zeros(6)
        for i in range(6):
            trim_ctrls[i] = trim_ctrls_ct[i]
        return trim_ctrls

    def find_trim_input(self, trim_states):
        trim_states_ct = self.convert_trim_states(trim_states)
        trim_input_ct = self.trim_func(self.obj, trim_states_ct)
        self.trim_input = self.convert_trim_controls(trim_input_ct)
        return self.trim_input

    def print_result(self):
        self.print_optimal_result(self.obj)


class TrimmerState:
    dll = None
    obj = None
    trim_func = None
    n_args = 7  # Number of optimization arguments
    input_type = ct.c_double * (
        n_args + 2
    )  # Plus 2 values for optimization cost and success flag
    output_type = ct.POINTER(ct.c_double)
    trim_state = None

    def __init__(self, uav_name):
        # Load the lib_trimmer dll
        # This is for the standalone cmake build
        # self.dll = ct.cdll.LoadLibrary('/home/george/ros_workspaces/uav_ftc/src/last_letter/last_letter_lib/build/liblib_trimmer.so')
        # This is for the catkin-made build
        dll_path = os.path.expanduser(
            "~/ros_workspaces/uav_ftc/devel/lib/liblib_trimmer.so"
        )
        self.dll = ct.cdll.LoadLibrary(dll_path)

        # Set the exposed functions data types
        self.dll.trimmer_state_new.argtypes = [ct.c_char_p]
        self.dll.trimmer_state_new.restype = ct.c_void_p

        self.trim_func = self.dll.find_state_trim
        self.dll.find_state_trim.argtypes = [ct.c_void_p, self.input_type]
        self.dll.find_state_trim.restype = self.output_type

        self.print_optimal_result = self.dll.print_optimal_state_result
        self.dll.print_optimal_state_result.argtypes = [ct.c_void_p]
        self.dll.print_optimal_state_result.restype = ct.c_double

        self.set_parameter_func = self.dll.set_parameter
        self.dll.set_parameter.argtypes = [
            ct.c_void_p,
            ct.c_int,
            ct.c_char_p,
            ct.c_double,
        ]
        self.dll.set_parameter.restype = ct.c_bool

        self.update_model_func = self.dll.update_model
        self.dll.update_model.argtypes = [ct.c_void_p]
        self.dll.update_model.restype = None

        self.uav_name = uav_name
        self.obj = self.dll.trimmer_state_new(uav_name)

    def set_model_parameter(self, param_type, name, value):
        # print('Received new model parameter: {}/{}:{}'.format(param_type, name, value))
        # Set the model parameters. They will not take effect (be written)
        # param_type defined as:
        # typedef enum {
        #     PARAM_TYPE_WORLD = 0,
        #     PARAM_TYPE_ENV,
        #     PARAM_TYPE_INIT,
        #     PARAM_TYPE_INERTIAL,
        #     PARAM_TYPE_AERO,
        #     PARAM_TYPE_PROP,
        #     PARAM_TYPE_GROUND
        # } ParamType_t;
        return self.set_parameter_func(self.obj, param_type, name, value)

    def update_model(self):
        # Update (write) the model paramters
        self.update_model_func(self.obj)

    # Convert trim trajectory from a numpy array of [Va, gamma, R]
    # to a suitable input type for find_trim()
    def convert_trim_trajectory(self, trim_trajectory_np):
        return self.input_type(*trim_trajectory_np)

    def convert_trim_state(self, trim_state_ct):
        trim_state = np.zeros(self.n_args + 2)
        for i in range(self.n_args + 2):
            trim_state[i] = trim_state_ct[i]
        return trim_state

    def find_trim_values(self, trim_trajectory):
        trim_trajectory_ct = self.convert_trim_trajectory(trim_trajectory)
        trim_state_ct = self.trim_func(self.obj, trim_trajectory_ct)
        self.trim_state = self.convert_trim_state(trim_state_ct)
        return self.trim_state

    def print_result(self):
        self.print_optimal_result(self.obj)


if __name__ == "__main__":

    print("Testing TrimmerState class functionality")
    uav_name = "skywalker_2013_mod"
    trimmer = TrimmerState(uav_name)
    print("Successfully loaded Trimmer object")

    # Use this to tweak model parameters
    # trimmer.set_model_parameter(5, 'motor1/k_motor', 0) # Zero-out motor output
    # trimmer.update_model()

    trim_states = np.array([10, np.deg2rad(-15), np.inf])
    # Va, gamma, R
    print("Obtaining trim input: 1000 repetitions")
    t_start = time.time()
    for i in range(1000):
        trim_state = trimmer.find_trim_values(trim_states)
    t_end = time.time()

    print("Trim found:\n" + f"{trim_state}")
    trimmer.print_result()
    print(f"Time required: {t_end - t_start}s")
    print("\n")

    # print('Testing TrimmerInput class functionality')
    # uav_name = 'skywalker_2013_mod'
    # trimmer = TrimmerInput(uav_name)
    # print('Successfully loaded Trimmer object')

    # trim_states = np.array([0, 0*pi/180, 15, -0.05, 0, 0])
    # # phi, theta, Va, alpha, beta, r
    # print('Obtaining trim input: 1000 repetitions')
    # t_start = time.time()
    # for i in range(1000):
    #     trim_ctrls = trimmer.find_trim_input(trim_states)
    # t_end = time.time()

    # print("Trim found:\n" + "{}".format(trim_ctrls))
    # trimmer.print_result()
    # print("Time required: {}s".format(t_end-t_start))
