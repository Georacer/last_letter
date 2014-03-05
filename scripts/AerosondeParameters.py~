#!/usr/bin/env python

#Parameters for the Aerosonde UAV
#as provided in the book Small Unmanned Aircraft - Theory and Practice by Beard and McLain
#modifications in some signs have been made. Please compare with the original
import rospy


#World Parameters
def worldParams():
	rospy.set_param('simRate', 100)
	rospy.set_param('world/rho', 1.2682) #Air density
	rospy.set_param('world/oswald', 0.9)
	rospy.set_param('world/g',9.81)

#Airframe Aerodynamic Parameters
def planeParams():
	rospy.set_param('airframe/m', 13.5)
	rospy.set_param('airframe/j_x', 0.8244)
	rospy.set_param('airframe/j_y', 1.135)
	rospy.set_param('airframe/j_z', 1.759)
	rospy.set_param('airframe/j_xz', 0.1204)
	rospy.set_param('airframe/s', 0.55)
	rospy.set_param('airframe/b', 2.8956)
	rospy.set_param('airframe/c', 0.18994)
	rospy.set_param('airframe/c_lift_0', 0.28)
	rospy.set_param('airframe/c_drag_0', 0.03)
	rospy.set_param('airframe/c_m_0', -0.02338)
	rospy.set_param('airframe/c_lift_a', 3.45)
	rospy.set_param('airframe/c_drag_a', 0.3)
	rospy.set_param('airframe/c_m_a', -0.38)
	rospy.set_param('airframe/c_lift_q', 0)
	rospy.set_param('airframe/c_drag_q', 0)
	rospy.set_param('airframe/c_m_q', -3.6)
	rospy.set_param('airframe/c_lift_deltae', 0.36)
	rospy.set_param('airframe/c_drag_deltae', 0)
	rospy.set_param('airframe/c_m_deltae', -0.5)
	rospy.set_param('airframe/mcoeff', 50)
	rospy.set_param('airframe/alpha_stall', 0.4712)
	rospy.set_param('airframe/eps', 0.1592)
	rospy.set_param('airframe/c_drag_p', 0.0437)
	rospy.set_param('airframe/c_n_deltar', -0.032)
	rospy.set_param('airframe/c_y_0', 0)
	rospy.set_param('airframe/c_l_0', 0)
	rospy.set_param('airframe/c_n_0', 0)
	rospy.set_param('airframe/c_y_b', -0.98)
	rospy.set_param('airframe/c_l_b', -0.12)
	rospy.set_param('airframe/c_n_b', 0.25)
	rospy.set_param('airframe/c_y_p', 0)
	rospy.set_param('airframe/c_l_p', -0.26)
	rospy.set_param('airframe/c_n_p', 0.022)
	rospy.set_param('airframe/c_y_r', 0)
	rospy.set_param('airframe/c_l_r', 0.14)
	rospy.set_param('airframe/c_n_r', -0.35)
	rospy.set_param('airframe/c_y_deltaa', 0)
	rospy.set_param('airframe/c_l_deltaa', 0.08)
	rospy.set_param('airframe/c_n_deltaa', 0.06)
	rospy.set_param('airframe/c_y_deltar', 0.17)
	rospy.set_param('airframe/c_l_deltar', 0.105)

#Airframe Propulsion Parameters
def motorParams():
	rospy.set_param('motor/s_prop', 0.2027)
	rospy.set_param('motor/k_motor', 80)
	rospy.set_param('motor/k_t_p', 0)
	rospy.set_param('motor/k_omega', 0)
	rospy.set_param('motor/c_prop', 1.0)


if __name__=="__main__":
	worldParams()
	planeParams()
	motorParams()
