.. last_letter documentation master file, created by
   sphinx-quickstart on Wed Sep 16 16:56:01 2015.
   You can adapt this file completely to your liking, but it should at least
   contain the root ``toctree`` directive.

===========================================
Welcome to the documentation of last_letter
===========================================

last_letter is a collection of `ROS <http://ros.org/>`_ packages suitable for Unmanned Aerial Vehicle research, control algorithm development, non-linear physics simulation, Software-in-the-Loop (SIL) simulation and Hardware-in-the-Loop (HIL) simulation. It is developed by George Zogopoulos-Papaliakos.

Gitter chatroom
===============
A Gitter chatroom dedicated to last_letter can be found in `this link <https://gitter.im/Georacer/last_letter>`_. Omnipresence is not guaranteed.

Scientific background
=====================
Great effort has been exerted for the simulator to be as realistic as possible. To that goal, the project incorporates bibliography related to flight dynamics, physics modeling and sensor modeling. A congreggated document discussing the modeling of a fixed-wing UAV can be found in the `Modeling a Fixed-Wing UAV <https://github.com/Georacer/uav-modeling>`_ (uav-modeling) Github project. It is a LaTeX project with a single .pdf output document.
Although uav-modeling is independent of last_letter, it will often be referred to in the documentation.

License
=======
The last_letter software project, which is a collection of ROS packages, uses the `GPLv3 license <http://choosealicense.com/licenses/gpl-3.0/>`_.

Screenshots
===========

.. figure:: _static/images/cover_init.*

    Avionics panel and visualization of an aircraft pre-flight

.. figure:: _static/images/cover_takeoff.*

    An aircraft 3D model during take-off

.. figure:: _static/images/cover_rosgraph.*

    An indicative rqt_graph showing the node and topic layout of the simulator


.. toctree::
	:hidden:

	getting_started/index
	launching/index
	scenarios/index
	theory_elements/index
	Parameters <parameters/index>
	utilities/index
	contributors