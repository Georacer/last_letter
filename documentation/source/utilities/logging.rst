Logging
=======

last_letter uses the ROS `bag system <http://wiki.ros.org/Bags>`_ to record logs.

The ``record`` executable of the ``rosbag`` package is called in the ``last_letter/launch/simulation.launch`` file with the argument

.. code-block:: xml

	<node if="$(arg log)" pkg="rosbag" name="recorder" type="record" args="-O UAV_recording.bag -a"/> <!-- topic activity recorder node -->

, provided that the ``log`` argument of the launch file is set to ``true``.

This command starts a `rosbag/record <http://wiki.ros.org/rosbag/Commandline#record>`_ executable with the name ``recorder``, with the arguments ``-O UAV_recording.bag -a``.
The arguments tells the executable to write a ``.bag`` file wit the name ``UAV_recording.bag`` and puts in there **all** of the topics which appear in the simulation.

The ``.bag`` file is stored in the ``~/.ros`` directory.

You can extract the information of each topic in the ``.bag`` file separately, or replay them to playback the simulation. These functions are described in the ROS documentation pages of the ``.bag`` files.
