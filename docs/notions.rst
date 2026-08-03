===============================================================================
Notions
===============================================================================

This page presents the core notions behind the *last_letter* package.

Aircraft
===============================================================================

An *Aircraft* is a class that refers to a specific configuration. It is considered
complete, i.e. there is no missing information in an Aircraft specification, for
the purposes of this package.
However, note that one can define an aircraft in varying levels of detail.

An Aircraft comprises of multiple *Components*.

.. autoclass:: last_letter.simulation.Aircraft
    :members:
    :noindex:

.. tip::

    Throughout this documentation, it is often said that an Aircraft specification
    is build to a *Recipe*. These terms are equivalent.


Component
===============================================================================

A *Component* is a physical part or subassembly of an Aircraft. Its primary
information is its location in terms of the Aircraft.
Optionally, it can hold :class:`~systems.Inertial` information and a :class:`~systems.Mesh`
for the visual appearance.

.. autoclass:: last_letter.systems.Component
    :members:
    :noindex:

A Component is the base class for more specialized parts such as :class:`~aerodynamics.Aerodynamic`,
:class:`~propulsion.Thruster` and :class:`~propulsion.Battery`.

Databases
===============================================================================

All data pertinent to Aircraft, Components and other models resides in .yaml files,
within the ``data`` folder.
Under this folder, the following subfolders exist:

``aircraft``: This is where all the top-level aircraft definitions exist. One file
is responsible for specifying a single, whole aircraft.

``aerodynamics``: This is where all aerodynamic data exists. Each specification file can
refer to the lumped aerodynamic model of an aircraft or an individual airfoil.
These models can be a :class:`~systems.Component` of an :class:`~simulation.Aircraft`.

``battery``: This is where all battery data exists. Each specification file can
refer to a single battery model.
These models can be a :class:`~systems.Component` of an :class:`~simulation.Aircraft`.

``motor``: This is where all motor data exists. Each specification file can refer to
an individual motor.
These models can be an attribute of a :class:`~propulsion.Thruster`, if the latter supports it.

``propeller``: This is where all propeller data exists. Each specification file can refer to
an individual propeller.
These models form each :class:`~propulsion.Propeller`, which can be an attribute of a :class:`~propulsion.Thruster`, if the latter supports it.

``mesh``: This subfolder holds all the visual meshes (typically .stl files), so
that they are part of the database and they can be refered to by the Components that use them.

Schema
===============================================================================

*last_letter* enforces a rigid schema on how the Aircraft and other model information
should be entered in the .yaml files.
Thus, any third party software that depends on *last_letter* can enjoy a unified and
stable API. Accessing information across different Aircraft should be the same.

Still, there is enough room for flexibility. For example, a :class:`~propulsion.Thruster`
can be specified as a :class:`~propulsion.ThrusterSimple`, a :class:`~propulsion.ThrusterBeard`,
a :class:`~propulsion.ThrusterElectric` or a :class:`~propulsion.ThrusterSpeedControlled`.
Each specification shares the same basic parameters but also provides different
specialized parameters, according to its underlying model.

Under the hood, the schema is defined as a class of the `pydantic https://pydantic-docs.helpmanual.io/`
Python package, that specializes in data validation and settings managment.
Pydantic provides amenities for defining mandatory and optional members of a
configuration, as well as their data types.

Derived Aircraft
===============================================================================

It is often useful to describe an aircraft configuration based on an existing Aircraft description.
This allows for defining multiple payload configurations without having to duplicate the aircraft description,
which also prevents database maintenance issues.

To find out how to define a Derived Aircraft, please read :ref:`derived_aircraft`.

Validation
===============================================================================

Pydantic loads the dictionaries found in the .yaml files and filters them through
the :class:`~simulation.Aircraft` definition and its members.
If a dictionary structure or content does not comply with the Aircraft definition,
then an error is raised.
Thus, it is impossible to accidentally enter the data in a wrong format.

Frames or reference
===============================================================================

Several frames of reference are employed for specifying an Aircraft and its Components.

At the root of an Aircraft definition lies the **Aircraft Frame**. This is a right-hand,
cartesian *FRD* frame. Its x-axis points Forward, along the longitudinal axis of the aircraft.
Its z-axis points Downwards and its y-axis points to the exterior product of the two, towards the Right.
The selection of the origin of this frame is arbitrary.

The **Aerodynamic Frame** is where the forces and moments of an airfoil are calculated.
It is defined relative to the Aircraft Frame. It should not be defined by the Center of Mass
of the aircraft, as this can move independently of the aerodynamic layout.
When the Aerodynamic Frame refers to the lumped aircraft aerodynamics, it is aligned with the
Aircraft Frame. The origin is placed, in terms of the Aircraft frame at (-chord/4 of the mid span
wing airfoil, 0, z coordinate of the chord/4 point of the mid span wing airfoil).

Each thruster has its own **Thruster Frame**. For thrusters with no mass, the frame
is placed on the point of wrench (force and torque) application. For thrusters with
mass, it is placed on the thruster center of mass.
The x-axis of the Thruster Frame points towards the direction of generated thrust.
The other two axes can be oriented at will.

The **Mesh Frame** of each component can be placed at will, relative to a component frame.
It is used to translate and orient a mesh file to its visually correct location.
It is helpful since the CAD model of a mesh doesn't always use the same coordinate system as
the Component frame.

The **Body Frame**, as typically used in the UAV and robotics literature, is considered
on top of the Center of Mass of the aircraft. Since the Center of Mass can move depending
on the component distribution, it is not defined in *last_letter*, but is a derivative of
an Aircraft recipe.
