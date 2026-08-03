.. _usage:

===============================================================================
Usage
===============================================================================


.. Standalone
.. ===============================================================================

.. *last_letter* offers a few commands that can be run in a console, standalone:

.. Complete command documentation
.. -------------------------------------------------------------------------------

.. This is the documentation for all the commands provided by the *last_letter* executable.

.. .. click:: last_letter.__main__:cli
..    :prog: last_letter
..    :nested: full

.. Export to Excel
.. -------------------------------------------------------------------------------

.. As hinted above, it is possible to export one or all Aircraft recipes into an
.. Excel document.

.. For example, using the following command all recipes will be exported in a folder
.. named ``output`` in the Excel format.

.. ``last_letter export -o output -f excel``


.. As part of another Python project
.. ===============================================================================

.. *last_letter* is most useful as a submodule of another Python project.
.. Import the ``last_letter`` module and load an :class:`~last_letter.Aircraft` recipe using
.. the :func:`~last_letter.load_aircraft` function.
.. Then, use it to access the parameters of its members.

.. Example:

.. .. code-block:: python

..    from last_letter import last_letter

..    # Load the aircraft recipe
..    v3 = last_letter.load_aircraft('v3')

..    # Access the battery internal resistance
..    r_bat = v3.batteries[0].spec.resistance

.. You can find out which attributes are provided by an :class:`~last_letter.Aircraft`
.. object and its members in the API documentation: :mod:`last_letter`.

.. Since the model variants (e.g. the various :class:`~last_letter.Thruster` subclasses)
.. don't provide exactly the same members, it is best to check the actual type of the
.. member you are trying to access or enclose your code in ``try`` blocks.


.. Writing an Aircraft specification
.. ===============================================================================

.. .. important::

..     All the values in this package are expected to be given in **S.I. units**,
..     including the angles in radians. Otherwise, an explicit unit will be specified.

.. When you need to modify an existing aircraft or model specification or create a
.. new one there are some simple rules to follow:

.. #. You are able to create new models in the form of .yaml files and then refer to them by name.
.. #. Define only model attributes that are supported by the corresponding specification. You do not need to define optional attributes.
.. #. Before using your new or edited recipes, validate them by running ``last_letter validate`` in the *last_letter* root folder.

.. .. _derived_aircraft:

.. Writing a Derived Aircraft specification
.. ===============================================================================

.. *last_letter* allows specifying *Derived Aircraft* which are based on existing Aircraft but alter the base recipe in
.. speficic ways.
.. Use-cases of this feature are defining additional payload configurations for the same aircraft or experimenting with
.. alternate propulsion setups.

.. To create a Derived Aircraft, first create a new ``.yaml`` file in the ``aircraft`` database. The content of this file
.. should be a :class:`last_letter.DerivedAircraft` model:

.. .. autoclass:: last_letter.DerivedAircraft
..     :members:
..     :noindex:

.. Basically, the Derived Aircraft decription is a series of edits to the main :class:`last_letter.Component` groups, either
.. replacing their contents altogether or appending to them or deleting a whole group.
.. These edits can be chained together.

.. The available actions are:

.. .. autoclass:: last_letter.ModificationActionEnum
..     :members:
..     :noindex:

.. .. note::
..     If the parent configuration hasn't defined an optional group, use the ``replace`` action to add it. ``append`` will not work.
