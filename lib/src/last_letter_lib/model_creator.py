import os
from math import pi
from pathlib import Path
from shutil import copyfile

import pcg_gazebo
import pcg_gazebo.parsers.sdf as sdf
import yaml

from last_letter_lib import default_params


MODELS_FOLDER = os.path.expanduser("~/last_letter_models/")


def traverse_sdf_collect(root, child_type, results):
    try:
        for child in root.children.values():
            if isinstance(child, child_type):  # This is a child_type.
                results.append(child)
            elif isinstance(child, list):  # This is a list of children.
                for item in child:
                    traverse_sdf_collect(item, child_type, results)
            else:  # This is a normal child.
                traverse_sdf_collect(child, child_type, results)
    except AttributeError:
        pass


def copy_meshes(mesh_list, output_dir):
    for mesh in mesh_list:
        mesh_file = Path(str(mesh.uri._value))
        new_path = output_dir / mesh_file.name
        copyfile(mesh_file, new_path)

        mesh.uri = str(new_path)


class Geometry:
    _sdf_desc = None

    def __init__(self):
        self._sdf_desc = sdf.create_sdf_element("geometry")


class Sphere(Geometry):
    def __init__(self, radius):
        super().__init__()
        self._sdf_desc.sphere = sdf.create_sdf_element("sphere")
        self._sdf_desc.sphere.radius = radius


class Box(Geometry):
    def __init__(self, dimensions):
        super().__init__()
        self._sdf_desc.box = sdf.create_sdf_element("box")
        self._sdf_desc.box.size = dimensions


class Cylinder(Geometry):
    def __init__(self, radius, length):
        super().__init__()
        self._sdf_desc.cylinder = sdf.create_sdf_element("cylinder")
        self._sdf_desc.cylinder.radius = radius
        self._sdf_desc.cylinder.length = length


class Mesh(Geometry):
    def __init__(self, uri, scale=[1, 1, 1]):
        super().__init__()
        if not Path(uri).exists:
            raise FileNotFoundError(f"The provided mesh {uri} does not exist")
        self._sdf_desc.mesh = sdf.create_sdf_element("mesh")
        self._sdf_desc.mesh.uri = (
            uri  # Refer to the mesh file relatively. It will exist next to the .sdf.
        )
        self._sdf_desc.mesh.scale = scale


class Collision:
    _sdf_desc = None

    def __init__(self, name):
        self._sdf_desc = sdf.create_sdf_element("collision")
        self._sdf_desc.name = name

    @property
    def name(self):
        return self._sdf_desc.name

    @property
    def pose(self):
        return self._sdf_desc.pose

    @pose.setter
    def pose(self, pose):
        self._sdf_desc.pose = pose

    def set_geometry(self, shape):
        self._sdf_desc.geometry = shape._sdf_desc


class Visual:
    _sdf_desc = None

    def __init__(self, name):
        self._sdf_desc = sdf.create_sdf_element("visual")
        self._sdf_desc.name = name
        material = sdf.create_sdf_element("material")
        material.name = "greenSprite"
        material.ambient = [0.1, 0.84, 0.06, 1]
        self._sdf_desc.material = material

    @property
    def name(self):
        return self._sdf_desc.name

    @property
    def pose(self):
        return self._sdf_desc.pose

    @pose.setter
    def pose(self, pose):
        self._sdf_desc.pose = pose

    def set_geometry(self, shape):
        self._sdf_desc.geometry = shape._sdf_desc


class Link:
    _sdf_desc = None

    def __init__(self, name):
        self._sdf_desc = sdf.create_sdf_element("link")
        self._sdf_desc.name = name

    @property
    def pose(self):
        return self._sdf_desc.pose

    @pose.setter
    def pose(self, pose):
        self._sdf_desc.pose = pose

    @property
    def name(self):
        return self._sdf_desc.name

    def set_mass(self, mass):
        self._sdf_desc.mass = mass
        self._sdf_desc.gravity = True

    def set_cog(self, cog):
        self._sdf_desc.center_of_mass = cog

    def set_inertia_ixx(self, value):
        self._sdf_desc.inertia.ixx = value

    def set_inertia_iyy(self, value):
        self._sdf_desc.inertia.iyy = value

    def set_inertia_izz(self, value):
        self._sdf_desc.inertia.izz = value

    def set_inertia_ixz(self, value):
        self._sdf_desc.inertia.ixz = value

    def add_collision(self, collision):
        self._sdf_desc.add_collision(collision._sdf_desc.name, collision._sdf_desc)

    def add_visual(self, visual):
        self._sdf_desc.add_visual(visual._sdf_desc.name, visual._sdf_desc)

    @property
    def kinematic(self):
        return self._sdf_desc.kinematic

    @kinematic.setter
    def kinematic(self, value):
        self._sdf_desc.kinematic = value


class Joint:
    _sdf_desc = None

    def __init__(self, name, joint_type):
        self._sdf_desc = sdf.create_sdf_element("joint")
        self._sdf_desc.name = name
        self._sdf_desc.type = joint_type

    @property
    def type(self):
        return self._sdf_desc.type

    @property
    def parent(self):
        return self._sdf_desc.parent

    @parent.setter
    def parent(self, parent):
        self._sdf_desc.parent = parent

    @property
    def child(self):
        return self._sdf_desc.child

    @child.setter
    def child(self, child):
        self._sdf_desc.child = child

    @property
    def pose(self):
        return self._sdf_desc.pose

    @pose.setter
    def pose(self, pose):
        self._sdf_desc.pose = pose


class Thruster:
    parameters = None
    link = None
    joint = None

    def __init__(self, name):
        self.parameters = dict()
        self.parameters["name"] = name
        self.parameters.update(default_params.default_thruster_params)

        # Create default thruster link
        self.link = Link(name)
        self.link.set_mass(0.1)
        self.link.set_inertia_ixx(0.01)
        self.link.set_inertia_iyy(0.01)
        self.link.set_inertia_izz(0.01)
        motor_geometry = Cylinder(0.04, 0.04)
        visual = Visual(name)
        visual.set_geometry(motor_geometry)
        visual.pose = [0, 0, 0, pi, pi / 2, 0]
        self.link.add_visual(visual)
        collision = Collision(name)
        collision.set_geometry(motor_geometry)
        collision.pose = [0, 0, 0, pi, pi / 2, 0]
        self.link.add_collision(collision)

        # Create default thruster joint
        self.joint = Joint(name + "_joint", "fixed")
        self.joint.parent = "body_flu"
        self.joint.child = self.link.name

    @property
    def name(self):
        return self.link.name

    @property
    def pose(self):
        return self.link.pose

    @pose.setter
    def pose(self, pose):
        self.link.pose = pose


class Airfoil:
    parameters = None
    link = None
    joint = None

    def __init__(self, name):
        self.parameters = dict()
        self.parameters["name"] = name
        self.parameters["aerodynamicsType"] = 0

        # Create airfoil link
        self.link = Link(name)
        self.link.set_mass(0.01)

        # Create airfoil joint
        self.joint = Joint(name + "_joint", "fixed")
        self.joint.parent = "body_frd"
        self.joint.child = self.link.name

    @property
    def pose(self):
        return self.link.pose

    @pose.setter
    def pose(self, pose):
        self.link.pose = pose


class AircraftBody(Airfoil):
    name = None
    parameters = dict()

    def __init__(self):
        # Do not initialize the same way as an airfoil

        # Add default aerodynamics
        self.parameters.update(default_params.default_aerodynamics_params)

        self.name = "body_frd"
        self.parameters["name"] = self.name

        # Create airfoil link
        self.link = Link(self.name)
        # TODO: Introduce default inertial features
        self.link.pose = [0, 0, 0, pi, 0, 0]

        # Create airfoil joint
        self.joint = Joint(self.name + "_joint", "fixed")
        self.joint.parent = "body_flu"
        self.joint.child = self.link.name


class Aircraft:
    _sdf_desc = None
    body = None
    airfoil_params = dict()
    thruster_params = dict()

    def __init__(self, name):
        self._sdf_desc = sdf.create_sdf_element("model")
        self._sdf_desc.name = name
        self._sdf_desc.static = "false"
        self._sdf_desc.self_collide = False

        # Attach the model plugin
        last_letter_plugin = sdf.create_sdf_element("plugin")
        last_letter_plugin.name = "gazebo_model_plugin"
        last_letter_plugin.filename = "libgazebo_model_plugin.so"
        self._sdf_desc.add_plugin("", plugin=last_letter_plugin)

        # build the necessary FLU and FRD frames
        root_link = Link("body_flu")
        root_link.set_mass(0.1)
        root_link.set_inertia_ixx(0.1)
        root_link.set_inertia_iyy(0.1)
        root_link.set_inertia_izz(0.1)
        self.add_link(root_link)  # First link doesn't need a joint

    def __str__(self):
        return self._sdf_desc.__str__()

    @property
    def name(self):
        return self._sdf_desc.name

    @property
    def sdf(self, name):
        return self._sdf_desc

    @property
    def pose(self):
        return self._sdf_desc.pose

    @pose.setter
    def pose(self, pose):
        self._sdf_desc.pose = pose

    def add_link(self, link):
        self._sdf_desc.add_link(link._sdf_desc.name, link._sdf_desc)

    def add_joint(self, joint):
        self._sdf_desc.add_joint(joint._sdf_desc.name, joint._sdf_desc)

    def add_thruster(self, thruster):
        self.add_link(thruster.link)
        self.add_joint(thruster.joint)
        self.thruster_params[thruster.name] = thruster.parameters

    def add_airfoil(self, airfoil):
        self.add_link(airfoil.link)
        self.add_joint(airfoil.joint)
        self.airfoil_params[airfoil.name] = airfoil.parameters

    def create_gazebo_model(self):
        models_dir = os.path.join(MODELS_FOLDER, "exported_models")
        model_dir = Path(models_dir) / self._sdf_desc.name
        if not os.path.exists(models_dir):
            os.makedirs(models_dir)
        if not os.path.exists(model_dir):
            os.makedirs(str(model_dir))

        # Copy over the meshes inside the model folder
        meshes = list()
        traverse_sdf_collect(self._sdf_desc, sdf.Mesh, meshes)
        copy_meshes(meshes, model_dir)

        gazebo_model = pcg_gazebo.simulation.SimulationModel.from_sdf(self._sdf_desc)
        gazebo_model.to_gazebo_model(output_dir=models_dir, overwrite=True)

    def create_model_folder(self, configs_folder):
        model_directory = os.path.join(configs_folder, self.name)
        if not os.path.exists(model_directory):
            os.makedirs(model_directory)
        return model_directory

    def dump_yaml(self, dictionary, full_filename):
        with open(full_filename, "w") as f:
            yaml.dump(dictionary, f)

    def create_environment_params(self, model_folder):
        full_filename = os.path.join(model_folder, "environment.yaml")
        self.dump_yaml(default_params.default_environment_params, full_filename)

    def create_world_params(self, model_folder):
        full_filename = os.path.join(model_folder, "world.yaml")
        self.dump_yaml(default_params.default_world_params, full_filename)

    def create_simulation_params(self, model_folder):
        full_filename = os.path.join(model_folder, "simulation.yaml")
        self.dump_yaml(default_params.default_simulation_params, full_filename)

    def create_aerodynamics_params(self, model_folder):
        full_filename = os.path.join(model_folder, "aerodynamics.yaml")
        params = dict()
        params["nWings"] = len(self.airfoil_params)
        for idx, airfoil_param_set in enumerate(self.airfoil_params.values()):
            params[f"airfoil{idx+1}"] = dict()
            for key, value in airfoil_param_set.items():
                params[f"airfoil{idx+1}"][f"{key}"] = value
        self.dump_yaml(params, full_filename)

    def create_propulsion_params(self, model_folder):
        full_filename = os.path.join(model_folder, "propulsion.yaml")
        params = dict()
        params["nMotors"] = len(self.thruster_params)
        for idx, thruster_param_set in enumerate(self.thruster_params.values()):
            params[f"motor{idx+1}"] = dict()
            for key, value in thruster_param_set.items():
                params[f"motor{idx+1}"][f"{key}"] = value
        self.dump_yaml(params, full_filename)

    def create_initialization_params(self, model_folder):
        full_filename = os.path.join(model_folder, "init.yaml")
        self.dump_yaml(default_params.default_initialization_params, full_filename)

    def set_home_coordinates(self, home_lat, home_lon):
        default_params.default_initialization_params["coordinates"][0] = home_lat
        default_params.default_initialization_params["coordinates"][1] = home_lon

    def create_ground_reaction_params(self, model_folder):
        full_filename = os.path.join(model_folder, "ground.yaml")
        self.dump_yaml(default_params.default_ground_reaction_params, full_filename)

    def create_randomizer_params(self, model_folder):
        full_filename = os.path.join(model_folder, "randomizer.yaml")
        self.dump_yaml(default_params.default_randomizer_params, full_filename)

    def create_inertial_params(self, model_folder):
        full_filename = os.path.join(model_folder, "inertial.yaml")
        self.dump_yaml(default_params.default_inertial_params, full_filename)

    def create_last_letter_model(self):
        configs_directory = os.path.expanduser(MODELS_FOLDER)
        model_directory = self.create_model_folder(configs_directory)
        self.create_world_params(configs_directory)
        self.create_simulation_params(configs_directory)
        self.create_environment_params(configs_directory)
        self.create_aerodynamics_params(model_directory)
        self.create_propulsion_params(model_directory)
        self.create_initialization_params(model_directory)
        self.create_ground_reaction_params(model_directory)
        self.create_inertial_params(model_directory)
        self.create_randomizer_params(model_directory)


class World:
    _sdf_desc = None
    _name = None
    _max_step_size = 0.004
    _real_time_factor = 1
    _real_time_update_rate = 250

    def __init__(self, name):
        self._name = name
        self._sdf_desc = sdf.create_sdf_element("world")
        self._sdf_desc.physics.reset(mode="ode", with_optional_elements=True)

    def set_step_size(self, size):
        self._max_step_size = size

    def set_sim_rate(self, rate):
        if self._max_step_size is None:
            raise ValueError("Apply set_step_size() first")
        else:
            self._real_time_update_rate = rate
            self._real_time_factor = self._real_time_update_rate * self._max_step_size

    def add_model(self, model):
        self._sdf_desc.add_model(model.name, model._sdf_desc)

    def attach_camera(self, model):
        visual_tracker = sdf.create_sdf_element("track_visual")
        visual_tracker.name = model.name
        visual_tracker.static = True
        visual_tracker.xyz = [-4.0, 0.0, 0.4]
        visual_tracker.inherit_yaw = True
        visual_tracker.min_dist = 15
        visual_tracker.max_dist = 30
        camera_gui = sdf.create_sdf_element("gui")
        camera = sdf.create_sdf_element("camera")
        camera_gui.camera = camera
        camera_gui.camera.track_visual = visual_tracker

        self._sdf_desc.gui = camera_gui

    def create_gazebo_world(self):
        self._sdf_desc.physics.max_step_size = self._max_step_size
        self._sdf_desc.physics.real_time_factor = self._real_time_factor
        self._sdf_desc.physics.real_time_update_rate = self._real_time_update_rate

        sky_sdf = sdf.create_sdf_element("sky")
        sky_sdf.sunset = 12
        scene_sdf = sdf.create_sdf_element("scene")
        scene_sdf.ambient = [0.5, 0.5, 0.5, 1]
        scene_sdf.sky = sky_sdf
        self._sdf_desc.scene = scene_sdf

        ros_state_plugin = sdf.create_sdf_element("plugin")
        ros_state_plugin.name = "gazebo_ros_state"
        ros_state_plugin.filename = "libgazebo_ros_state.so"
        ros_state_plugin.from_dict({"update_rate": self._real_time_update_rate})
        self._sdf_desc.add_plugin("", plugin=ros_state_plugin)

        gazebo_world = pcg_gazebo.simulation.World.from_sdf(self._sdf_desc)
        output_dir = os.path.join(
            os.path.expanduser(os.path.join(MODELS_FOLDER, "worlds"))
        )
        gazebo_world.export_to_file(
            output_dir=output_dir, filename=self._name, overwrite=True
        )
