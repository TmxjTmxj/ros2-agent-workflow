from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from smartcar_bringup.controller_core import ControllerConfig


MODEL = Path(__file__).resolve().parents[1] / "models" / "hospital_amr" / "model.sdf"
WORLD = Path(__file__).resolve().parents[1] / "worlds" / "hospital_world_gz10.world"
PACKAGE_XML = Path(__file__).resolve().parents[1] / "src" / "smartcar_bringup" / "package.xml"
BRIDGE_CONFIG = Path(__file__).resolve().parents[1] / "config" / "ros_gz_bridge.yaml"
ROBOT_PROFILE = Path(__file__).resolve().parents[3] / "profiles" / "robots" / "hospital-amr.yaml"


def _pose(element):
    return [float(value) for value in (element.findtext("pose") or "0 0 0 0 0 0").split()]


def test_wheel_collision_and_joint_use_the_official_turtlebot_layout():
    """Keep wheel geometry aligned with the ROBOTIS TurtleBot3 SDF pattern."""
    model = ET.parse(MODEL).getroot().find("model")
    for side in ("left", "right"):
        link = model.find(f"link[@name='wheel_{side}_link']")
        joint = model.find(f"joint[@name='wheel_{side}_joint']")
        assert link is not None and joint is not None
        assert abs(_pose(joint)[1]) == pytest.approx(0.08)
        assert _pose(link.find("inertial"))[:3] == pytest.approx(_pose(joint)[:3])
        assert _pose(link.find("collision"))[:3] == pytest.approx(_pose(joint)[:3])
        assert float(link.findtext("collision/geometry/cylinder/radius")) == pytest.approx(0.033)
        assert float(link.findtext("collision/geometry/cylinder/length")) == pytest.approx(0.018)


def test_diff_drive_uses_official_burger_kinematics_and_emits_odometry():
    """Use Gazebo Sim 9's real wheel-joint driver and its documented odom tag."""
    model = ET.parse(MODEL).getroot().find("model")
    diff = model.find("plugin[@name='gz::sim::systems::DiffDrive']")
    odometry = model.find("plugin[@name='gz::sim::systems::OdometryPublisher']")

    assert model.find("plugin[@name='gz::sim::systems::VelocityControl']") is None
    assert diff is not None
    assert diff.findtext("left_joint") == "wheel_left_joint"
    assert diff.findtext("right_joint") == "wheel_right_joint"
    assert float(diff.findtext("wheel_separation")) == pytest.approx(0.160)
    assert float(diff.findtext("wheel_radius")) == pytest.approx(0.033)
    assert diff.findtext("topic") == "cmd_vel"
    assert diff.findtext("odom_topic") == "odom"
    assert diff.findtext("frame_id") == "odom"
    assert diff.findtext("child_frame_id") == "base_footprint"
    assert diff.findtext("odom_publish_frequency") == "30"
    assert diff.find("odom_publisher_frequency") is None
    assert odometry is None


def test_profile_controller_and_diff_drive_share_conservative_burger_limits():
    model = ET.parse(MODEL).getroot().find("model")
    diff = model.find("plugin[@name='gz::sim::systems::DiffDrive']")
    declared = yaml.safe_load(ROBOT_PROFILE.read_text(encoding="utf-8"))["limits"]
    expected = {
        "max_linear_velocity": 0.22,
        "max_angular_velocity": 1.0,
        "max_linear_acceleration": 0.5,
        "max_angular_acceleration": 1.0,
    }
    controller = ControllerConfig()

    assert declared == expected
    assert {
        "max_linear_velocity": controller.max_linear,
        "max_angular_velocity": controller.max_angular,
        "max_linear_acceleration": controller.max_linear_acceleration,
        "max_angular_acceleration": controller.max_angular_acceleration,
    } == expected
    assert {
        key: float(diff.findtext(key))
        for key in expected
    } == expected


def test_base_collision_matches_official_burger_mesh_scale():
    model = ET.parse(MODEL).getroot().find("model")
    collision = model.find("link[@name='base_link']/collision[@name='base_chassis']")
    assert _pose(collision)[:3] == pytest.approx([-0.032, 0.0, 0.070])
    size = [float(value) for value in collision.findtext("geometry/box/size").split()]
    assert size == pytest.approx([0.14, 0.14, 0.14])


def test_ball_caster_supports_chassis_at_wheel_contact_height():
    """Catches the chassis dragging because the official ball caster is missing."""
    model = ET.parse(MODEL).getroot().find("model")
    caster = model.find("link[@name='caster_back_link']")
    joint = model.find("joint[@name='caster_back_joint']")
    assert caster is not None and joint is not None
    assert joint.get("type") == "ball"
    assert joint.findtext("parent") == "base_link"
    assert joint.findtext("child") == "caster_back_link"
    radius = float(caster.findtext("collision/geometry/sphere/radius"))
    caster_pose = _pose(caster)
    wheel = model.find("link[@name='wheel_left_link']/collision")
    wheel_pose = _pose(wheel)
    wheel_radius = float(wheel.findtext("geometry/cylinder/radius"))
    assert caster_pose[:3] == pytest.approx([-0.081, 0.0, -0.004])
    assert radius == pytest.approx(0.005)
    assert abs((caster_pose[2] - radius) - (wheel_pose[2] - wheel_radius)) <= 0.002


def test_decorative_floor_tiles_do_not_create_impassable_steps():
    world = ET.parse(WORLD).getroot().find("world")
    floor_models = [model for model in world.findall("model") if model.get("name", "").startswith("floor_")]

    assert floor_models
    assert all(model.find("link/collision") is None for model in floor_models)


def test_camera_is_above_the_chassis_and_uses_a_regular_forward_view():
    model = ET.parse(MODEL).getroot().find("model")
    camera = model.find("link[@name='base_link']/sensor[@name='front_camera']")

    assert camera is not None
    assert camera.get("type") == "camera"
    assert _pose(camera)[2] >= 0.16
    assert float(camera.findtext("camera/horizontal_fov")) == pytest.approx(1.396)
    assert camera.findtext("visualize") == "false"
    assert float(camera.findtext("update_rate")) <= 10.0


def test_lds_matches_official_burger_envelope_and_scan_density():
    model = ET.parse(MODEL).getroot().find("model")
    scan_link = model.find("link[@name='base_scan']")
    collision = scan_link.find("collision[@name='lidar_sensor_collision']")
    sensor = scan_link.find("sensor[@name='hls_lfcd_lds']")

    assert _pose(sensor)[:3] == pytest.approx([-0.032, 0.0, 0.171])
    assert float(collision.findtext("geometry/cylinder/radius")) == pytest.approx(0.0508)
    assert float(collision.findtext("geometry/cylinder/length")) == pytest.approx(0.055)
    assert sensor.findtext("lidar/scan/horizontal/samples") == "360"


def test_world_visual_materials_use_valid_sdf_nesting():
    world = ET.parse(WORLD).getroot().find("world")

    assert world.findall(".//visual/material/material") == []


def test_body_contact_sensor_and_world_contact_system_are_enabled():
    model = ET.parse(MODEL).getroot().find("model")
    world = ET.parse(WORLD).getroot().find("world")
    expected = {
        "base_chassis",
        "lidar_sensor_collision",
        "wheel_left_collision",
        "wheel_right_collision",
        "caster_collision",
    }
    contacts = model.findall("link/sensor[@type='contact']")

    assert {sensor.findtext("contact/collision") for sensor in contacts} == expected
    assert all(sensor.find("topic") is None for sensor in contacts)
    assert world.find("plugin[@name='gz::sim::systems::Contact']") is not None

    bridges = yaml.safe_load(BRIDGE_CONFIG.read_text())
    contact_bridges = [
        item for item in bridges if item["ros_topic_name"] == "/hospital_amr/contacts"
    ]
    assert len(contact_bridges) == len(expected)
    assert all("/sensor/" in item["gz_topic_name"] for item in contact_bridges)


def test_ros_package_declares_direct_runtime_dependencies():
    root = ET.parse(PACKAGE_XML).getroot()
    dependencies = {element.text for element in root.findall("depend")}

    assert {"ament_index_python", "rosgraph_msgs"}.issubset(dependencies)
