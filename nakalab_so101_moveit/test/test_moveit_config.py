from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import yaml


JOINTS = [
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
    'gripper',
]
ARM_JOINTS = JOINTS[:-1]


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE_ROOT / 'config'
LAUNCH = PACKAGE_ROOT / 'launch'


def load_yaml(name):
    return yaml.safe_load((CONFIG / name).read_text(encoding='utf-8'))


def generated_urdf():
    controller_share = Path(get_package_share_directory('nakalab_so101_controller'))
    result = subprocess.run(
        [
            'xacro',
            str(controller_share / 'urdf' / 'nakalab_so101_controlled.urdf.xacro'),
            'use_sim_hardware:=true',
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(result.stdout)


def group_state(root, group):
    for state in root.findall('group_state'):
        if state.attrib == {'name': 'zero', 'group': group}:
            return {joint.attrib['name']: float(joint.attrib['value'])
                    for joint in state.findall('joint')}
    raise AssertionError(f'Missing zero state for {group}')


def test_srdf_matches_urdf():
    urdf = generated_urdf()
    srdf = ET.parse(CONFIG / 'nakalab_so101.srdf').getroot()
    links = {link.attrib['name'] for link in urdf.findall('link')}
    joints = {joint.attrib['name'] for joint in urdf.findall('joint')}

    assert srdf.attrib['name'] == urdf.attrib['name']
    groups = {group.attrib['name']: group for group in srdf.findall('group')}
    assert {'so101_follower', 'so101_gripper', 'so101'} <= groups.keys()
    assert 'so101_all' not in groups
    assert 'zero' not in groups

    chain = groups['so101_follower'].find('chain')
    assert chain is not None
    assert chain.attrib == {
        'base_link': 'arm_base_link',
        'tip_link': 'gripper_frame_link',
    }
    assert chain.attrib['base_link'] in links
    assert chain.attrib['tip_link'] in links

    assert group_state(srdf, 'so101_follower') == dict.fromkeys(ARM_JOINTS, 0.0)
    assert group_state(srdf, 'so101_gripper') == {'gripper': 0.0}
    assert group_state(srdf, 'so101') == dict.fromkeys(JOINTS, 0.0)

    for group in srdf.findall('group'):
        for joint in group.findall('joint'):
            assert joint.attrib['name'] in joints
        for chain_element in group.findall('chain'):
            assert chain_element.attrib['base_link'] in links
            assert chain_element.attrib['tip_link'] in links
    end_effector = srdf.find('end_effector')
    assert end_effector is not None
    assert end_effector.attrib['name'] == 'so101_end_effector'
    assert end_effector.attrib['group'] == 'so101_gripper'
    assert end_effector.attrib['parent_group'] == 'so101_follower'
    assert end_effector.attrib['parent_link'] == 'gripper_link'
    assert end_effector.attrib['parent_link'] in links


def test_kinematics_configuration():
    kinematics = load_yaml('kinematics.yaml')
    follower = kinematics['so101_follower']
    assert follower['kinematics_solver'] == 'pick_ik/PickIkPlugin'
    assert follower['rotation_scale'] == 0.0
    assert follower['position_scale'] > 0.0
    assert follower['position_threshold'] > 0.0
    assert 'so101' not in kinematics
    assert 'so101_gripper' not in kinematics


def test_joint_limits_configuration():
    limits = load_yaml('joint_limits.yaml')
    assert 0 < limits['default_velocity_scaling_factor'] <= 1
    assert 0 < limits['default_acceleration_scaling_factor'] <= 1
    assert set(limits['joint_limits']) == set(JOINTS)
    for name, limit in limits['joint_limits'].items():
        assert limit['has_velocity_limits'] is True, name
        assert limit['max_velocity'] == 1.0, name
        assert limit['has_acceleration_limits'] is True, name
        assert limit['max_acceleration'] == 2.0, name
        assert 'min_position' not in limit and 'max_position' not in limit, name


def test_ompl_configuration():
    ompl = load_yaml('ompl_planning.yaml')
    assert ompl['planning_plugin'] == 'ompl_interface/OMPLPlanner'
    assert 'RRTConnectkConfigDefault' in ompl['planner_configs']
    for group in ('so101_follower', 'so101_gripper', 'so101'):
        assert ompl[group]['default_planner_config'] == 'RRTConnectkConfigDefault'
        assert ompl[group]['planner_configs'] == ['RRTConnectkConfigDefault']


def test_controller_mapping_matches_ros2_control():
    moveit_controllers = load_yaml('moveit_controllers.yaml')
    controller_share = Path(get_package_share_directory('nakalab_so101_controller'))
    ros2_controllers = yaml.safe_load(
        (controller_share / 'config' / 'ros2_controllers.yaml').read_text(encoding='utf-8')
    )
    manager = moveit_controllers['moveit_simple_controller_manager']
    names = manager['controller_names']
    assert moveit_controllers['moveit_manage_controllers'] is False
    assert set(names) == {
        'so101_follower_controller',
        'so101_gripper_controller',
    }
    mapped_joints = []
    for name in names:
        mapped = manager[name]
        configured = ros2_controllers[name]['ros__parameters']['joints']
        assert mapped['type'] == 'FollowJointTrajectory'
        assert mapped['action_ns'] == 'follow_joint_trajectory'
        assert mapped['joints'] == configured
        mapped_joints.extend(mapped['joints'])
    assert len(mapped_joints) == len(set(mapped_joints))
    assert set(mapped_joints) == set(JOINTS)
    assert len(manager['so101_follower_controller']['joints']) == 5
    assert manager['so101_gripper_controller']['joints'] == ['gripper']


def test_launch_contract_and_rviz_configuration():
    launch_text = (LAUNCH / 'moveit.launch.py').read_text(encoding='utf-8')
    sim_text = (LAUNCH / 'sim.launch.py').read_text(encoding='utf-8')
    all_launch_text = launch_text + sim_text
    assert 'MoveIt' + 'ConfigsBuilder' not in all_launch_text
    assert 'moveit_configs_utils' + '.launches' not in all_launch_text
    assert 'use_sim_hardware' in launch_text
    assert 'use_' + 'fake_hardware' not in all_launch_text
    assert 'controller.launch.py' in launch_text
    assert "executable='move_group'" in launch_text
    assert 'gaze' + 'bo' not in all_launch_text.lower()
    assert "'use_sim_hardware': 'true'" in sim_text
    assert not (LAUNCH / 'demo.launch.py').exists()
    assert not (LAUNCH / 'real.launch.py').exists()

    rviz = (CONFIG / 'moveit.rviz').read_text(encoding='utf-8')
    for value in (
        'rviz_default_plugins/Grid',
        'moveit_rviz_plugin/MotionPlanning',
        'moveit_rviz_plugin/Trajectory',
        'Planning Group: so101_follower',
        'Planning Scene Topic: /monitored_planning_scene',
        'Fixed Frame: world',
        'Robot Description: robot_description',
        'Topic: /display_planned_path',
        'Loop Animation: false',
    ):
        assert value in rviz
