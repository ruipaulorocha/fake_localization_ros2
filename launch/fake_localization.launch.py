import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription

from launch.actions import (DeclareLaunchArgument, GroupAction,
	OpaqueFunction, SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import PushRosNamespace, Node

def generate_launch_description():
	# launch configuration variables
	use_namespace = LaunchConfiguration('use_namespace')
	namespace =		LaunchConfiguration('namespace')
	delta = {
		'x':   LaunchConfiguration('delta_x'),
		'y':   LaunchConfiguration('delta_y'),
		'yaw': LaunchConfiguration('delta_yaw'),
	}

	# declare launch arguments
	use_namespace_arg = DeclareLaunchArgument(
		'use_namespace',
		default_value = 'false',
		description = 'Whether to apply a namespace')
	namespace_arg = DeclareLaunchArgument(
		'namespace',
		default_value = '',
		description = 'Top-level namespace')
	delta_x_arg = DeclareLaunchArgument(
		'delta_x',
		default_value = '0.0',
		description = 'xx offset between <namespace>/odom and /map')
	delta_y_arg = DeclareLaunchArgument(
		'delta_y',
		default_value = '0.0',
		description = 'yy offset between <namespace>/odom and /map')
	delta_yaw_arg = DeclareLaunchArgument(
		'delta_yaw',
		default_value = '0.0',
		description = 'delta_yaw offset between <namespace>/odom and /map')

	# opaque functions
	def get_prefix(context):
		if context.launch_configurations['use_namespace'] == 'true':
			prefix = context.launch_configurations['namespace']
		else:
			prefix = ''
		return [SetLaunchConfiguration('frame_id_prefix', prefix)]

	def get_odom_frame_id(context):
		frame_id = os.path.join(context.launch_configurations['frame_id_prefix'], 'odom')
		return [SetLaunchConfiguration('odom_frame_id', frame_id)]

	def get_base_frame_id(context):
		frame_id = os.path.join(context.launch_configurations['frame_id_prefix'], 'base_link')
		return [SetLaunchConfiguration('base_frame_id', frame_id)]

	get_prefix_fn = OpaqueFunction(function = get_prefix)
	get_odom_frame_id_fn = OpaqueFunction(function = get_odom_frame_id)
	get_base_frame_id_fn = OpaqueFunction(function = get_base_frame_id)

    # specify the actions
	cmd_group = GroupAction([
		# use_namespace_arg,
		# namespace_arg,
		# get_prefix_fn,
		PushRosNamespace(
			condition = IfCondition(use_namespace),
			namespace = namespace
		),
		Node(
			package = 'fake_localization_ros2',
			executable='fake_localization',
			name='fake_localization',
			parameters=[
				{
				'odom_frame_id': [LaunchConfiguration('odom_frame_id') ],
				'base_frame_id': [LaunchConfiguration('base_frame_id') ],
				'global_frame_id': '/map',
				'delta_x':   delta['x'],
				'delta_y':   delta['y'],
				'delta_yaw': delta['yaw']
				}
			]#,
            #arguments=['--ros-args', '--log-level', 'DEBUG'] # to increase the logger level from INFO to DEBUG
		)
	])

    # create the launch description and populate
	ld = LaunchDescription()

	# arguments
	ld.add_action(use_namespace_arg)
	ld.add_action(namespace_arg)
	ld.add_action(delta_x_arg)
	ld.add_action(delta_y_arg)
	ld.add_action(delta_yaw_arg)

	# opaque functions
	ld.add_action(get_prefix_fn)
	ld.add_action(get_odom_frame_id_fn)
	ld.add_action(get_base_frame_id_fn)

	# actions
	ld.add_action(cmd_group)

	return ld
