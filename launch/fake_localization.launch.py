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
	use_sim_time_arg = DeclareLaunchArgument(
		'use_sim_time',
		default_value = 'true',
		description = 'Whether to use simulated time')
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
	odom_topic_arg = DeclareLaunchArgument(
		'odom_topic',
		default_value = 'odom',
		description = 'odom topic')
	odom_frame_id_arg = DeclareLaunchArgument(
		'odom_frame_id',
		default_value = 'odom',
		description = 'odom frame id')
	global_frame_id_arg = DeclareLaunchArgument(
		'global_frame_id',
		default_value = 'map',
		description = 'global frame id')
	base_frame_id_arg = DeclareLaunchArgument(
		'base_frame_id',
		default_value = 'base_link',
		description = 'base frame id')
	tf_tolerance_arg = DeclareLaunchArgument(
		'transform_tolerance',
		default_value = '0.1',
		description = 'TF tolerance')	

	# opaque functions
	def get_prefix(context):
		if context.launch_configurations['use_namespace'] == 'true':
			prefix = context.launch_configurations['namespace']
		else:
			prefix = ''
		return [SetLaunchConfiguration('frame_id_prefix', prefix)]

	def get_odom_frame_id(context):
		frame_id = os.path.join(context.launch_configurations['frame_id_prefix'],
			context.launch_configurations['odom_frame_id'])
		return [SetLaunchConfiguration('odom_frame_id_with_prefix', frame_id)]

	def get_base_frame_id(context):
		frame_id = os.path.join(context.launch_configurations['frame_id_prefix'],
			context.launch_configurations['base_frame_id'])
		return [SetLaunchConfiguration('base_frame_id_with_prefix', frame_id)]

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
				'odom_topic':		[LaunchConfiguration('odom_topic') ],
				'odom_frame_id':	[LaunchConfiguration('odom_frame_id_with_prefix') ],
				'base_frame_id':	[LaunchConfiguration('base_frame_id_with_prefix') ],
				'global_frame_id':	[LaunchConfiguration('global_frame_id') ],
				'delta_x':			delta['x'],
				'delta_y':			delta['y'],
				'delta_yaw':		delta['yaw'],
				'transform_tolerance': [LaunchConfiguration('transform_tolerance') ],
				'use_sim_time':		[LaunchConfiguration('use_sim_time') ]
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
	ld.add_action(use_sim_time_arg)
	ld.add_action(delta_x_arg)
	ld.add_action(delta_y_arg)
	ld.add_action(delta_yaw_arg)
	ld.add_action(odom_topic_arg)
	ld.add_action(odom_frame_id_arg)
	ld.add_action(global_frame_id_arg)
	ld.add_action(base_frame_id_arg)
	ld.add_action(tf_tolerance_arg)	

	# opaque functions
	ld.add_action(get_prefix_fn)
	ld.add_action(get_odom_frame_id_fn)
	ld.add_action(get_base_frame_id_fn)

	# actions
	ld.add_action(cmd_group)

	return ld
