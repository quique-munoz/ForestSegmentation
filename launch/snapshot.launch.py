from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ---- Launch args (con defaults) ----
    params_file = LaunchConfiguration('params_file')
    period_sec  = LaunchConfiguration('period_sec')
    slop_sec    = LaunchConfiguration('sync_slop_sec')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('forest_segmentation'),
            'config',
            'snapshot.yaml'
        ]),
        description='Ruta al YAML de parámetros para snapshot_saver_action_param'
    )

    declare_period = DeclareLaunchArgument(
        'period_sec',
        default_value='10.0',
        description='Periodo (s) entre snapshots solicitados por el cliente'
    )

    declare_slop = DeclareLaunchArgument(
        'sync_slop_sec',
        default_value='0.08',
        description='Slop (s) para la sincronización aproximada en el servidor/cliente'
    )

    # ---- (action server) ----
    server = Node(
        package='forest_segmentation',
        executable='snapshot_action',
        name='snapshot_saver_action',
        output='screen',
        parameters=[params_file], 
    )

    # ---- (action client) ----
    client = Node(
        package='forest_segmentation',
        executable='snapshot_client',
        name='snapshot_client',
        output='screen',
        parameters=[{
            'period_sec': period_sec,
            'sync_slop_sec': slop_sec,
        }]
    )

    return LaunchDescription([
        declare_params_file,
        declare_period,
        declare_slop,
        server,
        client
    ])
