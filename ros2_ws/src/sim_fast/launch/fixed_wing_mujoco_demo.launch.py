from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_delay = DeclareLaunchArgument(
        'start_delay_sec', default_value='3.0',
        description='Delay before the demonstration arms and publishes its mission.')
    wind_x = DeclareLaunchArgument(
        'wind_x_mps', default_value='0.0',
        description='World-frame east wind component.')
    wind_y = DeclareLaunchArgument(
        'wind_y_mps', default_value='0.0',
        description='World-frame north wind component.')
    viewer = DeclareLaunchArgument(
        'viewer', default_value='false',
        description='Open the live MuJoCo viewer (chase camera). If no '
                    'display/OpenGL is available the plant keeps running headless.')
    airframe = DeclareLaunchArgument(
        'airframe', default_value='twin_wing',
        description='twin_wing | single_wing (single-motor pusher flying wing).')
    airframe_spec = DeclareLaunchArgument(
        'airframe_spec', default_value='',
        description='Twin-wing airframe YAML; empty uses the bundled '
                    'aerial_kit/airframes/twin_wing/twin_wing.yaml.')
    estimator_mode = DeclareLaunchArgument(
        'estimator_mode', default_value='truth',
        description='truth or ekf; ekf inserts simulated GPS, baro and IMU.')
    estimator_seed = DeclareLaunchArgument(
        'estimator_seed', default_value='7',
        description='Seed for deterministic simulated sensor noise.')

    return LaunchDescription([
        start_delay,
        wind_x,
        wind_y,
        viewer,
        airframe,
        airframe_spec,
        estimator_mode,
        estimator_seed,
        Node(
            package='sim_bridge',
            executable='fixedwing_backend_adapter_node',
            name='fixedwing_backend_adapter_node',
            output='screen',
            parameters=[{
                'backend': 'mujoco',
                'start_z_m': 55.0,
                'start_speed_mps': 15.0,
                'cruise_airspeed_mps': 15.0,
                'wind_x_mps': LaunchConfiguration('wind_x_mps'),
                'wind_y_mps': LaunchConfiguration('wind_y_mps'),
                'mujoco_timestep': 0.002,
                'mujoco_thrust_tau_s': 0.03,
                'mujoco_viewer': LaunchConfiguration('viewer'),
                'airframe_spec': LaunchConfiguration('airframe_spec'),
                'airframe': LaunchConfiguration('airframe'),
                'estimator_mode': LaunchConfiguration('estimator_mode'),
                'estimator_seed': LaunchConfiguration('estimator_seed'),
            }],
        ),
        Node(
            package='uav_control',
            executable='fixed_wing_guidance_node',
            name='fixed_wing_guidance_node',
            output='screen',
        ),
        Node(
            package='sim_fast',
            executable='fixed_wing_demo_node',
            name='fixed_wing_demo_node',
            output='screen',
            parameters=[{
                'start_delay_sec': LaunchConfiguration('start_delay_sec'),
            }],
        ),
    ])
