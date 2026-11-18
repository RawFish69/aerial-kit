from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]


def test_mujoco_demo_selects_the_physics_backend():
    launch = (
        PACKAGE / 'launch' / 'fixed_wing_mujoco_demo.launch.py'
    ).read_text(encoding='utf-8')
    assert "'backend': 'mujoco'" in launch
    assert 'fixed_wing_guidance_node' in launch
    assert 'fixed_wing_demo_node' in launch
    assert "mujoco_viewer" in launch


def test_mujoco_demo_exposes_the_shared_airframe_spec():
    launch = (
        PACKAGE / 'launch' / 'fixed_wing_mujoco_demo.launch.py'
    ).read_text(encoding='utf-8')
    assert "'airframe_spec'" in launch
    for arg in ('wind_x_mps', 'wind_y_mps', 'estimator_mode', 'estimator_seed',
                'viewer', 'start_delay_sec'):
        assert f"'{arg}'" in launch, arg


def _teleop_launch():
    import importlib.util

    path = PACKAGE / 'launch' / 'fixed_wing_mujoco_teleop.launch.py'
    spec = importlib.util.spec_from_file_location('teleop_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description(), path.read_text(encoding='utf-8')


def test_teleop_launch_flies_mujoco_without_mission_guidance():
    from launch.actions import DeclareLaunchArgument
    from launch_ros.actions import Node

    description, text = _teleop_launch()
    nodes = [e for e in description.entities if isinstance(e, Node)]
    executables = {n.node_executable for n in nodes}
    assert executables == {'fixedwing_backend_adapter_node', 'fixed_wing_teleop_node'}
    assert 'fixed_wing_guidance_node' not in text
    assert 'fixed_wing_demo_node' not in text
    assert "'backend': 'mujoco'" in text

    args = {e.name: e.default_value for e in description.entities
            if isinstance(e, DeclareLaunchArgument)}
    for name in ('airframe_spec', 'start_z_m', 'start_speed_mps',
                 'wind_x_mps', 'wind_y_mps', 'viewer'):
        assert name in args, name
    assert args['viewer'][0].text == 'true'
    # It starts airborne at cruise: a fixed wing cannot hover.
    assert float(args['start_z_m'][0].text) > 20.0
    assert float(args['start_speed_mps'][0].text) >= 12.0


def test_teleop_exit_shuts_the_session_down():
    _, text = _teleop_launch()
    assert 'OnProcessExit' in text and 'Shutdown' in text


def test_mujoco_launches_offer_the_single_wing():
    for name in ('fixed_wing_mujoco_demo.launch.py', 'fixed_wing_mujoco_teleop.launch.py'):
        text = (PACKAGE / 'launch' / name).read_text(encoding='utf-8')
        assert "'airframe'" in text and 'single_wing' in text, name


def _bringup():
    import importlib.util

    path = PACKAGE / 'launch' / 'bringup.launch.py'
    spec = importlib.util.spec_from_file_location('bringup_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_bringup_keeps_pointmass_default_and_offers_mujoco():
    from launch.actions import DeclareLaunchArgument

    module = _bringup()
    args = {e.name: e.default_value[0].text for e in module.generate_launch_description().entities
            if isinstance(e, DeclareLaunchArgument)}
    assert args['sim_backend'] == 'pointmass'
    assert args['airframe'] == 'fpv_quad_5in'


def test_predictive_tracker_tau_follows_the_plant():
    import sys

    repo = PACKAGE.parents[2]
    if str(repo) not in sys.path:
        sys.path.insert(0, str(repo))
    module = _bringup()
    assert module.velocity_loop_tau_s('pointmass', 'research_hex') == module._FAST_SIM_VELOCITY_TAU_S
    from aerial_kit.airframes.multirotor_specs import load_multirotor_spec

    for name in ('fpv_quad_5in', 'research_octo'):
        kp = load_multirotor_spec(name).control['velocity_kp']
        assert module.velocity_loop_tau_s('mujoco', name) == 1.0 / kp
