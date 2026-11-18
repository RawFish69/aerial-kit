import subprocess
import sys
from pathlib import Path
from xml.etree import ElementTree


PACKAGE = Path(__file__).resolve().parents[1]
MODELS = PACKAGE / 'models'
REPO = PACKAGE.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))


AIRFRAMES = (
    'fpv_quad_5in',
    'micro_lr_quad',
    'research_hex',
    'research_octo',
)


def _model_root(airframe):
    return ElementTree.parse(
        MODELS / f'{airframe}_visual' / 'model.sdf').getroot()


def _visuals(airframe):
    return list(_model_root(airframe).iter('visual'))


def _resolve_uri(uri):
    assert uri.startswith('model://'), uri
    return MODELS / uri[len('model://'):]


def test_multirotor_assets_match_their_specs():
    result = subprocess.run(
        [sys.executable, str(REPO / 'scripts' / 'generate_multirotor_assets.py'),
         '--check'],
        capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr


def test_every_mesh_uri_resolves_and_models_stay_under_two_megabytes():
    for airframe in AIRFRAMES:
        model_dir = MODELS / f'{airframe}_visual'
        for visual in _visuals(airframe):
            uri = visual.findtext('geometry/mesh/uri')
            assert uri is not None
            path = _resolve_uri(uri)
            assert path.is_file(), path
            assert path.stat().st_size > 200, path
        size = sum(
            path.stat().st_size for path in model_dir.rglob('*')
            if path.is_file())
        assert size < 2_000_000, f'{airframe}: {size} bytes'


def test_gazebo_and_mujoco_use_the_same_visual_parts():
    from aerial_kit.airframes.multirotor_specs import load_multirotor_spec
    from aerial_kit.airframes.multirotor_specs.mjcf import multirotor_mjcf

    for airframe in AIRFRAMES:
        spec = load_multirotor_spec(airframe)
        mjcf = ElementTree.fromstring(multirotor_mjcf(spec, ground=False))
        mujoco_meshes = {
            geom.attrib['name']
            for geom in mjcf.iter('geom')
            if geom.attrib.get('type') == 'mesh'
        }
        gazebo_visuals = {
            visual.attrib['name']
            for visual in _visuals(airframe)
        }
        assert gazebo_visuals == mujoco_meshes, airframe


def test_generated_models_are_static_and_named_for_their_airframes():
    for airframe in AIRFRAMES:
        model = _model_root(airframe).find('model')
        assert model is not None
        assert model.attrib['name'] == airframe
        assert model.findtext('static') == 'true'


def test_micro_quad_reuses_cad_stls_without_copying_them():
    model_dir = MODELS / 'micro_lr_quad_visual'
    uris = {
        visual.findtext('geometry/mesh/uri')
        for visual in _visuals('micro_lr_quad')
    }
    cad_uris = {uri for uri in uris if uri.startswith('model://lr_drone_urdf/')}
    assert len(cad_uris) == 5
    assert not list((model_dir / 'meshes').glob('*.stl'))


def test_multirotor_launch_bridges_the_dynamic_pose_service():
    launch = (
        PACKAGE / 'launch' / 'multirotor_gazebo_demo.launch.py'
    ).read_text(encoding='utf-8')
    assert '/world/uav_multirotor/set_pose' in launch
    assert 'ros_gz_interfaces/srv/SetEntityPose' in launch
    assert 'gazebo_multirotor_pose_node' in launch
    assert 'model://' in launch


def test_multirotor_world_has_the_expected_runtime_systems():
    world = ElementTree.parse(
        PACKAGE / 'worlds' / 'multirotor_demo.sdf').getroot().find('world')
    assert world is not None
    assert world.attrib['name'] == 'uav_multirotor'
    plugin_names = {plugin.attrib['name'] for plugin in world.findall('plugin')}
    assert 'gz::sim::systems::Physics' in plugin_names
    assert 'gz::sim::systems::UserCommands' in plugin_names
    assert 'gz::sim::systems::SceneBroadcaster' in plugin_names
