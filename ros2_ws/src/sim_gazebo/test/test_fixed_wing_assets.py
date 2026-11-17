import subprocess
import sys
from pathlib import Path
from xml.etree import ElementTree


PACKAGE = Path(__file__).resolve().parents[1]
MODEL = PACKAGE / 'models' / 'twin_wing_visual'
REPO = PACKAGE.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))


def test_fixed_wing_world_includes_the_visual_model():
    root = ElementTree.parse(
        PACKAGE / 'worlds' / 'fixed_wing_demo.sdf').getroot()
    world = root.find('world')
    assert world is not None
    assert world.attrib['name'] == 'uav_fixed_wing'

    include = world.find('include')
    assert include is not None
    assert include.findtext('name') == 'twin_wing'
    assert include.findtext('uri') == 'model://twin_wing_visual'


def _visuals():
    root = ElementTree.parse(MODEL / 'model.sdf').getroot()
    model = root.find('model')
    assert model is not None
    assert model.attrib['name'] == 'twin_wing'
    assert model.findtext('static') == 'true'
    body = model.find("link[@name='body']")
    assert body is not None
    return body.findall('visual')


def test_visual_model_has_the_twin_wing_surfaces():
    names = {visual.attrib['name'] for visual in _visuals()}
    assert {
        'center_body',
        'left_wing_top',
        'right_wing_top',
        'left_wing_bottom',
        'right_wing_bottom',
        'left_elevon',
        'right_elevon',
        'left_hinge_rod',
        'right_hinge_rod',
        'left_wingtip_fin',
        'right_wingtip_fin',
        'left_nacelle',
        'right_nacelle',
        'left_prop_blades',
        'right_prop_blades',
        'left_spinner',
        'right_spinner',
    } <= names


def test_every_visual_mesh_is_shipped():
    for visual in _visuals():
        uri = visual.find('geometry/mesh/uri').text
        assert uri.startswith('model://twin_wing_visual/')
        path = MODEL / uri[len('model://twin_wing_visual/'):]
        assert path.is_file(), path
        assert path.stat().st_size > 200


def test_gazebo_model_matches_the_shared_airframe_yaml():
    # Fails if someone edits the YAML or the generated files by hand.
    result = subprocess.run(
        [sys.executable, str(REPO / 'scripts' / 'generate_twin_wing_assets.py'),
         '--check'],
        capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr


def test_gazebo_and_mujoco_use_the_same_parts():
    from aerial_kit.airframes.twin_wing import load_twin_wing_spec
    from aerial_kit.airframes.twin_wing.mjcf import twin_wing_mjcf

    mjcf = ElementTree.fromstring(twin_wing_mjcf(load_twin_wing_spec()))
    mujoco_meshes = {
        geom.attrib['name'] for geom in mjcf.iter('geom')
        if geom.attrib.get('type') == 'mesh'}
    gazebo = {visual.attrib['name'] for visual in _visuals()}
    assert mujoco_meshes == gazebo


def test_launch_bridges_the_dynamic_pose_service():
    launch = (
        PACKAGE / 'launch' / 'fixed_wing_gazebo_demo.launch.py'
    ).read_text(encoding='utf-8')
    assert '/world/uav_fixed_wing/set_pose' in launch
    assert 'ros_gz_interfaces/srv/SetEntityPose' in launch
    assert 'gazebo_fixedwing_pose_node' in launch
