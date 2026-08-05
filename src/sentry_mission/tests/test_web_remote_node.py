"""Tests for web_remote_node service call handling."""

import sys
import threading
import types
import signal
from pathlib import Path
from unittest import mock

import pytest

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))


@pytest.fixture(scope='module', autouse=True)
def mock_ros2():
    """Mock ROS2 modules so web_remote_node can be imported off-board."""
    modules = {
        'rclpy': types.ModuleType('rclpy'),
        'rclpy.node': types.ModuleType('rclpy.node'),
        'rclpy.qos': types.ModuleType('rclpy.qos'),
        'geometry_msgs': types.ModuleType('geometry_msgs'),
        'geometry_msgs.msg': types.ModuleType('geometry_msgs.msg'),
        'std_srvs': types.ModuleType('std_srvs'),
        'std_srvs.srv': types.ModuleType('std_srvs.srv'),
        'std_msgs': types.ModuleType('std_msgs'),
        'std_msgs.msg': types.ModuleType('std_msgs.msg'),
        'sentry_interfaces': types.ModuleType('sentry_interfaces'),
        'sentry_interfaces.srv': types.ModuleType('sentry_interfaces.srv'),
        'sentry_interfaces.msg': types.ModuleType('sentry_interfaces.msg'),
        'sensor_msgs': types.ModuleType('sensor_msgs'),
        'sensor_msgs.msg': types.ModuleType('sensor_msgs.msg'),
    }

    modules['rclpy'].__path__ = []
    modules['rclpy'].init = mock.MagicMock()
    modules['sentry_interfaces'].__path__ = []
    modules['rclpy'].shutdown = mock.MagicMock()
    modules['rclpy'].ok = mock.MagicMock(return_value=True)
    modules['rclpy'].Node = object
    modules['rclpy.node'].Node = object
    modules['rclpy.qos'].DurabilityPolicy = type('DurabilityPolicy', (), {})
    modules['rclpy.qos'].QoSProfile = type('QoSProfile', (), {})

    Twist = type('Twist', (), {})
    modules['geometry_msgs.msg'].Twist = Twist

    SetBool = type('SetBool', (), {})
    SetBool.Request = type('Request', (), {})
    modules['std_srvs.srv'].SetBool = SetBool

    SetCropType = type('SetCropType', (), {})
    modules['sentry_interfaces.srv'].SetCropType = SetCropType
    MissionStatus = type('MissionStatus', (), {})
    modules['sentry_interfaces.msg'].MissionStatus = MissionStatus
    PlantDetection = type('PlantDetection', (), {})
    modules['sentry_interfaces.msg'].PlantDetection = PlantDetection
    Diagnosis = type('Diagnosis', (), {})
    modules['sentry_interfaces.msg'].Diagnosis = Diagnosis
    CompressedImage = type('CompressedImage', (), {})
    modules['sensor_msgs.msg'].CompressedImage = CompressedImage
    String = type('String', (), {})
    modules['std_msgs.msg'].String = String


    for name, mod in modules.items():
        sys.modules[name] = mod

    yield

    for name in modules:
        sys.modules.pop(name, None)


def test_set_mode_auto_adds_done_callback():
    from sentry_mission.web_remote_node import WebRemoteNode

    node = mock.MagicMock()
    node.create_publisher = mock.MagicMock()
    node.create_client = mock.MagicMock()
    node.declare_parameter = mock.MagicMock()
    node.get_parameter = mock.MagicMock(return_value=mock.MagicMock(value=1.0))
    node.get_logger = mock.MagicMock(return_value=mock.MagicMock())
    node.create_timer = mock.MagicMock()

    fake_future = mock.MagicMock()
    fake_client = mock.MagicMock()
    fake_client.service_is_ready = mock.MagicMock(return_value=True)
    fake_client.call_async = mock.MagicMock(return_value=fake_future)
    node.create_client.return_value = fake_client

    # Patch super().__init__ to avoid ROS2 node initialization
    with mock.patch('builtins.super'):
        web = WebRemoteNode.__new__(WebRemoteNode)
        web.__dict__['node'] = node
        # Manually set attributes normally set by __init__
        web.cmd_pub = node.create_publisher()
        web.mode_srv = fake_client
        web.max_linear = 0.5
        web.max_angular = 1.0
        web.mode = 'AUTO'
        web.linear = 0.0
        web.angular = 0.0
        web.lock = threading.Lock()
        web.last_cmd_time = 0.0
        web.TIMEOUT = 0.5
        web.timer = None
        web.get_logger = node.get_logger
        from sentry_mission.batch_recorder import BatchRecorder
        web.batch_recorder = BatchRecorder()

        result = web.set_mode_auto(False)

    assert result is True
    fake_client.call_async.assert_called_once()
    fake_future.add_done_callback.assert_called_once()


def test_on_mode_response_logs_success():
    from sentry_mission.web_remote_node import WebRemoteNode

    web = mock.MagicMock()
    logger = mock.MagicMock()
    web.get_logger = mock.MagicMock(return_value=logger)

    fake_future = mock.MagicMock()
    response = mock.MagicMock()
    response.success = True
    response.message = 'OK'
    fake_future.result = mock.MagicMock(return_value=response)

    WebRemoteNode._on_mode_response(web, fake_future, True)
    logger.info.assert_called_once()


def test_on_mode_response_logs_failure():
    from sentry_mission.web_remote_node import WebRemoteNode

    web = mock.MagicMock()
    logger = mock.MagicMock()
    web.get_logger = mock.MagicMock(return_value=logger)

    fake_future = mock.MagicMock()
    response = mock.MagicMock()
    response.success = False
    response.message = 'Rejected'
    fake_future.result = mock.MagicMock(return_value=response)

    WebRemoteNode._on_mode_response(web, fake_future, True)
    logger.warn.assert_called_once()


def test_on_stop_response_logs_success():
    from sentry_mission.web_remote_node import WebRemoteNode

    web = mock.MagicMock()
    logger = mock.MagicMock()
    web.get_logger = mock.MagicMock(return_value=logger)

    fake_future = mock.MagicMock()
    response = mock.MagicMock()
    response.success = True
    response.message = 'Stopped'
    fake_future.result = mock.MagicMock(return_value=response)

    WebRemoteNode._on_stop_response(web, fake_future)
    logger.info.assert_called_once()

def test_stack_script_env_preserves_frontend_control_plane():
    from sentry_mission.web_remote_node import _stack_script_env

    env = _stack_script_env({'KEEP': 'yes'})

    assert env['KEEP'] == 'yes'
    assert env['SENTRY_PRESERVE_WEB'] == '1'
    assert env['ENABLE_WEB'] == 'false'


def test_stack_script_env_enables_vision_and_advisory_for_cruise():
    from sentry_mission.web_remote_node import _stack_script_env

    env = _stack_script_env({
        'ENABLE_VISION': 'false',
        'ENABLE_ADVISORY': 'false',
    })

    assert env['SENTRY_PRESERVE_WEB'] == '1'
    assert env['ENABLE_WEB'] == 'false'
    assert env['ENABLE_VISION'] == 'true'
    assert env['ENABLE_ADVISORY'] == 'true'


def test_stack_script_env_defaults_to_mipi_camera():
    from sentry_mission.web_remote_node import _stack_script_env

    env = _stack_script_env({})

    assert env['CAMERA_BACKEND'] == 'mipi'


def test_mission_status_complete_when_all_waypoints_reached():
    from sentry_mission.web_remote_node import _mission_status_is_complete

    msg = types.SimpleNamespace(
        state='PATROL',
        current_wp_idx=3,
        total_wps=3,
    )

    assert _mission_status_is_complete(msg) is True


def test_mission_status_not_complete_before_last_waypoint():
    from sentry_mission.web_remote_node import _mission_status_is_complete

    msg = types.SimpleNamespace(
        state='PATROL',
        current_wp_idx=2,
        total_wps=3,
    )

    assert _mission_status_is_complete(msg) is False


def test_mission_status_syncs_manual_mode_after_external_stop():
    from sentry_mission.web_remote_node import WebRemoteNode

    web = WebRemoteNode.__new__(WebRemoteNode)
    web.lock = threading.Lock()
    web.mode = 'AUTO'
    web.frontend_started_auto = True
    web.completion_stop_started = True
    web._last_mission_state = None
    web.batch_recorder = mock.MagicMock()

    WebRemoteNode.on_mission_status(
        web,
        types.SimpleNamespace(state='MANUAL', current_wp_idx=2, total_wps=3),
    )

    assert web.mode == 'MANUAL'
    assert web.frontend_started_auto is False
    assert web.completion_stop_started is False


def test_validate_vision_inference_mode_accepts_known_modes():
    from sentry_mission.web_remote_node import _validate_vision_inference_mode

    assert _validate_vision_inference_mode('triggered') == 'triggered'
    assert _validate_vision_inference_mode('independent') == 'independent'


def test_validate_vision_inference_mode_rejects_unknown_mode():
    from sentry_mission.web_remote_node import _validate_vision_inference_mode

    with pytest.raises(ValueError):
        _validate_vision_inference_mode('always')


def test_set_vision_inference_mode_updates_mode_on_success():
    from sentry_mission.web_remote_node import WebRemoteNode

    web = WebRemoteNode.__new__(WebRemoteNode)
    web.stack_lock = threading.Lock()
    web.lock = threading.Lock()
    web.vision_inference_mode = 'triggered'
    web._start_independent_diagnosis_locked = mock.MagicMock(
        return_value=(True, 'started'))
    web._stop_independent_diagnosis_locked = mock.MagicMock()

    ok, message = web.set_vision_inference_mode('independent')

    assert ok is True
    assert message == 'started'
    assert web.vision_inference_mode == 'independent'


def test_stop_independent_diagnosis_only_terminates_owned_process_group():
    from sentry_mission.web_remote_node import WebRemoteNode

    web = WebRemoteNode.__new__(WebRemoteNode)
    web.vision_diagnosis_proc = mock.MagicMock(pid=42)
    web.vision_diagnosis_proc.poll.return_value = None
    web.vision_diagnosis_log = None
    web.get_logger = mock.MagicMock(return_value=mock.MagicMock())

    with mock.patch('sentry_mission.web_remote_node.os.getpgid', return_value=42,
                    create=True), \
         mock.patch('sentry_mission.web_remote_node.os.killpg', create=True) as killpg, \
         mock.patch('sentry_mission.web_remote_node.subprocess.run') as run:
        ok, _ = web._stop_independent_diagnosis_locked()

    assert ok is True
    killpg.assert_called_once_with(42, signal.SIGTERM)
    run.assert_not_called()


def test_start_vision_stack_runs_camera_script():
    from sentry_mission.web_remote_node import WebRemoteNode

    web = WebRemoteNode.__new__(WebRemoteNode)
    web.lock = threading.Lock()
    web.stack_lock = threading.Lock()
    web.camera_ready = False
    web.camera_start_script = '/tmp/start_camera_stack.sh'
    web.get_logger = mock.MagicMock(return_value=mock.MagicMock())
    web._run_stack_script = mock.MagicMock(return_value=(True, 'camera up'))

    assert web.start_vision_stack() == (True, 'camera up')
    web._run_stack_script.assert_called_once_with('/tmp/start_camera_stack.sh')
    assert web.camera_ready is True


def test_start_inference_stack_runs_inference_script():
    from sentry_mission.web_remote_node import WebRemoteNode

    web = WebRemoteNode.__new__(WebRemoteNode)
    web.lock = threading.Lock()
    web.stack_lock = threading.Lock()
    web.inference_ready = False
    web.inference_start_script = '/tmp/start_inference_stack.sh'
    web.get_logger = mock.MagicMock(return_value=mock.MagicMock())
    web._run_stack_script = mock.MagicMock(return_value=(True, 'inference up'))

    assert web.start_inference_stack() == (True, 'inference up')
    web._run_stack_script.assert_called_once_with('/tmp/start_inference_stack.sh')
    assert web.inference_ready is True


def test_stop_vision_stack_runs_camera_stop_script():
    from sentry_mission.web_remote_node import WebRemoteNode

    web = WebRemoteNode.__new__(WebRemoteNode)
    web.lock = threading.Lock()
    web.stack_lock = threading.Lock()
    web.camera_ready = True
    web.camera_stop_script = '/tmp/stop_camera_stack.sh'
    web.get_logger = mock.MagicMock(return_value=mock.MagicMock())
    web._run_stack_script = mock.MagicMock(return_value=(True, 'camera down'))

    assert web.stop_vision_stack() == (True, 'camera down')
    web._run_stack_script.assert_called_once_with('/tmp/stop_camera_stack.sh')
    assert web.camera_ready is False


def test_stop_inference_stack_runs_inference_stop_script():
    from sentry_mission.web_remote_node import WebRemoteNode

    web = WebRemoteNode.__new__(WebRemoteNode)
    web.lock = threading.Lock()
    web.stack_lock = threading.Lock()
    web.inference_ready = True
    web.inference_stop_script = '/tmp/stop_inference_stack.sh'
    web.get_logger = mock.MagicMock(return_value=mock.MagicMock())
    web._run_stack_script = mock.MagicMock(return_value=(True, 'inference down'))

    assert web.stop_inference_stack() == (True, 'inference down')
    web._run_stack_script.assert_called_once_with('/tmp/stop_inference_stack.sh')
    assert web.inference_ready is False


def test_validate_cruise_speed_limits_value():
    from sentry_mission.web_remote_node import _validate_cruise_speed

    assert _validate_cruise_speed(0.18) == 0.18
    with pytest.raises(ValueError):
        _validate_cruise_speed(0.04)
    with pytest.raises(ValueError):
        _validate_cruise_speed(0.36)


def test_cruise_speed_config_round_trip(tmp_path):
    from sentry_mission.web_remote_node import (
        _read_cruise_speed_file,
        _write_cruise_speed_file,
    )

    config_path = tmp_path / 'mission_params.yaml'
    _write_cruise_speed_file(config_path, 0.22)

    assert _read_cruise_speed_file(config_path) == 0.22
    assert 'cruise_speed: 0.22' in config_path.read_text(encoding='utf-8')


def test_set_cruise_speed_before_preheat_saves_requested_value():
    from sentry_mission.web_remote_node import WebRemoteNode

    web = WebRemoteNode.__new__(WebRemoteNode)
    web.stack_ready = False
    web.lock = threading.Lock()
    web.cruise_speed = 0.18
    web._set_remote_parameter = mock.MagicMock()

    ok, message = web.set_cruise_speed(0.22)

    assert ok is True
    assert web.cruise_speed == 0.22
    assert 'saved for preheat' in message
    web._set_remote_parameter.assert_not_called()


def test_set_cruise_speed_updates_mission_and_nav_controller():
    from sentry_mission.web_remote_node import WebRemoteNode

    web = WebRemoteNode.__new__(WebRemoteNode)
    web.stack_ready = True
    web.lock = threading.Lock()
    web.cruise_speed = 0.18
    web._set_remote_parameter = mock.MagicMock(return_value=(True, 'ok'))

    ok, _ = web.set_cruise_speed(0.22)

    assert ok is True
    assert web.cruise_speed == 0.22
    assert web._set_remote_parameter.call_args_list == [
        mock.call('/mission_control_node', 'cruise_speed', 0.22),
        mock.call('/controller_server', 'FollowPath.desired_linear_vel', 0.22),
    ]


def test_fixed_point_stops_round_trip_preserves_cruise_speed(tmp_path):
    from sentry_mission.web_remote_node import (
        _read_mission_params,
        _write_mission_params,
    )

    config_path = tmp_path / 'mission_params.yaml'
    _write_mission_params(config_path, {
        'cruise_speed': 0.22,
        'fixed_point_stops': [{
            'x': 1.2,
            'y': -0.5,
            'radius': 0.2,
            'disease_class': 'early_blight',
        }],
    })

    params = _read_mission_params(config_path)

    assert params['cruise_speed'] == 0.22
    assert params['fixed_point_stops'] == [{
        'x': 1.2,
        'y': -0.5,
        'radius': 0.2,
        'disease_class': 'early_blight',
    }]


def test_validate_fixed_point_stops_rejects_unknown_tomato_disease():
    from sentry_mission.web_remote_node import _validate_fixed_point_stops

    with pytest.raises(ValueError, match='disease_class'):
        _validate_fixed_point_stops([{
            'x': 0,
            'y': 0,
            'radius': 0.2,
            'disease_class': 'unknown',
        }])


def test_write_fixed_point_stops_preserves_existing_mission_params(tmp_path):
    from sentry_mission.web_remote_node import (
        _read_mission_params,
        _write_fixed_point_stops_file,
        _write_mission_params,
    )

    config_path = tmp_path / 'mission_params.yaml'
    _write_mission_params(config_path, {'cruise_speed': 0.23})

    _write_fixed_point_stops_file(config_path, [{
        'x': 0.0,
        'y': 1.0,
        'radius': 0.25,
        'disease_class': 'healthy',
    }])

    assert _read_mission_params(config_path) == {
        'cruise_speed': 0.23,
        'fixed_point_stops': [{
            'x': 0.0,
            'y': 1.0,
            'radius': 0.25,
            'disease_class': 'healthy',
        }],
    }


def test_capture_camera_image_saves_latest_jpeg_to_configured_directory(tmp_path):
    from sentry_mission.web_remote_node import WebRemoteNode

    web = WebRemoteNode.__new__(WebRemoteNode)
    web.lock = threading.Lock()
    web.capture_dir = str(tmp_path)
    web.latest_camera_jpeg = b'\xff\xd8mock-jpeg\xff\xd9'
    web.get_logger = mock.MagicMock(return_value=mock.MagicMock())

    ok, filename = web.capture_camera_image()

    assert ok is True
    image_path = tmp_path / filename
    assert image_path.suffix == '.jpg'
    assert image_path.read_bytes() == b'\xff\xd8mock-jpeg\xff\xd9'


def test_capture_camera_image_rejects_when_no_frame_has_arrived(tmp_path):
    from sentry_mission.web_remote_node import WebRemoteNode

    web = WebRemoteNode.__new__(WebRemoteNode)
    web.lock = threading.Lock()
    web.capture_dir = str(tmp_path)
    web.latest_camera_jpeg = None

    ok, message = web.capture_camera_image()

    assert ok is False
    assert 'No camera frame' in message


def _make_wired_node():
    import time as _time
    from sentry_mission.web_remote_node import WebRemoteNode
    from sentry_mission.batch_recorder import BatchRecorder

    node = WebRemoteNode.__new__(WebRemoteNode)
    node.batch_recorder = BatchRecorder()
    node.latest_plant = None
    node.latest_plant_time = 0.0
    node.latest_camera_jpeg = b'frame'
    node._last_mission_state = None
    node.lock = threading.Lock()
    node.get_logger = mock.MagicMock()
    return node


def _status(state):
    return types.SimpleNamespace(state=state, total_wps=3, current_wp_idx=0)


def test_patrol_to_stopped_records_snapshot_with_fresh_plant():
    import time
    node = _make_wired_node()
    node.batch_recorder.on_mode_change('AUTO')
    node.latest_plant = ([0.1, 0.1, 0.5, 0.5], 0.9)
    node.latest_plant_time = time.time()

    node.on_mission_status(_status('PATROL'))
    node.on_mission_status(_status('STOPPED'))

    assert len(node.batch_recorder.current.records) == 1


def test_patrol_to_stopped_without_plant_records_nothing():
    node = _make_wired_node()
    node.batch_recorder.on_mode_change('AUTO')

    node.on_mission_status(_status('PATROL'))
    node.on_mission_status(_status('STOPPED'))

    assert node.batch_recorder.current.records == []


def test_stale_plant_detection_is_ignored():
    import time
    node = _make_wired_node()
    node.batch_recorder.on_mode_change('AUTO')
    node.latest_plant = ([0.1, 0.1, 0.5, 0.5], 0.9)
    node.latest_plant_time = time.time() - 5.0

    node.on_mission_status(_status('PATROL'))
    node.on_mission_status(_status('STOPPED'))

    assert node.batch_recorder.current.records == []


def test_diagnosis_sentinel_class_id_ignored():
    import time
    node = _make_wired_node()
    node.batch_recorder.on_mode_change('AUTO')
    node.latest_plant = ([0.1, 0.1, 0.5, 0.5], 0.9)
    node.latest_plant_time = time.time()
    node.on_mission_status(_status('PATROL'))
    node.on_mission_status(_status('STOPPED'))

    node._on_diagnosis(types.SimpleNamespace(
        class_id=254, disease_class='', confidence=0.0))
    assert node.batch_recorder.current.records[0].disease_class is None

    node._on_diagnosis(types.SimpleNamespace(
        class_id=1, disease_class='early_blight', confidence=0.8))
    assert node.batch_recorder.current.records[0].disease_class == 'early_blight'


def test_get_status_includes_message_unread():
    import time
    node = _make_wired_node()
    node.mode = 'MANUAL'
    node.linear = 0.0
    node.angular = 0.0
    node.last_cmd_time = time.time()
    node.TIMEOUT = 0.5
    node.mode_srv = mock.MagicMock()
    node.frontend_started_auto = False
    node.completion_stop_started = False
    node.stack_ready = False
    node.stack_lock = threading.Lock()
    node.stack_operation = None
    node.cruise_speed = 0.18
    node.vision_inference_mode = 'triggered'
    node.batch_recorder.unread = 2

    assert node.get_status()['message_unread'] == 2


def test_mission_status_flips_mode_to_auto_on_patrol():
    node = _make_wired_node()
    node.mode = 'MANUAL'
    node.frontend_started_auto = True
    node.completion_stop_started = False

    node.on_mission_status(_status('PATROL'))

    assert node.mode == 'AUTO'


def test_manual_status_racing_set_mode_auto_does_not_stick():
    # set_mode_auto just ran, then a stale MANUAL status lands before
    # PATROL starts: the following PATROL status must restore AUTO or the
    # joystick watchdog floods /cmd_vel with zeros all cruise long.
    node = _make_wired_node()
    node.mode = 'AUTO'
    node.frontend_started_auto = True
    node.completion_stop_started = False

    node.on_mission_status(_status('MANUAL'))
    assert node.mode == 'MANUAL'
    assert node.frontend_started_auto is False

    node.on_mission_status(_status('PATROL'))
    assert node.mode == 'AUTO'


def test_mission_owned_states_keep_mode_auto():
    # STOPPED/SCANNING/OBSTACLE_*: mission_control owns /cmd_vel there too.
    node = _make_wired_node()
    node.mode = 'MANUAL'
    node.frontend_started_auto = True
    node.completion_stop_started = False

    for state in ('STOPPED', 'SCANNING', 'OBSTACLE_BACKUP', 'ANALYZING'):
        node.mode = 'MANUAL'
        node.on_mission_status(_status(state))
        assert node.mode == 'AUTO', state
