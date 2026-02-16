"""Smoke tests: verify all public API symbols are importable."""


def test_import_walkie_robot():
    from walkie_sdk import WalkieRobot

    assert WalkieRobot is not None


def test_import_multi_camera():
    from walkie_sdk import MultiCamera

    assert MultiCamera is not None


def test_import_protocol_enums():
    from walkie_sdk import ROSProtocol, CameraProtocol

    assert ROSProtocol is not None
    assert CameraProtocol is not None


def test_import_arm_control_mode():
    from walkie_sdk import ArmControlMode

    assert ArmControlMode is not None


def test_import_visualization_constants():
    from walkie_sdk import (
        ARROW,
        CUBE,
        SPHERE,
        CYLINDER,
        LINE_STRIP,
        LINE_LIST,
        CUBE_LIST,
        SPHERE_LIST,
        POINTS,
        TEXT_VIEW_FACING,
        MESH_RESOURCE,
        TRIANGLE_LIST,
        DEFAULT_AXIS_TOPIC,
        Visualization,
    )

    assert SPHERE == 2
    assert Visualization is not None


def test_version_string():
    from walkie_sdk import __version__

    assert isinstance(__version__, str)
    parts = __version__.split(".")
    assert len(parts) == 3, f"Expected semver, got {__version__!r}"


def test_all_exports_resolve():
    import walkie_sdk

    for name in walkie_sdk.__all__:
        assert hasattr(walkie_sdk, name), f"Missing export: {name}"
