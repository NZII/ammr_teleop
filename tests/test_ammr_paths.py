import json
from pathlib import Path

import pytest

from ammr_teleop.common.paths import ENV, configured_path, project_root, settings

@pytest.fixture
def isolated(tmp_path, monkeypatch):
    for name in (*ENV.values(), 'AMMR_CONFIG', 'TELEOP_ENV', 'XR_TELEOP_ENV'):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setenv('AMMR_PROJECT_ROOT', str(tmp_path))
    (tmp_path / 'configs').mkdir()
    return tmp_path

def test_precedence_and_relative_base(isolated, monkeypatch):
    (isolated / 'configs/paths.yaml').write_text(json.dumps({'data_root': 'configured data'}))
    assert configured_path('data_root') == str(isolated / 'configured data')
    monkeypatch.setenv('AMMR_DATA_ROOT', 'env data')
    assert configured_path('data_root') == str(isolated / 'env data')
    assert configured_path('data_root', 'explicit data') == str(isolated / 'explicit data')

def test_remote_paths_not_expanded_locally(isolated, monkeypatch):
    monkeypatch.setenv('AMMR_AGX_ROOT', '~/robot')
    assert configured_path('agx_root') == '~/robot'

def test_explicit_missing_config_fails(isolated, monkeypatch):
    monkeypatch.setenv('AMMR_CONFIG', 'missing.yaml')
    with pytest.raises(FileNotFoundError):
        settings()

def test_invalid_config_key_fails(isolated):
    (isolated / 'configs/paths.yaml').write_text('{"assset_root": "assets"}')
    with pytest.raises(ValueError):
        settings()

def test_optional_remote_overlay_can_be_empty(isolated):
    (isolated / 'configs/paths.yaml').write_text('{"agx_overlay_setup": ""}')
    assert configured_path('agx_overlay_setup') == ''

def test_legacy_interpreter_and_explicit_override(isolated, monkeypatch):
    monkeypatch.setenv('TELEOP_ENV', '/opt/old env')
    assert configured_path('teleop_python') == '/opt/old env/bin/python'
    monkeypatch.setenv('AMMR_TELEOP_PYTHON', '/opt/new/python')
    assert configured_path('teleop_python') == '/opt/new/python'
