"""Tests for sentry_bringup package setup."""

import ast
from pathlib import Path

import pytest


@pytest.fixture
def setup_ast():
    setup_py = Path(__file__).parent.parent / 'setup.py'
    return ast.parse(setup_py.read_text(encoding='utf-8'))


def _find_entry_points_dict(tree):
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            for kw in node.keywords:
                if kw.arg == 'entry_points':
                    return ast.literal_eval(kw.value)
    raise ValueError('entry_points not found in setup.py')


def test_no_legacy_ai_inference_entry_point(setup_ast):
    eps = _find_entry_points_dict(setup_ast)
    console_scripts = eps.get('console_scripts', [])
    names = [line.split('=')[0].strip() for line in console_scripts]
    assert 'ai_inference_node' not in names


def test_no_legacy_uart_bridge_entry_point(setup_ast):
    eps = _find_entry_points_dict(setup_ast)
    console_scripts = eps.get('console_scripts', [])
    names = [line.split('=')[0].strip() for line in console_scripts]
    assert 'uart_bridge_node' not in names


def test_mipi_camera_entry_point_present(setup_ast):
    eps = _find_entry_points_dict(setup_ast)
    console_scripts = eps.get('console_scripts', [])
    names = [line.split('=')[0].strip() for line in console_scripts]
    assert 'mipi_camera_node' in names


def test_legacy_nodes_files_removed():
    pkg_dir = Path(__file__).parent.parent / 'sentry_bringup'
    assert not (pkg_dir / 'ai_inference_node.py').exists()
    assert not (pkg_dir / 'uart_bridge_node.py').exists()
