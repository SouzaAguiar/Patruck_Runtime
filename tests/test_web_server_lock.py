"""Load only command state definitions: no pygame, server, IMU or motors."""
import ast
import dataclasses
import threading
import time

import pytest
from test_telemetry import ROOT


def state(head=False):
    source = (ROOT/'mini_bdx_runtime/mini_bdx_runtime/web_controller.py').read_text()
    nodes = [n for n in ast.parse(source).body if isinstance(n, ast.FunctionDef)
             or isinstance(n, ast.ClassDef) and n.name == 'WebCommandState']
    future = ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0)
    env = dict(dataclass=dataclasses.dataclass, field=dataclasses.field,
               threading=threading, time=time, X_RANGE=[-.15,.15], Y_RANGE=[-.2,.2],
               YAW_RANGE=[-1,1], HEAD_PITCH_RANGE=[-.78,.3],
               HEAD_YAW_RANGE=[-.5,.5], HEAD_ROLL_RANGE=[-.5,.5])
    exec(compile(ast.fix_missing_locations(ast.Module(body=[future]+nodes, type_ignores=[])),
                 '<web-state-only>', 'exec'), env)
    return env['WebCommandState'](allow_head_control=head)


@pytest.mark.parametrize('forward', [-1., 1.])
def test_server_blocks_lateral_even_when_client_sends_nonzero_axis(forward):
    s = state()
    s.update(dict(left_x=.7, left_y=forward, right_x=.4, lateral_locked=True))
    assert s.snapshot()[0][:3] == [forward*.15, 0., .4]
    d = s.last_snapshot_metadata
    assert d['received_axes']['left_x'] == .7 and d['lateral_lock_applied']
    s.update(dict(left_x=.7, left_y=forward, lateral_locked=False))
    assert s.snapshot()[0][1] == .14
    assert s.last_snapshot_metadata['command_sequence'] == 2
    assert d['effective_commands'][1] == 0  # Old snapshots are independent.


def test_legacy_client_is_explicitly_unknown_and_does_not_inherit_lock():
    s = state()
    s.update(dict(lateral_locked=True))
    s.update(dict(left_x=.5))
    assert s.snapshot()[0][1] == .1
    assert s.last_snapshot_metadata['lateral_lock_requested'] is None
    assert s.last_snapshot_metadata['lateral_lock_reported'] is False
    assert s.last_snapshot_metadata['client_version'] is None


@pytest.mark.parametrize('head_allowed', [True, False])
def test_lock_uses_effective_mode_including_disabled_head_fallback(head_allowed):
    s = state(head_allowed)
    s.update(dict(mode='head', left_x=.5, left_y=1, lateral_locked=True))
    commands = s.snapshot()[0]
    assert commands[1] == 0
    assert commands[5] == (.25 if head_allowed else 0)
    assert s.last_snapshot_metadata['lateral_lock_applied'] is (not head_allowed)


def test_invalid_lock_flag_is_visible_and_conservatively_blocks_lateral():
    s = state()
    s.update(dict(left_x=1, lateral_locked='false'))
    assert s.snapshot()[0][1] == 0
    assert s.last_snapshot_metadata['lateral_lock_reported'] is False
    assert s.last_snapshot_metadata['lateral_lock_applied'] is True


@pytest.mark.parametrize('limit', [.03, .05, .08, .15])
@pytest.mark.parametrize('direction', [-1, 1])
def test_server_caps_both_directions_and_records_hand_support(limit, direction):
    s = state()
    s.update(dict(left_x=.7, left_y=direction, right_x=.4, lateral_locked=True,
                  longitudinal_limit=limit, hand_support=True))
    assert s.snapshot()[0][:3] == [direction*limit, 0, .4]
    assert s.last_snapshot_metadata['longitudinal_limit_m_s']==limit
    assert s.last_snapshot_metadata['hand_support'] is True
    s.update(dict(left_y=direction*.1,longitudinal_limit=limit,hand_support=False))
    assert s.snapshot()[0][0]==direction*.015
    assert s.last_snapshot_metadata['hand_support'] is False


@pytest.mark.parametrize('limit', [None, 'bad', float('nan'), float('inf'), -.05, .3])
def test_invalid_explicit_limit_falls_back_to_low_speed(limit):
    s = state()
    s.update(dict(left_y=1,longitudinal_limit=limit))
    assert s.snapshot()[0][0]==.03
