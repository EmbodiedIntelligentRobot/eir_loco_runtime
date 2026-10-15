#!/usr/bin/env python3
"""Decode EIR flight schema 1 without third-party dependencies. Readable records survive truncation."""
import argparse
import csv
import json
import struct
import sys
from pathlib import Path

SOURCES = ['command', 'feedback', 'watchdog', 'application', 'lifecycle', 'manual']
EVENTS = ['none', 'started', 'stopped', 'mode_changed', 'invalid_command', 'invalid_feedback',
          'feedback_timeout', 'command_timeout', 'backend_failure', 'motor_fault', 'estop',
          'application_fault', 'manual_save', 'clock_jump', 'warning', 'hardware_offline']
FIELDS = {10: 'submit_times_ns', 11: 'accepted', 20: 'input_q', 21: 'input_dq', 22: 'input_tau',
          23: 'input_kp', 24: 'input_kd', 30: 'wire_q', 31: 'wire_dq', 32: 'wire_tau',
          33: 'wire_kp', 34: 'wire_kd', 35: 'wire_mode', 40: 'hand_command', 41: 'hand_kp',
          42: 'hand_kd', 50: 'actuator_torque', 51: 'actuator_sequence_u32', 80: 'component',
          90: 'runtime_mode', 99: 'raw_dimensions', 100: 'raw_q', 101: 'raw_dq', 102: 'raw_tau',
          103: 'current', 104: 'motor_temperature', 105: 'mos_temperature', 110: 'motor_status',
          111: 'motor_error', 112: 'motor_loss', 120: 'joint_q', 121: 'joint_dq', 122: 'joint_tau',
          200: 'imu_wxyz', 201: 'imu_gyro', 202: 'imu_acc', 210: 'encoder_q', 211: 'encoder_dq',
          212: 'encoder_status', 220: 'bms', 221: 'bms_status', 230: 'hand_q', 231: 'hand_dq',
          232: 'hand_tau', 233: 'hand_code', 240: 'force_x', 241: 'force_y', 242: 'force_z',
          243: 'moment_x', 244: 'moment_y', 245: 'moment_z', 246: 'force_code', 250: 'sbus',
          260: 'alarm', 270: 'head_q', 271: 'head_online', 500: 'app_codes', 501: 'app_timing',
          502: 'velocity_command', 503: 'logical_command', 504: 'logical_feedback_q', 505: 'logical_feedback_dq', 510: 'mechanism_torque_warning_masks', 12: 'command_dimensions', 900: 'snapshot_ages_ms'}
FIELDS.update({506:'logical_command_dq',507:'logical_kp',508:'logical_kd',509:'policy_target_q',
    512:'app_identity_u32',513:'policy_state',514:'control_state_u32',515:'action_ages',
    516:'requested_motor_torque',517:'mechanism_fallback_mask',520:'policy_raw_action',
    521:'policy_applied_action',522:'amp_phase',523:'policy_joint_indices',530:'failure_cause',531:'failure_session_u32',
    532:'mode_request',533:'request_identity_u32',534:'request_origin',535:'application_event_drops_u32',536:'rejected_wire_mode',537:'mimic_a_e_variant'})
for mechanism in range(4):
    FIELDS[540+2*mechanism]=f'mechanism_{mechanism}_codes'
    FIELDS[541+2*mechanism]=f'mechanism_{mechanism}_values'
WIDE_FIELDS={512:['session','feedback_sequence','feedback_stamp_ns','target_sequence',
    'target_stamp_ns','target_session','target_source_stamp_ns','inference_sequence',
    'depth_stamp_ns','feature_stamp_ns','encoder_calls','actor_calls','io_feedback_sequence','io_feedback_stamp_ns'],
    514:['fault_time_ns','measured_limit_mask','target_limit_mask','motion_frame','motion_frames',
         'motion_generation','gain_mask','hand_mask','control_interval_ns','control_overruns'],
    531:['failure_session'],533:['request_id','request_session'],535:['application_event_drops']}


def checksum(data):
    value = 2166136261
    for byte in data:
        value = ((value ^ byte) * 16777619) & 0xffffffff
    return value


def records(path, issues):
    with Path(path).open('rb') as stream:
        header = stream.read(64)
        if len(header) != 64 or header[:8] != b'EIRFLT01' or struct.unpack_from('<II', header, 8) != (1, 64):
            raise ValueError('unsupported or invalid flight header')
        while True:
            prefix = stream.read(8)
            if not prefix:
                return
            if len(prefix) != 8:
                issues.append('truncated record prefix')
                return
            size, expected = struct.unpack('<II', prefix)
            if not 52 <= size <= 52 + 12288:
                issues.append('invalid record length')
                return
            body = stream.read(size)
            if len(body) != size:
                issues.append('truncated record body')
                return
            if checksum(body) != expected:
                issues.append('checksum mismatch')
                continue
            source, event, flags, system, mono, origin, sequence, correlation, used = struct.unpack_from('<HHIqqqQQI', body)
            if used != size - 52 or source >= len(SOURCES) or event >= len(EVENTS):
                issues.append('invalid record header')
                continue
            row = dict(source=SOURCES[source], event=EVENTS[event], flags=flags, system_ns=system,
                       monotonic_ns=mono, source_ns=origin, sequence=sequence, correlation=correlation)
            offset = 52
            try:
                while offset < size:
                    tag, kind, count = struct.unpack_from('<HHI', body, offset)
                    offset += 8
                    if kind not in (1, 2) or count > 2048:
                        raise ValueError('invalid field')
                    values = struct.unpack_from('<' + str(count) + ('d' if kind == 1 else 'I'), body, offset)
                    offset += count * (8 if kind == 1 else 4)
                    name = FIELDS.get(tag, 'field_' + str(tag))
                    for index, value in enumerate(values):
                        row[f'{name}.{index}'] = value
                    if tag in WIDE_FIELDS and kind == 2 and count % 2 == 0 and count <= 2*len(WIDE_FIELDS[tag]):
                        for index, label in enumerate(WIDE_FIELDS[tag][:count//2]):
                            row[label] = values[2*index] | (values[2*index+1] << 32)
                if offset != size:
                    raise ValueError('field length mismatch')
            except (ValueError, struct.error):
                issues.append('invalid field payload')
                continue
            yield row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input')
    parser.add_argument('--csv', metavar='OUTPUT')
    args = parser.parse_args()
    issues = []
    if args.csv:
        columns = set()
        for row in records(args.input, issues):
            columns.update(row)
        with Path(args.csv).open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=sorted(columns))
            writer.writeheader()
            for row in records(args.input, []):
                writer.writerow(row)
    else:
        for row in records(args.input, issues):
            print(json.dumps(row, ensure_ascii=False))
    for issue in issues:
        print(issue, file=sys.stderr)
    return 2 if issues else 0


if __name__ == '__main__':
    sys.exit(main())
