"""One A39 fixed-address UART sender. No port opens until explicit connection."""
import binascii
import json
import os
from pathlib import Path
import re
import threading
import time

SOURCE = 23120
SEND_INTERVAL = .5  # 2.4 kbit/s radio: latest target only, at most two frames/s.
HEARTBEAT_INTERVAL = 1.0
TARGET_MAX_AGE = 1.0


def frame(sequence, action, payload):
    if action == 'heartbeat':
        text = f'102,1,{SOURCE},{sequence}'
    else:
        code = {'target': 20, 'mode': 22, 'stop': 17, 'attitude_on': 23, 'attitude_off': 24}[action]
        axes = payload.get('axes', {})
        values = [round(axes.get(axis, 0) * 10) for axis in ('surge', 'yaw', 'heave')]
        text = ','.join(map(str, (101, 1, SOURCE, sequence, code, payload['mode'],
                                  *values, round(payload.get('propulsion_scale', 100) * 10))))
    return f'{text},{binascii.crc_hqx(text.encode("ascii"), 0xffff)}\n'.encode('ascii')


class LoRaLink:
    def __init__(self, serial_factory=None, sequence_file=None, clock=time.monotonic):
        self.lock = threading.RLock()
        self.clock = clock
        self.serial_factory = serial_factory
        self.sequence_file = Path(sequence_file or Path(os.environ.get('LOCALAPPDATA', Path.home())) /
                                  'RobotFish' / 'lora-sequence.json')
        self.serial = None
        self.host = self.port = self.error = ''
        self.address, self.channel = 0x0102, 0x20
        self.sequence = self.sent = 0
        self.last_sent = float('-inf')
        self.pending = None
        self.thread = None

    def configure(self, host, port='COM28', address='0102', channel='20', connected=True):
        if type(connected) is not bool:
            raise ValueError('connected must be boolean')
        if not connected:
            self.close()
            return self.status()
        if not isinstance(port, str) or not re.fullmatch(r'COM[1-9][0-9]{0,2}', port.upper()):
            raise ValueError('LoRa port must be COM1..COM999')
        if not isinstance(address, str) or not re.fullmatch(r'[0-9a-fA-F]{4}', address):
            raise ValueError('LoRa address requires four hex digits')
        if not isinstance(channel, str) or not re.fullmatch(r'[0-9a-fA-F]{2}', channel):
            raise ValueError('LoRa channel requires two hex digits')
        self.close()
        factory = self.serial_factory
        if factory is None:
            import serial
            factory = serial.Serial
        with self.lock:
            try:
                if self.sequence_file.exists():
                    value = json.loads(self.sequence_file.read_text(encoding='utf-8'))['sequence']
                    if type(value) is not int or not 0 <= value <= 65535:
                        raise ValueError('invalid saved sequence')
                    self.sequence = value
                self.serial = factory(port.upper(), 115200, timeout=0, write_timeout=.15)
            except (OSError, ValueError, KeyError) as exc:
                self.error = str(exc)
                raise ValueError('LoRa connection failed: ' + str(exc)) from exc
            self.host, self.port = host, port.upper()
            self.address, self.channel = int(address, 16), int(channel, 16)
            self.error = ''
            self.pending = None
            self.last_sent = float('-inf')
            if self.thread is None:
                self.thread = threading.Thread(target=self._run, daemon=True, name='lora-sender')
                self.thread.start()
        return self.status()

    def close(self):
        with self.lock:
            if self.serial:
                self.serial.close()
            self.serial = None
            self.pending = None

    def status(self):
        with self.lock:
            return {'connected': self.serial is not None, 'host': self.host, 'port': self.port,
                    'address': f'{self.address:04X}', 'channel': f'{self.channel:02X}', 'baud': 115200,
                    'source': SOURCE, 'last_sequence': self.sequence, 'sent': self.sent,
                    'error': self.error, 'queued': self.pending is not None}

    def synchronize(self, source, sequence):
        if source != SOURCE or type(sequence) is not int or not 0 <= sequence <= 65535:
            return
        with self.lock:
            if 0 < ((sequence - self.sequence) & 65535) < 32768:
                self.sequence = sequence

    def clear_target(self):
        with self.lock:
            if self.pending and self.pending[0] != 'stop':
                self.pending = None

    def submit(self, host, action, payload):
        with self.lock:
            if self.serial is None or host != self.host:
                raise ValueError('LoRa sender is not connected to this device')
            # STOP cannot be overwritten before its one bounded UART submission.
            if self.pending and self.pending[0] == 'stop' and action != 'stop':
                return {'accepted': False, 'waiting': True, 'transport': 'lora', 'simulated': False}
            self.pending = (action, dict(payload), self.clock())
        return {'accepted': False, 'queued': True, 'transport': 'lora', 'simulated': False}

    def step(self):
        with self.lock:
            now = self.clock()
            if self.serial is None:
                return
            if self.pending and now - self.pending[2] >= TARGET_MAX_AGE:
                self.pending = None
            urgent = self.pending and self.pending[0] == 'stop'
            if now - self.last_sent < (0 if urgent else SEND_INTERVAL if self.pending else HEARTBEAT_INTERVAL):
                return
            action, payload = self.pending[:2] if self.pending else ('heartbeat', {})
            self.pending = None
            sequence = (self.sequence + 1) & 65535
            try:
                # Persist before TX so a process restart never reuses a sent sequence.
                self.sequence_file.parent.mkdir(parents=True, exist_ok=True)
                temp = self.sequence_file.with_suffix('.tmp')
                temp.write_text(json.dumps({'sequence': sequence}), encoding='utf-8')
                temp.replace(self.sequence_file)
                self.sequence = sequence
                data = self.address.to_bytes(2, 'big') + bytes([self.channel]) + frame(sequence, action, payload)
                if self.serial.write(data) != len(data):
                    raise OSError('partial UART frame')
                self.sent += 1
                self.last_sent = now
            except (OSError, ValueError) as exc:
                self.error = str(exc)
                self.close()  # No delayed target, background reconnect or automatic retry.

    def _run(self):
        while True:
            self.step()
            time.sleep(.05)
