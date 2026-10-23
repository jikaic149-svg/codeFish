"""Real A39 bench check: configure with both MD0 low, then test with MD0 high."""
import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import secrets
import tempfile
import time
import zlib

import serial

ROOT = Path(__file__).resolve().parents[2]
PRIVATE = Path(tempfile.gettempdir()) / "robotfish-lora-pair-20261022"
CONFIG = ROOT / "validation/lora-fixed-pair-config.json"
RESULT = ROOT / "validation/lora-fixed-pair-test.json"


class Link:
    def __init__(self, port, bridge=False):
        self.bridge = bridge
        self.pending = bytearray()
        self.status = []
        self.serial = serial.Serial(port=None, baudrate=115200, timeout=0.025,
                                    write_timeout=1)
        self.serial.dtr = self.serial.rts = False
        self.serial.port = port
        self.serial.open()
        if bridge:
            self.serial.write(b"P\n")
            self.receive(0.4)
            if not any(s.startswith("READY LORA_BRIDGE") for s in self.status):
                self.close()
                raise RuntimeError("ESP32 did not identify as the UART test bridge")

    def close(self):
        self.serial.close()

    def receive(self, duration):
        result = bytearray()
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            chunk = self.serial.read(self.serial.in_waiting or 1)
            if not self.bridge:
                result.extend(chunk)
                continue
            self.pending.extend(chunk)
            while b"\n" in self.pending:
                line, _, rest = self.pending.partition(b"\n")
                self.pending[:] = rest
                if line.startswith(b"R "):
                    result.extend(bytes.fromhex(line[2:].decode("ascii")))
                elif line.startswith(b"ERR "):
                    raise RuntimeError(line.decode("ascii"))
                else:
                    self.status.append(line.decode("ascii", errors="replace"))
        return bytes(result)

    def baud(self, value):
        self.receive(0.05)
        if self.bridge:
            self.serial.write(f"B{value}\n".encode())
            self.status.clear()
            self.receive(0.12)
            if f"OK B{value}" not in self.status:
                raise RuntimeError("ESP32 UART baud change was not acknowledged")
        else:
            self.serial.baudrate = value
        self.serial.reset_input_buffer()

    def send(self, data):
        frame = b"T " + data.hex().encode() + b"\n" if self.bridge else data
        if self.serial.write(frame) != len(frame):
            raise RuntimeError("Partial host serial write")
        self.serial.flush()

    def exchange(self, request):
        self.receive(0.04)
        self.send(request)
        return self.receive(0.35)

    def query(self, register, count=1):
        header = bytes((0, register, count))
        response = self.exchange(header)
        if not response.startswith(header) or len(response) == 3:
            raise RuntimeError(f"No valid A39 response on {self.serial.port}, reg {register:02X}")
        return response[3:]

    def save(self, register, payload, count=1):
        header = bytes((0x80, register, count))
        if self.exchange(header + payload) != header:
            raise RuntimeError(f"A39 write not acknowledged, reg {register:02X}")
        if self.query(register, count) != payload:
            raise RuntimeError(f"A39 readback mismatch, reg {register:02X}")


def save_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def configure(links):
    PRIVATE.mkdir(exist_ok=True)
    key_file = PRIVATE / "pair-key.bin"
    if not key_file.exists():
        key_file.write_bytes(secrets.token_bytes(16))
    key = key_file.read_bytes()
    if len(key) != 16:
        raise RuntimeError("Invalid saved pair key length")
    identities = []
    for link in links:
        link.baud(9600)
        version = link.query(0).decode("ascii")
        if not version.startswith("A39-T400A30D1a-9."):
            raise RuntimeError("Unexpected module: " + version)
        identities.append(version)
    original = []
    registers = [4, 5, 6, 7, 8, 11, 20, 23, 24, 25, 26, 27]
    for link, version in zip(links, identities):
        original.append({"port": link.serial.port, "version": version,
                         "registers": {f"{r:02X}": link.query(r).hex() for r in registers}})
    backup = PRIVATE / "original-radio-config.json"
    if not backup.exists():
        save_json(backup, original)
    report = {"time_utc": datetime.now(timezone.utc).isoformat(),
              "mode": "fixed-point", "working_baud": 115200, "uart": "8N1",
              "channel_hex": "0x20", "air_rate_kbps": 2.4,
              "broadcast_addresses_used": False, "automatic_retransmission": False,
              "aes_key_sha256": hashlib.sha256(key).hexdigest(), "nodes": []}
    for number, (link, version) in enumerate(zip(links, identities), 1):
        peer = 3 - number
        values = {4: (115200).to_bytes(4, "big"), 5: b"\0", 6: b"\x04\x19",
                  7: b"\0\x02", 8: b"\x05", 20: b"\0", 23: b"\0\0", 11: key}
        for register, payload in values.items():
            link.save(register, payload)
        addresses = bytes((1, number, 1, peer))
        link.save(24, addresses, count=4)
        report["nodes"].append({"port": link.serial.port, "version": version,
                                "address_hex": f"0x01{number:02X}",
                                "peer_address_hex": f"0x01{peer:02X}",
                                "all_written_values_readback_verified": True})
        print(f"CONFIGURED {link.serial.port}: {version}, 01{number:02X} -> 01{peer:02X}", flush=True)
    report["both_keys_readback_matched"] = all(link.query(11) == key for link in links)
    report["complete"] = len(report["nodes"]) == 2 and report["both_keys_readback_matched"]
    save_json(CONFIG, report)
    if not report["complete"]:
        raise RuntimeError("Pair configuration is incomplete")
    print("CONFIGURATION_VERIFIED: both MD0 pins may now be changed to 3.3V", flush=True)


def run_test(links, count, floating_stage=None):
    config = json.loads(CONFIG.read_text(encoding="utf-8"))
    if not config.get("complete"):
        raise RuntimeError("Both modules must first pass configuration readback")
    destination_path = (ROOT / f"validation/lora-floating-{floating_stage}.json"
                        if floating_stage else RESULT)
    for link in links:
        link.baud(115200)
    run_id = secrets.token_hex(4)
    records, negatives = [], []
    for index, sender in enumerate(links):
        receiver = links[1 - index]
        source, destination = index + 1, 2 - index
        for sequence in range(1, count + 1):
            body = f"LTEST|{run_id}|{source}>{destination}|{sequence:03}|{secrets.token_hex(4)}".encode()
            payload = body + f"|{zlib.crc32(body):08X}\n".encode()
            receiver.receive(0.08)
            sender.receive(0.05)
            started = time.monotonic()
            sender.send(bytes((1, destination, 0x20)) + payload)
            got = bytearray()
            deadline = started + 3
            while len(got) < len(payload) and time.monotonic() < deadline:
                got.extend(receiver.receive(0.06))
            elapsed = (time.monotonic() - started) * 1000
            got.extend(receiver.receive(0.06))
            row = {"direction": f"{source}->{destination}", "sequence": sequence,
                   "expected_hex": payload.hex(), "received_hex": bytes(got).hex(),
                   "exact_match": bytes(got) == payload, "host_elapsed_ms": round(elapsed, 2)}
            records.append(row)
            print(f"{source}->{destination} {sequence}/{count}: {'PASS' if row['exact_match'] else 'FAIL'} {elapsed:.1f}ms", flush=True)
            if not row["exact_match"] and not floating_stage:
                save_json(RESULT, {"time_utc": datetime.now(timezone.utc).isoformat(),
                                  "run_id": run_id, "configuration": config,
                                  "records": records, "wrong_address_checks": negatives,
                                  "all_passed": False, "complete": False})
                raise RuntimeError("RF packet mismatch; stopped before further transmissions")
        if floating_stage:
            continue
        receiver.receive(0.1)
        sender.send(b"\x01\x7E\x20" + f"NEG|{run_id}|{source}\n".encode())
        unexpected = receiver.receive(1.5)
        negatives.append({"direction": f"{source}->{destination}", "destination_hex": "0x017E",
                          "correctly_filtered": not unexpected, "unexpected_hex": unexpected.hex()})
    report = {"time_utc": datetime.now(timezone.utc).isoformat(), "run_id": run_id,
              "configuration": config, "packets_per_direction": count,
              "complete": True,
              "mode_pin_condition": ("MD0 high, MD1 low" if floating_stage in (None, "recovery")
                                     else "both modules: MD0 and MD1 disconnected"),
              "experiment_stage": floating_stage or "normal",
              "powercycled_before_measurement": floating_stage == "cold_start",
              "records": records, "wrong_address_checks": negatives,
              "all_passed": all(r["exact_match"] for r in records)
                            and all(r["correctly_filtered"] for r in negatives)}
    save_json(destination_path, report)
    if floating_stage:
        for direction in ["1->2", "2->1"]:
            successes = sum(r["exact_match"] for r in records if r["direction"] == direction)
            print(f"FLOATING {floating_stage} {direction}: {successes}/{count} received", flush=True)
        return
    if not report["all_passed"]:
        raise RuntimeError("Radio test failed; see " + str(RESULT))
    print(f"BOTH_DIRECTIONS_PASS: {count}/{count} each; wrong-address frames filtered", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=["configure", "test", "floating"])
    parser.add_argument("--esp", default="COM29")
    parser.add_argument("--usb", default="COM28")
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--stage", choices=["powered", "cold_start", "recovery"], default="powered")
    args = parser.parse_args()
    if not 1 <= args.count <= 100:
        parser.error("count must be 1..100")
    with ExitStack() as stack:
        links = [Link(args.esp, bridge=True)]
        stack.callback(links[0].close)
        links.append(Link(args.usb))
        stack.callback(links[1].close)
        if args.phase == "configure":
            configure(links)
        else:
            run_test(links, args.count, args.stage if args.phase == "floating" else None)
