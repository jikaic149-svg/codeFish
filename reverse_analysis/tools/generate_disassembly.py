#!/usr/bin/env python3
"""Generate deterministic Xtensa disassembly for key CameraTest functions."""

from __future__ import annotations

import argparse
import json
import subprocess
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class FunctionRange:
    filename: str
    start: int
    stop: int
    description: str


RANGES = (
    FunctionRange("01_control_command_received.asm", 0x420036DC, 0x42003708, "控制到达/退出失联保护"),
    FunctionRange("02_control_tick.asm", 0x42003708, 0x42003A8C, "CPG 与推进周期控制主体"),
    FunctionRange("03_cpg_task.asm", 0x42003A8C, 0x42003AA8, "CPGTask 入口与 20 ms 周期"),
    FunctionRange("04_failsafe_trigger.asm", 0x42003D14, 0x42003D60, "失联保护触发"),
    FunctionRange("05_failsafe_watchdog.asm", 0x42003D60, 0x42003E28, "AP/STA 双链路看门狗"),
    FunctionRange("06_demo_request_guard.asm", 0x42003F18, 0x42003F34, "Demo 启动/取消入口"),
    FunctionRange("07_demo_request_start.asm", 0x42003F34, 0x42003F9C, "Demo 启动路径"),
    FunctionRange("08_demo_tick_guard.asm", 0x42003F9C, 0x42003FC1, "Demo 状态机入口"),
    FunctionRange("09_demo_tick_states_a.asm", 0x42003FC1, 0x42003FEC, "Demo 状态分派前半"),
    FunctionRange("10_demo_tick_states_b.asm", 0x42003FEC, 0x42004070, "Demo 状态分派后半"),
    FunctionRange("11_setup.asm", 0x4200447C, 0x42004864, "setup、摄像头、网络与任务创建"),
    FunctionRange("12_loop.asm", 0x42004864, 0x4200487C, "Arduino loop：安全与状态机调度"),
    FunctionRange("13_partner_http_handlers.asm", 0x42004910, 0x42004E80, "Partner HTTP 处理函数"),
    FunctionRange("14_ota_upload.asm", 0x420054B0, 0x420058EC, "Web OTA 上传处理函数"),
    FunctionRange("15_camera_control.asm", 0x42005FAC, 0x4200657C, "相机参数及机器鱼控制分派"),
    FunctionRange("16_partner_stop.asm", 0x42007288, 0x420072BC, "Partner 脚本安全停止"),
    FunctionRange("17_partner_parse_save.asm", 0x420072BC, 0x420073BC, "Partner 脚本解析与持久化"),
    FunctionRange("18_partner_start.asm", 0x420073BC, 0x42007438, "Partner 脚本启动"),
    FunctionRange("19_partner_tick_guard.asm", 0x42007438, 0x4200745C, "Partner 步骤推进入口"),
    FunctionRange("20_partner_tick_body.asm", 0x4200745C, 0x420074D4, "Partner 步骤及循环推进"),
    FunctionRange("21_partner_get_script.asm", 0x420074D4, 0x420074DC, "读取当前脚本文本"),
    FunctionRange("22_partner_status_idle.asm", 0x420074DC, 0x4200755E, "格式化空闲状态 JSON"),
    FunctionRange("23_partner_status_running.asm", 0x4200755E, 0x42007594, "计算运行中步骤剩余时间"),
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("analysis_root", type=Path)
    parser.add_argument("objdump", type=Path)
    args = parser.parse_args()

    root = args.analysis_root.resolve()
    repository = root.parent
    manifest = json.loads((root / "metadata" / "image_manifest.json").read_text(encoding="utf-8"))
    irom = next(segment for segment in manifest["segments"] if segment["memory_type"] == "IROM")
    segment = root / "segments" / irom["filename"]
    segment_argument = segment.relative_to(repository)
    output_dir = root / "disassembly" / "key_functions"
    output_dir.mkdir(parents=True, exist_ok=True)
    for old_output in output_dir.glob("*.asm"):
        old_output.unlink()

    version = subprocess.run(
        [str(args.objdump.resolve()), "--version"],
        check=True,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    ).stdout.splitlines()[0]

    index_lines = [
        "filename\tstart\tstop\tdescription\n",
    ]
    for item in RANGES:
        command = [
            str(args.objdump.resolve()),
            "-D",
            "-b",
            "binary",
            "-m",
            "xtensa",
            f"--adjust-vma=0x{irom['load_address']:08x}",
            f"--start-address=0x{item.start:08x}",
            f"--stop-address=0x{item.stop:08x}",
            str(segment_argument),
        ]
        result = subprocess.run(
            command,
            cwd=repository,
            check=True,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
        )
        header = (
            f"# {item.description}\n"
            f"# address range: 0x{item.start:08x}..0x{item.stop:08x}\n"
            f"# generator: {version}\n"
            "# Note: inline Xtensa jump tables can appear as instructions in linear output.\n\n"
        )
        (output_dir / item.filename).write_text(header + result.stdout, encoding="utf-8")
        index_lines.append(
            f"{item.filename}\t0x{item.start:08x}\t0x{item.stop:08x}\t{item.description}\n"
        )

    (output_dir / "index.tsv").write_text("".join(index_lines), encoding="utf-8")
    print(f"generated={len(RANGES)}")
    print(f"output={output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
