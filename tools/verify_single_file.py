#!/usr/bin/env python3
"""Check that the two-file C program builds alone and matches the modular demo.

Uses only Python's standard library. Every invocation keeps its own source copy,
input files, command logs, and summary under results/single_file/run-*.
Run from any directory; --reference paths are relative to the repository root.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback
from datetime import datetime, timezone


ROOT = Path(__file__).resolve().parents[1]
FRAME_HEADER = "tick,F,L,R,D,state,elapsed,motor,cleaner,events"
SHUTDOWN = "# shutdown,C:OFF|M:STOP"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def create_run_directory() -> Path:
    base = ROOT / "results" / "single_file"
    base.mkdir(parents=True, exist_ok=True)
    name = "run-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    for suffix in range(10000):
        candidate = base / (name if suffix == 0 else f"{name}-{suffix}")
        try:
            candidate.mkdir()
            return candidate
        except FileExistsError:
            continue
    raise RuntimeError("Could not create a unique result directory.")


def decode_output(value: str | bytes | None) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace").replace("\r\n", "\n")
    return value or ""


def run_command(command: list[str], cwd: Path, logs: Path, label: str,
                timeout: int, environment: dict[str, str]) -> dict:
    started = time.monotonic()
    result = {"command": command, "cwd": str(cwd), "timeout_seconds": timeout,
              "returncode": None, "timed_out": False, "launch_error": None}
    stdout = stderr = ""
    try:
        completed = subprocess.run(command, cwd=cwd, env=environment,
                                   capture_output=True, text=True, encoding="utf-8",
                                   errors="replace", timeout=timeout, check=False)
        result["returncode"] = completed.returncode
        stdout, stderr = completed.stdout, completed.stderr
    except subprocess.TimeoutExpired as error:
        result["timed_out"] = True
        stdout, stderr = decode_output(error.stdout), decode_output(error.stderr)
    except OSError as error:
        result["launch_error"] = str(error)
    result["elapsed_seconds"] = round(time.monotonic() - started, 6)
    for stream, content in (("stdout", stdout), ("stderr", stderr)):
        path = logs / f"{label}.{stream}.txt"
        path.write_text(content, encoding="utf-8")
        result[f"{stream}_log"] = str(path)
        result[stream] = content
    write_json(logs / f"{label}.command.json", result)
    return result


def execution_succeeded(result: dict, expected_code: int) -> bool:
    return (not result["timed_out"] and result["launch_error"] is None
            and result["returncode"] == expected_code)


def frame_checks(stdout: str, inputs: list[tuple[int, int, int, int]]) -> dict[str, bool]:
    lines = stdout.splitlines()
    checks = {"csv_header": bool(lines) and lines[0] == FRAME_HEADER,
              "shutdown_commands": bool(lines) and lines[-1] == SHUTDOWN}
    try:
        frames = list(csv.reader(io.StringIO("\n".join(lines[1:-1]))))
        checks["frame_count"] = len(frames) == len(inputs) + 1
        checks["frame_width"] = all(len(frame) == 10 for frame in frames)
        checks["consecutive_ticks"] = [int(frame[0]) for frame in frames] == list(range(len(inputs) + 1))
        expected_sensors = [inputs[0], *inputs]
        checks["sensor_history"] = [tuple(map(int, frame[1:5])) for frame in frames] == expected_sensors
        checks["initial_state_and_commands"] = bool(frames) and frames[0][5:] == [
            "STOP_OFF", "0", "STOP", "OFF", "M:STOP|C:OFF"]
        checks["valid_states"] = all(frame[5] in {
            "STOP_OFF", "F_ON", "F_POWER1", "LEFT_OFF", "RIGHT_OFF", "BACK_OFF"
        } for frame in frames)
    except (ValueError, IndexError, csv.Error):
        checks["parseable_frames"] = False
    return checks


def expectations(result: dict, case: dict, golden: str) -> dict[str, bool]:
    checks = {"expected_exit_code": execution_succeeded(result, case["exit_code"])}
    stdout, stderr = result["stdout"], result["stderr"]
    if "inputs" in case:
        checks.update(frame_checks(stdout, case["inputs"]))
    if case.get("golden"):
        checks["matches_demo_golden"] = stdout == golden
    if "stderr_exact" in case:
        checks["expected_stderr"] = stderr == case["stderr_exact"]
    elif "stderr_contains" in case:
        checks["expected_stderr"] = case["stderr_contains"] in stderr
    else:
        checks["empty_stderr"] = stderr == ""
    if case.get("empty_stdout"):
        checks["empty_stdout"] = stdout == ""
    if "stdout_contains" in case:
        checks["expected_help"] = all(text in stdout for text in case["stdout_contains"])
    return checks


def write_inputs(path: Path, rows: list[tuple[int, int, int, int]]) -> None:
    text = "front,left,right,dust\n" + "".join(
        ",".join(map(str, row)) + "\n" for row in rows)
    path.write_text(text, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gcc", default="gcc", help="C compiler executable (default: gcc)")
    parser.add_argument("--reference", default="build/cmake/rvc_demo.exe" if os.name == "nt"
                        else "build/cmake/rvc_demo", help="Built modular demo executable")
    args = parser.parse_args()
    run_dir = create_run_directory()
    summary_path = run_dir / "summary.json"
    summary = {"status": "running", "started_utc": datetime.now(timezone.utc).isoformat(),
               "repository": str(ROOT), "run_directory": str(run_dir), "cases": []}
    write_json(summary_path, summary)
    try:
        # The compile working directory starts with exactly these two files.
        isolated = run_dir / "isolated_source"
        isolated.mkdir()
        logs = run_dir / "logs"
        logs.mkdir()
        inputs_dir = run_dir / "inputs"
        inputs_dir.mkdir()
        summary["source_hashes"] = {}
        for name in ("controller.c", "controller.h"):
            source = ROOT / "single_file" / name
            destination = isolated / name
            source_hash = sha256(source)
            shutil.copyfile(source, destination)
            copied_hash = sha256(destination)
            summary["source_hashes"][name] = {"source": str(source), "sha256": source_hash,
                                                "copied_sha256": copied_hash}
            if source_hash != copied_hash:
                raise RuntimeError(f"Source changed while copying {name}; rerun verification.")
        summary["compile_directory_initial_files"] = sorted(path.name for path in isolated.iterdir())
        if summary["compile_directory_initial_files"] != ["controller.c", "controller.h"]:
            raise RuntimeError("Isolated source directory contains unexpected files.")

        environment = os.environ.copy()
        # Avoid project/user include paths inherited from the invoking shell.
        include_variables = ("CPATH", "C_INCLUDE_PATH", "CPLUS_INCLUDE_PATH", "OBJC_INCLUDE_PATH")
        summary["removed_include_environment_variables"] = [name for name in include_variables if name in environment]
        for name in include_variables:
            environment.pop(name, None)
        compiler = shutil.which(args.gcc) or args.gcc
        if Path(compiler).is_file():
            compiler = str(Path(compiler).resolve())
            environment["PATH"] = str(Path(compiler).parent) + os.pathsep + environment.get("PATH", "")
        executable = isolated / ("rvc.exe" if os.name == "nt" else "rvc")
        command = [compiler, "-std=c17", "-Wall", "-Wextra", "-Wpedantic", "-Werror",
                   "controller.c", "-o", executable.name]
        summary["compile"] = run_command(command, isolated, logs, "compile", 60, environment)
        write_json(summary_path, summary)
        if not execution_succeeded(summary["compile"], 0) or not executable.is_file():
            raise RuntimeError("Standalone compilation failed; see compile logs.")
        summary["standalone_executable_sha256"] = sha256(executable)

        reference = Path(args.reference)
        if not reference.is_absolute():
            reference = ROOT / reference
        reference = reference.resolve(strict=True)
        if not reference.is_file():
            raise RuntimeError("Reference executable is not a file.")
        summary["reference"] = {"path": str(reference), "sha256": sha256(reference)}
        golden_path = ROOT / "examples" / "demo_output.csv"
        # Saved PowerShell output may have a UTF-8 BOM; it is an encoding marker,
        # not part of the program's CSV header or behavioral output.
        golden = golden_path.read_text(encoding="utf-8-sig")
        summary["golden"] = {"path": str(golden_path), "sha256": sha256(golden_path)}
        demo_path = ROOT / "scenarios" / "demo.csv"
        with demo_path.open(encoding="utf-8", newline="") as file:
            reader = csv.reader(file)
            if next(reader) != ["front", "left", "right", "dust"]:
                raise RuntimeError("Unexpected demo CSV header.")
            demo_inputs = [tuple(map(int, row)) for row in reader]
        if len(demo_inputs) != 18 or any(len(row) != 4 or any(v not in (0, 1) for v in row) for row in demo_inputs):
            raise RuntimeError("Expected the known 18-tick demo input sequence.")

        combinations = [tuple((bits >> bit) & 1 for bit in (3, 2, 1, 0))
                        for _ in range(8) for bits in range(16)]
        short_rows = [(0, 0, 0, 0), (0, 0, 0, 1), (1, 0, 0, 0)]
        combination_path, short_path = inputs_dir / "combinations.csv", inputs_dir / "realtime.csv"
        invalid_path, empty_path = inputs_dir / "invalid_second_row.csv", inputs_dir / "empty.csv"
        missing_path = inputs_dir / "does_not_exist.csv"
        write_inputs(combination_path, combinations)
        write_inputs(short_path, short_rows)
        write_inputs(invalid_path, [(0, 0, 0, 0), (2, 0, 0, 0)])
        empty_path.write_text("", encoding="utf-8")
        cases = [
            {"name": "builtin_18_ticks", "args": [], "exit_code": 0, "inputs": demo_inputs, "golden": True},
            {"name": "demo_csv", "args": ["--scenario", str(demo_path)], "exit_code": 0,
             "inputs": demo_inputs, "golden": True},
            {"name": "all_16_inputs_repeated_128_ticks", "args": ["--scenario", str(combination_path)],
             "exit_code": 0, "inputs": combinations},
            {"name": "invalid_second_row", "args": ["--scenario", str(invalid_path)], "exit_code": 1,
             "inputs": [(0, 0, 0, 0)], "stderr_exact": "Invalid scenario at line 3.\n"},
            {"name": "empty_csv", "args": ["--scenario", str(empty_path)], "exit_code": 1,
             "empty_stdout": True, "stderr_exact": "Scenario is empty or its first row is invalid.\n"},
            {"name": "missing_csv", "args": ["--scenario", str(missing_path)], "exit_code": 1,
             "empty_stdout": True, "stderr_contains": "scenario:"},
            {"name": "help", "args": ["--help"], "exit_code": 0,
             "stdout_contains": ["rvc_demo [--scenario file.csv]", "CSV: front,left,right,dust", "built-in scenario"]},
            {"name": "invalid_zero_period", "args": ["--realtime-ms", "0"], "exit_code": 1,
             "empty_stdout": True, "stderr_exact": "Period must be an integer from 1 to 60000 ms.\n"},
            {"name": "realtime_three_ticks", "args": ["--scenario", str(short_path), "--realtime-ms", "1"],
             "exit_code": 0, "inputs": short_rows},
        ]
        summary["input_hashes"] = {str(path): sha256(path) for path in [demo_path, combination_path,
                                                                         short_path, invalid_path, empty_path]}
        for case in cases:
            actual = {}
            for label, program in (("reference", reference), ("standalone", executable)):
                actual[label] = run_command([str(program), *case["args"]], isolated, logs,
                                            f"{case['name']}.{label}", 30, environment)
            checks = {label: expectations(result, case, golden) for label, result in actual.items()}
            parity = {key: actual["reference"][key] == actual["standalone"][key]
                      for key in ("returncode", "stdout", "stderr")}
            passed = all(parity.values()) and all(all(values.values()) for values in checks.values())
            summary["cases"].append({"name": case["name"], "status": "passed" if passed else "failed",
                                     "arguments": case["args"], "expected_exit_code": case["exit_code"],
                                     "checks": checks, "equivalence": parity, "executions": actual})
            write_json(summary_path, summary)
        summary["passed_cases"] = sum(case["status"] == "passed" for case in summary["cases"])
        summary["total_cases"] = len(cases)
        summary["status"] = "passed" if summary["passed_cases"] == len(cases) else "failed"
    except (Exception, KeyboardInterrupt) as error:
        summary["status"] = "failed"
        summary["error"] = f"{type(error).__name__}: {error}"
        (run_dir / "verification_error.txt").write_text(traceback.format_exc(), encoding="utf-8")
    summary["finished_utc"] = datetime.now(timezone.utc).isoformat()
    write_json(summary_path, summary)
    print(f"{summary['status'].upper()}: {summary_path}")
    return 0 if summary["status"] == "passed" else 1


if __name__ == "__main__":
    sys.exit(main())
