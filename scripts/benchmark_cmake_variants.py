#!/usr/bin/env python3

import argparse
import os
import pathlib
import re
import select
import shutil
import socket
import subprocess
import sys
import termios
import threading
import time


VARIANTS = ("serial", "openmp", "sycl")
PROCESS_SAMPLE_PATTERN = re.compile(r"\[profile\] process=([0-9.]+) ms")
RENDER_SAMPLE_PATTERN = re.compile(r"\[profile\] render=([0-9.]+) ms draw=([0-9.]+) ms")


def run(cmd: str, cwd: pathlib.Path) -> None:
    subprocess.run(["bash", "-lc", cmd], cwd=cwd, check=True)


def configure_tty(fd: int) -> None:
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 1
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def capture_serial(
    device: pathlib.Path,
    output: pathlib.Path,
    target_process_samples: int,
) -> int:
    fd = None
    process_samples = 0
    line_buffer = b""
    with output.open("wb") as out:
        while process_samples < target_process_samples:
            if fd is None:
                try:
                    fd = os.open(device, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
                    configure_tty(fd)
                except OSError:
                    continue
            try:
                ready, _, _ = select.select([fd], [], [], 0.2)
                if not ready:
                    continue
                chunk = os.read(fd, 4096)
                if chunk:
                    out.write(chunk)
                    out.flush()
                    line_buffer += chunk
                    complete_lines = line_buffer.split(b"\n")
                    line_buffer = complete_lines.pop()
                    for line in complete_lines:
                        if PROCESS_SAMPLE_PATTERN.search(line.decode("latin1", "ignore")):
                            process_samples += 1
                            if process_samples >= target_process_samples:
                                break
                else:
                    os.close(fd)
                    fd = None
            except OSError:
                if fd is not None:
                    os.close(fd)
                    fd = None
    if fd is not None:
        os.close(fd)
    return process_samples


def wait_for_port(port: int) -> None:
    while True:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.2):
                return
        except OSError:
            time.sleep(0.1)


def build_variant(repo: pathlib.Path, build_dir: pathlib.Path, variant: str, jobs: int) -> pathlib.Path:
    run(
        "source ~/miosix/compilers/gcc.sh && "
        f"cmake --fresh -S . -B {build_dir.name} "
        f"-DTHERMAL_CAMERA_FRAME_PROCESS_VARIANT={variant} && "
        f"cmake --build {build_dir.name} -j{jobs}",
        repo,
    )
    return build_dir / "main.elf"


def run_variant(
    repo: pathlib.Path,
    elf: pathlib.Path,
    variant: str,
    serial_device: pathlib.Path,
    target_process_samples: int,
    out_dir: pathlib.Path,
) -> tuple[pathlib.Path, int]:
    openocd_cfg = repo / "miosix-kernel/miosix/arch/cortexM0plus_rp2040/rp2040_raspberry_pi_pico/openocd.cfg"
    openocd_log = out_dir / f"openocd_{variant}.log"
    gdb_log = out_dir / f"gdb_{variant}.log"
    serial_log = out_dir / f"{variant}.log"

    with openocd_log.open("w") as log:
        openocd = subprocess.Popen(
            ["openocd", "-f", str(openocd_cfg)],
            cwd=repo,
            stdout=log,
            stderr=subprocess.STDOUT,
        )
        try:
            wait_for_port(3333)
            captured_process_samples: int | None = None

            def capture_worker() -> None:
                nonlocal captured_process_samples
                captured_process_samples = capture_serial(
                    serial_device,
                    serial_log,
                    target_process_samples,
                )

            capture_thread = threading.Thread(target=capture_worker, daemon=True)
            capture_thread.start()
            gdb_cmd = (
                "source ~/miosix/compilers/gcc.sh && "
                f"arm-miosix-eabi-gdb -q {elf} "
                "-ex 'set confirm off' "
                "-ex 'target extended-remote :3333' "
                "-ex 'monitor reset halt' "
                "-ex 'load' "
                "-ex 'monitor reset run' "
                "-ex 'detach' "
                "-ex 'quit'"
            )
            with gdb_log.open("w") as glog:
                subprocess.run(["bash", "-lc", gdb_cmd], cwd=repo, stdout=glog, stderr=subprocess.STDOUT, check=True)
            capture_thread.join()
        finally:
            openocd.terminate()
            try:
                openocd.wait(timeout=5)
            except subprocess.TimeoutExpired:
                openocd.kill()
                openocd.wait()
    if captured_process_samples is None:
        raise RuntimeError(f"Serial capture failed for {variant}")
    return serial_log, captured_process_samples


def parse_stats(log_path: pathlib.Path, warmup_samples: int, measured_samples: int) -> dict[str, float | int | None]:
    text = log_path.read_bytes().decode("latin1", "ignore")
    process = [float(x) for x in PROCESS_SAMPLE_PATTERN.findall(text)]
    render_draw = [(float(a), float(b)) for a, b in RENDER_SAMPLE_PATTERN.findall(text)]
    render = [a for a, _ in render_draw]
    draw = [b for _, b in render_draw]
    stable_process = process[warmup_samples:warmup_samples + measured_samples]
    stable_render = render[warmup_samples:warmup_samples + measured_samples]
    stable_draw = draw[warmup_samples:warmup_samples + measured_samples]

    def avg(values: list[float]) -> float | None:
        return round(sum(values) / len(values), 3) if values else None

    return {
        "process_count": len(stable_process),
        "process_total_count": len(process),
        "process_avg_ms": avg(stable_process),
        "process_min_ms": round(min(stable_process), 3) if stable_process else None,
        "process_max_ms": round(max(stable_process), 3) if stable_process else None,
        "render_avg_ms": avg(stable_render),
        "draw_avg_ms": avg(stable_draw),
        "dropped_frames": len(re.findall(r"Dropped frame", text)),
        "hardfaults": len(re.findall(r"Unexpected HardFault", text)),
    }


def print_summary(results: dict[str, dict[str, float | int | None]]) -> None:
    headers = (
        "variant",
        "process_avg",
        "process_min",
        "process_max",
        "render_avg",
        "draw_avg",
        "samples",
        "drops",
        "faults",
    )
    rows = [headers]
    for variant in VARIANTS:
        stats = results[variant]
        rows.append(
            (
                variant,
                str(stats["process_avg_ms"]),
                str(stats["process_min_ms"]),
                str(stats["process_max_ms"]),
                str(stats["render_avg_ms"]),
                str(stats["draw_avg_ms"]),
                str(stats["process_count"]),
                str(stats["dropped_frames"]),
                str(stats["hardfaults"]),
            )
        )
    widths = [max(len(row[i]) for row in rows) for i in range(len(headers))]
    for index, row in enumerate(rows):
        print("  ".join(cell.ljust(widths[i]) for i, cell in enumerate(row)))
        if index == 0:
            print("  ".join("-" * width for width in widths))


def main() -> int:
    parser = argparse.ArgumentParser(description="Build and benchmark the CMake serial/openmp/sycl variants on the board.")
    parser.add_argument("--repo", type=pathlib.Path, default=pathlib.Path(__file__).resolve().parents[1])
    parser.add_argument("--serial-device", type=pathlib.Path, default=pathlib.Path("/dev/ttyACM0"))
    parser.add_argument("--warmup-samples", type=int, default=10, help="Number of initial process samples to discard.")
    parser.add_argument("--samples", type=int, default=100, help="Number of post-warmup samples to collect per variant.")
    parser.add_argument("--jobs", type=int, default=os.cpu_count() or 8)
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--output-dir", type=pathlib.Path, default=pathlib.Path("/tmp/thermal_camera_bench"))
    args = parser.parse_args()

    if args.warmup_samples < 0:
        print("--warmup-samples must be >= 0", file=sys.stderr)
        return 1
    if args.samples <= 0:
        print("--samples must be > 0", file=sys.stderr)
        return 1

    if shutil.which("openocd") is None:
        print("openocd not found in PATH", file=sys.stderr)
        return 1

    args.output_dir.mkdir(parents=True, exist_ok=True)

    elfs: dict[str, pathlib.Path] = {}
    for variant in VARIANTS:
        build_dir = args.repo / f"build-bench-{variant}"
        if args.skip_build:
            elfs[variant] = build_dir / "main.elf"
        else:
            print(f"Building {variant}...")
            elfs[variant] = build_variant(args.repo, build_dir, variant, args.jobs)

    results: dict[str, dict[str, float | int | None]] = {}
    target_process_samples = args.warmup_samples + args.samples
    for variant in VARIANTS:
        print(f"Running {variant}...")
        log_path, captured_process_samples = run_variant(
            args.repo,
            elfs[variant],
            variant,
            args.serial_device,
            target_process_samples,
            args.output_dir,
        )
        results[variant] = parse_stats(log_path, args.warmup_samples, args.samples)
        if captured_process_samples != target_process_samples:
            print(f"{variant} captured {captured_process_samples}/{target_process_samples} process samples", file=sys.stderr)
            return 1
        if results[variant]["process_count"] != args.samples:
            print(
                f"{variant} collected {results[variant]['process_count']}/{args.samples} "
                "post-warmup process samples",
                file=sys.stderr,
            )
            return 1

    print()
    print_summary(results)
    print()
    print(f"Logs saved in {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
