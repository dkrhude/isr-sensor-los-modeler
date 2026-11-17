#!/usr/bin/env python3
"""
run_modeler.py
Cross-platform build & execution orchestrator for isr-sensor-los-modeler.
Automatically compiles the ISO C11 telemetry state machine via MSVC (cl.exe) or GCC,
executes the hardware telemetry benchmark, and runs the 60-minute Python LOS simulation.
"""

import os
import sys
import shutil
import subprocess

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
BIN_DIR = os.path.join(BASE_DIR, "bin")
PROC_DIR = os.path.join(BASE_DIR, "data", "processed")
EXE_NAME = "telemetry_parser.exe" if os.name == "nt" else "telemetry_parser"
EXE_PATH = os.path.join(BIN_DIR, EXE_NAME)


def find_vsdevcmd_windows() -> str:
    """Uses Visual Studio Installer's vswhere.exe to locate VsDevCmd.bat on Windows."""
    vswhere = r"C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe"
    if not os.path.exists(vswhere):
        return ""
    res = subprocess.run(
        [vswhere, "-latest", "-products", "*", "-requires",
         "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-property", "installationPath"],
        capture_output=True, text=True
    )
    install_path = res.stdout.strip()
    if install_path:
        devcmd = os.path.join(install_path, "Common7", "Tools", "VsDevCmd.bat")
        if os.path.exists(devcmd):
            return devcmd
    return ""


def compile_and_run_c_telemetry() -> bool:
    os.makedirs(BIN_DIR, exist_ok=True)
    os.makedirs(PROC_DIR, exist_ok=True)

    c_sources = [
        os.path.join("src", "telemetry_parser.c"),
        os.path.join("src", "main_telemetry_demo.c")
    ]

    compiled = False
    if shutil.which("cl"):
        print("[*] Compiling C11 telemetry parser using active MSVC (cl.exe)...")
        cmd = ["cl", "/nologo", "/O2", "/W3", "/Iinclude"] + c_sources + [f"/Fe:{EXE_PATH}"]
        compiled = (subprocess.run(cmd, cwd=BASE_DIR).returncode == 0)
    elif shutil.which("gcc"):
        print("[*] Compiling C11 telemetry parser using GCC...")
        cmd = ["gcc", "-O2", "-Wall", "-Wextra", "-std=c11", "-Iinclude"] + c_sources + ["-o", EXE_PATH, "-lm"]
        compiled = (subprocess.run(cmd, cwd=BASE_DIR).returncode == 0)
    elif os.name == "nt":
        devcmd = find_vsdevcmd_windows()
        if devcmd:
            print("[*] Initializing Visual Studio x64 C/C++ environment via VsDevCmd.bat...")
            cmdline = (
                f'"{devcmd}" -arch=x64 -no_logo && '
                f'cl /nologo /O2 /W3 /Iinclude src\\telemetry_parser.c src\\main_telemetry_demo.c /Fe:"{EXE_PATH}"'
            )
            compiled = (subprocess.run(["cmd.exe", "/c", cmdline], cwd=BASE_DIR).returncode == 0)

    if not compiled:
        print("[!] C compiler (MSVC cl.exe or GCC) not found. Install 'Desktop development with C++' in VS Installer.")
        print("[*] Proceeding with analytical IMU jitter fallback for Python LOS simulation...\n")
        return False

    # Clean intermediate MSVC .obj files from root if generated
    for fname in ("telemetry_parser.obj", "main_telemetry_demo.obj"):
        fpath = os.path.join(BASE_DIR, fname)
        if os.path.exists(fpath):
            os.remove(fpath)

    subprocess.run([EXE_PATH], cwd=BASE_DIR, check=True)
    return True


if __name__ == "__main__":
    compile_and_run_c_telemetry()
    from sim.sensor_los_sim import run_simulation
    run_simulation()
    print("\n[SUCCESS] Hybrid C11 + Python ISR Sensor LOS Modeling complete.")