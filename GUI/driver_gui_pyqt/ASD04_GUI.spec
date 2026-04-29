# -*- mode: python ; coding: utf-8 -*-

from pathlib import Path

from PyInstaller.utils.hooks import copy_metadata


project_dir = Path(SPECPATH)
main_script = project_dir / "main.py"
icon_file = project_dir / "robotic-arm.ico"

hiddenimports = [
    "PyQt6.QtCore",
    "PyQt6.QtGui",
    "PyQt6.QtWidgets",
    "serial",
    "serial.tools.list_ports",
    "serial.tools.list_ports_windows",
    "serial.win32",
]

datas = []
datas += copy_metadata("pyserial")
if icon_file.exists():
    datas.append((str(icon_file), "."))

a = Analysis(
    [str(main_script)],
    pathex=[str(project_dir)],
    binaries=[],
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        "PySide6",
        "serial.tools.list_ports_linux",
        "serial.tools.list_ports_osx",
        "serial.tools.list_ports_posix",
    ],
    noarchive=False,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name="ASD04_GUI",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False,
    icon=str(icon_file) if icon_file.exists() else None,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name="ASD04_GUI",
)
