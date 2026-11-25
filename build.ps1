# Run from a Windows Python 3.11 x64 environment with requirements.txt installed.
$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    $icon = Join-Path $PSScriptRoot 'assets/pidtuner.ico'
    $fwDir = Join-Path $PSScriptRoot 'firmware_example'
    python -m PyInstaller --noconfirm --clean --onefile --windowed --noupx --name PIDTuner --icon $icon --add-data "${icon}:assets" --add-data "${fwDir}:firmware_example" --distpath release --workpath .build/pack --specpath .build --exclude-module PySide6.QtQuick --exclude-module PySide6.QtQml --exclude-module PySide6.QtOpenGLWidgets --exclude-module matplotlib --exclude-module scipy --exclude-module IPython main.py
    if ($LASTEXITCODE -ne 0) { throw "Packaging failed: $LASTEXITCODE" }
} finally { Pop-Location }

