# Run from a Windows Python 3.11 x64 environment with requirements.txt installed.
$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    $icon = Join-Path $PSScriptRoot 'assets/pidtuner.ico'
    python -m PyInstaller --noconfirm --clean --onefile --windowed --noupx --name PIDTuner --icon $icon --add-data "${icon}:assets" --distpath release --workpath .build/pack --specpath .build --exclude-module PySide6.QtQuick --exclude-module PySide6.QtQml --exclude-module PySide6.QtOpenGLWidgets --exclude-module matplotlib --exclude-module scipy --exclude-module IPython main.py
    if ($LASTEXITCODE -ne 0) { throw "Packaging failed: $LASTEXITCODE" }
} finally { Pop-Location }

