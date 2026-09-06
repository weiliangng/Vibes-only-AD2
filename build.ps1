[CmdletBinding()]
param(
    [string]$Python = ""
)

$ErrorActionPreference = "Stop"
$projectRoot = $PSScriptRoot
$buildPython = Join-Path $projectRoot ".venv-build\Scripts\python.exe"
$source = Join-Path $projectRoot "tools\ad2_can_monitor.py"
$versionFile = Join-Path $projectRoot "packaging\version_info.txt"

if (-not $Python) {
    $pythonCommand = Get-Command python -ErrorAction SilentlyContinue
    if ($pythonCommand) {
        $Python = $pythonCommand.Source
    } else {
        $Python = Join-Path $env:LOCALAPPDATA "Programs\Python\Python312\python.exe"
    }
}

if (-not (Test-Path -LiteralPath $Python)) {
    throw "Python was not found. Pass its full path with -Python."
}

if (-not (Test-Path -LiteralPath $buildPython)) {
    & $Python -m venv (Join-Path $projectRoot ".venv-build")
}

& $buildPython -m pip install --disable-pip-version-check -r (Join-Path $projectRoot "requirements-build.txt")
if ($LASTEXITCODE -ne 0) { throw "Installing build dependencies failed." }

& $buildPython -m unittest discover -s (Join-Path $projectRoot "tools") -p "test_*.py"
if ($LASTEXITCODE -ne 0) { throw "Tests failed; the executable was not built." }

& $buildPython -m PyInstaller `
    --noconfirm `
    --clean `
    --onefile `
    --console `
    --name "AD2-CAN-Monitor" `
    --version-file $versionFile `
    --paths (Join-Path $projectRoot "tools") `
    --distpath (Join-Path $projectRoot "dist") `
    --workpath (Join-Path $projectRoot "build") `
    --specpath (Join-Path $projectRoot "build") `
    $source
if ($LASTEXITCODE -ne 0) { throw "PyInstaller failed." }

$exe = Join-Path $projectRoot "dist\AD2-CAN-Monitor.exe"
Write-Host "Built $exe"
