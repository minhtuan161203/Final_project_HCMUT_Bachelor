param(
    [switch]$SkipInstall,
    [switch]$SkipClean,
    [switch]$UseSystemPython
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $scriptDir

$venvPython = Join-Path $scriptDir ".venv\\Scripts\\python.exe"
if ((-not $UseSystemPython) -and (Test-Path $venvPython))
{
    $pythonExe = $venvPython
}
else
{
    $pythonExe = "python"
}

Write-Host "Using Python:" $pythonExe

if (-not $SkipInstall)
{
    Write-Host "Installing build dependencies..."
    & $pythonExe -m pip install --upgrade pip
    & $pythonExe -m pip install -r requirements.txt pyinstaller
}

if (-not $SkipClean)
{
    Write-Host "Cleaning previous build artifacts..."
    Remove-Item -Recurse -Force build, dist -ErrorAction SilentlyContinue
}

$env:PYTHONNOUSERSITE = "1"

Write-Host "Running PyInstaller spec..."
& $pythonExe -m PyInstaller --noconfirm --clean ASD04_GUI.spec

if ($LASTEXITCODE -ne 0)
{
    throw "PyInstaller build failed with exit code $LASTEXITCODE."
}

$exePath = Join-Path $scriptDir "dist\\ASD04_GUI\\ASD04_GUI.exe"
Write-Host "Build complete:" $exePath
