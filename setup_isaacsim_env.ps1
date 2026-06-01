param(
    [string]$EnvPath = "",
    [string]$PythonVersion = "3.11",
    [switch]$Force
)

$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
if ([string]::IsNullOrWhiteSpace($EnvPath)) {
    $EnvPath = Join-Path $ProjectRoot ".conda\env_isaacsim_5_1_0"
}

$PythonExe = Join-Path $EnvPath "python.exe"
$RequirementsFile = Join-Path $ProjectRoot "requirements-isaacsim.txt"

if (-not (Test-Path -LiteralPath $RequirementsFile)) {
    throw "Isaac Sim requirements file was not found: $RequirementsFile"
}

if ((Test-Path -LiteralPath $EnvPath) -and -not $Force) {
    Write-Host "Environment already exists: $EnvPath"
    Write-Host "Use -Force to recreate it manually after deleting or moving the existing folder."
    exit 0
}

if ($Force -and (Test-Path -LiteralPath $EnvPath)) {
    throw "Refusing to delete an existing environment automatically: $EnvPath. Move or delete it manually, then rerun."
}

$condaCommand = Get-Command conda -ErrorAction SilentlyContinue
if ($null -eq $condaCommand) {
    throw "conda was not found on PATH. Install Miniconda/Anaconda or open a conda-enabled PowerShell, then rerun."
}

Write-Host "Creating conda environment: $EnvPath"
conda create -p $EnvPath "python=$PythonVersion" -y

if (-not (Test-Path -LiteralPath $PythonExe)) {
    throw "Python executable was not created: $PythonExe"
}

Write-Host "Installing Isaac Sim 5.1 pip packages. This can take a long time."
& $PythonExe -m pip install --upgrade pip
& $PythonExe -m pip install -r $RequirementsFile

Write-Host "Environment ready: $PythonExe"
Write-Host "Run the GUI demo with:"
Write-Host "powershell -ExecutionPolicy Bypass -File .\run_harim_demo.ps1 -AcceptEula -Cycles 1"
