# Build the C64 program from Windows through WSL (from ../talonhd/scripts/build-windows.ps1).
# Bake the REU image first: cargo run --release -- bake
param([string]$Distribution = 'Ubuntu-24.04')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$linuxRoot = (& wsl -d $Distribution -- wslpath -a -u $projectRoot.Replace('\', '/')).Trim()
if ($LASTEXITCODE -ne 0) { throw 'Cannot access the WSL build environment.' }
& wsl -d $Distribution -- bash "$linuxRoot/scripts/build-target.sh"
if ($LASTEXITCODE -ne 0) { throw 'MODE 7 C64 build failed.' }
