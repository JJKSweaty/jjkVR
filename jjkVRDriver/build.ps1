$ErrorActionPreference = "Stop"

$vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path -LiteralPath $vswhere)) {
    throw "Visual Studio 2022 C++ Build Tools are required."
}

$installPath = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $installPath) {
    $installPath = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\2019\BuildTools"
}

$msbuild = Join-Path $installPath "MSBuild\Current\Bin\MSBuild.exe"
if (-not (Test-Path -LiteralPath $msbuild)) {
    throw "Visual Studio C++ Build Tools with the v142 toolset are required."
}
& $msbuild (Join-Path $PSScriptRoot "JJKVR_Driver.sln") /m /p:Configuration=Release /p:Platform=x64
if ($LASTEXITCODE -ne 0) {
    throw "JJKVR driver build failed with exit code $LASTEXITCODE."
}

$driver = Join-Path $PSScriptRoot "jjkvr\bin\win64\driver_jjkvr.dll"
if (-not (Test-Path -LiteralPath $driver)) {
    throw "Build completed without producing $driver."
}

Write-Host "Built $driver"
