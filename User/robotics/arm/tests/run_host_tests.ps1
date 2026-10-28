$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = Resolve-Path (Join-Path $ScriptDir "..\..\..\..")
$OutDir = Join-Path $RepoRoot "build\host_tests"
$Exe = Join-Path $OutDir "arm_host_tests.exe"

New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

Push-Location $RepoRoot
try {
  & g++ -std=c++17 -Wall -Wextra -Wpedantic -fno-exceptions -fno-rtti `
    -IUser `
    User/robotics/arm/tests/arm_host_tests.cpp `
    User/robotics/core/rotation.cpp `
    User/robotics/core/transform.cpp `
    -o $Exe
  if ($LASTEXITCODE -ne 0) {
    throw "g++ failed with exit code $LASTEXITCODE"
  }

  & $Exe
  if ($LASTEXITCODE -ne 0) {
    throw "arm host tests failed with exit code $LASTEXITCODE"
  }
} finally {
  Pop-Location
}
