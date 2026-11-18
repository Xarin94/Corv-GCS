# Local Windows build: MSVC environment from the Visual Studio 2022 Build Tools,
# Qt from QT_DIR (default C:\Users\<you>\Qt\6.12.0\msvc2022_64), Ninja and CMake
# from Visual Studio. OpenSSL (libcrypto, for the cellular link) from OPENSSL_DIR,
# else from build\tools\openssl-*\x64 (tools\package-windows.ps1 fetches it),
# else from a system install; without it the build has no cellular link.
#   powershell -ExecutionPolicy Bypass -File tools\build.ps1 [-Preset windows-release] [-Test] [-Run]
param(
    [string]$Preset = "windows-release",
    [switch]$Test,
    [switch]$Run
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$qt = if ($env:QT_DIR) { $env:QT_DIR } else { Join-Path $env:USERPROFILE "Qt\6.12.0\msvc2022_64" }
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$vs = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vs) { throw "Visual Studio 2022 with the C++ tools not found" }

# Import the x64 developer environment into this PowerShell
$envDump = cmd /c "`"$vs\VC\Auxiliary\Build\vcvars64.bat`" >nul && set"
foreach ($line in $envDump) {
    if ($line -match '^([^=]+)=(.*)$') { [Environment]::SetEnvironmentVariable($matches[1], $matches[2]) }
}
$env:PATH = "$vs\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin;$vs\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja;$qt\bin;$env:PATH"

$ssl = $env:OPENSSL_DIR
if (-not $ssl) {
    $local = Get-ChildItem (Join-Path $root "build\tools") -Directory -Filter "openssl-*" -ErrorAction SilentlyContinue |
             Sort-Object Name -Descending | Select-Object -First 1
    if ($local -and (Test-Path (Join-Path $local.FullName "x64\lib\libcrypto.lib"))) { $ssl = Join-Path $local.FullName "x64" }
}
$sslArgs = @()
if ($ssl) {
    $sslArgs = @("-DOPENSSL_ROOT_DIR=$ssl", "-DOPENSSL_USE_STATIC_LIBS=OFF")
    # libcrypto-3-x64.dll for the tests and -Run
    $env:PATH = "$ssl\bin;$env:PATH"
}

# SDL3 (joystick input) from SDL3_DIR, else build\tools\sdl3\SDL3-*\cmake (tools\package-windows.ps1 fetches it)
$sdl = $env:SDL3_DIR
if (-not $sdl) {
    $local = Get-ChildItem (Join-Path $root "build\tools\sdl3") -Directory -Filter "SDL3-*" -ErrorAction SilentlyContinue |
             Sort-Object Name -Descending | Select-Object -First 1
    if ($local -and (Test-Path (Join-Path $local.FullName "cmake\SDL3Config.cmake"))) { $sdl = Join-Path $local.FullName "cmake" }
}
if ($sdl) {
    $sslArgs += "-DSDL3_DIR=$sdl"
    $env:PATH = "$(Join-Path (Split-Path -Parent $sdl) 'lib\x64');$env:PATH"
}

Push-Location $root
try {
    cmake --preset $Preset -DCMAKE_PREFIX_PATH="$qt" @sslArgs
    if ($LASTEXITCODE) { throw "configure failed" }
    cmake --build --preset $Preset
    if ($LASTEXITCODE) { throw "build failed" }
    if ($Test) {
        ctest --preset $Preset
        if ($LASTEXITCODE) { throw "tests failed" }
    }
    if ($Run) { & "$root\build\$Preset\app\corv-gcs.exe" }
} finally {
    Pop-Location
}
