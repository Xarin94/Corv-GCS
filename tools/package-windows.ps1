# Windows installer: build (unless -SkipBuild), stage the application with
# windeployqt, the Visual C++ runtime next to the executable (app-local, so the
# per-user installer needs no administrator rights) and OpenSSL's libcrypto,
# then NSIS makes dist\CORV-GCS-<version>-setup-x64.exe.
#   powershell -ExecutionPolicy Bypass -File tools\package-windows.ps1 [-SkipBuild] [-Zip] [-NoDownload]
#
# NSIS: makensis on PATH, the NSIS install, or build\tools\nsis-*; OpenSSL:
# OPENSSL_DIR, build\tools\openssl-*\x64 or a system install. What is missing
# is downloaded into build\tools (NSIS 3.11 from SourceForge, OpenSSL 3.5 for
# MSVC from FireDaemon) unless -NoDownload.
param(
    [string]$Preset = "windows-release",
    [switch]$SkipBuild,
    [switch]$Zip,
    [switch]$NoDownload
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$tools = Join-Path $root "build\tools"
$qt = if ($env:QT_DIR) { $env:QT_DIR } else { Join-Path $env:USERPROFILE "Qt\6.12.0\msvc2022_64" }
$version = ([regex]::Match((Get-Content (Join-Path $root "CMakeLists.txt") -Raw), 'project\(CorvGcs VERSION ([0-9.]+)')).Groups[1].Value
if (-not $version) { throw "version not found in CMakeLists.txt" }

function Get-Zip([string]$url, [string]$name, [string]$dest) {
    if ($NoDownload) { throw "$name missing and -NoDownload given" }
    $dl = Join-Path $tools "dl"
    New-Item -ItemType Directory -Force $dl | Out-Null
    $zip = Join-Path $dl "$name.zip"
    if (-not (Test-Path $zip)) {
        Write-Host "Downloading $name ..."
        Invoke-WebRequest -Uri $url -OutFile $zip -UseBasicParsing -UserAgent "curl/8"
    }
    Expand-Archive -Path $zip -DestinationPath $dest -Force
}

# ── NSIS ──
$makensis = (Get-Command makensis.exe -ErrorAction SilentlyContinue).Source
if (-not $makensis -and (Test-Path "${env:ProgramFiles(x86)}\NSIS\makensis.exe")) { $makensis = "${env:ProgramFiles(x86)}\NSIS\makensis.exe" }
if (-not $makensis) {
    $local = Get-ChildItem $tools -Directory -Filter "nsis-*" -ErrorAction SilentlyContinue | Sort-Object Name -Descending | Select-Object -First 1
    if (-not $local) {
        Get-Zip "https://sourceforge.net/projects/nsis/files/NSIS%203/3.11/nsis-3.11.zip/download" "nsis-3.11" $tools
        $local = Get-Item (Join-Path $tools "nsis-3.11")
    }
    $makensis = Join-Path $local.FullName "makensis.exe"
}
Write-Host "NSIS: $makensis"

# ── OpenSSL (the cellular link) ──
$ssl = $env:OPENSSL_DIR
if (-not $ssl) {
    $local = Get-ChildItem $tools -Directory -Filter "openssl-*" -ErrorAction SilentlyContinue | Sort-Object Name -Descending | Select-Object -First 1
    if (-not $local -and -not $NoDownload) {
        Get-Zip "https://download.firedaemon.com/FireDaemon-OpenSSL/openssl-3.5.4.zip" "openssl-3.5.4" (Join-Path $tools "openssl-3.5.4")
        $local = Get-Item (Join-Path $tools "openssl-3.5.4")
    }
    if ($local) { $ssl = Join-Path $local.FullName "x64" }
}
if ($ssl) { $env:OPENSSL_DIR = $ssl; Write-Host "OpenSSL: $ssl" }

# ── SDL3 (joystick input) ──
$sdl = $env:SDL3_DIR
if (-not $sdl) {
    $local = Get-ChildItem (Join-Path $tools "sdl3") -Directory -Filter "SDL3-*" -ErrorAction SilentlyContinue | Sort-Object Name -Descending | Select-Object -First 1
    if (-not $local -and -not $NoDownload) {
        Get-Zip "https://github.com/libsdl-org/SDL/releases/download/release-3.4.18/SDL3-devel-3.4.18-VC.zip" "SDL3-devel-3.4.18-VC" (Join-Path $tools "sdl3")
        $local = Get-Item (Join-Path $tools "sdl3\SDL3-3.4.18")
    }
    if ($local) { $sdl = Join-Path $local.FullName "cmake" }
}
if ($sdl) { $env:SDL3_DIR = $sdl; Write-Host "SDL3: $sdl" }

# ── Build ──
if (-not $SkipBuild) {
    & powershell -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot "build.ps1") -Preset $Preset
    if ($LASTEXITCODE) { throw "build failed" }
}
$out = Join-Path $root "build\$Preset"
$exe = Join-Path $out "corv-gcs.exe"
if (-not (Test-Path $exe)) { throw "$exe not found: build first" }

# ── Stage ──
$dist = Join-Path $root "dist"
$stage = Join-Path $dist "stage\CORV GCS 2"
if (Test-Path $stage) { Remove-Item -Recurse -Force $stage }
New-Item -ItemType Directory -Force $stage | Out-Null
Copy-Item $exe $stage
$cli = Join-Path $out "corv-link-cli.exe"
if (Test-Path $cli) { Copy-Item $cli $stage }

$windeployqt = Join-Path $qt "bin\windeployqt.exe"
$deployArgs = @("--qmldir", (Join-Path $root "libs\ui\qml"), "--release", "--no-compiler-runtime", "--no-translations", "--no-opengl-sw")
& $windeployqt @deployArgs (Join-Path $stage "corv-gcs.exe")
if ($LASTEXITCODE) { throw "windeployqt failed" }
if (Test-Path (Join-Path $stage "corv-link-cli.exe")) {
    & $windeployqt --release --no-compiler-runtime --no-translations --no-opengl-sw (Join-Path $stage "corv-link-cli.exe")
    if ($LASTEXITCODE) { throw "windeployqt (cli) failed" }
}

# Visual C++ runtime, app-local (Microsoft.VC143.CRT of the build tools)
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$vs = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
$crt = Get-ChildItem (Join-Path $vs "VC\Redist\MSVC") -Directory -ErrorAction SilentlyContinue | Sort-Object Name -Descending |
       ForEach-Object { Join-Path $_.FullName "x64\Microsoft.VC143.CRT" } | Where-Object { Test-Path $_ } | Select-Object -First 1
if ($crt) { Copy-Item (Join-Path $crt "*.dll") $stage; Write-Host "VC runtime: $crt" }
else { Write-Warning "Microsoft.VC143.CRT not found: the target needs the Visual C++ 2015-2022 runtime installed" }

# libcrypto when the build links it
$linksCrypto = Select-String -Path $exe -Pattern "libcrypto-3-x64.dll" -SimpleMatch -Quiet
if ($linksCrypto) {
    $dll = if ($ssl) { Join-Path $ssl "bin\libcrypto-3-x64.dll" } else { $null }
    if (-not $dll -or -not (Test-Path $dll)) {
        $dll = Get-ChildItem "C:\Program Files\OpenSSL*\bin\libcrypto-3-x64.dll", "C:\Program Files\OpenSSL*\libcrypto-3-x64.dll" -ErrorAction SilentlyContinue |
               Select-Object -First 1 -ExpandProperty FullName
    }
    if (-not $dll) { throw "the build links libcrypto but libcrypto-3-x64.dll was not found" }
    Copy-Item $dll $stage
} else {
    Write-Warning "built without OpenSSL: the package has no cellular (LTE) link"
}

# SDL3.dll when the build links it (joystick input)
if (Select-String -Path $exe -Pattern "SDL3.dll" -SimpleMatch -Quiet) {
    $sdlDll = if ($sdl) { Join-Path (Split-Path -Parent $sdl) "lib\x64\SDL3.dll" } else { $null }
    if (-not $sdlDll -or -not (Test-Path $sdlDll)) { throw "the build links SDL3 but SDL3.dll was not found" }
    Copy-Item $sdlDll $stage
} else {
    Write-Warning "built without SDL3: the package has no joystick input"
}

Copy-Item (Join-Path $root "LICENSE") $stage
# The bundled LTE cells (CC BY-SA 4.0, see celldb\README.md)
Copy-Item -Recurse (Join-Path $root "resources\celldb") (Join-Path $stage "celldb")
Copy-Item (Join-Path $root "resources\icons\icon.ico") (Join-Path $stage "corv-gcs.ico")
@"
CORV GCS $version - third-party components
==========================================

Qt $((Split-Path -Leaf (Split-Path -Parent $qt))) (LGPLv3), dynamically linked: the Qt6*.dll files and the
plugin folders can be replaced with a compatible build. Sources:
https://download.qt.io/official_releases/qt/ - license text: https://www.gnu.org/licenses/lgpl-3.0.txt

OpenSSL 3 libcrypto (Apache License 2.0), libcrypto-3-x64.dll: https://www.openssl.org/source/
MAVLink C library, generated headers (MIT): https://github.com/mavlink/c_library_v2
Rajdhani fonts (SIL Open Font License 1.1), Roboto Mono fonts (Apache License 2.0)
SDL 3 (zlib license), SDL3.dll: https://github.com/libsdl-org/SDL
LTE cells in celldb\: OpenCelliD (https://opencellid.org, CC BY-SA 4.0) and the Mozilla Location
Service (public domain); the merged file is distributed under CC BY-SA 4.0 (celldb\README.md).
Microsoft Visual C++ runtime (msvcp140*.dll, vcruntime140*.dll, concrt140.dll): redistributed
under the Visual Studio redistribution terms.
"@ | Set-Content -Encoding utf8 (Join-Path $stage "THIRD-PARTY-NOTICES.txt")

# The uninstaller removes exactly what was staged (the data folder stays)
$files = Join-Path $dist "stage\files.nsh"
$lines = @()
Get-ChildItem $stage -Recurse -File | ForEach-Object { $lines += "  Delete `"`$INSTDIR\$($_.FullName.Substring($stage.Length + 1))`"" }
Get-ChildItem $stage -Recurse -Directory | Sort-Object { $_.FullName.Length } -Descending |
    ForEach-Object { $lines += "  RMDir `"`$INSTDIR\$($_.FullName.Substring($stage.Length + 1))`"" }
$lines | Set-Content -Encoding utf8 $files

# ── Installer ──
$setup = Join-Path $dist "CORV-GCS-$version-setup-x64.exe"
& $makensis /V2 "/DVERSION=$version" "/DSTAGE=$stage" "/DOUTFILE=$setup" "/DFILES=$files" (Join-Path $root "packaging\windows\installer.nsi")
if ($LASTEXITCODE) { throw "makensis failed" }
Write-Host ("Installer: {0} ({1:N1} MB)" -f $setup, ((Get-Item $setup).Length / 1MB))

if ($Zip) {
    $zipOut = Join-Path $dist "CORV-GCS-$version-windows-x64.zip"
    if (Test-Path $zipOut) { Remove-Item $zipOut }
    Compress-Archive -Path (Join-Path $stage "*") -DestinationPath $zipOut
    Write-Host "Portable zip: $zipOut"
}
