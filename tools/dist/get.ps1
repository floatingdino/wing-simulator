# One-line installer for a released build of the Wing Simulator command-line tools on Windows:
#
#   irm https://github.com/floatingdino/wing-simulator/releases/latest/download/get.ps1 | iex
#
# The copy attached to a release installs that release; set $env:WINGSIM_VERSION = 'v0.3.0' to pick
# another. Installs to %LOCALAPPDATA%\WingSimulator (bin\ is added to your user PATH). Windows has no
# virtual audio device: wing-sim runs control-only (WING-Edit, OSC, scenes) and wing-render renders
# offline. Binaries are not code-signed.
$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'

$repo = if ($env:WINGSIM_REPO) { $env:WINGSIM_REPO } else { 'floatingdino/wing-simulator' }
$version = if ($env:WINGSIM_VERSION) { $env:WINGSIM_VERSION } else { '@VERSION@' }
if ($version -eq ('@' + 'VERSION@') -or $version -eq 'latest') {
    $base = "https://github.com/$repo/releases/latest/download"
    $version = 'latest'
} else {
    $base = "https://github.com/$repo/releases/download/$version"
}
$asset = 'wing-sim-tools-windows-x86_64.zip'

$tmp = Join-Path ([IO.Path]::GetTempPath()) ("wingsim-" + [guid]::NewGuid())
New-Item -ItemType Directory -Path $tmp | Out-Null
try {
    Write-Host "Wing Simulator ($version): downloading $asset"
    Invoke-WebRequest -UseBasicParsing -Uri "$base/$asset" -OutFile (Join-Path $tmp $asset)
    $sums = Join-Path $tmp 'SHA256SUMS.txt'
    try {
        Invoke-WebRequest -UseBasicParsing -Uri "$base/SHA256SUMS.txt" -OutFile $sums
    } catch {
        $sums = $null
        Write-Warning 'no SHA256SUMS.txt in this release; not verified'
    }
    if ($sums) {
        $line = Get-Content $sums | Where-Object { $_ -match "\s\*?$([regex]::Escape($asset))$" }
        $want = (@($line)[0] -split '\s+')[0]
        $got = (Get-FileHash -Algorithm SHA256 (Join-Path $tmp $asset)).Hash
        if (-not $want -or $want.ToLower() -ne $got.ToLower()) { throw "checksum mismatch for $asset" }
        Write-Host 'checksum ok'
    }

    Expand-Archive -Path (Join-Path $tmp $asset) -DestinationPath (Join-Path $tmp 'x') -Force
    $src = Get-ChildItem (Join-Path $tmp 'x') -Directory | Select-Object -First 1
    $dest = Join-Path $env:LOCALAPPDATA 'WingSimulator'
    New-Item -ItemType Directory -Path $dest -Force | Out-Null
    Copy-Item -Path (Join-Path $src.FullName '*') -Destination $dest -Recurse -Force

    $bin = Join-Path $dest 'bin'
    $path = [Environment]::GetEnvironmentVariable('Path', 'User')
    if (-not (($path -split ';') -contains $bin)) {
        [Environment]::SetEnvironmentVariable('Path', ($(if ($path) { "$path;" } else { '' }) + $bin), 'User')
        Write-Host "added $bin to your user PATH (open a new terminal)"
    }
    Write-Host "installed to $dest"
} finally {
    Remove-Item -Recurse -Force $tmp -ErrorAction SilentlyContinue
}
