<#
.SYNOPSIS
    Opens the Understand-Anything dashboard for this repo's knowledge graph.

.DESCRIPTION
    Serves .ua/knowledge-graph.json and source previews from this checkout on
    127.0.0.1, behind a one-time token. The tokenized URL is printed and opened
    in the browser. Stop with Ctrl+C.

    Default: the standalone read-only viewer. Needs only Node.js >= 18; no
    Claude Code, no LLM. The viewer tarball is downloaded once from the
    Understand-Anything GitHub release, checked against the SHA-256 pinned
    below, and cached under %LOCALAPPDATA%\understand-anything-viewer.

    -Local: the dashboard of an installed Understand-Anything Claude Code
    plugin instead (no download).

    The graph shows the code as of the commit in .ua/meta.json; rebuild or
    update it with /understand in Claude Code.

    Installing Node.js:
        winget install OpenJS.NodeJS.LTS    # into %ProgramFiles%\nodejs
    Alternative with uv (nodejs-wheel is an unofficial PyPI repackaging of
    Node.js LTS):
        uv tool install nodejs-wheel        # node, npm, npx into %USERPROFILE%\.local\bin
        uv tool update-shell                # optional: put that folder on PATH
    This script also looks in both folders, so neither has to be on PATH.

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File .ua\dashboard.ps1

.EXAMPLE
    .\.ua\dashboard.ps1 -Port 5180 -NoOpen
#>
param(
    [int]$Port,         # default 5173, next free port if taken (an explicit -Port must be free)
    [switch]$NoOpen,    # print the URL without opening the browser
    [switch]$Local      # use the installed Claude Code plugin's dashboard
)
$ErrorActionPreference = 'Stop'

# Pinned viewer release. To upgrade: pick a release that ships
# understand-anything-viewer.tgz and copy its sha256 digest from the release page.
# Release page: https://github.com/Egonex-AI/Understand-Anything/releases/tag/v2.9.0
$ViewerVersion = 'v2.9.0'
$ViewerSha256  = 'a8626ff3ad90041e807bfdb8994eefdd986e891593c4759d08222667e5405330'

$RepoRoot = Split-Path -Parent $PSScriptRoot
if (-not (Test-Path (Join-Path $PSScriptRoot 'knowledge-graph.json'))) {
    throw "No knowledge graph at $PSScriptRoot\knowledge-graph.json. Build it with /understand in Claude Code."
}

# Node.js >= 18: from PATH, else the nodejs.org installer's folder or uv's tool folder
if (-not (Get-Command node -ErrorAction SilentlyContinue)) {
    $nodeDir = @((Join-Path $env:ProgramFiles 'nodejs'), (Join-Path $HOME '.local\bin')) |
        Where-Object { Test-Path (Join-Path $_ 'node.exe') } |
        Select-Object -First 1
    if ($nodeDir) { $env:Path = "$nodeDir;$env:Path" }
    else { throw 'Node.js >= 18 is required: run "winget install OpenJS.NodeJS.LTS" (or "uv tool install nodejs-wheel").' }
}
$nodeVersion = (& node --version).Trim()
if ([int]($nodeVersion.TrimStart('v').Split('.')[0]) -lt 18) {
    throw "Node.js >= 18 is required; found $nodeVersion."
}

if ($Local) {
    $pluginCache = Join-Path $HOME '.claude\plugins\cache\understand-anything\understand-anything'
    $dashboard = Get-ChildItem $pluginCache -Directory -ErrorAction SilentlyContinue |
        Where-Object { $_.Name -as [version] } |
        Sort-Object { [version]$_.Name } -Descending |
        ForEach-Object { Join-Path $_.FullName 'packages\dashboard' } |
        Where-Object { Test-Path (Join-Path $_ 'node_modules\.bin\vite.cmd') } |
        Select-Object -First 1
    if (-not $dashboard) { throw "No installed plugin dashboard under $pluginCache. Run without -Local." }

    $env:GRAPH_DIR = $RepoRoot
    if ($NoOpen) { $env:BROWSER = 'none' }
    $viteArgs = @('--host', '127.0.0.1')
    if ($PSBoundParameters.ContainsKey('Port')) { $viteArgs += @('--port', $Port, '--strictPort') }
    Push-Location $dashboard
    try { & (Join-Path $dashboard 'node_modules\.bin\vite.cmd') @viteArgs }
    finally { Pop-Location }
    return
}

# Standalone viewer: download once, verify, extract, run with node (it has no dependencies)
$cacheDir = Join-Path $env:LOCALAPPDATA "understand-anything-viewer\$ViewerVersion"
$tarball  = Join-Path $cacheDir 'understand-anything-viewer.tgz'
$viewer   = Join-Path $cacheDir 'package\bin\viewer.mjs'

function Test-Tarball { (Test-Path $tarball) -and ((Get-FileHash $tarball -Algorithm SHA256).Hash -eq $ViewerSha256) }

if (-not (Test-Tarball)) {
    New-Item -ItemType Directory -Force $cacheDir | Out-Null
    $url = "https://github.com/Egonex-AI/Understand-Anything/releases/download/$ViewerVersion/understand-anything-viewer.tgz"
    Write-Host "Downloading viewer $ViewerVersion ..."
    [Net.ServicePointManager]::SecurityProtocol = [Net.ServicePointManager]::SecurityProtocol -bor [Net.SecurityProtocolType]::Tls12
    $ProgressPreference = 'SilentlyContinue'
    Invoke-WebRequest -Uri $url -OutFile "$tarball.part" -UseBasicParsing
    $hash = (Get-FileHash "$tarball.part" -Algorithm SHA256).Hash
    if ($hash -ne $ViewerSha256) {
        Remove-Item "$tarball.part" -Force
        throw "Viewer download failed verification: SHA-256 $hash, expected $ViewerSha256."
    }
    Move-Item "$tarball.part" $tarball -Force
    if (Test-Path (Join-Path $cacheDir 'package')) { Remove-Item (Join-Path $cacheDir 'package') -Recurse -Force }
}
if (-not (Test-Path $viewer)) {
    & "$env:SystemRoot\System32\tar.exe" -xzf $tarball -C $cacheDir
    if ($LASTEXITCODE -ne 0 -or -not (Test-Path $viewer)) { throw "Could not extract $tarball." }
}

$viewerArgs = @($viewer, $RepoRoot)
if ($PSBoundParameters.ContainsKey('Port')) { $viewerArgs += @('--port', $Port) }
if ($NoOpen) { $viewerArgs += '--no-open' }
& node @viewerArgs
