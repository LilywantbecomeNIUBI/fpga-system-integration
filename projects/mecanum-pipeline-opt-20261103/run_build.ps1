param(
    [string]$GowinRoot = 'C:\Gowin\Gowin_V1.9.12.04_x64',
    [string]$Python = 'python',
    [ValidateSet('all','syn','pnr')][string]$Stage = 'all'
)
$ErrorActionPreference = 'Stop'
$tool = Join-Path $GowinRoot 'IDE/bin/gw_sh.exe'
if (-not (Test-Path -LiteralPath $tool)) { throw "Gowin tool not found: $tool" }
$manifest = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'manifest.json') -Raw | ConvertFrom-Json
foreach ($entry in $manifest.source_sha256.PSObject.Properties) {
    $file = Join-Path (Join-Path $PSScriptRoot 'source') $entry.Name
    if ((Get-FileHash -LiteralPath $file -Algorithm SHA256).Hash.ToLowerInvariant() -ne $entry.Value) {
        throw "Source hash mismatch: $($entry.Name)"
    }
}
$scriptName = switch ($Stage) { 'syn' { 'synthesis.tcl' } 'pnr' { 'pnr.tcl' } default { 'build.tcl' } }
Push-Location $PSScriptRoot
try {
    if ($Stage -ne 'syn') {
        & $Python (Join-Path $PSScriptRoot 'prepare_reference.py')
        if ($LASTEXITCODE -ne 0) { throw 'Incremental reference preparation failed; no clean fallback' }
    }
    & $tool (Join-Path $PSScriptRoot $scriptName) 2>&1 | Tee-Object -FilePath 'build.log'
    if ($LASTEXITCODE -ne 0) { throw "Gowin exited with code $LASTEXITCODE; see build.log" }
} finally { Pop-Location }
