$root = Split-Path -Parent $PSScriptRoot
$esp32Dir = Join-Path $root 'src/esp32'
$esp32SrcDir = Join-Path $esp32Dir 'src'
$destDir = Join-Path $root 'doc/generated'
$dest = Join-Path $destDir 'esp32_src_full_dump.txt'

if (-not (Test-Path $destDir)) {
  New-Item -ItemType Directory -Path $destDir -Force | Out-Null
}

$files = @(
  (Join-Path $esp32Dir 'CMakeLists.txt'),
  (Join-Path $esp32SrcDir 'CMakeLists.txt'),
  (Join-Path $esp32SrcDir 'main.cpp')
)

$commDir = Join-Path $esp32SrcDir 'comm'
$motionDir = Join-Path $esp32SrcDir 'motion'
$dynamicFiles = @()

foreach ($dir in @($commDir, $motionDir)) {
  if (Test-Path $dir) {
    $dynamicFiles += Get-ChildItem -Path $dir -Recurse -File -Include *.h, *.cpp |
      Sort-Object FullName |
      ForEach-Object { $_.FullName }
  }
}

$files += $dynamicFiles

if (Test-Path $dest) { Remove-Item $dest }

foreach ($f in $files) {
  if (-not (Test-Path $f)) {
    Write-Warning "File not found: $f"
    continue
  }
  Add-Content -Path $dest -Value "--- FILE: $f ---"
  Get-Content -Path $f | Add-Content -Path $dest
  Add-Content -Path $dest -Value ""
  Add-Content -Path $dest -Value "--- END FILE ---"
  Add-Content -Path $dest -Value ""
}

Write-Output "Wrote $dest with the full raw concatenated source files."