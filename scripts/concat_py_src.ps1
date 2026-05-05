$root = Split-Path -Parent $PSScriptRoot
$destDir = Join-Path $root 'doc/generated'
$dest = Join-Path $destDir 'rpi_python_src_full_dump.txt'

$searchDirs = @(
  (Join-Path $root 'src/rpi'),
  (Join-Path $root 'src/wendy')
)

if (-not (Test-Path $destDir)) {
  New-Item -ItemType Directory -Path $destDir -Force | Out-Null
}

if (Test-Path $dest) {
  Remove-Item $dest
}

$files = @()
foreach ($dir in $searchDirs) {
  if (-not (Test-Path $dir)) {
    continue
  }

  $files += Get-ChildItem -Path $dir -Recurse -File -Filter *.py |
    Where-Object { $_.FullName -notmatch '\\__pycache__\\' } |
    Sort-Object FullName |
    ForEach-Object { $_.FullName }
}

$files = $files | Sort-Object -Unique

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

Write-Output "Wrote $dest with concatenated Python host sources."
