$dest = 'c:\temp\pw\doc\generated\esp32_src_full_dump.txt'
$files = @(
  'c:\temp\pw\src\esp32\src\step_types.h',
  'c:\temp\pw\src\esp32\src\stepper_queue.h',
  'c:\temp\pw\src\esp32\src\stepper_queue.cpp',
  'c:\temp\pw\src\esp32\src\stepper_driver.h',
  'c:\temp\pw\src\esp32\src\stepper_driver.cpp',
  'c:\temp\pw\src\esp32\src\motion_planner.h',
  'c:\temp\pw\src\esp32\src\motion_planner.cpp',
  'c:\temp\pw\src\esp32\src\messages.h',
  'c:\temp\pw\src\esp32\src\main.cpp',
  'c:\temp\pw\src\esp32\src\comm_interface.h',
  'c:\temp\pw\src\esp32\src\comm_interface.cpp',
  'c:\temp\pw\src\esp32\src\CMakeLists.txt'
)
if (Test-Path $dest) { Remove-Item $dest }
foreach ($f in $files) {
  Add-Content -Path $dest -Value "--- FILE: $f ---"
  Get-Content -Path $f | Add-Content -Path $dest
  Add-Content -Path $dest -Value "--- END FILE ---"
  Add-Content -Path $dest -Value ""
}
Write-Output "Wrote $dest with the full raw concatenated source files."