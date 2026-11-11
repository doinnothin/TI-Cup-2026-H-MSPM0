param([string]$ProjectRoot = (Split-Path $PSScriptRoot -Parent),
      [string]$BaselineRoot = $ProjectRoot,
      [string]$Gcc = '')
$ErrorActionPreference = 'Stop'
if ([string]::IsNullOrWhiteSpace($Gcc)) {
    if (-not [string]::IsNullOrWhiteSpace($env:ARM_GCC_ROOT)) {
        $Gcc = Join-Path $env:ARM_GCC_ROOT 'bin\arm-none-eabi-gcc.exe'
    } else {
        $GccCommand = Get-Command arm-none-eabi-gcc -ErrorAction Stop
        $Gcc = $GccCommand.Source
    }
}
if (-not (Test-Path -LiteralPath $Gcc -PathType Leaf)) {
    throw "Arm GNU compiler not found at '$Gcc'. Set ARM_GCC_ROOT or pass -Gcc."
}
Push-Location $ProjectRoot
try {
    $commonArgs = @('-mcpu=cortex-m0plus','-mthumb','-O0','-g','-std=c11','-Wall','-Wextra','-ffunction-sections','-fdata-sections','-nostartfiles','-specs=nosys.specs','-Wl,-e,main,-Ttext=0x1000,-Tdata=0x20000000,--gc-sections','-I','tests/q5_stubs','-I','user','-I',(Join-Path $BaselineRoot 'user'),'-I',(Join-Path $BaselineRoot 'user/app/q4'))
    $sources = @('tests/q6_target_test.c','user/app/q4/q5_vehicle_model.c','user/app/q4/q45_vehicle_control.c','user/app/q6/q6_target.c',(Join-Path $BaselineRoot 'user/app/q3/pendulum_geometry.c'),(Join-Path $BaselineRoot 'user/app/q3/ball_state_estimator.c'))

    $testElf = Join-Path $ProjectRoot 'tests\q6_target_test.elf'
    & $Gcc @commonArgs @sources -lm -lc -o $testElf
    if ($LASTEXITCODE -ne 0) { throw 'Q6 test compilation failed' }
    python -B tests/run_q5_test.py $testElf --deps (Join-Path $BaselineRoot 'build/testdeps')
    if ($LASTEXITCODE -ne 0) { throw 'Q6 ARM regressions failed' }

    $staticElf = Join-Path $ProjectRoot 'tests\q6_static_test.elf'
    $staticSources = @('tests/q6_static_test.c','user/app/q4/q5_vehicle_model.c','user/app/q4/q45_vehicle_control.c','user/app/q6/q6_target.c',(Join-Path $BaselineRoot 'user/app/q3/pendulum_geometry.c'),(Join-Path $BaselineRoot 'user/app/q3/ball_state_estimator.c'))
    & $Gcc @commonArgs '-DQ6_STATIC_TEST_MODE=1' @staticSources -lm -lc -o $staticElf
    if ($LASTEXITCODE -ne 0) { throw 'Q6 static-image compilation failed' }
    python -B tests/run_q5_test.py $staticElf --deps (Join-Path $BaselineRoot 'build/testdeps')
    if ($LASTEXITCODE -ne 0) { throw 'Q6 static-image regressions failed' }
} finally { Pop-Location }
