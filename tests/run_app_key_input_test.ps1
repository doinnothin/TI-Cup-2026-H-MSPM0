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
    $testElf = Join-Path $ProjectRoot 'tests\app_key_input_test.elf'
    $arguments = @(
        '-mcpu=cortex-m0plus', '-mthumb', '-O0', '-g', '-std=c11', '-Wall', '-Wextra',
        '-ffunction-sections', '-fdata-sections', '-nostartfiles', '-specs=nosys.specs',
        '-Wl,-e,main,-Ttext=0x1000,-Tdata=0x20000000,--gc-sections',
        '-I', 'tests/stubs', '-I', 'user',
        'tests/app_key_input_test.c', 'user/app/app_key_input.c', '-lc', '-o', $testElf
    )
    & $Gcc @arguments
    if ($LASTEXITCODE -ne 0) { throw 'App key-input test compilation failed' }
    python -B tests/run_q5_test.py $testElf --deps (Join-Path $BaselineRoot 'build/testdeps')
    if ($LASTEXITCODE -ne 0) { throw 'App key-input regressions failed' }
} finally { Pop-Location }
