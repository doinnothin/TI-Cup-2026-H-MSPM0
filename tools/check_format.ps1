param([switch]$Fix)

$ErrorActionPreference = 'Stop'

$formatterCommand = Get-Command clang-format -ErrorAction SilentlyContinue
if ($null -eq $formatterCommand) {
    throw 'clang-format 18.1.8 is required. Install it and add it to PATH.'
}

$versionOutput = & $formatterCommand.Source --version
if ($LASTEXITCODE -ne 0 -or $versionOutput -notmatch '^clang-format version 18\.1\.8$') {
    throw "Expected clang-format 18.1.8, found '$versionOutput'."
}

$sourceFiles = @(git ls-files | Where-Object {
    ($_ -match '^(user|tests)/.*\.(c|h)$') -and
    (Test-Path -LiteralPath $_ -PathType Leaf)
})

if ($sourceFiles.Count -eq 0) {
    throw 'No maintained C/H files were found. Run this script from the repository root.'
}

$failedFiles = @()
foreach ($sourceFile in $sourceFiles) {
    if ($Fix) {
        & $formatterCommand.Source -i --style=file -- $sourceFile 2>&1 | Out-Null
    }
    else {
        & $formatterCommand.Source --dry-run --Werror --style=file -- $sourceFile 2>&1 | Out-Null
    }

    if ($LASTEXITCODE -ne 0) {
        $failedFiles += $sourceFile
    }
}

if ($failedFiles.Count -gt 0) {
    $failedFiles | ForEach-Object { Write-Output "Formatting check failed: $_" }
    exit 1
}

Write-Output "clang-format 18.1.8: $($sourceFiles.Count) C/H files checked."
