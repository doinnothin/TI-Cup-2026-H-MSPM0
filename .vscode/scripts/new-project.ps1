param(
    [Parameter(Mandatory=$true, HelpMessage="新工程名称")]
    [string]$ProjectName
)

$ErrorActionPreference = "Stop"

# 模板工程路径（当前脚本所在工程的根目录）
$TemplateDir = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $PSScriptRoot))

# 新工程路径（跟模板工程放在同一级目录下）
$NewProjectDir = Join-Path (Split-Path -Parent $TemplateDir) $ProjectName

if (Test-Path $NewProjectDir) {
    Write-Error "目录 '$NewProjectDir' 已存在，请换一个工程名或先删除旧目录"
    exit 1
}

Write-Host "从模板工程创建新工程: $TemplateDir"
Write-Host "目标路径: $NewProjectDir"
Write-Host ""

# 复制整个模板文件夹，排除不需要的目录和文件
$excludeDirs = @(
    'build',
    'build-release',
    '.history',
    'user'
)

$excludeFiles = @(
    '.code-workspace'
)

Write-Host "正在复制工程骨架..."
$items = Get-ChildItem $TemplateDir -Force | Where-Object {
    $name = $_.Name
    if ($excludeDirs -contains $name) { return $false }
    if ($excludeFiles -contains $name) { return $false }
    if ($name -like '*.log') { return $false }
    return $true
}

New-Item -ItemType Directory -Force $NewProjectDir | Out-Null

foreach ($item in $items) {
    Copy-Item -Path $item.FullName -Destination (Join-Path $NewProjectDir $item.Name) -Recurse -Force
}

# 重建干净的 user 目录
$userDir = Join-Path $NewProjectDir 'user'
New-Item -ItemType Directory -Force $userDir | Out-Null

$mainContent = @'
#include "ti_msp_dl_config.h"

int main(void)
{
    SYSCFG_DL_init();

    while (1)
    {

    }
}
'@

Set-Content -Path (Join-Path $userDir 'main.c') -Value $mainContent -Encoding UTF8

Write-Host ""
Write-Host "新工程已创建: $NewProjectDir"
Write-Host "  user/main.c — 从这里开始写代码"
Write-Host ""
Write-Host "下一步: 在 VSCode 中打开 $NewProjectDir 文件夹即可开始开发"
