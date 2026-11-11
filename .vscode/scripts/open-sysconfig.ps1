param(
    [Parameter(Mandatory = $true)]
    [string]$WorkspaceFolder,

    [switch]$ValidateOnly
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Get-DotEnvValue {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,

        [Parameter(Mandatory = $true)]
        [string]$Name
    )

    if (-not (Test-Path -LiteralPath $Path)) {
        return $null
    }

    foreach ($line in Get-Content -LiteralPath $Path) {
        $trimmedLine = $line.Trim()
        if (($trimmedLine.Length -eq 0) -or $trimmedLine.StartsWith("#")) {
            continue
        }

        $parts = $trimmedLine -split "=", 2
        if (($parts.Count -eq 2) -and ($parts[0].Trim() -eq $Name)) {
            return $parts[1].Trim().Trim('"').Trim("'")
        }
    }

    return $null
}

$envPath = Join-Path $WorkspaceFolder ".env"
$sysconfigRoot = $env:SYSCONFIG_ROOT
$dotEnvSysconfigRoot = Get-DotEnvValue -Path $envPath -Name "SYSCONFIG_ROOT"
$sdkRoot = $env:MSPM0_SDK_ROOT
$dotEnvSdkRoot = Get-DotEnvValue -Path $envPath -Name "MSPM0_SDK_ROOT"

if (-not [string]::IsNullOrWhiteSpace($dotEnvSysconfigRoot)) {
    $sysconfigRoot = $dotEnvSysconfigRoot
}

if ([string]::IsNullOrWhiteSpace($sysconfigRoot)) {
    throw "未设置 SYSCONFIG_ROOT，请在 .env 中配置 SysConfig 安装目录。"
}

if (-not [string]::IsNullOrWhiteSpace($dotEnvSdkRoot)) {
    $sdkRoot = $dotEnvSdkRoot
}

if ([string]::IsNullOrWhiteSpace($sdkRoot)) {
    $sdkRoot = Join-Path (Split-Path -Parent $sysconfigRoot) "mspm0_sdk_2_10_00_04"
}

$nwPath = Join-Path $sysconfigRoot "nw\nw.exe"
$appPath = Join-Path $sysconfigRoot "app"
$productPath = Join-Path $sdkRoot ".metadata\product.json"
$syscfgFile = Get-ChildItem -LiteralPath $WorkspaceFolder -Filter "*.syscfg" -File |
    Sort-Object -Property Name |
    Select-Object -First 1

if ($null -eq $syscfgFile) {
    throw "工作区根目录未找到 .syscfg 文件：$WorkspaceFolder"
}

$syscfgPath = $syscfgFile.FullName

if (-not (Test-Path -LiteralPath $nwPath)) {
    throw "SysConfig 可执行文件不存在：$nwPath"
}

if (-not (Test-Path -LiteralPath $appPath)) {
    throw "SysConfig app 目录不存在：$appPath"
}

if (-not (Test-Path -LiteralPath $productPath -PathType Leaf)) {
    throw "MSPM0 SDK 产品描述文件不存在：$productPath"
}

if (-not (Test-Path -LiteralPath $syscfgPath)) {
    throw "SysConfig 脚本不存在：$syscfgPath"
}

if ($ValidateOnly) {
    Write-Host "SysConfig 配置检查通过：$nwPath"
    Write-Host "MSPM0 SDK 产品：$productPath"
    Write-Host "SysConfig 脚本：$syscfgPath"
    exit 0
}

& $nwPath $appPath --product $productPath --script $syscfgPath
