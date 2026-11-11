# 代码风格与格式化

## 约定

- 应用与测试代码使用 C11、四空格缩进、UTF-8（无 BOM），以及 LF 行尾。
- `.clang-format` 是仓库的格式化来源；`.editorconfig` 与 `.gitattributes` 定义编辑器与 Git 文本文件行为。
- 本次清理所用的固定版本格式化工具为 `clang-format` 18.1.8。
- 格式化范围为 `user/` 与 `tests/` 下所有第一方 C/H 文件，包括 Q3。Q3 的算法、标定值、控制参数与序列仍受行为保护；仅样式改动是允许的。TI/CMSIS 厂商代码与 SysConfig 生成的文件不重新格式化。
- 保留厂商版权与许可头。格式化并不构成修改厂商代码的许可。

## 检查或应用格式化

创建一个被忽略的项目本地格式化环境，并从仓库根目录运行：

```powershell
python -m venv .venv-format
.\.venv-format\Scripts\python.exe -m pip install clang-format==18.1.8
$env:PATH = "$PWD\.venv-format\Scripts;$env:PATH"
powershell -ExecutionPolicy Bypass -File tools/check_format.ps1
```

将仓库风格应用到受维护的源码范围：

```powershell
powershell -ExecutionPolicy Bypass -File tools/check_format.ps1 -Fix
```

应用格式化后请审查产生的 diff。格式化工具不得改变控制表达式、常量、初始化顺序或生成代码。对于 Q3，请将每个改动的 hunk 视为仅布局变更来审查；预处理器的行续接格式化可能使 Git 的忽略空白 diff 仍然显示出一个 hunk。
