@echo off
setlocal

set "WORKSPACE=%~1"
set "CMAKE=%~2"
set "OPENOCD=%~3"
set "OPENOCD_SCRIPTS=%~4"
set "BUILD=%WORKSPACE%\build"
rem The C control regressions also create *.elf files in build.  The firmware
rem target is unambiguous: CMake names the MSPM0 executable MSPM0.elf.
set "ELF=%BUILD%\MSPM0.elf"
set "FLASH_CFG=%BUILD%\flash-openocd.cfg"

if not exist "%ELF%" (
    echo Firmware not found: %ELF%
    echo Build the MSPM0 target first.
    exit /b 1
)

(
    echo program "%ELF:\=/%" verify
    echo reset run
    echo exit
) > "%FLASH_CFG%"

"%OPENOCD%" -s "%OPENOCD_SCRIPTS%" -f "%WORKSPACE%\linker\CFG.cfg" -f "%FLASH_CFG%"
exit /b %ERRORLEVEL%
