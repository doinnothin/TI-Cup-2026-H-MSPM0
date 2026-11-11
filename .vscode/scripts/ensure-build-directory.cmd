@echo off
setlocal

set "WORKSPACE=%~1"
set "CMAKE=%~2"
set "BUILD=%WORKSPACE%\build"
set "CACHE=%BUILD%\CMakeCache.txt"
set "NEEDS_CONFIGURE=0"
set "CACHE_HOME="

if not exist "%CACHE%" (
    set "NEEDS_CONFIGURE=1"
    goto configure_check_done
)

for /f "tokens=1,* delims==" %%A in ('findstr /b /c:"CMAKE_HOME_DIRECTORY:INTERNAL=" "%CACHE%"') do (
    set "CACHE_HOME=%%B"
)

if "%CACHE_HOME%"=="" (
    set "NEEDS_CONFIGURE=1"
    goto configure_check_done
)

if /i not "%CACHE_HOME:/=\%"=="%WORKSPACE%" (
    echo CMake cache source path changed.
    echo Old: %CACHE_HOME%
    echo New: %WORKSPACE%
    echo Removing stale build directory: %BUILD%
    rmdir /s /q "%BUILD%"
    set "NEEDS_CONFIGURE=1"
)

:configure_check_done
if "%NEEDS_CONFIGURE%"=="1" (
    "%CMAKE%" --preset debug-armgcc -S "%WORKSPACE%"
    exit /b %ERRORLEVEL%
)

exit /b 0
