@echo off
REM ===========================================================================
REM  build.bat <app>            e.g.  build.bat ch_probe
REM
REM  Compiles every src\*.c plus apps\<app>.c into out\<app>.exe and runs it.
REM  C17, /W4, strict.  build.bat <app> [asan] [app args...]
REM ===========================================================================
setlocal EnableDelayedExpansion

if "%~1"=="" (
    echo usage: build.bat ^<app^> [asan]
    echo   apps available:
    for %%F in ("%~dp0apps\*.c") do echo     %%~nF
    exit /b 1
)

set "ROOT=%~dp0"
set "APPNAME=%~1"
set "APPSRC=%ROOT%apps\%APPNAME%.c"
if not exist "%APPSRC%" set "APPSRC=%ROOT%tests\%APPNAME%.c"
if not exist "%APPSRC%" (
    echo error: no such source: %APPNAME%.c in apps\ or tests\
    exit /b 1
)

set "VCVARS="
for %%V in (
    "C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
    "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
) do (
    if not defined VCVARS if exist %%V set "VCVARS=%%~V"
)
if not defined VCVARS (
    echo error: MSVC Build Tools not found
    exit /b 1
)
if not defined VSCMD_VER call "%VCVARS%" >nul 2>nul

if not exist "%ROOT%out" mkdir "%ROOT%out"

set "FLAGS=/nologo /std:c17 /W4 /O2 /I "%ROOT%include" /diagnostics:caret"
if /i "%~2"=="asan" set "FLAGS=%FLAGS% /fsanitize=address /Zi"

REM forward remaining args to the program: drop the app name, and "asan" if given
shift
if /i "%~1"=="asan" shift
set "APPARGS="
:collect
if "%~1"=="" goto collected
set "APPARGS=!APPARGS! %~1"
shift
goto collect
:collected

pushd "%ROOT%out"
cl %FLAGS% "%ROOT%src\*.c" "%APPSRC%" /Fe:"%APPNAME%.exe"
if errorlevel 1 (
    popd
    echo.
    echo === BUILD FAILED ===
    exit /b 1
)
echo.
echo === run ===
"%ROOT%out\%APPNAME%.exe"!APPARGS!
set "RC=!errorlevel!"
popd
exit /b %RC%


