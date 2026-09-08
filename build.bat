@echo off
REM ===========================================================================
REM  build.bat <target> [asan] [args...]        e.g.  build.bat ch_probe 30
REM
REM  Compiles every src\*.c plus apps\<target>.c (or tests\<target>.c) into
REM  out\<target>.exe and runs it.  C17, /W4, strict.
REM
REM  No GOTO labels are used below. A .bat file with LF-only line endings
REM  loses its labels -- cmd re-seeks the file by byte offset and lands in the
REM  wrong place -- which is why .gitattributes pins *.bat to CRLF and why the
REM  argument handling here is done with FOR rather than a SHIFT loop.
REM ===========================================================================
setlocal EnableDelayedExpansion

set "ROOT=%~dp0"

if "%~1"=="" (
    echo usage: build.bat ^<target^> [asan] [args...]
    echo   targets available:
    for %%F in ("%ROOT%apps\*.c" "%ROOT%tests\*.c") do echo     %%~nF
    exit /b 1
)

set "APPNAME=%~1"
set "APPSRC=%ROOT%apps\%APPNAME%.c"
if not exist "%APPSRC%" set "APPSRC=%ROOT%tests\%APPNAME%.c"
if not exist "%APPSRC%" (
    echo error: no such target: %APPNAME%.c in apps\ or tests\
    exit /b 1
)

REM everything after the target name
set "REST="
for /f "tokens=1,* delims= " %%a in ("%*") do set "REST=%%b"

set "ASAN="
if /i "%~2"=="asan" (
    set "ASAN=1"
    set "REST="
    for /f "tokens=1,2,* delims= " %%a in ("%*") do set "REST=%%c"
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
if defined ASAN set "FLAGS=%FLAGS% /fsanitize=address /Zi"

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
"%ROOT%out\%APPNAME%.exe" !REST!
set "RC=!errorlevel!"
popd
exit /b %RC%
