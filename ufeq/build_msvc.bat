@echo off
setlocal
cd /d "%~dp0"
if defined VSCMD_ARG_TGT_ARCH goto :build
if exist "%VSINSTALLDIR%\VC\Auxiliary\Build\vcvars64.bat" (
  call "%VSINSTALLDIR%\VC\Auxiliary\Build\vcvars64.bat" >nul
) else if exist "D:\software_store\VS_Builder\VS\BuildTools\VC\Auxiliary\Build\vcvars64.bat" (
  call "D:\software_store\VS_Builder\VS\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul
) else (
  echo Run this script from an x64 Native Tools / Developer Command Prompt.
  exit /b 1
)

:build
if not exist build mkdir build
cd build
del /q *.obj 2>nul
del /q *.lib 2>nul

cl /nologo /std:c11 /O2 /W3 /utf-8 /I..\include /c ..\src\*.c ..\platform\*.c
if errorlevel 1 exit /b 1
lib /nologo /out:ufeq.lib *.obj
if errorlevel 1 exit /b 1
cl /nologo /std:c11 /O2 /W3 /utf-8 /I..\include ..\tests\test_smoke.c ufeq.lib /Fe:test_smoke.exe
if errorlevel 1 exit /b 1
cl /nologo /std:c11 /O2 /W3 /utf-8 /I..\include ..\tests\test_modules.c ufeq.lib /Fe:test_modules.exe
if errorlevel 1 exit /b 1
cl /nologo /std:c11 /O2 /W3 /utf-8 /I..\include ..\tests\test_extended.c ufeq.lib /Fe:test_extended.exe
if errorlevel 1 exit /b 1
cl /nologo /std:c11 /O2 /W3 /utf-8 /I..\include ..\tests\test_all.c ufeq.lib /Fe:test_all.exe
if errorlevel 1 exit /b 1
echo === smoke ===
test_smoke.exe
if errorlevel 1 exit /b 1
echo === modules ===
test_modules.exe
if errorlevel 1 exit /b 1
echo === extended ===
test_extended.exe
if errorlevel 1 exit /b 1
echo === all ===
test_all.exe
exit /b %ERRORLEVEL%
