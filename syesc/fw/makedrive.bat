@echo off
setlocal enabledelayedexpansion

set "DESTDIR=G:\Meine Ablage\VERVE\50 Development DEV\50.1 Embedded\builds"
set "SOURCEDIR=.\build"
set "SOURCEDIR_FULL=%~f1"

pushd "%SOURCEDIR%"

REM Copy .bin files
for /R %%F in (*.bin) do (
    set "relativePath=%%~dpF"
    set "relativePath=!relativePath:%CD%=!"
    xcopy "%%F" "%DESTDIR%!relativePath!" /I /Y /S
)

REM Copy .hex files
for /R %%F in (*.hex) do (
    set "relativePath=%%~dpF"
    set "relativePath=!relativePath:%CD%=!"
    xcopy "%%F" "%DESTDIR%!relativePath!" /I /Y /S
)

popd
endlocal
