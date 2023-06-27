@echo off
set "DRIVEDIR=G:\Meine Ablage\VERVE\50 Development DEV\code"
set "SOURCE_DIR=.\build"

for /R "%SOURCE_DIR%" %%F in (*.bin *.hex) do (
    xcopy /Y "%%F" "%DRIVEDIR%\"
)