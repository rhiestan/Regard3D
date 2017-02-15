@ECHO OFF
REM This batch file creates the setup for Regard3D
REM
REM Please adjust the path to NSIS/NSIS Unicode

SET NSIS_PATH="C:\Program Files (x86)\NSIS\Unicode"
SET PATH=%PATH%;%NSIS_PATH%

makensis.exe /V3 Regard3D_msvc2015.nsi
pause
