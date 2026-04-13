@echo off
setlocal

set "CONDA_EXE=%USERPROFILE%\anaconda3\Scripts\conda.exe"
if exist "%CONDA_EXE%" (
  "%CONDA_EXE%" run -n bwsi python %*
  exit /b %ERRORLEVEL%
)

set "BWSI_PYTHON=%USERPROFILE%\anaconda3\envs\bwsi\python.exe"
if exist "%BWSI_PYTHON%" (
  "%BWSI_PYTHON%" %*
  exit /b %ERRORLEVEL%
)

echo Could not locate the bwsi Python interpreter or conda.exe. 1>&2
exit /b 1
