@echo off
cd /d "%~dp0"
if not exist "prusa-slicer-console.exe" (
  echo No se encuentra prusa-slicer-console.exe en esta carpeta.
  pause
  exit /b
)
echo Abriendo Prusa Colorin con log... Importa, lamina y cierra el programa.
prusa-slicer-console.exe --loglevel=4 > log_colorin.txt 2>&1
echo Listo. El log quedo en: %~dp0log_colorin.txt
pause
