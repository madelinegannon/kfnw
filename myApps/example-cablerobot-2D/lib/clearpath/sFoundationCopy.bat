@echo off
REM USAGE: <Architecture> <Config> <DestDir>
set cwdDir=%~dp0
set srcArch=%1
pushd "%cwdDir%"
set srcConfig=%2
set destDir=%3
set devDir=.\sFoundation\win\%srcConfig%\%srcArch%\
REM Developer or End-User Mode?
if exist "%devDir%" (
set dllDir=%devDir%
set devRoot=..\sFoundation
) else (
set dllDir=.\lib\win\%srcConfig%\%srcArch%
set devRoot=.\lib\win\Release\Win32
)
echo cwd=%cwdDir% srcArch=%1 srcConfig=%2 dllDir=%dllDir% destDir=%3
copy /y "%dllDir%\sFoundation20.dll" %destDir%
copy /y "%dllDir%\sFoundation20.lib" %destDir%
copy /y "%devRoot%\MNuserDriver20.xml" %destDir%
popd
