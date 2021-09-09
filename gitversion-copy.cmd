@echo off
rem

IF "%~1" == "" (
	echo Specify build location
	exit
) else (
	set BUILD_LOCATION=%1
)

for /f %%a in ('type %BUILD_LOCATION%\..\git_version.txt') do set GIT_VERSION=%%a

echo git version is: %GIT_VERSION%

rem ${SCToolsBaseDir}/StarCore_Support\SmartDSP\b4860_bin_creator.exe -cw '${SC_TOOLS_HOME}' -dir '${BuildLocation}' -arch b4860 -file enb_l1_b4860_v1
C:\Freescale\CW_SC_3900FP_v10.9.0\SC\StarCore_Support\SmartDSP\b4860_bin_creator.exe -cw C:\Freescale\CW_SC_3900FP_v10.9.0\SC -dir %BUILD_LOCATION% -arch b4860 -file enb_l1_b4860_v1

copy /b %BUILD_LOCATION%\enb_l1_b4860_v1.bin %BUILD_LOCATION%\enb_l1_b4860_v1-%GIT_VERSION%.bin