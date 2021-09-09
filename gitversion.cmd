@echo off
rem

echo Generating header for git hash

IF "%~1" == "" (
	set GIT_HEADER=git_version.h
	set GIT_HEADER_TXT=git_version.txt
) else (
	set GIT_HEADER=%1\git_version.h
	set GIT_HEADER_TXT=%1\git_version.txt
)

for /f "tokens=2 delims==" %%I in ('wmic os get localdatetime /format:list') do set datetime=%%I
rem echo %datetime%

rem Build the reverse date string YYYY-MM-DD
set rdate=%datetime:~0,4%%datetime:~4,2%%datetime:~6,2%
echo rdate=%rdate%

rem Built a datetime string YYYY-MM-DD-hhmmss
set datetime=%rdate%-%datetime:~8,6%

for /f %%a in ('git -C . describe --always --abbrev^=16 --dirty --exclude "*"') do set GIT_VERSION=%%a

echo git version is: %datetime%-%GIT_VERSION%

echo #ifndef GIT_VERSION_H > %GIT_HEADER%
echo #define GIT_VERSION_H >> %GIT_HEADER%
echo.  >> %GIT_HEADER%
echo #define GIT_CURRENT_SHA1 "%datetime%-%GIT_VERSION%" >> %GIT_HEADER%
echo.  >> %GIT_HEADER%
echo #endif //GIT_VERSION_H >> %GIT_HEADER%

echo file is generated into %GIT_HEADER%

echo %datetime%-%GIT_VERSION% > %GIT_HEADER_TXT%