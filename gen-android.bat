@echo off
setlocal enabledelayedexpansion
set default_value=E:\Components\Android-sdk\ndk\17.2.4988734
::格式化括号
set "default_value=%default_value:(=^(%"
set "default_value=%default_value:)=^)%"
if "%1"=="" ( 
	set prestr=%default_value%
) else ( 
	set prestr=%1
)
set PM_AndroidNDK_PATH=%prestr%
echo %PM_AndroidNDK_PATH%

call ./physx/generate_projects.bat
