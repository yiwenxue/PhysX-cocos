@echo off
setlocal enabledelayedexpansion
@REM set default_value=F:\INCLUDE\AndroidSDK\ndk\17.2.4988734
set default_value=E:\physics\emscripten-core\emsdk-windows\upstream\emscripten
::格式化括号
set "default_value=%default_value:(=^(%"
set "default_value=%default_value:)=^)%"
if "%1"=="" ( 
	set prestr=%default_value%
) else ( 
	set prestr=%1
)
@REM set PM_AndroidNDK_PATH=%prestr%
@REM echo %PM_AndroidNDK_PATH%

set EMSCRIPTEN=%prestr%

call .\physx\generate_projects.bat emscripten
copy .\physx\compiler\emscripten-debug\compile_commands.json .\