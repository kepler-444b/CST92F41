@echo off
::Keil执行文件位置
set UV="D:\Program Files\Keil_v5\UV4\UV4.exe"
::查找uvprojx工程文件
echo Find Project ...
echo .>build_log.txt
for /f "usebackq delims=" %%j in (`dir /s /b %cd%\*.uvprojx`) do (
if exist %%j (
echo --------------------------------------------
echo %%j
echo Init building ...
%UV% -j0 -b %%j -l %cd%\build_log.txt
type build_log.txt)
echo --------------------------------------------
)
echo Done.
pause

