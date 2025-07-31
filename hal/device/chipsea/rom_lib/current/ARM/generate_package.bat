@echo off

REM @echo %1
REM @echo %2
REM @echo %3
REM @echo %4

set tmp=%1
set toolchain=%tmp:~0,-8%\bin
REM @echo ":"%toolchain%

REM Add toolchain path to PATH variable
path = %toolchain%;%path%;
REM @echo %path%

fromelf.exe --text -c -o %2..\bin\%3.asm %4
fromelf.exe --bin  -o %2..\bin\%3.bin %4
fromelf.exe --i32  -o %2..\bin\%3.hex %4
copy %4 %2..\bin\
copy %2..\Listings\%3.map %2..\bin\
