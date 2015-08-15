@echo off
REM ///////////////////////////////////////////////////////////////////////////
REM // Set Paths
REM ///////////////////////////////////////////////////////////////////////////
set "MSVC_PATH=C:\Program Files\Microsoft Visual Studio 12.0\VC"
set "NSIS_PATH=C:\Program Files\NSIS\Unicode"
set "UPX3_PATH=C:\Program Files\UPX"
set "PDOC_PATH=C:\Program Files\Pandoc"
set "TOOLS_VER=120"

REM ###############################################
REM # DO NOT MODIFY ANY LINES BELOW THIS LINE !!! #
REM ###############################################

REM ///////////////////////////////////////////////////////////////////////////
REM // Setup environment
REM ///////////////////////////////////////////////////////////////////////////
call "%MSVC_PATH%\vcvarsall.bat" x86

REM ///////////////////////////////////////////////////////////////////////////
REM // Check environment
REM ///////////////////////////////////////////////////////////////////////////
if "%VCINSTALLDIR%"=="" (
	echo %%VCINSTALLDIR%% not specified. Please check your MSVC_PATH var!
	goto BuildError
)
if "%QTDIR%"=="" (
	echo %%QTDIR%% not specified. Please check your MSVC_PATH var!
	goto BuildError
)
if not exist "%VCINSTALLDIR%\bin\cl.exe" (
	echo C++ compiler not found. Please check your MSVC_PATH var!
	goto BuildError
)
if not exist "%PDOC_PATH%\pandoc.exe" (
	echo Pandoc binary could not be found. Please check your PDOC_PATH var!
	goto BuildError
)

REM ///////////////////////////////////////////////////////////////////////////
REM // Get current date and time (in ISO format)
REM ///////////////////////////////////////////////////////////////////////////
set "ISO_DATE="
set "ISO_TIME="
if not exist "%~dp0\etc\date.exe" BuildError
for /F "tokens=1,2 delims=:" %%a in ('"%~dp0\etc\date.exe" +ISODATE:%%Y-%%m-%%d') do (
	if "%%a"=="ISODATE" set "ISO_DATE=%%b"
)
for /F "tokens=1,2,3,4 delims=:" %%a in ('"%~dp0\etc\date.exe" +ISOTIME:%%T') do (
	if "%%a"=="ISOTIME" set "ISO_TIME=%%b:%%c:%%d"
)
if "%ISO_DATE%"=="" goto BuildError
if "%ISO_TIME%"=="" goto BuildError

REM ///////////////////////////////////////////////////////////////////////////
REM // Build the binaries
REM ///////////////////////////////////////////////////////////////////////////
echo ---------------------------------------------------------------------
echo BEGIN BUILD
echo ---------------------------------------------------------------------
MSBuild.exe /property:Configuration=release /target:clean   "%~dp0\x264_launcher_MSVC2013.sln"
if not "%ERRORLEVEL%"=="0" goto BuildError
MSBuild.exe /property:Configuration=release /target:rebuild "%~dp0\x264_launcher_MSVC2013.sln"
if not "%ERRORLEVEL%"=="0" goto BuildError

REM ///////////////////////////////////////////////////////////////////////////
REM // Detect build number
REM ///////////////////////////////////////////////////////////////////////////
set "BUILD_NO="
for /F "tokens=2,*" %%s in (%~dp0\src\version.h) do (
	if "%%s"=="VER_X264_BUILD" set "BUILD_NO=%%~t"
)
if "%BUILD_NO%"=="" goto BuildError

REM ///////////////////////////////////////////////////////////////////////////
REM // Copy base files
REM ///////////////////////////////////////////////////////////////////////////
echo ---------------------------------------------------------------------
echo BEGIN PACKAGING
echo ---------------------------------------------------------------------
set "PACK_PATH=%TMP%\~%RANDOM%%RANDOM%.tmp"
mkdir "%PACK_PATH%"
mkdir "%PACK_PATH%\imageformats"
mkdir "%PACK_PATH%\toolset\x86"
mkdir "%PACK_PATH%\toolset\x64"
mkdir "%PACK_PATH%\toolset\common"
mkdir "%PACK_PATH%\sources"
copy "%~dp0\bin\Win32\Release\x264_launcher.exe"    "%PACK_PATH%"
copy "%~dp0\bin\Win32\Release\MUtils32-?.dll"       "%PACK_PATH%"
copy "%~dp0\bin\Win32\Release\toolset\x86\*.exe"    "%PACK_PATH%\toolset\x86"
copy "%~dp0\bin\Win32\Release\toolset\x86\*.dll"    "%PACK_PATH%\toolset\x86"
copy "%~dp0\bin\Win32\Release\toolset\x64\*.exe"    "%PACK_PATH%\toolset\x64"
copy "%~dp0\bin\Win32\Release\toolset\x64\*.dll"    "%PACK_PATH%\toolset\x64"
copy "%~dp0\bin\Win32\Release\toolset\common\*.exe" "%PACK_PATH%\toolset\common"
copy "%~dp0\bin\Win32\Release\toolset\common\*.gpg" "%PACK_PATH%\toolset\common"
copy "%~dp0\etc\sources\*.xz"                       "%PACK_PATH%\sources"
copy "%~dp0\LICENSE.html"                           "%PACK_PATH%"
copy "%~dp0\*.txt"                                  "%PACK_PATH%"

REM ///////////////////////////////////////////////////////////////////////////
REM // Copy dependencies
REM ///////////////////////////////////////////////////////////////////////////
copy "%MSVC_PATH%\redist\x86\Microsoft.VC%TOOLS_VER%.CRT\*.dll"                     "%PACK_PATH%"
copy "%~dp0\..\Prerequisites\Qt4\v%TOOLS_VER%_xp\Shared\bin\QtCore4.dll"            "%PACK_PATH%"
copy "%~dp0\..\Prerequisites\Qt4\v%TOOLS_VER%_xp\Shared\bin\QtGui4.dll"             "%PACK_PATH%"
copy "%~dp0\..\Prerequisites\Qt4\v%TOOLS_VER%_xp\Shared\bin\QtSvg4.dll"             "%PACK_PATH%"
copy "%~dp0\..\Prerequisites\Qt4\v%TOOLS_VER%_xp\Shared\bin\QtXml4.dll"             "%PACK_PATH%"
copy "%~dp0\..\Prerequisites\Qt4\v%TOOLS_VER%_xp\Shared\bin\QtXml4.dll"             "%PACK_PATH%"
copy "%~dp0\..\Prerequisites\Qt4\v%TOOLS_VER%_xp\Shared\plugins\imageformats\*.dll" "%PACK_PATH%\imageformats"
del "%PACK_PATH%\imageformats\*d4.dll" 2> NUL

REM ///////////////////////////////////////////////////////////////////////////
REM // Generate Docs
REM ///////////////////////////////////////////////////////////////////////////
"%PDOC_PATH%\pandoc.exe" --from markdown_github+pandoc_title_block+header_attributes+implicit_figures --to html5 --toc -N --standalone -H "%~dp0\etc\css\style.inc" --output "%PACK_PATH%\README.html" "%~dp0\README.md"

REM ///////////////////////////////////////////////////////////////////////////
REM // Compress
REM ///////////////////////////////////////////////////////////////////////////
"%UPX3_PATH%\upx.exe" --brute "%PACK_PATH%\*.exe"
"%UPX3_PATH%\upx.exe" --best  "%PACK_PATH%\*.dll"

REM ///////////////////////////////////////////////////////////////////////////
REM // Attributes
REM ///////////////////////////////////////////////////////////////////////////
attrib +R "%PACK_PATH%\*.exe"
attrib +R "%PACK_PATH%\*.dll"
attrib +R "%PACK_PATH%\*.txt"
attrib +R "%PACK_PATH%\*.html"

REM ///////////////////////////////////////////////////////////////////////////
REM // Setup install parameters
REM ///////////////////////////////////////////////////////////////////////////
mkdir "%~dp0\out" 2> NUL
set "OUT_PATH=%~dp0\out\x264_launcher.%ISO_DATE%"
:GenerateOutfileName
if exist "%OUT_PATH%.exe" (
	set "OUT_PATH=%OUT_PATH%.new"
	goto GenerateOutfileName
)
if exist "%OUT_PATH%.sfx" (
	set "OUT_PATH=%OUT_PATH%.new"
	goto GenerateOutfileName
)
if exist "%OUT_PATH%.zip" (
	set "OUT_PATH=%OUT_PATH%.new"
	goto GenerateOutfileName
)

REM ///////////////////////////////////////////////////////////////////////////
REM // Create Tag
REM ///////////////////////////////////////////////////////////////////////////
echo Simple x264/x265 Launcher - graphical front-end for x264 and x265 > "%PACK_PATH%\BUILD_TAG.txt"
echo Copyright (C) 2004-2015 LoRd_MuldeR ^<MuldeR2@GMX.de^> >> "%PACK_PATH%\BUILD_TAG.txt"
echo. >> "%PACK_PATH%\BUILD_TAG.txt"
echo Build #%BUILD_NO%, created on %ISO_DATE% at %ISO_TIME% >> "%PACK_PATH%\BUILD_TAG.txt"
echo. >> "%PACK_PATH%\BUILD_TAG.txt"
echo. >> "%PACK_PATH%\BUILD_TAG.txt"
"%~dp0\etc\cat.exe" "%~dp0\etc\setup\build.nfo" >> "%PACK_PATH%\BUILD_TAG.txt"

REM ///////////////////////////////////////////////////////////////////////////
REM // Build the installer
REM ///////////////////////////////////////////////////////////////////////////
"%NSIS_PATH%\makensis.exe" "/DX264_UPX_PATH=%UPX3_PATH%" "/DX264_DATE=%ISO_DATE%" "/DX264_BUILD=%BUILD_NO%" "/DX264_OUTPUT_FILE=%OUT_PATH%.sfx" "/DX264_SOURCE_PATH=%PACK_PATH%"    "%~dp0\etc\setup\setup.nsi"
if not "%ERRORLEVEL%"=="0" goto BuildError

"%NSIS_PATH%\makensis.exe" "/DX264_UPX_PATH=%UPX3_PATH%" "/DX264_DATE=%ISO_DATE%" "/DX264_BUILD=%BUILD_NO%" "/DX264_OUTPUT_FILE=%OUT_PATH%.exe" "/DX264_SOURCE_FILE=%OUT_PATH%.sfx" "%~dp0\etc\setup\wrapper.nsi"
if not "%ERRORLEVEL%"=="0" goto BuildError

attrib +R "%OUT_PATH%.exe"
attrib +R "%OUT_PATH%.sfx"

REM ///////////////////////////////////////////////////////////////////////////
REM // Build ZIP package
REM ///////////////////////////////////////////////////////////////////////////
pushd "%PACK_PATH%"
"%~dp0\etc\zip.exe" -r -9 -z "%OUT_PATH%.zip" "*.*" < "%PACK_PATH%\BUILD_TAG.txt"
popd

if not "%ERRORLEVEL%"=="0" goto BuildError
attrib +R "%OUT_PATH%.zip"

REM ///////////////////////////////////////////////////////////////////////////
REM // Clean up
REM ///////////////////////////////////////////////////////////////////////////
rmdir /Q /S "%PACK_PATH%"

REM ///////////////////////////////////////////////////////////////////////////
REM // COMPLETE
REM ///////////////////////////////////////////////////////////////////////////
echo.
echo Build completed.
echo.
pause
goto:eof

REM ///////////////////////////////////////////////////////////////////////////
REM // FAILED
REM ///////////////////////////////////////////////////////////////////////////
:BuildError
echo.
echo Build has failed !!!
echo.
pause
