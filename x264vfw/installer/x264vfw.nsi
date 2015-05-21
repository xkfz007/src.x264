;x264vfw - H.264/MPEG-4 AVC codec
;Written by BugMaster

!define FullName  "x264vfw - H.264/MPEG-4 AVC codec"
!define ShortName "x264vfw"

;--------------------------------
;Includes

  !include "MUI.nsh"
  !include "Sections.nsh"
  !include "WinVer.nsh"
  !include "x64.nsh"
  !include "FileFunc.nsh"

;--------------------------------
;General

  SetCompressor /SOLID lzma

  ;Name and file
  Name "${FullName}"
  OutFile "${ShortName}.exe"

  ;Default installation folder
  InstallDir ""

  ;System32 or SysWOW64 dir according to OS
  Var SYSDIR_32bit

  ;Previous version installation folder
  Var PREV_INSTDIR

;--------------------------------
;Interface Configuration

  !define MUI_HEADERIMAGE
  !define MUI_COMPONENTSPAGE_NODESC
  !define MUI_FINISHPAGE_NOAUTOCLOSE
  !define MUI_UNFINISHPAGE_NOAUTOCLOSE
  !define MUI_ABORTWARNING

;--------------------------------
;Pages

  !define MUI_WELCOMEPAGE_TITLE_3LINES
  !insertmacro MUI_PAGE_WELCOME
  !insertmacro MUI_PAGE_LICENSE "..\COPYING"
  !insertmacro MUI_PAGE_DIRECTORY
  !insertmacro MUI_PAGE_INSTFILES
  !define MUI_FINISHPAGE_TITLE_3LINES
  !insertmacro MUI_PAGE_FINISH

  !insertmacro MUI_UNPAGE_CONFIRM
  !insertmacro MUI_UNPAGE_INSTFILES
  !define MUI_FINISHPAGE_TITLE_3LINES
  !insertmacro MUI_UNPAGE_FINISH

;--------------------------------
;Languages

  !insertmacro MUI_LANGUAGE "English"

;--------------------------------
;Macros

!macro RegCodec CODEC_ID CODEC_NAME CODEC_DESC

  Push $R0
  Push $R1
  Push $R2
    StrCpy $R0 "${CODEC_ID}"
    StrCpy $R1 "${CODEC_NAME}"
    StrCpy $R2 "${CODEC_DESC}"
    ${If} ${IsNT}
      WriteRegStr HKLM "Software\Microsoft\Windows NT\CurrentVersion\drivers32"    $R0 $R1
      WriteRegStr HKLM "Software\Microsoft\Windows NT\CurrentVersion\drivers.desc" $R1 $R2
    ${Else}
      WriteINIStr "$WINDIR\system.ini" "drivers32" $R0 $R1
      WriteRegStr HKLM "System\CurrentControlSet\Control\MediaResources\icm\$R0" "Driver"       $R1
      WriteRegStr HKLM "System\CurrentControlSet\Control\MediaResources\icm\$R0" "Description"  $R2
      WriteRegStr HKLM "System\CurrentControlSet\Control\MediaResources\icm\$R0" "FriendlyName" $R2
    ${EndIf}
  Pop $R2
  Pop $R1
  Pop $R0

!macroend

!macro UnRegCodec CODEC_ID

  Push $R0
  Push $R1
    StrCpy $R0 "${CODEC_ID}"
    ${If} ${IsNT}
      ReadRegStr $R1 HKLM "Software\Microsoft\Windows NT\CurrentVersion\drivers32"    $R0
      DeleteRegValue HKLM "Software\Microsoft\Windows NT\CurrentVersion\drivers.desc" $R1
      DeleteRegValue HKLM "Software\Microsoft\Windows NT\CurrentVersion\drivers32"    $R0
    ${Else}
      DeleteINIStr "$WINDIR\system.ini" "drivers32" $R0
      DeleteRegKey HKLM "System\CurrentControlSet\Control\MediaResources\icm\$R0"
    ${EndIf}
  Pop $R1
  Pop $R0

!macroend

;--------------------------------
;Installer Sections

Section "-Remove previous version"

  ${If} $PREV_INSTDIR != ""
    Delete "$SYSDIR_32bit\x264vfw.dll"

    SetShellVarContext all
    Delete "$SMPROGRAMS\${ShortName}\Configure ${ShortName}.lnk"
    SetShellVarContext current
    Delete "$SMPROGRAMS\${ShortName}\Configure ${ShortName}.lnk"

    Delete "$PREV_INSTDIR\x264vfw.dll"
    Delete "$PREV_INSTDIR\x264vfw.ico"

    !insertmacro UnRegCodec "vidc.x264"

    SetShellVarContext all
    Delete "$SMPROGRAMS\${ShortName}\Uninstall ${ShortName}.lnk"
    RMDir  "$SMPROGRAMS\${ShortName}"
    SetShellVarContext current
    Delete "$SMPROGRAMS\${ShortName}\Uninstall ${ShortName}.lnk"
    RMDir  "$SMPROGRAMS\${ShortName}"

    Delete "$PREV_INSTDIR\${ShortName}-uninstall.exe"
    RMDir  $PREV_INSTDIR
    DeleteRegKey HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${ShortName}"
  ${EndIf}

SectionEnd

Section "-Install"

  SetOutPath $INSTDIR

  ;Create uninstaller
  WriteUninstaller "$INSTDIR\${ShortName}-uninstall.exe"

  ; Write the uninstall keys for Windows
  WriteRegStr   HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${ShortName}" "UninstallString" "$INSTDIR\${ShortName}-uninstall.exe"
  WriteRegStr   HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${ShortName}" "DisplayIcon" "$INSTDIR\${ShortName}-uninstall.exe"
  WriteRegStr   HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${ShortName}" "DisplayName" "${FullName} (remove only)"
  WriteRegDWORD HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${ShortName}" "NoModify" 1
  WriteRegDWORD HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${ShortName}" "NoRepair" 1

  SetShellVarContext all
  CreateDirectory "$SMPROGRAMS\${ShortName}"
  CreateShortcut  "$SMPROGRAMS\${ShortName}\Uninstall ${ShortName}.lnk" "$INSTDIR\${ShortName}-uninstall.exe"

  File ".\x264vfw.ico"

  SetOutPath $SYSDIR_32bit
  File ".\x264vfw.dll"

  SetShellVarContext all
  ${If} ${FileExists} "$SYSDIR_32bit\rundll32.exe"
    CreateShortcut "$SMPROGRAMS\${ShortName}\Configure ${ShortName}.lnk" "$SYSDIR_32bit\rundll32.exe" "x264vfw.dll,Configure" "$INSTDIR\x264vfw.ico"
  ${Else}
    CreateShortcut "$SMPROGRAMS\${ShortName}\Configure ${ShortName}.lnk" "rundll32.exe" "x264vfw.dll,Configure" "$INSTDIR\x264vfw.ico"
  ${EndIf}

  !insertmacro RegCodec "vidc.x264" "x264vfw.dll" "${FullName}"

SectionEnd

;--------------------------------
;Installer Functions

Function .onInit

  ${If} ${RunningX64}
    StrCpy $SYSDIR_32bit "$WINDIR\SysWOW64"
  ${Else}
    StrCpy $SYSDIR_32bit $SYSDIR
  ${EndIf}

  ; Check Administrator's rights
  ClearErrors
  UserInfo::GetAccountType
  IfErrors not_admin
  Pop $R0
  StrCmp $R0 "Admin" admin
not_admin:
  ; No Administrator's rights
  MessageBox MB_YESNO|MB_ICONEXCLAMATION "For correct installation of this program you must have $\"Administrator's rights$\"!$\r$\nDo you wish to continue installation anyway?" \
    /SD IDNO IDYES admin
  Abort
admin:

  StrCpy $PREV_INSTDIR ""
  ReadRegStr $R0 HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${ShortName}" "UninstallString"
  ${GetParent} $R0 $R1
  StrCmp $R1 "" +2
  StrCpy $PREV_INSTDIR $R1

  StrCmp $INSTDIR "" +1 +4
  StrCpy $INSTDIR $PREV_INSTDIR
  StrCmp $INSTDIR "" +1 +2
  StrCpy $INSTDIR "$PROGRAMFILES32\${ShortName}"

FunctionEnd

;--------------------------------
;Uninstaller Section

Section "Uninstall"

  MessageBox MB_YESNO|MB_ICONQUESTION "Keep ${ShortName}'s settings in registry?" /SD IDYES IDYES keep_settings
  DeleteRegKey HKCU "Software\GNU\x264"
keep_settings:

  Delete /REBOOTOK "$SYSDIR_32bit\x264vfw.dll"

  SetShellVarContext all
  Delete /REBOOTOK "$SMPROGRAMS\${ShortName}\Configure ${ShortName}.lnk"
  SetShellVarContext current
  Delete /REBOOTOK "$SMPROGRAMS\${ShortName}\Configure ${ShortName}.lnk"

  Delete /REBOOTOK "$INSTDIR\x264vfw.dll"
  Delete /REBOOTOK "$INSTDIR\x264vfw.ico"

  !insertmacro UnRegCodec "vidc.x264"

  SetShellVarContext all
  Delete /REBOOTOK "$SMPROGRAMS\${ShortName}\Uninstall ${ShortName}.lnk"
  RMDir  /REBOOTOK "$SMPROGRAMS\${ShortName}"
  SetShellVarContext current
  Delete /REBOOTOK "$SMPROGRAMS\${ShortName}\Uninstall ${ShortName}.lnk"
  RMDir  /REBOOTOK "$SMPROGRAMS\${ShortName}"

  Delete /REBOOTOK "$INSTDIR\${ShortName}-uninstall.exe"
  RMDir  /REBOOTOK $INSTDIR
  DeleteRegKey HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${ShortName}"

SectionEnd

;--------------------------------
;Uninstaller Functions

Function un.onInit

  ${If} ${RunningX64}
    StrCpy $SYSDIR_32bit "$WINDIR\SysWOW64"
  ${Else}
    StrCpy $SYSDIR_32bit $SYSDIR
  ${EndIf}

FunctionEnd
