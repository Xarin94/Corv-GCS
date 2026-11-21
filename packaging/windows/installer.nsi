; CORV GCS - Windows installer (NSIS 3, Unicode). Built by tools\package-windows.ps1:
;   makensis /DVERSION=2.0.2 /DSTAGE=<staged folder> /DOUTFILE=<setup.exe> /DFILES=<files.nsh> installer.nsi
;
; Per-user like the Electron installer (no administrator rights), with the
; choice of the folder. 1.x (Electron) is CORV GCS too, but in Programs\CORV GCS,
; a folder its uninstaller deletes whole: 2.x installs next to it, in
; Programs\CORV Systems\CORV GCS, with its own uninstall entry, and the
; "CORV GCS" shortcuts start 2.x. The data folder next to the executable
; (missions, logs, terrain, settings) is left alone on uninstall.
;
; The first 2.0.x installers called it "CORV GCS 2" (Programs\CORV GCS 2). Over
; such an installation it stays in that folder, where its data are, and the old uninstall
; entry and shortcuts give way to the "CORV GCS" ones.
;
; /UPDATE (the GCS's one-click update): silent, into the folder of the
; installed copy (InstallDirRegKey), the shortcuts only where they already
; are, and the new version started at the end.

Unicode true
ManifestDPIAware true
SetCompressor /SOLID lzma
RequestExecutionLevel user

!ifndef VERSION
  !error "VERSION not defined"
!endif
!ifndef STAGE
  !error "STAGE not defined"
!endif
!ifndef OUTFILE
  !define OUTFILE "CORV-GCS-${VERSION}-setup-x64.exe"
!endif

!define APPNAME "CORV GCS"
!define COMPANY "CORV Systems"
!define EXE "corv-gcs.exe"
!define REGKEY "Software\${COMPANY}\${APPNAME}"
!define UNINSTKEY "Software\Microsoft\Windows\CurrentVersion\Uninstall\${APPNAME}"
; The name in the first 2.0.x installers
!define OLDNAME "CORV GCS 2"
!define OLDREGKEY "Software\${COMPANY}\${OLDNAME}"
!define OLDUNINSTKEY "Software\Microsoft\Windows\CurrentVersion\Uninstall\${OLDNAME}"

Name "${APPNAME}"
Caption "${APPNAME} ${VERSION} Setup"
OutFile "${OUTFILE}"
InstallDir "$LOCALAPPDATA\Programs\${COMPANY}\${APPNAME}"
InstallDirRegKey HKCU "${REGKEY}" "InstallDir"
BrandingText "${APPNAME} ${VERSION}"

VIProductVersion "${VERSION}.0"
VIAddVersionKey "ProductName" "${APPNAME}"
VIAddVersionKey "CompanyName" "${COMPANY}"
VIAddVersionKey "FileDescription" "${APPNAME} installer"
VIAddVersionKey "FileVersion" "${VERSION}"
VIAddVersionKey "ProductVersion" "${VERSION}"
VIAddVersionKey "LegalCopyright" "Apache-2.0"

!include "MUI2.nsh"
!include "FileFunc.nsh"
!include "LogicLib.nsh"

Var Update   ; "1" when started by the GCS with /UPDATE

!define MUI_ICON "${STAGE}\corv-gcs.ico"
!define MUI_UNICON "${STAGE}\corv-gcs.ico"
!define MUI_ABORTWARNING
!define MUI_FINISHPAGE_RUN "$INSTDIR\${EXE}"
!define MUI_FINISHPAGE_RUN_TEXT "Start ${APPNAME}"

!insertmacro MUI_PAGE_WELCOME
!insertmacro MUI_PAGE_LICENSE "${STAGE}\LICENSE"
!insertmacro MUI_PAGE_COMPONENTS
!insertmacro MUI_PAGE_DIRECTORY
!insertmacro MUI_PAGE_INSTFILES
!insertmacro MUI_PAGE_FINISH

!insertmacro MUI_UNPAGE_CONFIRM
!insertmacro MUI_UNPAGE_INSTFILES

!insertmacro MUI_LANGUAGE "English"
!insertmacro MUI_LANGUAGE "Italian"

Section "${APPNAME}" SecApp
  SectionIn RO
  SetOutPath "$INSTDIR"
  ; A running copy would keep its files locked
  nsExec::Exec 'taskkill /IM ${EXE} /F'
  File /r "${STAGE}\*.*"
  WriteUninstaller "$INSTDIR\uninstall.exe"

  ; This folder was "CORV GCS 2": its entries give way to the new name
  ReadRegStr $0 HKCU "${OLDREGKEY}" "InstallDir"
  ${If} $0 == $INSTDIR
    DeleteRegKey HKCU "${OLDUNINSTKEY}"
    DeleteRegKey HKCU "${OLDREGKEY}"
    Delete "$SMPROGRAMS\${OLDNAME}.lnk"
    Delete "$DESKTOP\${OLDNAME}.lnk"
  ${EndIf}

  WriteRegStr HKCU "${REGKEY}" "InstallDir" "$INSTDIR"
  WriteRegStr HKCU "${UNINSTKEY}" "DisplayName" "${APPNAME}"
  WriteRegStr HKCU "${UNINSTKEY}" "DisplayVersion" "${VERSION}"
  WriteRegStr HKCU "${UNINSTKEY}" "Publisher" "${COMPANY}"
  WriteRegStr HKCU "${UNINSTKEY}" "DisplayIcon" "$INSTDIR\${EXE}"
  WriteRegStr HKCU "${UNINSTKEY}" "InstallLocation" "$INSTDIR"
  WriteRegStr HKCU "${UNINSTKEY}" "UninstallString" '"$INSTDIR\uninstall.exe"'
  WriteRegStr HKCU "${UNINSTKEY}" "QuietUninstallString" '"$INSTDIR\uninstall.exe" /S'
  WriteRegDWORD HKCU "${UNINSTKEY}" "NoModify" 1
  WriteRegDWORD HKCU "${UNINSTKEY}" "NoRepair" 1
  ${GetSize} "$INSTDIR" "/S=0K" $0 $1 $2
  IntFmt $0 "0x%08X" $0
  WriteRegDWORD HKCU "${UNINSTKEY}" "EstimatedSize" "$0"
SectionEnd

Section "Start menu shortcut" SecStart
  CreateShortcut "$SMPROGRAMS\${APPNAME}.lnk" "$INSTDIR\${EXE}" "" "$INSTDIR\${EXE}" 0
SectionEnd

Section "Desktop shortcut" SecDesktop
  CreateShortcut "$DESKTOP\${APPNAME}.lnk" "$INSTDIR\${EXE}" "" "$INSTDIR\${EXE}" 0
SectionEnd

; Last: the new version starts where the update came from
Section "-Relaunch"
  StrCmp $Update "1" 0 +2
  Exec '"$INSTDIR\${EXE}"'
SectionEnd

Function .onInit
  ; Over a "CORV GCS 2" installation: into its folder, where its data are
  ReadRegStr $0 HKCU "${REGKEY}" "InstallDir"
  ${If} $0 == ""
    ReadRegStr $0 HKCU "${OLDREGKEY}" "InstallDir"
    ${If} $0 != ""
      StrCpy $INSTDIR $0
    ${EndIf}
  ${EndIf}

  ${GetParameters} $R0
  ClearErrors
  ${GetOptions} $R0 "/UPDATE" $R1
  ${If} ${Errors}
    Return
  ${EndIf}
  StrCpy $Update "1"
  SetSilent silent
  ; An update keeps the shortcuts the user has (under either name), and adds none
  ${IfNot} ${FileExists} "$SMPROGRAMS\${APPNAME}.lnk"
  ${AndIfNot} ${FileExists} "$SMPROGRAMS\${OLDNAME}.lnk"
    SectionSetFlags ${SecStart} 0
  ${EndIf}
  ${IfNot} ${FileExists} "$DESKTOP\${APPNAME}.lnk"
  ${AndIfNot} ${FileExists} "$DESKTOP\${OLDNAME}.lnk"
    SectionSetFlags ${SecDesktop} 0
  ${EndIf}
FunctionEnd

LangString DESC_App ${LANG_ENGLISH} "The ground control station (required)."
LangString DESC_App ${LANG_ITALIAN} "La stazione di terra (obbligatoria)."
LangString DESC_Start ${LANG_ENGLISH} "A shortcut in the Start menu."
LangString DESC_Start ${LANG_ITALIAN} "Un collegamento nel menu Start."
LangString DESC_Desktop ${LANG_ENGLISH} "A shortcut on the desktop."
LangString DESC_Desktop ${LANG_ITALIAN} "Un collegamento sul desktop."
!insertmacro MUI_FUNCTION_DESCRIPTION_BEGIN
  !insertmacro MUI_DESCRIPTION_TEXT ${SecApp} $(DESC_App)
  !insertmacro MUI_DESCRIPTION_TEXT ${SecStart} $(DESC_Start)
  !insertmacro MUI_DESCRIPTION_TEXT ${SecDesktop} $(DESC_Desktop)
!insertmacro MUI_FUNCTION_DESCRIPTION_END

Section "Uninstall"
  nsExec::Exec 'taskkill /IM ${EXE} /F'
  ; Exactly what the installer put there (generated list); data\ stays
  !include "${FILES}"
  Delete "$INSTDIR\uninstall.exe"
  RMDir "$INSTDIR"
  ; Programs\CORV Systems, when it is now empty
  RMDir "$LOCALAPPDATA\Programs\${COMPANY}"
  Delete "$SMPROGRAMS\${APPNAME}.lnk"
  Delete "$DESKTOP\${APPNAME}.lnk"
  DeleteRegKey HKCU "${UNINSTKEY}"
  DeleteRegKey HKCU "${REGKEY}"
SectionEnd
