; Regard3D.nsi
;
; This is an installer script for the Nullsoft Installer System (NSIS).
; See http://nsis.sourceforge.net/
;
;--------------------------------

; Use Modern UI 2
!include MUI2.nsh

!include "x64.nsh"
!include "LogicLib.nsh"

;Get installation folder from registry if available
InstallDirRegKey HKCU "Software\hiesti.ch\Regard3D" ""

!searchparse /file ..\version.h "#define REGARD3D_VERSION_MAJOR " VER_MAJOR
!searchparse /file ..\version.h "#define REGARD3D_VERSION_MINOR " VER_MINOR
!searchparse /file ..\version.h "#define REGARD3D_VERSION_BUILD " VER_BUILD
!searchparse /file ..\version.h '#define REGARD3D_NAME "' APP_NAME '"'
!searchparse /file ..\version.h '#define REGARD3D_COPYRIGHT_NAME "' COMPANY_NAME '"'
!searchparse /file ..\version.h '#define REGARD3D_COPYRIGHT_YEAR "' COPYRIGHT_YEAR '"'

; The name of the installer
Name "Regard 3D ${VER_MAJOR}.${VER_MINOR}.${VER_BUILD}"

; The Installer to create
OutFile "..\..\Regard3D_${VER_MAJOR}.${VER_MINOR}.${VER_BUILD}_Setup.exe"

; The default installation directory
InstallDir $PROGRAMFILES64\Regard3D

; Request application privileges for Windows Vista
RequestExecutionLevel admin

XPStyle on

; Set best compression
SetCompressor lzma

; Set file and version information in the installer itself
VIAddVersionKey "ProductName" "Regard3D_Setup"
VIAddVersionKey "Comments" "Installer for ${APP_NAME}"
VIAddVersionKey "CompanyName" "${COMPANY_NAME}"
VIAddVersionKey "LegalCopyright" "Copyright (C) ${COPYRIGHT_YEAR} ${COMPANY_NAME}"
VIAddVersionKey "FileDescription" "Installer for ${APP_NAME}"
VIAddVersionKey "FileVersion" "${VER_MAJOR}.${VER_MINOR}.${VER_BUILD}"
VIProductVersion "${VER_MAJOR}.${VER_MINOR}.${VER_BUILD}.0"

; Do not automatically jump to the finish page, to allow the user to check the install log.
!define MUI_FINISHPAGE_NOAUTOCLOSE
!define MUI_UNFINISHPAGE_NOAUTOCLOSE
!define MUI_HEADERIMAGE
!define MUI_HEADERIMAGE_BITMAP "${NSISDIR}\Contrib\Graphics\Header\nsis.bmp"
!define MUI_HEADERIMAGE_UNBITMAP "${NSISDIR}\Contrib\Graphics\Header\nsis.bmp"

Var StartMenuFolder

;--------------------------------

; Pages

!insertmacro MUI_PAGE_LICENSE ..\licenses\Copyright.txt
!insertmacro MUI_PAGE_DIRECTORY

;Start Menu Folder Page Configuration
!define MUI_STARTMENUPAGE_REGISTRY_ROOT "HKCU" 
!define MUI_STARTMENUPAGE_REGISTRY_KEY "Software\hiesti.ch\Regard3D" 
!define MUI_STARTMENUPAGE_REGISTRY_VALUENAME "Start Menu Folder"
!define MUI_STARTMENUPAGE_DEFAULTFOLDER "Regard3D"
!insertmacro MUI_PAGE_STARTMENU Application $StartMenuFolder

!insertmacro MUI_PAGE_INSTFILES
!insertmacro MUI_PAGE_FINISH

!insertmacro MUI_UNPAGE_CONFIRM
!insertmacro MUI_UNPAGE_INSTFILES
!insertmacro MUI_UNPAGE_FINISH


!insertmacro MUI_LANGUAGE "English"
!insertmacro MUI_LANGUAGE "French"
!insertmacro MUI_LANGUAGE "German"
!insertmacro MUI_RESERVEFILE_LANGDLL

;--------------------------------


Section "Main program" Section_Main

	SectionIn 1

	; Set output path to the installation directory.
	SetOutPath $INSTDIR

	; Store install path in registry
	WriteRegStr HKCU "Software\hiesti.ch\Regard3D" "" $INSTDIR

	; Put files there
	File ..\..\build_msvc64_2015\Regard3D.exe
	File ..\..\openblas_files\libgcc_s_seh-1.dll
	File ..\..\openblas_files\libgfortran-3.dll
	File ..\..\openblas_files\libopenblas.dll
	File ..\..\openblas_files\libquadmath-0.dll
	File ..\..\openblas_files\libwinpthread-1.dll
	WriteUninstaller "$INSTDIR\Uninstall.exe"

	SetOutPath $INSTDIR\poisson
	File ..\..\thirdparty\poisson\*.*
	SetOutPath $INSTDIR\mve
	File ..\..\thirdparty\mve\*.*
	SetOutPath $INSTDIR\pmvs
	File ..\..\thirdparty\pmvs\*.*
	WriteRegStr HKCU "Software\hiesti.ch\Regard3D" "ExternalEXEPath" "$INSTDIR"

	; Set output path to local AppData
	SetOutPath $LOCALAPPDATA\Regard3D
	File ..\..\thirdparty\font\dejavu-fonts-ttf-2.34\ttf\DejaVuSans.ttf
	File ..\..\sensor_database.csv

	WriteRegStr HKCU "Software\hiesti.ch\Regard3D" "FontFilename" "$LOCALAPPDATA\Regard3D\DejaVuSans.ttf"
	WriteRegStr HKCU "Software\hiesti.ch\Regard3D" "CameraDBFilename" "$LOCALAPPDATA\Regard3D\sensor_database.csv"
	
	; Insert links in start menu
	!insertmacro MUI_STARTMENU_WRITE_BEGIN Application
		CreateDirectory "$SMPROGRAMS\$StartMenuFolder"
		CreateShortCut "$SMPROGRAMS\$StartMenuFolder\Regard3D.lnk" "$INSTDIR\Regard3D.exe"
		CreateShortCut "$SMPROGRAMS\$StartMenuFolder\Uninstall.lnk" "$INSTDIR\Uninstall.exe"
  	!insertmacro MUI_STARTMENU_WRITE_END

	; Write uninstall routine and some additional info into registry
	WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\Regard3D" "DisplayName" "EncFS MP"
	WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\Regard3D" "UninstallString" '"$INSTDIR\Uninstall.exe"'
	WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\Regard3D" "QuietUninstallString" "$\"$INSTDIR\Uninstall.exe$\" /S"
	WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\Regard3D" "InstallLocation" "$\"$INSTDIR$\""
	WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\Regard3D" "DisplayIcon" "$\"$INSTDIR\Regard3D.exe$\""
	WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\Regard3D" "Publisher" "${COMPANY_NAME}"
	WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\Regard3D" "DisplayVersion" "${VER_MAJOR}.${VER_MINOR}.${VER_BUILD}"
	WriteRegDword HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\Regard3D" "NoModify" "1"
	WriteRegDword HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\Regard3D" "NoRepair" "1"

SectionEnd

LangString Message64bit ${LANG_ENGLISH} "Regard3D is a 64-bit program and can't be installed on a 32 bit version Windows."
LangString Message64bit ${LANG_FRENCH} "Regard3D est un programme 64-bit et ne peut pas être installée sur une version 32 bit de Windows."
LangString Message64bit ${LANG_GERMAN} "Regard3D ist ein 64-bit Programm und kann nicht auf einer 32-Bit-Version von Windows installiert werden."

; Install the Visual C++ 2015 Update 3 Redistributable Package (x64)
Section "Visual C++ 2015 libraries" VCRedist

	SectionIn 1

	SetOutPath $TEMP
	
	IfFileExists "$TEMP\vc_redist.x64.exe" ErrorVCRedistFileExists
	DetailPrint "Installing Visual C++ 2015 Redistributable Package (x64)"
	File res\vc_redist.x64.exe
	ExecWait '"$TEMP\vc_redist.x64.exe" /q'
	Delete "$TEMP\vc_redist.x64.exe"

	Return

ErrorVCRedistFileExists:
	MessageBox MB_OK|MB_ICONEXCLAMATION "Error while extracting Visual C++ 2015 Redistributable Package (x64): File already exists"
SectionEnd

Function .onInit
	${If} ${RunningX64}
		SetRegView 64
	${Else}
		MessageBox MB_OK $(Message64bit)
		Abort
	${EndIf}
	!insertmacro MUI_LANGDLL_DISPLAY
FunctionEnd

Section "Uninstall" Uninstall
	
	Delete "$LOCALAPPDATA\Regard3D\DejaVuSans.ttf"
	Delete "$LOCALAPPDATA\Regard3D\sensor_database.csv"
	RMDir "$LOCALAPPDATA\Regard3D"
	RMDir /r "$INSTDIR\licenses"
	RMDir /r "$INSTDIR\poisson"
	RMDir /r "$INSTDIR\mve"
	RMDir /r "$INSTDIR\pmvs"
	Delete "$INSTDIR\Uninstall.exe"
	Delete "$INSTDIR\*omp*.dll"
	Delete "$INSTDIR\*pthread*.dll"
	Delete "$INSTDIR\libgcc_s_seh-1.dll"
	Delete "$INSTDIR\libgfortran-3.dll"
	Delete "$INSTDIR\libopenblas.dll"
	Delete "$INSTDIR\libquadmath-0.dll"
	Delete "$INSTDIR\libwinpthread-1.dll"
	Delete "$INSTDIR\Regard3D.exe"
	RMDir "$INSTDIR"

	!insertmacro MUI_STARTMENU_GETFOLDER Application $StartMenuFolder
	Delete "$SMPROGRAMS\$StartMenuFolder\Licenses.lnk"
	Delete "$SMPROGRAMS\$StartMenuFolder\Uninstall.lnk"
	Delete "$SMPROGRAMS\$StartMenuFolder\Regard3D.lnk"
	RMDir "$SMPROGRAMS\$StartMenuFolder"
	DeleteRegKey /ifempty HKCU "Software\hiesti.ch\Regard3D"
	DeleteRegKey HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\Regard3D"

SectionEnd

Function un.onInit
	${If} ${RunningX64}
		SetRegView 64
	${EndIf}
  !insertmacro MUI_UNGETLANGUAGE
FunctionEnd
