# CMAKE generated file: DO NOT EDIT!
# Generated by "NMake Makefiles" Generator, CMake Version 3.1

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

!IF "$(OS)" == "Windows_NT"
NULL=
!ELSE
NULL=nul
!ENDIF
SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files (x86)\CMake\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files (x86)\CMake\bin\cmake.exe" -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\build

# Utility rule file for safepdf.

# Include the progress variables for this target.
include doc\CMakeFiles\safepdf.dir\progress.make

doc\CMakeFiles\safepdf:
	cd C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\build\doc
	echo >nul && "C:\Program Files (x86)\CMake\bin\cmake.exe" -E chdir C:/Users/Filip/Workspace/cmp/EAST_libs_build/flann-1.8.4-src/build/doc "C:/Program Files/MiKTeX 2.9/miktex/bin/x64/ps2pdf14.exe" -dMaxSubsetPct=100 -dCompatibilityLevel=1.3 -dSubsetFonts=true -dEmbedAllFonts=true -dAutoFilterColorImages=false -dAutoFilterGrayImages=false -dColorImageFilter=/FlateEncode -dGrayImageFilter=/FlateEncode -dMonoImageFilter=/FlateEncode manual.ps manual.pdf
	cd C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\build

safepdf: doc\CMakeFiles\safepdf
safepdf: doc\CMakeFiles\safepdf.dir\build.make
.PHONY : safepdf

# Rule to build all files generated by this target.
doc\CMakeFiles\safepdf.dir\build: safepdf
.PHONY : doc\CMakeFiles\safepdf.dir\build

doc\CMakeFiles\safepdf.dir\clean:
	cd C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\build\doc
	$(CMAKE_COMMAND) -P CMakeFiles\safepdf.dir\cmake_clean.cmake
	cd C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\build
.PHONY : doc\CMakeFiles\safepdf.dir\clean

doc\CMakeFiles\safepdf.dir\depend:
	$(CMAKE_COMMAND) -E cmake_depends "NMake Makefiles" C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\doc C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\build C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\build\doc C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\build\doc\CMakeFiles\safepdf.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : doc\CMakeFiles\safepdf.dir\depend

