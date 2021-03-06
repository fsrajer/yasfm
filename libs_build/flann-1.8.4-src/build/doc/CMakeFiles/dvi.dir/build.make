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

# Utility rule file for dvi.

# Include the progress variables for this target.
include doc\CMakeFiles\dvi.dir\progress.make

doc\CMakeFiles\dvi: doc\manual.dvi
	cd C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\build\doc
	cd C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\build

doc\manual.dvi: doc\images\cmake-gui.eps
doc\manual.dvi: doc\manual.tex
doc\manual.dvi: doc\references.bib
	$(CMAKE_COMMAND) -E cmake_progress_report C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\build\CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating manual.dvi"
	cd C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\build\doc
	echo >nul && "C:\Program Files (x86)\CMake\bin\cmake.exe" -E chdir C:/Users/Filip/Workspace/cmp/EAST_libs_build/flann-1.8.4-src/build/doc "C:/Program Files/MiKTeX 2.9/miktex/bin/x64/latex.exe" -interaction=batchmode manual.tex
	echo >nul && "C:\Program Files (x86)\CMake\bin\cmake.exe" -E chdir C:/Users/Filip/Workspace/cmp/EAST_libs_build/flann-1.8.4-src/build/doc "C:/Program Files/MiKTeX 2.9/miktex/bin/x64/bibtex.exe" manual
	echo >nul && "C:\Program Files (x86)\CMake\bin\cmake.exe" -E chdir C:/Users/Filip/Workspace/cmp/EAST_libs_build/flann-1.8.4-src/build/doc "C:/Program Files/MiKTeX 2.9/miktex/bin/x64/latex.exe" -interaction=batchmode manual.tex
	echo >nul && "C:\Program Files (x86)\CMake\bin\cmake.exe" -E chdir C:/Users/Filip/Workspace/cmp/EAST_libs_build/flann-1.8.4-src/build/doc "C:/Program Files/MiKTeX 2.9/miktex/bin/x64/latex.exe" -interaction=batchmode manual.tex
	cd C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\build

doc\images\cmake-gui.eps: ..\doc\images\cmake-gui.png
	$(CMAKE_COMMAND) -E cmake_progress_report C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\build\CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating images/cmake-gui.eps"
	cd C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\build\doc
	C:\Windows\System32\convert.exe C:/Users/Filip/Workspace/cmp/EAST_libs_build/flann-1.8.4-src/doc/images/cmake-gui.png C:/Users/Filip/Workspace/cmp/EAST_libs_build/flann-1.8.4-src/build/doc/images/cmake-gui.eps
	cd C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\build

doc\manual.tex: ..\doc\manual.tex
	$(CMAKE_COMMAND) -E cmake_progress_report C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\build\CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating manual.tex"
	cd C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\build\doc
	echo >nul && "C:\Program Files (x86)\CMake\bin\cmake.exe" -E copy C:/Users/Filip/Workspace/cmp/EAST_libs_build/flann-1.8.4-src/doc/manual.tex C:/Users/Filip/Workspace/cmp/EAST_libs_build/flann-1.8.4-src/build/doc/manual.tex
	cd C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\build

doc\references.bib: ..\doc\references.bib
	$(CMAKE_COMMAND) -E cmake_progress_report C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\build\CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating references.bib"
	cd C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\build\doc
	echo >nul && "C:\Program Files (x86)\CMake\bin\cmake.exe" -E copy C:/Users/Filip/Workspace/cmp/EAST_libs_build/flann-1.8.4-src/doc/references.bib C:/Users/Filip/Workspace/cmp/EAST_libs_build/flann-1.8.4-src/build/doc/references.bib
	cd C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\build

dvi: doc\CMakeFiles\dvi
dvi: doc\manual.dvi
dvi: doc\images\cmake-gui.eps
dvi: doc\manual.tex
dvi: doc\references.bib
dvi: doc\CMakeFiles\dvi.dir\build.make
.PHONY : dvi

# Rule to build all files generated by this target.
doc\CMakeFiles\dvi.dir\build: dvi
.PHONY : doc\CMakeFiles\dvi.dir\build

doc\CMakeFiles\dvi.dir\clean:
	cd C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\build\doc
	$(CMAKE_COMMAND) -P CMakeFiles\dvi.dir\cmake_clean.cmake
	cd C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\build
.PHONY : doc\CMakeFiles\dvi.dir\clean

doc\CMakeFiles\dvi.dir\depend:
	$(CMAKE_COMMAND) -E cmake_depends "NMake Makefiles" C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\doc C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\build C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\build\doc C:\Users\Filip\Workspace\cmp\EAST_libs_build\flann-1.8.4-src\build\doc\CMakeFiles\dvi.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : doc\CMakeFiles\dvi.dir\depend

