# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Produce verbose output by default.
VERBOSE = 1

# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/seheonha/git/AutoHYU-Control/src/lib/acados

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/seheonha/git/AutoHYU-Control/src/lib/acados/build

# Include any dependencies generated for this target.
include external/blasfeo/examples/CMakeFiles/getting_started.dir/depend.make

# Include the progress variables for this target.
include external/blasfeo/examples/CMakeFiles/getting_started.dir/progress.make

# Include the compile flags for this target's objects.
include external/blasfeo/examples/CMakeFiles/getting_started.dir/flags.make

external/blasfeo/examples/CMakeFiles/getting_started.dir/getting_started.c.o: external/blasfeo/examples/CMakeFiles/getting_started.dir/flags.make
external/blasfeo/examples/CMakeFiles/getting_started.dir/getting_started.c.o: ../external/blasfeo/examples/getting_started.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/seheonha/git/AutoHYU-Control/src/lib/acados/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object external/blasfeo/examples/CMakeFiles/getting_started.dir/getting_started.c.o"
	cd /home/seheonha/git/AutoHYU-Control/src/lib/acados/build/external/blasfeo/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/getting_started.dir/getting_started.c.o   -c /home/seheonha/git/AutoHYU-Control/src/lib/acados/external/blasfeo/examples/getting_started.c

external/blasfeo/examples/CMakeFiles/getting_started.dir/getting_started.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/getting_started.dir/getting_started.c.i"
	cd /home/seheonha/git/AutoHYU-Control/src/lib/acados/build/external/blasfeo/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/seheonha/git/AutoHYU-Control/src/lib/acados/external/blasfeo/examples/getting_started.c > CMakeFiles/getting_started.dir/getting_started.c.i

external/blasfeo/examples/CMakeFiles/getting_started.dir/getting_started.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/getting_started.dir/getting_started.c.s"
	cd /home/seheonha/git/AutoHYU-Control/src/lib/acados/build/external/blasfeo/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/seheonha/git/AutoHYU-Control/src/lib/acados/external/blasfeo/examples/getting_started.c -o CMakeFiles/getting_started.dir/getting_started.c.s

# Object files for target getting_started
getting_started_OBJECTS = \
"CMakeFiles/getting_started.dir/getting_started.c.o"

# External object files for target getting_started
getting_started_EXTERNAL_OBJECTS =

external/blasfeo/examples/getting_started: external/blasfeo/examples/CMakeFiles/getting_started.dir/getting_started.c.o
external/blasfeo/examples/getting_started: external/blasfeo/examples/CMakeFiles/getting_started.dir/build.make
external/blasfeo/examples/getting_started: external/blasfeo/libblasfeo.so
external/blasfeo/examples/getting_started: external/blasfeo/examples/CMakeFiles/getting_started.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/seheonha/git/AutoHYU-Control/src/lib/acados/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable getting_started"
	cd /home/seheonha/git/AutoHYU-Control/src/lib/acados/build/external/blasfeo/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/getting_started.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
external/blasfeo/examples/CMakeFiles/getting_started.dir/build: external/blasfeo/examples/getting_started

.PHONY : external/blasfeo/examples/CMakeFiles/getting_started.dir/build

external/blasfeo/examples/CMakeFiles/getting_started.dir/clean:
	cd /home/seheonha/git/AutoHYU-Control/src/lib/acados/build/external/blasfeo/examples && $(CMAKE_COMMAND) -P CMakeFiles/getting_started.dir/cmake_clean.cmake
.PHONY : external/blasfeo/examples/CMakeFiles/getting_started.dir/clean

external/blasfeo/examples/CMakeFiles/getting_started.dir/depend:
	cd /home/seheonha/git/AutoHYU-Control/src/lib/acados/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/seheonha/git/AutoHYU-Control/src/lib/acados /home/seheonha/git/AutoHYU-Control/src/lib/acados/external/blasfeo/examples /home/seheonha/git/AutoHYU-Control/src/lib/acados/build /home/seheonha/git/AutoHYU-Control/src/lib/acados/build/external/blasfeo/examples /home/seheonha/git/AutoHYU-Control/src/lib/acados/build/external/blasfeo/examples/CMakeFiles/getting_started.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : external/blasfeo/examples/CMakeFiles/getting_started.dir/depend

