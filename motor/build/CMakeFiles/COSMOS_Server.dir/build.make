# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.0

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

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
CMAKE_SOURCE_DIR = /home/pi/spintable/motor

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/spintable/motor/build

# Include any dependencies generated for this target.
include CMakeFiles/COSMOS_Server.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/COSMOS_Server.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/COSMOS_Server.dir/flags.make

CMakeFiles/COSMOS_Server.dir/server.cpp.o: CMakeFiles/COSMOS_Server.dir/flags.make
CMakeFiles/COSMOS_Server.dir/server.cpp.o: ../server.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pi/spintable/motor/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/COSMOS_Server.dir/server.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/COSMOS_Server.dir/server.cpp.o -c /home/pi/spintable/motor/server.cpp

CMakeFiles/COSMOS_Server.dir/server.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/COSMOS_Server.dir/server.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/pi/spintable/motor/server.cpp > CMakeFiles/COSMOS_Server.dir/server.cpp.i

CMakeFiles/COSMOS_Server.dir/server.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/COSMOS_Server.dir/server.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/pi/spintable/motor/server.cpp -o CMakeFiles/COSMOS_Server.dir/server.cpp.s

CMakeFiles/COSMOS_Server.dir/server.cpp.o.requires:
.PHONY : CMakeFiles/COSMOS_Server.dir/server.cpp.o.requires

CMakeFiles/COSMOS_Server.dir/server.cpp.o.provides: CMakeFiles/COSMOS_Server.dir/server.cpp.o.requires
	$(MAKE) -f CMakeFiles/COSMOS_Server.dir/build.make CMakeFiles/COSMOS_Server.dir/server.cpp.o.provides.build
.PHONY : CMakeFiles/COSMOS_Server.dir/server.cpp.o.provides

CMakeFiles/COSMOS_Server.dir/server.cpp.o.provides.build: CMakeFiles/COSMOS_Server.dir/server.cpp.o

CMakeFiles/COSMOS_Server.dir/gps/gps.cpp.o: CMakeFiles/COSMOS_Server.dir/flags.make
CMakeFiles/COSMOS_Server.dir/gps/gps.cpp.o: ../gps/gps.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pi/spintable/motor/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/COSMOS_Server.dir/gps/gps.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/COSMOS_Server.dir/gps/gps.cpp.o -c /home/pi/spintable/motor/gps/gps.cpp

CMakeFiles/COSMOS_Server.dir/gps/gps.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/COSMOS_Server.dir/gps/gps.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/pi/spintable/motor/gps/gps.cpp > CMakeFiles/COSMOS_Server.dir/gps/gps.cpp.i

CMakeFiles/COSMOS_Server.dir/gps/gps.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/COSMOS_Server.dir/gps/gps.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/pi/spintable/motor/gps/gps.cpp -o CMakeFiles/COSMOS_Server.dir/gps/gps.cpp.s

CMakeFiles/COSMOS_Server.dir/gps/gps.cpp.o.requires:
.PHONY : CMakeFiles/COSMOS_Server.dir/gps/gps.cpp.o.requires

CMakeFiles/COSMOS_Server.dir/gps/gps.cpp.o.provides: CMakeFiles/COSMOS_Server.dir/gps/gps.cpp.o.requires
	$(MAKE) -f CMakeFiles/COSMOS_Server.dir/build.make CMakeFiles/COSMOS_Server.dir/gps/gps.cpp.o.provides.build
.PHONY : CMakeFiles/COSMOS_Server.dir/gps/gps.cpp.o.provides

CMakeFiles/COSMOS_Server.dir/gps/gps.cpp.o.provides.build: CMakeFiles/COSMOS_Server.dir/gps/gps.cpp.o

CMakeFiles/COSMOS_Server.dir/motor/dcmotor.cpp.o: CMakeFiles/COSMOS_Server.dir/flags.make
CMakeFiles/COSMOS_Server.dir/motor/dcmotor.cpp.o: ../motor/dcmotor.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pi/spintable/motor/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/COSMOS_Server.dir/motor/dcmotor.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/COSMOS_Server.dir/motor/dcmotor.cpp.o -c /home/pi/spintable/motor/motor/dcmotor.cpp

CMakeFiles/COSMOS_Server.dir/motor/dcmotor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/COSMOS_Server.dir/motor/dcmotor.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/pi/spintable/motor/motor/dcmotor.cpp > CMakeFiles/COSMOS_Server.dir/motor/dcmotor.cpp.i

CMakeFiles/COSMOS_Server.dir/motor/dcmotor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/COSMOS_Server.dir/motor/dcmotor.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/pi/spintable/motor/motor/dcmotor.cpp -o CMakeFiles/COSMOS_Server.dir/motor/dcmotor.cpp.s

CMakeFiles/COSMOS_Server.dir/motor/dcmotor.cpp.o.requires:
.PHONY : CMakeFiles/COSMOS_Server.dir/motor/dcmotor.cpp.o.requires

CMakeFiles/COSMOS_Server.dir/motor/dcmotor.cpp.o.provides: CMakeFiles/COSMOS_Server.dir/motor/dcmotor.cpp.o.requires
	$(MAKE) -f CMakeFiles/COSMOS_Server.dir/build.make CMakeFiles/COSMOS_Server.dir/motor/dcmotor.cpp.o.provides.build
.PHONY : CMakeFiles/COSMOS_Server.dir/motor/dcmotor.cpp.o.provides

CMakeFiles/COSMOS_Server.dir/motor/dcmotor.cpp.o.provides.build: CMakeFiles/COSMOS_Server.dir/motor/dcmotor.cpp.o

CMakeFiles/COSMOS_Server.dir/motor/pwm.cpp.o: CMakeFiles/COSMOS_Server.dir/flags.make
CMakeFiles/COSMOS_Server.dir/motor/pwm.cpp.o: ../motor/pwm.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pi/spintable/motor/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/COSMOS_Server.dir/motor/pwm.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/COSMOS_Server.dir/motor/pwm.cpp.o -c /home/pi/spintable/motor/motor/pwm.cpp

CMakeFiles/COSMOS_Server.dir/motor/pwm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/COSMOS_Server.dir/motor/pwm.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/pi/spintable/motor/motor/pwm.cpp > CMakeFiles/COSMOS_Server.dir/motor/pwm.cpp.i

CMakeFiles/COSMOS_Server.dir/motor/pwm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/COSMOS_Server.dir/motor/pwm.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/pi/spintable/motor/motor/pwm.cpp -o CMakeFiles/COSMOS_Server.dir/motor/pwm.cpp.s

CMakeFiles/COSMOS_Server.dir/motor/pwm.cpp.o.requires:
.PHONY : CMakeFiles/COSMOS_Server.dir/motor/pwm.cpp.o.requires

CMakeFiles/COSMOS_Server.dir/motor/pwm.cpp.o.provides: CMakeFiles/COSMOS_Server.dir/motor/pwm.cpp.o.requires
	$(MAKE) -f CMakeFiles/COSMOS_Server.dir/build.make CMakeFiles/COSMOS_Server.dir/motor/pwm.cpp.o.provides.build
.PHONY : CMakeFiles/COSMOS_Server.dir/motor/pwm.cpp.o.provides

CMakeFiles/COSMOS_Server.dir/motor/pwm.cpp.o.provides.build: CMakeFiles/COSMOS_Server.dir/motor/pwm.cpp.o

# Object files for target COSMOS_Server
COSMOS_Server_OBJECTS = \
"CMakeFiles/COSMOS_Server.dir/server.cpp.o" \
"CMakeFiles/COSMOS_Server.dir/gps/gps.cpp.o" \
"CMakeFiles/COSMOS_Server.dir/motor/dcmotor.cpp.o" \
"CMakeFiles/COSMOS_Server.dir/motor/pwm.cpp.o"

# External object files for target COSMOS_Server
COSMOS_Server_EXTERNAL_OBJECTS =

COSMOS_Server: CMakeFiles/COSMOS_Server.dir/server.cpp.o
COSMOS_Server: CMakeFiles/COSMOS_Server.dir/gps/gps.cpp.o
COSMOS_Server: CMakeFiles/COSMOS_Server.dir/motor/dcmotor.cpp.o
COSMOS_Server: CMakeFiles/COSMOS_Server.dir/motor/pwm.cpp.o
COSMOS_Server: CMakeFiles/COSMOS_Server.dir/build.make
COSMOS_Server: CMakeFiles/COSMOS_Server.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable COSMOS_Server"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/COSMOS_Server.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/COSMOS_Server.dir/build: COSMOS_Server
.PHONY : CMakeFiles/COSMOS_Server.dir/build

CMakeFiles/COSMOS_Server.dir/requires: CMakeFiles/COSMOS_Server.dir/server.cpp.o.requires
CMakeFiles/COSMOS_Server.dir/requires: CMakeFiles/COSMOS_Server.dir/gps/gps.cpp.o.requires
CMakeFiles/COSMOS_Server.dir/requires: CMakeFiles/COSMOS_Server.dir/motor/dcmotor.cpp.o.requires
CMakeFiles/COSMOS_Server.dir/requires: CMakeFiles/COSMOS_Server.dir/motor/pwm.cpp.o.requires
.PHONY : CMakeFiles/COSMOS_Server.dir/requires

CMakeFiles/COSMOS_Server.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/COSMOS_Server.dir/cmake_clean.cmake
.PHONY : CMakeFiles/COSMOS_Server.dir/clean

CMakeFiles/COSMOS_Server.dir/depend:
	cd /home/pi/spintable/motor/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/spintable/motor /home/pi/spintable/motor /home/pi/spintable/motor/build /home/pi/spintable/motor/build /home/pi/spintable/motor/build/CMakeFiles/COSMOS_Server.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/COSMOS_Server.dir/depend

