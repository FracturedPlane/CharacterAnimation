# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704

# Include any dependencies generated for this target.
include Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/depend.make

# Include the progress variables for this target.
include Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/progress.make

# Include the compile flags for this target's objects.
include Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/flags.make

Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/MotorDemo.o: Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/flags.make
Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/MotorDemo.o: Demos/DynamicControlDemo/MotorDemo.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/MotorDemo.o"
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Demos/DynamicControlDemo && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/AppMotorDemo.dir/MotorDemo.o -c /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Demos/DynamicControlDemo/MotorDemo.cpp

Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/MotorDemo.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AppMotorDemo.dir/MotorDemo.i"
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Demos/DynamicControlDemo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Demos/DynamicControlDemo/MotorDemo.cpp > CMakeFiles/AppMotorDemo.dir/MotorDemo.i

Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/MotorDemo.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AppMotorDemo.dir/MotorDemo.s"
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Demos/DynamicControlDemo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Demos/DynamicControlDemo/MotorDemo.cpp -o CMakeFiles/AppMotorDemo.dir/MotorDemo.s

Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/MotorDemo.o.requires:
.PHONY : Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/MotorDemo.o.requires

Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/MotorDemo.o.provides: Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/MotorDemo.o.requires
	$(MAKE) -f Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/build.make Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/MotorDemo.o.provides.build
.PHONY : Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/MotorDemo.o.provides

Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/MotorDemo.o.provides.build: Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/MotorDemo.o

Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/main.o: Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/flags.make
Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/main.o: Demos/DynamicControlDemo/main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/main.o"
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Demos/DynamicControlDemo && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/AppMotorDemo.dir/main.o -c /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Demos/DynamicControlDemo/main.cpp

Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/main.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AppMotorDemo.dir/main.i"
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Demos/DynamicControlDemo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Demos/DynamicControlDemo/main.cpp > CMakeFiles/AppMotorDemo.dir/main.i

Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/main.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AppMotorDemo.dir/main.s"
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Demos/DynamicControlDemo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Demos/DynamicControlDemo/main.cpp -o CMakeFiles/AppMotorDemo.dir/main.s

Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/main.o.requires:
.PHONY : Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/main.o.requires

Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/main.o.provides: Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/main.o.requires
	$(MAKE) -f Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/build.make Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/main.o.provides.build
.PHONY : Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/main.o.provides

Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/main.o.provides.build: Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/main.o

# Object files for target AppMotorDemo
AppMotorDemo_OBJECTS = \
"CMakeFiles/AppMotorDemo.dir/MotorDemo.o" \
"CMakeFiles/AppMotorDemo.dir/main.o"

# External object files for target AppMotorDemo
AppMotorDemo_EXTERNAL_OBJECTS =

Demos/DynamicControlDemo/AppMotorDemo: Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/MotorDemo.o
Demos/DynamicControlDemo/AppMotorDemo: Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/main.o
Demos/DynamicControlDemo/AppMotorDemo: Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/build.make
Demos/DynamicControlDemo/AppMotorDemo: Demos/OpenGL/libOpenGLSupport.a
Demos/DynamicControlDemo/AppMotorDemo: src/BulletDynamics/libBulletDynamics.a
Demos/DynamicControlDemo/AppMotorDemo: src/BulletCollision/libBulletCollision.a
Demos/DynamicControlDemo/AppMotorDemo: src/LinearMath/libLinearMath.a
Demos/DynamicControlDemo/AppMotorDemo: /usr/lib/x86_64-linux-gnu/libglut.so
Demos/DynamicControlDemo/AppMotorDemo: /usr/lib/x86_64-linux-gnu/libGL.so
Demos/DynamicControlDemo/AppMotorDemo: /usr/lib/x86_64-linux-gnu/libGLU.so
Demos/DynamicControlDemo/AppMotorDemo: Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable AppMotorDemo"
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Demos/DynamicControlDemo && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/AppMotorDemo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/build: Demos/DynamicControlDemo/AppMotorDemo
.PHONY : Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/build

Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/requires: Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/MotorDemo.o.requires
Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/requires: Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/main.o.requires
.PHONY : Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/requires

Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/clean:
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Demos/DynamicControlDemo && $(CMAKE_COMMAND) -P CMakeFiles/AppMotorDemo.dir/cmake_clean.cmake
.PHONY : Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/clean

Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/depend:
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704 /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Demos/DynamicControlDemo /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704 /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Demos/DynamicControlDemo /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Demos/DynamicControlDemo/CMakeFiles/AppMotorDemo.dir/depend

