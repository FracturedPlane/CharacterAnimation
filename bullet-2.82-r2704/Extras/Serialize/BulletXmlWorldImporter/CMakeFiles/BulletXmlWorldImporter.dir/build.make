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
include Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/depend.make

# Include the progress variables for this target.
include Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/progress.make

# Include the compile flags for this target's objects.
include Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/flags.make

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/btBulletXmlWorldImporter.o: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/flags.make
Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/btBulletXmlWorldImporter.o: Extras/Serialize/BulletXmlWorldImporter/btBulletXmlWorldImporter.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/btBulletXmlWorldImporter.o"
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/BulletXmlWorldImporter.dir/btBulletXmlWorldImporter.o -c /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter/btBulletXmlWorldImporter.cpp

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/btBulletXmlWorldImporter.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BulletXmlWorldImporter.dir/btBulletXmlWorldImporter.i"
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter/btBulletXmlWorldImporter.cpp > CMakeFiles/BulletXmlWorldImporter.dir/btBulletXmlWorldImporter.i

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/btBulletXmlWorldImporter.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BulletXmlWorldImporter.dir/btBulletXmlWorldImporter.s"
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter/btBulletXmlWorldImporter.cpp -o CMakeFiles/BulletXmlWorldImporter.dir/btBulletXmlWorldImporter.s

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/btBulletXmlWorldImporter.o.requires:
.PHONY : Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/btBulletXmlWorldImporter.o.requires

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/btBulletXmlWorldImporter.o.provides: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/btBulletXmlWorldImporter.o.requires
	$(MAKE) -f Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/build.make Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/btBulletXmlWorldImporter.o.provides.build
.PHONY : Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/btBulletXmlWorldImporter.o.provides

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/btBulletXmlWorldImporter.o.provides.build: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/btBulletXmlWorldImporter.o

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/string_split.o: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/flags.make
Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/string_split.o: Extras/Serialize/BulletXmlWorldImporter/string_split.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/string_split.o"
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/BulletXmlWorldImporter.dir/string_split.o -c /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter/string_split.cpp

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/string_split.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BulletXmlWorldImporter.dir/string_split.i"
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter/string_split.cpp > CMakeFiles/BulletXmlWorldImporter.dir/string_split.i

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/string_split.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BulletXmlWorldImporter.dir/string_split.s"
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter/string_split.cpp -o CMakeFiles/BulletXmlWorldImporter.dir/string_split.s

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/string_split.o.requires:
.PHONY : Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/string_split.o.requires

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/string_split.o.provides: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/string_split.o.requires
	$(MAKE) -f Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/build.make Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/string_split.o.provides.build
.PHONY : Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/string_split.o.provides

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/string_split.o.provides.build: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/string_split.o

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxml.o: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/flags.make
Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxml.o: Extras/Serialize/BulletXmlWorldImporter/tinyxml.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxml.o"
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/BulletXmlWorldImporter.dir/tinyxml.o -c /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter/tinyxml.cpp

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxml.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BulletXmlWorldImporter.dir/tinyxml.i"
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter/tinyxml.cpp > CMakeFiles/BulletXmlWorldImporter.dir/tinyxml.i

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxml.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BulletXmlWorldImporter.dir/tinyxml.s"
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter/tinyxml.cpp -o CMakeFiles/BulletXmlWorldImporter.dir/tinyxml.s

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxml.o.requires:
.PHONY : Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxml.o.requires

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxml.o.provides: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxml.o.requires
	$(MAKE) -f Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/build.make Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxml.o.provides.build
.PHONY : Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxml.o.provides

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxml.o.provides.build: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxml.o

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinystr.o: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/flags.make
Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinystr.o: Extras/Serialize/BulletXmlWorldImporter/tinystr.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinystr.o"
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/BulletXmlWorldImporter.dir/tinystr.o -c /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter/tinystr.cpp

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinystr.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BulletXmlWorldImporter.dir/tinystr.i"
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter/tinystr.cpp > CMakeFiles/BulletXmlWorldImporter.dir/tinystr.i

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinystr.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BulletXmlWorldImporter.dir/tinystr.s"
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter/tinystr.cpp -o CMakeFiles/BulletXmlWorldImporter.dir/tinystr.s

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinystr.o.requires:
.PHONY : Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinystr.o.requires

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinystr.o.provides: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinystr.o.requires
	$(MAKE) -f Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/build.make Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinystr.o.provides.build
.PHONY : Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinystr.o.provides

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinystr.o.provides.build: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinystr.o

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlerror.o: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/flags.make
Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlerror.o: Extras/Serialize/BulletXmlWorldImporter/tinyxmlerror.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlerror.o"
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlerror.o -c /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter/tinyxmlerror.cpp

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlerror.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlerror.i"
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter/tinyxmlerror.cpp > CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlerror.i

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlerror.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlerror.s"
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter/tinyxmlerror.cpp -o CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlerror.s

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlerror.o.requires:
.PHONY : Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlerror.o.requires

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlerror.o.provides: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlerror.o.requires
	$(MAKE) -f Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/build.make Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlerror.o.provides.build
.PHONY : Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlerror.o.provides

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlerror.o.provides.build: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlerror.o

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlparser.o: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/flags.make
Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlparser.o: Extras/Serialize/BulletXmlWorldImporter/tinyxmlparser.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlparser.o"
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlparser.o -c /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter/tinyxmlparser.cpp

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlparser.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlparser.i"
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter/tinyxmlparser.cpp > CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlparser.i

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlparser.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlparser.s"
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter/tinyxmlparser.cpp -o CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlparser.s

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlparser.o.requires:
.PHONY : Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlparser.o.requires

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlparser.o.provides: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlparser.o.requires
	$(MAKE) -f Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/build.make Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlparser.o.provides.build
.PHONY : Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlparser.o.provides

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlparser.o.provides.build: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlparser.o

# Object files for target BulletXmlWorldImporter
BulletXmlWorldImporter_OBJECTS = \
"CMakeFiles/BulletXmlWorldImporter.dir/btBulletXmlWorldImporter.o" \
"CMakeFiles/BulletXmlWorldImporter.dir/string_split.o" \
"CMakeFiles/BulletXmlWorldImporter.dir/tinyxml.o" \
"CMakeFiles/BulletXmlWorldImporter.dir/tinystr.o" \
"CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlerror.o" \
"CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlparser.o"

# External object files for target BulletXmlWorldImporter
BulletXmlWorldImporter_EXTERNAL_OBJECTS =

Extras/Serialize/BulletXmlWorldImporter/libBulletXmlWorldImporter.a: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/btBulletXmlWorldImporter.o
Extras/Serialize/BulletXmlWorldImporter/libBulletXmlWorldImporter.a: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/string_split.o
Extras/Serialize/BulletXmlWorldImporter/libBulletXmlWorldImporter.a: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxml.o
Extras/Serialize/BulletXmlWorldImporter/libBulletXmlWorldImporter.a: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinystr.o
Extras/Serialize/BulletXmlWorldImporter/libBulletXmlWorldImporter.a: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlerror.o
Extras/Serialize/BulletXmlWorldImporter/libBulletXmlWorldImporter.a: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlparser.o
Extras/Serialize/BulletXmlWorldImporter/libBulletXmlWorldImporter.a: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/build.make
Extras/Serialize/BulletXmlWorldImporter/libBulletXmlWorldImporter.a: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libBulletXmlWorldImporter.a"
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter && $(CMAKE_COMMAND) -P CMakeFiles/BulletXmlWorldImporter.dir/cmake_clean_target.cmake
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/BulletXmlWorldImporter.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/build: Extras/Serialize/BulletXmlWorldImporter/libBulletXmlWorldImporter.a
.PHONY : Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/build

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/requires: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/btBulletXmlWorldImporter.o.requires
Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/requires: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/string_split.o.requires
Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/requires: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxml.o.requires
Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/requires: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinystr.o.requires
Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/requires: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlerror.o.requires
Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/requires: Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/tinyxmlparser.o.requires
.PHONY : Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/requires

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/clean:
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter && $(CMAKE_COMMAND) -P CMakeFiles/BulletXmlWorldImporter.dir/cmake_clean.cmake
.PHONY : Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/clean

Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/depend:
	cd /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704 /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704 /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter /home/glenpb/playground/CharacterAnimation/bullet-2.82-r2704/Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Extras/Serialize/BulletXmlWorldImporter/CMakeFiles/BulletXmlWorldImporter.dir/depend

