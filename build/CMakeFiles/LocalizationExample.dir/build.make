# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

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
CMAKE_SOURCE_DIR = /home/jack/Documents/test_gtsam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jack/Documents/test_gtsam/build

# Include any dependencies generated for this target.
include CMakeFiles/LocalizationExample.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/LocalizationExample.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/LocalizationExample.dir/flags.make

CMakeFiles/LocalizationExample.dir/src/LocalizationExample.cpp.o: CMakeFiles/LocalizationExample.dir/flags.make
CMakeFiles/LocalizationExample.dir/src/LocalizationExample.cpp.o: ../src/LocalizationExample.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/jack/Documents/test_gtsam/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/LocalizationExample.dir/src/LocalizationExample.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/LocalizationExample.dir/src/LocalizationExample.cpp.o -c /home/jack/Documents/test_gtsam/src/LocalizationExample.cpp

CMakeFiles/LocalizationExample.dir/src/LocalizationExample.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LocalizationExample.dir/src/LocalizationExample.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/jack/Documents/test_gtsam/src/LocalizationExample.cpp > CMakeFiles/LocalizationExample.dir/src/LocalizationExample.cpp.i

CMakeFiles/LocalizationExample.dir/src/LocalizationExample.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LocalizationExample.dir/src/LocalizationExample.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/jack/Documents/test_gtsam/src/LocalizationExample.cpp -o CMakeFiles/LocalizationExample.dir/src/LocalizationExample.cpp.s

CMakeFiles/LocalizationExample.dir/src/LocalizationExample.cpp.o.requires:
.PHONY : CMakeFiles/LocalizationExample.dir/src/LocalizationExample.cpp.o.requires

CMakeFiles/LocalizationExample.dir/src/LocalizationExample.cpp.o.provides: CMakeFiles/LocalizationExample.dir/src/LocalizationExample.cpp.o.requires
	$(MAKE) -f CMakeFiles/LocalizationExample.dir/build.make CMakeFiles/LocalizationExample.dir/src/LocalizationExample.cpp.o.provides.build
.PHONY : CMakeFiles/LocalizationExample.dir/src/LocalizationExample.cpp.o.provides

CMakeFiles/LocalizationExample.dir/src/LocalizationExample.cpp.o.provides.build: CMakeFiles/LocalizationExample.dir/src/LocalizationExample.cpp.o

# Object files for target LocalizationExample
LocalizationExample_OBJECTS = \
"CMakeFiles/LocalizationExample.dir/src/LocalizationExample.cpp.o"

# External object files for target LocalizationExample
LocalizationExample_EXTERNAL_OBJECTS =

LocalizationExample: CMakeFiles/LocalizationExample.dir/src/LocalizationExample.cpp.o
LocalizationExample: CMakeFiles/LocalizationExample.dir/build.make
LocalizationExample: /usr/local/lib/libgtsam.so.4.0.0
LocalizationExample: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
LocalizationExample: /usr/lib/x86_64-linux-gnu/libboost_system.so
LocalizationExample: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
LocalizationExample: /usr/lib/x86_64-linux-gnu/libboost_thread.so
LocalizationExample: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
LocalizationExample: /usr/lib/x86_64-linux-gnu/libboost_timer.so
LocalizationExample: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
LocalizationExample: /usr/lib/libtbb.so
LocalizationExample: /usr/lib/libtbbmalloc.so
LocalizationExample: /usr/local/lib/libmetis.so
LocalizationExample: CMakeFiles/LocalizationExample.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable LocalizationExample"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/LocalizationExample.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/LocalizationExample.dir/build: LocalizationExample
.PHONY : CMakeFiles/LocalizationExample.dir/build

CMakeFiles/LocalizationExample.dir/requires: CMakeFiles/LocalizationExample.dir/src/LocalizationExample.cpp.o.requires
.PHONY : CMakeFiles/LocalizationExample.dir/requires

CMakeFiles/LocalizationExample.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/LocalizationExample.dir/cmake_clean.cmake
.PHONY : CMakeFiles/LocalizationExample.dir/clean

CMakeFiles/LocalizationExample.dir/depend:
	cd /home/jack/Documents/test_gtsam/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jack/Documents/test_gtsam /home/jack/Documents/test_gtsam /home/jack/Documents/test_gtsam/build /home/jack/Documents/test_gtsam/build /home/jack/Documents/test_gtsam/build/CMakeFiles/LocalizationExample.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/LocalizationExample.dir/depend

