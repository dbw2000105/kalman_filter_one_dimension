# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/dbstg/CLionProjects/kalmanFilter

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dbstg/CLionProjects/kalmanFilter/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/kalmanFilter.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/kalmanFilter.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/kalmanFilter.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/kalmanFilter.dir/flags.make

CMakeFiles/kalmanFilter.dir/main.cpp.o: CMakeFiles/kalmanFilter.dir/flags.make
CMakeFiles/kalmanFilter.dir/main.cpp.o: ../main.cpp
CMakeFiles/kalmanFilter.dir/main.cpp.o: CMakeFiles/kalmanFilter.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dbstg/CLionProjects/kalmanFilter/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/kalmanFilter.dir/main.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/kalmanFilter.dir/main.cpp.o -MF CMakeFiles/kalmanFilter.dir/main.cpp.o.d -o CMakeFiles/kalmanFilter.dir/main.cpp.o -c /home/dbstg/CLionProjects/kalmanFilter/main.cpp

CMakeFiles/kalmanFilter.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kalmanFilter.dir/main.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dbstg/CLionProjects/kalmanFilter/main.cpp > CMakeFiles/kalmanFilter.dir/main.cpp.i

CMakeFiles/kalmanFilter.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kalmanFilter.dir/main.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dbstg/CLionProjects/kalmanFilter/main.cpp -o CMakeFiles/kalmanFilter.dir/main.cpp.s

# Object files for target kalmanFilter
kalmanFilter_OBJECTS = \
"CMakeFiles/kalmanFilter.dir/main.cpp.o"

# External object files for target kalmanFilter
kalmanFilter_EXTERNAL_OBJECTS =

kalmanFilter: CMakeFiles/kalmanFilter.dir/main.cpp.o
kalmanFilter: CMakeFiles/kalmanFilter.dir/build.make
kalmanFilter: CMakeFiles/kalmanFilter.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dbstg/CLionProjects/kalmanFilter/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable kalmanFilter"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kalmanFilter.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/kalmanFilter.dir/build: kalmanFilter
.PHONY : CMakeFiles/kalmanFilter.dir/build

CMakeFiles/kalmanFilter.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/kalmanFilter.dir/cmake_clean.cmake
.PHONY : CMakeFiles/kalmanFilter.dir/clean

CMakeFiles/kalmanFilter.dir/depend:
	cd /home/dbstg/CLionProjects/kalmanFilter/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dbstg/CLionProjects/kalmanFilter /home/dbstg/CLionProjects/kalmanFilter /home/dbstg/CLionProjects/kalmanFilter/cmake-build-debug /home/dbstg/CLionProjects/kalmanFilter/cmake-build-debug /home/dbstg/CLionProjects/kalmanFilter/cmake-build-debug/CMakeFiles/kalmanFilter.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/kalmanFilter.dir/depend

