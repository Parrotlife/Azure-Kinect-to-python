# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_SOURCE_DIR = /home/logimaster/Documents/jeanmarc/Azure-Kinect-SDK-to-python

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/logimaster/Documents/jeanmarc/Azure-Kinect-SDK-to-python

# Include any dependencies generated for this target.
include src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/depend.make

# Include the progress variables for this target.
include src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/progress.make

# Include the compile flags for this target's objects.
include src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/flags.make

src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/main.cpp.o: src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/flags.make
src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/main.cpp.o: src/simple_3d_viewer/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/logimaster/Documents/jeanmarc/Azure-Kinect-SDK-to-python/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/main.cpp.o"
	cd /home/logimaster/Documents/jeanmarc/Azure-Kinect-SDK-to-python/src/simple_3d_viewer && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/simple_3d_viewer.dir/main.cpp.o -c /home/logimaster/Documents/jeanmarc/Azure-Kinect-SDK-to-python/src/simple_3d_viewer/main.cpp

src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simple_3d_viewer.dir/main.cpp.i"
	cd /home/logimaster/Documents/jeanmarc/Azure-Kinect-SDK-to-python/src/simple_3d_viewer && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/logimaster/Documents/jeanmarc/Azure-Kinect-SDK-to-python/src/simple_3d_viewer/main.cpp > CMakeFiles/simple_3d_viewer.dir/main.cpp.i

src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simple_3d_viewer.dir/main.cpp.s"
	cd /home/logimaster/Documents/jeanmarc/Azure-Kinect-SDK-to-python/src/simple_3d_viewer && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/logimaster/Documents/jeanmarc/Azure-Kinect-SDK-to-python/src/simple_3d_viewer/main.cpp -o CMakeFiles/simple_3d_viewer.dir/main.cpp.s

src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/main.cpp.o.requires:

.PHONY : src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/main.cpp.o.requires

src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/main.cpp.o.provides: src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/main.cpp.o.requires
	$(MAKE) -f src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/build.make src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/main.cpp.o.provides.build
.PHONY : src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/main.cpp.o.provides

src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/main.cpp.o.provides.build: src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/main.cpp.o


# Object files for target simple_3d_viewer
simple_3d_viewer_OBJECTS = \
"CMakeFiles/simple_3d_viewer.dir/main.cpp.o"

# External object files for target simple_3d_viewer
simple_3d_viewer_EXTERNAL_OBJECTS =

bin/simple_3d_viewer: src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/main.cpp.o
bin/simple_3d_viewer: src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/build.make
bin/simple_3d_viewer: src/sample_helper_libs/window_controller_3d/libwindow_controller_3d.a
bin/simple_3d_viewer: src/extern/glfw/src/src/libglfw3.a
bin/simple_3d_viewer: /usr/lib/x86_64-linux-gnu/librt.so
bin/simple_3d_viewer: /usr/lib/x86_64-linux-gnu/libm.so
bin/simple_3d_viewer: /usr/lib/x86_64-linux-gnu/libX11.so
bin/simple_3d_viewer: /usr/lib/x86_64-linux-gnu/libXrandr.so
bin/simple_3d_viewer: /usr/lib/x86_64-linux-gnu/libXinerama.so
bin/simple_3d_viewer: /usr/lib/x86_64-linux-gnu/libXcursor.so
bin/simple_3d_viewer: src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/logimaster/Documents/jeanmarc/Azure-Kinect-SDK-to-python/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/simple_3d_viewer"
	cd /home/logimaster/Documents/jeanmarc/Azure-Kinect-SDK-to-python/src/simple_3d_viewer && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/simple_3d_viewer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/build: bin/simple_3d_viewer

.PHONY : src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/build

src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/requires: src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/main.cpp.o.requires

.PHONY : src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/requires

src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/clean:
	cd /home/logimaster/Documents/jeanmarc/Azure-Kinect-SDK-to-python/src/simple_3d_viewer && $(CMAKE_COMMAND) -P CMakeFiles/simple_3d_viewer.dir/cmake_clean.cmake
.PHONY : src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/clean

src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/depend:
	cd /home/logimaster/Documents/jeanmarc/Azure-Kinect-SDK-to-python && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/logimaster/Documents/jeanmarc/Azure-Kinect-SDK-to-python /home/logimaster/Documents/jeanmarc/Azure-Kinect-SDK-to-python/src/simple_3d_viewer /home/logimaster/Documents/jeanmarc/Azure-Kinect-SDK-to-python /home/logimaster/Documents/jeanmarc/Azure-Kinect-SDK-to-python/src/simple_3d_viewer /home/logimaster/Documents/jeanmarc/Azure-Kinect-SDK-to-python/src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/depend

