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
CMAKE_SOURCE_DIR = /home/junchao/biped_control

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/junchao/biped_control/build

# Include any dependencies generated for this target.
include CMakeFiles/src.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/src.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/src.dir/flags.make

CMakeFiles/src.dir/BalanceController/BalanceController.cpp.o: CMakeFiles/src.dir/flags.make
CMakeFiles/src.dir/BalanceController/BalanceController.cpp.o: ../BalanceController/BalanceController.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/junchao/biped_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/src.dir/BalanceController/BalanceController.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/BalanceController/BalanceController.cpp.o -c /home/junchao/biped_control/BalanceController/BalanceController.cpp

CMakeFiles/src.dir/BalanceController/BalanceController.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/BalanceController/BalanceController.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/junchao/biped_control/BalanceController/BalanceController.cpp > CMakeFiles/src.dir/BalanceController/BalanceController.cpp.i

CMakeFiles/src.dir/BalanceController/BalanceController.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/BalanceController/BalanceController.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/junchao/biped_control/BalanceController/BalanceController.cpp -o CMakeFiles/src.dir/BalanceController/BalanceController.cpp.s

CMakeFiles/src.dir/BalanceController/BalanceControllerWrapper.cpp.o: CMakeFiles/src.dir/flags.make
CMakeFiles/src.dir/BalanceController/BalanceControllerWrapper.cpp.o: ../BalanceController/BalanceControllerWrapper.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/junchao/biped_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/src.dir/BalanceController/BalanceControllerWrapper.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/BalanceController/BalanceControllerWrapper.cpp.o -c /home/junchao/biped_control/BalanceController/BalanceControllerWrapper.cpp

CMakeFiles/src.dir/BalanceController/BalanceControllerWrapper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/BalanceController/BalanceControllerWrapper.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/junchao/biped_control/BalanceController/BalanceControllerWrapper.cpp > CMakeFiles/src.dir/BalanceController/BalanceControllerWrapper.cpp.i

CMakeFiles/src.dir/BalanceController/BalanceControllerWrapper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/BalanceController/BalanceControllerWrapper.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/junchao/biped_control/BalanceController/BalanceControllerWrapper.cpp -o CMakeFiles/src.dir/BalanceController/BalanceControllerWrapper.cpp.s

CMakeFiles/src.dir/ConvexMPC/Gait.cpp.o: CMakeFiles/src.dir/flags.make
CMakeFiles/src.dir/ConvexMPC/Gait.cpp.o: ../ConvexMPC/Gait.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/junchao/biped_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/src.dir/ConvexMPC/Gait.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/ConvexMPC/Gait.cpp.o -c /home/junchao/biped_control/ConvexMPC/Gait.cpp

CMakeFiles/src.dir/ConvexMPC/Gait.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/ConvexMPC/Gait.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/junchao/biped_control/ConvexMPC/Gait.cpp > CMakeFiles/src.dir/ConvexMPC/Gait.cpp.i

CMakeFiles/src.dir/ConvexMPC/Gait.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/ConvexMPC/Gait.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/junchao/biped_control/ConvexMPC/Gait.cpp -o CMakeFiles/src.dir/ConvexMPC/Gait.cpp.s

CMakeFiles/src.dir/ConvexMPC/MPCLocoMotion.cpp.o: CMakeFiles/src.dir/flags.make
CMakeFiles/src.dir/ConvexMPC/MPCLocoMotion.cpp.o: ../ConvexMPC/MPCLocoMotion.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/junchao/biped_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/src.dir/ConvexMPC/MPCLocoMotion.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/ConvexMPC/MPCLocoMotion.cpp.o -c /home/junchao/biped_control/ConvexMPC/MPCLocoMotion.cpp

CMakeFiles/src.dir/ConvexMPC/MPCLocoMotion.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/ConvexMPC/MPCLocoMotion.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/junchao/biped_control/ConvexMPC/MPCLocoMotion.cpp > CMakeFiles/src.dir/ConvexMPC/MPCLocoMotion.cpp.i

CMakeFiles/src.dir/ConvexMPC/MPCLocoMotion.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/ConvexMPC/MPCLocoMotion.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/junchao/biped_control/ConvexMPC/MPCLocoMotion.cpp -o CMakeFiles/src.dir/ConvexMPC/MPCLocoMotion.cpp.s

CMakeFiles/src.dir/ConvexMPC/NominalMPC.cpp.o: CMakeFiles/src.dir/flags.make
CMakeFiles/src.dir/ConvexMPC/NominalMPC.cpp.o: ../ConvexMPC/NominalMPC.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/junchao/biped_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/src.dir/ConvexMPC/NominalMPC.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/ConvexMPC/NominalMPC.cpp.o -c /home/junchao/biped_control/ConvexMPC/NominalMPC.cpp

CMakeFiles/src.dir/ConvexMPC/NominalMPC.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/ConvexMPC/NominalMPC.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/junchao/biped_control/ConvexMPC/NominalMPC.cpp > CMakeFiles/src.dir/ConvexMPC/NominalMPC.cpp.i

CMakeFiles/src.dir/ConvexMPC/NominalMPC.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/ConvexMPC/NominalMPC.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/junchao/biped_control/ConvexMPC/NominalMPC.cpp -o CMakeFiles/src.dir/ConvexMPC/NominalMPC.cpp.s

CMakeFiles/src.dir/ConvexMPC/RobotState.cpp.o: CMakeFiles/src.dir/flags.make
CMakeFiles/src.dir/ConvexMPC/RobotState.cpp.o: ../ConvexMPC/RobotState.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/junchao/biped_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/src.dir/ConvexMPC/RobotState.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/ConvexMPC/RobotState.cpp.o -c /home/junchao/biped_control/ConvexMPC/RobotState.cpp

CMakeFiles/src.dir/ConvexMPC/RobotState.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/ConvexMPC/RobotState.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/junchao/biped_control/ConvexMPC/RobotState.cpp > CMakeFiles/src.dir/ConvexMPC/RobotState.cpp.i

CMakeFiles/src.dir/ConvexMPC/RobotState.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/ConvexMPC/RobotState.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/junchao/biped_control/ConvexMPC/RobotState.cpp -o CMakeFiles/src.dir/ConvexMPC/RobotState.cpp.s

CMakeFiles/src.dir/ConvexMPC/SolverMPC.cpp.o: CMakeFiles/src.dir/flags.make
CMakeFiles/src.dir/ConvexMPC/SolverMPC.cpp.o: ../ConvexMPC/SolverMPC.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/junchao/biped_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/src.dir/ConvexMPC/SolverMPC.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/ConvexMPC/SolverMPC.cpp.o -c /home/junchao/biped_control/ConvexMPC/SolverMPC.cpp

CMakeFiles/src.dir/ConvexMPC/SolverMPC.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/ConvexMPC/SolverMPC.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/junchao/biped_control/ConvexMPC/SolverMPC.cpp > CMakeFiles/src.dir/ConvexMPC/SolverMPC.cpp.i

CMakeFiles/src.dir/ConvexMPC/SolverMPC.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/ConvexMPC/SolverMPC.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/junchao/biped_control/ConvexMPC/SolverMPC.cpp -o CMakeFiles/src.dir/ConvexMPC/SolverMPC.cpp.s

CMakeFiles/src.dir/ConvexMPC/StairMPC.cpp.o: CMakeFiles/src.dir/flags.make
CMakeFiles/src.dir/ConvexMPC/StairMPC.cpp.o: ../ConvexMPC/StairMPC.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/junchao/biped_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/src.dir/ConvexMPC/StairMPC.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/ConvexMPC/StairMPC.cpp.o -c /home/junchao/biped_control/ConvexMPC/StairMPC.cpp

CMakeFiles/src.dir/ConvexMPC/StairMPC.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/ConvexMPC/StairMPC.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/junchao/biped_control/ConvexMPC/StairMPC.cpp > CMakeFiles/src.dir/ConvexMPC/StairMPC.cpp.i

CMakeFiles/src.dir/ConvexMPC/StairMPC.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/ConvexMPC/StairMPC.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/junchao/biped_control/ConvexMPC/StairMPC.cpp -o CMakeFiles/src.dir/ConvexMPC/StairMPC.cpp.s

CMakeFiles/src.dir/ConvexMPC/convexMPC_interface.cpp.o: CMakeFiles/src.dir/flags.make
CMakeFiles/src.dir/ConvexMPC/convexMPC_interface.cpp.o: ../ConvexMPC/convexMPC_interface.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/junchao/biped_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/src.dir/ConvexMPC/convexMPC_interface.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/ConvexMPC/convexMPC_interface.cpp.o -c /home/junchao/biped_control/ConvexMPC/convexMPC_interface.cpp

CMakeFiles/src.dir/ConvexMPC/convexMPC_interface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/ConvexMPC/convexMPC_interface.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/junchao/biped_control/ConvexMPC/convexMPC_interface.cpp > CMakeFiles/src.dir/ConvexMPC/convexMPC_interface.cpp.i

CMakeFiles/src.dir/ConvexMPC/convexMPC_interface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/ConvexMPC/convexMPC_interface.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/junchao/biped_control/ConvexMPC/convexMPC_interface.cpp -o CMakeFiles/src.dir/ConvexMPC/convexMPC_interface.cpp.s

CMakeFiles/src.dir/src/FSM/FSM.cpp.o: CMakeFiles/src.dir/flags.make
CMakeFiles/src.dir/src/FSM/FSM.cpp.o: ../src/FSM/FSM.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/junchao/biped_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/src.dir/src/FSM/FSM.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/src/FSM/FSM.cpp.o -c /home/junchao/biped_control/src/FSM/FSM.cpp

CMakeFiles/src.dir/src/FSM/FSM.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/src/FSM/FSM.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/junchao/biped_control/src/FSM/FSM.cpp > CMakeFiles/src.dir/src/FSM/FSM.cpp.i

CMakeFiles/src.dir/src/FSM/FSM.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/src/FSM/FSM.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/junchao/biped_control/src/FSM/FSM.cpp -o CMakeFiles/src.dir/src/FSM/FSM.cpp.s

CMakeFiles/src.dir/src/FSM/FSMState.cpp.o: CMakeFiles/src.dir/flags.make
CMakeFiles/src.dir/src/FSM/FSMState.cpp.o: ../src/FSM/FSMState.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/junchao/biped_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/src.dir/src/FSM/FSMState.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/src/FSM/FSMState.cpp.o -c /home/junchao/biped_control/src/FSM/FSMState.cpp

CMakeFiles/src.dir/src/FSM/FSMState.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/src/FSM/FSMState.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/junchao/biped_control/src/FSM/FSMState.cpp > CMakeFiles/src.dir/src/FSM/FSMState.cpp.i

CMakeFiles/src.dir/src/FSM/FSMState.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/src/FSM/FSMState.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/junchao/biped_control/src/FSM/FSMState.cpp -o CMakeFiles/src.dir/src/FSM/FSMState.cpp.s

CMakeFiles/src.dir/src/FSM/FSMState_Climb.cpp.o: CMakeFiles/src.dir/flags.make
CMakeFiles/src.dir/src/FSM/FSMState_Climb.cpp.o: ../src/FSM/FSMState_Climb.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/junchao/biped_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object CMakeFiles/src.dir/src/FSM/FSMState_Climb.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/src/FSM/FSMState_Climb.cpp.o -c /home/junchao/biped_control/src/FSM/FSMState_Climb.cpp

CMakeFiles/src.dir/src/FSM/FSMState_Climb.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/src/FSM/FSMState_Climb.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/junchao/biped_control/src/FSM/FSMState_Climb.cpp > CMakeFiles/src.dir/src/FSM/FSMState_Climb.cpp.i

CMakeFiles/src.dir/src/FSM/FSMState_Climb.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/src/FSM/FSMState_Climb.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/junchao/biped_control/src/FSM/FSMState_Climb.cpp -o CMakeFiles/src.dir/src/FSM/FSMState_Climb.cpp.s

CMakeFiles/src.dir/src/FSM/FSMState_PDStand.cpp.o: CMakeFiles/src.dir/flags.make
CMakeFiles/src.dir/src/FSM/FSMState_PDStand.cpp.o: ../src/FSM/FSMState_PDStand.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/junchao/biped_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object CMakeFiles/src.dir/src/FSM/FSMState_PDStand.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/src/FSM/FSMState_PDStand.cpp.o -c /home/junchao/biped_control/src/FSM/FSMState_PDStand.cpp

CMakeFiles/src.dir/src/FSM/FSMState_PDStand.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/src/FSM/FSMState_PDStand.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/junchao/biped_control/src/FSM/FSMState_PDStand.cpp > CMakeFiles/src.dir/src/FSM/FSMState_PDStand.cpp.i

CMakeFiles/src.dir/src/FSM/FSMState_PDStand.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/src/FSM/FSMState_PDStand.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/junchao/biped_control/src/FSM/FSMState_PDStand.cpp -o CMakeFiles/src.dir/src/FSM/FSMState_PDStand.cpp.s

CMakeFiles/src.dir/src/FSM/FSMState_Passive.cpp.o: CMakeFiles/src.dir/flags.make
CMakeFiles/src.dir/src/FSM/FSMState_Passive.cpp.o: ../src/FSM/FSMState_Passive.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/junchao/biped_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object CMakeFiles/src.dir/src/FSM/FSMState_Passive.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/src/FSM/FSMState_Passive.cpp.o -c /home/junchao/biped_control/src/FSM/FSMState_Passive.cpp

CMakeFiles/src.dir/src/FSM/FSMState_Passive.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/src/FSM/FSMState_Passive.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/junchao/biped_control/src/FSM/FSMState_Passive.cpp > CMakeFiles/src.dir/src/FSM/FSMState_Passive.cpp.i

CMakeFiles/src.dir/src/FSM/FSMState_Passive.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/src/FSM/FSMState_Passive.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/junchao/biped_control/src/FSM/FSMState_Passive.cpp -o CMakeFiles/src.dir/src/FSM/FSMState_Passive.cpp.s

CMakeFiles/src.dir/src/FSM/FSMState_QPStand.cpp.o: CMakeFiles/src.dir/flags.make
CMakeFiles/src.dir/src/FSM/FSMState_QPStand.cpp.o: ../src/FSM/FSMState_QPStand.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/junchao/biped_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Building CXX object CMakeFiles/src.dir/src/FSM/FSMState_QPStand.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/src/FSM/FSMState_QPStand.cpp.o -c /home/junchao/biped_control/src/FSM/FSMState_QPStand.cpp

CMakeFiles/src.dir/src/FSM/FSMState_QPStand.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/src/FSM/FSMState_QPStand.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/junchao/biped_control/src/FSM/FSMState_QPStand.cpp > CMakeFiles/src.dir/src/FSM/FSMState_QPStand.cpp.i

CMakeFiles/src.dir/src/FSM/FSMState_QPStand.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/src/FSM/FSMState_QPStand.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/junchao/biped_control/src/FSM/FSMState_QPStand.cpp -o CMakeFiles/src.dir/src/FSM/FSMState_QPStand.cpp.s

CMakeFiles/src.dir/src/FSM/FSMState_ThreeFoot.cpp.o: CMakeFiles/src.dir/flags.make
CMakeFiles/src.dir/src/FSM/FSMState_ThreeFoot.cpp.o: ../src/FSM/FSMState_ThreeFoot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/junchao/biped_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Building CXX object CMakeFiles/src.dir/src/FSM/FSMState_ThreeFoot.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/src/FSM/FSMState_ThreeFoot.cpp.o -c /home/junchao/biped_control/src/FSM/FSMState_ThreeFoot.cpp

CMakeFiles/src.dir/src/FSM/FSMState_ThreeFoot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/src/FSM/FSMState_ThreeFoot.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/junchao/biped_control/src/FSM/FSMState_ThreeFoot.cpp > CMakeFiles/src.dir/src/FSM/FSMState_ThreeFoot.cpp.i

CMakeFiles/src.dir/src/FSM/FSMState_ThreeFoot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/src/FSM/FSMState_ThreeFoot.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/junchao/biped_control/src/FSM/FSMState_ThreeFoot.cpp -o CMakeFiles/src.dir/src/FSM/FSMState_ThreeFoot.cpp.s

CMakeFiles/src.dir/src/FSM/FSMState_Walking.cpp.o: CMakeFiles/src.dir/flags.make
CMakeFiles/src.dir/src/FSM/FSMState_Walking.cpp.o: ../src/FSM/FSMState_Walking.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/junchao/biped_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_17) "Building CXX object CMakeFiles/src.dir/src/FSM/FSMState_Walking.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/src/FSM/FSMState_Walking.cpp.o -c /home/junchao/biped_control/src/FSM/FSMState_Walking.cpp

CMakeFiles/src.dir/src/FSM/FSMState_Walking.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/src/FSM/FSMState_Walking.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/junchao/biped_control/src/FSM/FSMState_Walking.cpp > CMakeFiles/src.dir/src/FSM/FSMState_Walking.cpp.i

CMakeFiles/src.dir/src/FSM/FSMState_Walking.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/src/FSM/FSMState_Walking.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/junchao/biped_control/src/FSM/FSMState_Walking.cpp -o CMakeFiles/src.dir/src/FSM/FSMState_Walking.cpp.s

CMakeFiles/src.dir/src/common/DesiredCommand.cpp.o: CMakeFiles/src.dir/flags.make
CMakeFiles/src.dir/src/common/DesiredCommand.cpp.o: ../src/common/DesiredCommand.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/junchao/biped_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_18) "Building CXX object CMakeFiles/src.dir/src/common/DesiredCommand.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/src/common/DesiredCommand.cpp.o -c /home/junchao/biped_control/src/common/DesiredCommand.cpp

CMakeFiles/src.dir/src/common/DesiredCommand.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/src/common/DesiredCommand.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/junchao/biped_control/src/common/DesiredCommand.cpp > CMakeFiles/src.dir/src/common/DesiredCommand.cpp.i

CMakeFiles/src.dir/src/common/DesiredCommand.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/src/common/DesiredCommand.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/junchao/biped_control/src/common/DesiredCommand.cpp -o CMakeFiles/src.dir/src/common/DesiredCommand.cpp.s

CMakeFiles/src.dir/src/common/FootSwingTrajectory.cpp.o: CMakeFiles/src.dir/flags.make
CMakeFiles/src.dir/src/common/FootSwingTrajectory.cpp.o: ../src/common/FootSwingTrajectory.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/junchao/biped_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_19) "Building CXX object CMakeFiles/src.dir/src/common/FootSwingTrajectory.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/src/common/FootSwingTrajectory.cpp.o -c /home/junchao/biped_control/src/common/FootSwingTrajectory.cpp

CMakeFiles/src.dir/src/common/FootSwingTrajectory.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/src/common/FootSwingTrajectory.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/junchao/biped_control/src/common/FootSwingTrajectory.cpp > CMakeFiles/src.dir/src/common/FootSwingTrajectory.cpp.i

CMakeFiles/src.dir/src/common/FootSwingTrajectory.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/src/common/FootSwingTrajectory.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/junchao/biped_control/src/common/FootSwingTrajectory.cpp -o CMakeFiles/src.dir/src/common/FootSwingTrajectory.cpp.s

CMakeFiles/src.dir/src/common/LegController.cpp.o: CMakeFiles/src.dir/flags.make
CMakeFiles/src.dir/src/common/LegController.cpp.o: ../src/common/LegController.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/junchao/biped_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_20) "Building CXX object CMakeFiles/src.dir/src/common/LegController.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/src/common/LegController.cpp.o -c /home/junchao/biped_control/src/common/LegController.cpp

CMakeFiles/src.dir/src/common/LegController.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/src/common/LegController.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/junchao/biped_control/src/common/LegController.cpp > CMakeFiles/src.dir/src/common/LegController.cpp.i

CMakeFiles/src.dir/src/common/LegController.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/src/common/LegController.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/junchao/biped_control/src/common/LegController.cpp -o CMakeFiles/src.dir/src/common/LegController.cpp.s

CMakeFiles/src.dir/src/common/OrientationEstimator.cpp.o: CMakeFiles/src.dir/flags.make
CMakeFiles/src.dir/src/common/OrientationEstimator.cpp.o: ../src/common/OrientationEstimator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/junchao/biped_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_21) "Building CXX object CMakeFiles/src.dir/src/common/OrientationEstimator.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/src/common/OrientationEstimator.cpp.o -c /home/junchao/biped_control/src/common/OrientationEstimator.cpp

CMakeFiles/src.dir/src/common/OrientationEstimator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/src/common/OrientationEstimator.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/junchao/biped_control/src/common/OrientationEstimator.cpp > CMakeFiles/src.dir/src/common/OrientationEstimator.cpp.i

CMakeFiles/src.dir/src/common/OrientationEstimator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/src/common/OrientationEstimator.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/junchao/biped_control/src/common/OrientationEstimator.cpp -o CMakeFiles/src.dir/src/common/OrientationEstimator.cpp.s

CMakeFiles/src.dir/src/common/PositionVelocityEstimator.cpp.o: CMakeFiles/src.dir/flags.make
CMakeFiles/src.dir/src/common/PositionVelocityEstimator.cpp.o: ../src/common/PositionVelocityEstimator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/junchao/biped_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_22) "Building CXX object CMakeFiles/src.dir/src/common/PositionVelocityEstimator.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/src/common/PositionVelocityEstimator.cpp.o -c /home/junchao/biped_control/src/common/PositionVelocityEstimator.cpp

CMakeFiles/src.dir/src/common/PositionVelocityEstimator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/src/common/PositionVelocityEstimator.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/junchao/biped_control/src/common/PositionVelocityEstimator.cpp > CMakeFiles/src.dir/src/common/PositionVelocityEstimator.cpp.i

CMakeFiles/src.dir/src/common/PositionVelocityEstimator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/src/common/PositionVelocityEstimator.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/junchao/biped_control/src/common/PositionVelocityEstimator.cpp -o CMakeFiles/src.dir/src/common/PositionVelocityEstimator.cpp.s

CMakeFiles/src.dir/src/interface/IOSDK.cpp.o: CMakeFiles/src.dir/flags.make
CMakeFiles/src.dir/src/interface/IOSDK.cpp.o: ../src/interface/IOSDK.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/junchao/biped_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_23) "Building CXX object CMakeFiles/src.dir/src/interface/IOSDK.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/src/interface/IOSDK.cpp.o -c /home/junchao/biped_control/src/interface/IOSDK.cpp

CMakeFiles/src.dir/src/interface/IOSDK.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/src/interface/IOSDK.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/junchao/biped_control/src/interface/IOSDK.cpp > CMakeFiles/src.dir/src/interface/IOSDK.cpp.i

CMakeFiles/src.dir/src/interface/IOSDK.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/src/interface/IOSDK.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/junchao/biped_control/src/interface/IOSDK.cpp -o CMakeFiles/src.dir/src/interface/IOSDK.cpp.s

CMakeFiles/src.dir/src/interface/KeyBoard.cpp.o: CMakeFiles/src.dir/flags.make
CMakeFiles/src.dir/src/interface/KeyBoard.cpp.o: ../src/interface/KeyBoard.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/junchao/biped_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_24) "Building CXX object CMakeFiles/src.dir/src/interface/KeyBoard.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/src/interface/KeyBoard.cpp.o -c /home/junchao/biped_control/src/interface/KeyBoard.cpp

CMakeFiles/src.dir/src/interface/KeyBoard.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/src/interface/KeyBoard.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/junchao/biped_control/src/interface/KeyBoard.cpp > CMakeFiles/src.dir/src/interface/KeyBoard.cpp.i

CMakeFiles/src.dir/src/interface/KeyBoard.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/src/interface/KeyBoard.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/junchao/biped_control/src/interface/KeyBoard.cpp -o CMakeFiles/src.dir/src/interface/KeyBoard.cpp.s

CMakeFiles/src.dir/src/interface/WirelessHandle.cpp.o: CMakeFiles/src.dir/flags.make
CMakeFiles/src.dir/src/interface/WirelessHandle.cpp.o: ../src/interface/WirelessHandle.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/junchao/biped_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_25) "Building CXX object CMakeFiles/src.dir/src/interface/WirelessHandle.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/src/interface/WirelessHandle.cpp.o -c /home/junchao/biped_control/src/interface/WirelessHandle.cpp

CMakeFiles/src.dir/src/interface/WirelessHandle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/src/interface/WirelessHandle.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/junchao/biped_control/src/interface/WirelessHandle.cpp > CMakeFiles/src.dir/src/interface/WirelessHandle.cpp.i

CMakeFiles/src.dir/src/interface/WirelessHandle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/src/interface/WirelessHandle.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/junchao/biped_control/src/interface/WirelessHandle.cpp -o CMakeFiles/src.dir/src/interface/WirelessHandle.cpp.s

# Object files for target src
src_OBJECTS = \
"CMakeFiles/src.dir/BalanceController/BalanceController.cpp.o" \
"CMakeFiles/src.dir/BalanceController/BalanceControllerWrapper.cpp.o" \
"CMakeFiles/src.dir/ConvexMPC/Gait.cpp.o" \
"CMakeFiles/src.dir/ConvexMPC/MPCLocoMotion.cpp.o" \
"CMakeFiles/src.dir/ConvexMPC/NominalMPC.cpp.o" \
"CMakeFiles/src.dir/ConvexMPC/RobotState.cpp.o" \
"CMakeFiles/src.dir/ConvexMPC/SolverMPC.cpp.o" \
"CMakeFiles/src.dir/ConvexMPC/StairMPC.cpp.o" \
"CMakeFiles/src.dir/ConvexMPC/convexMPC_interface.cpp.o" \
"CMakeFiles/src.dir/src/FSM/FSM.cpp.o" \
"CMakeFiles/src.dir/src/FSM/FSMState.cpp.o" \
"CMakeFiles/src.dir/src/FSM/FSMState_Climb.cpp.o" \
"CMakeFiles/src.dir/src/FSM/FSMState_PDStand.cpp.o" \
"CMakeFiles/src.dir/src/FSM/FSMState_Passive.cpp.o" \
"CMakeFiles/src.dir/src/FSM/FSMState_QPStand.cpp.o" \
"CMakeFiles/src.dir/src/FSM/FSMState_ThreeFoot.cpp.o" \
"CMakeFiles/src.dir/src/FSM/FSMState_Walking.cpp.o" \
"CMakeFiles/src.dir/src/common/DesiredCommand.cpp.o" \
"CMakeFiles/src.dir/src/common/FootSwingTrajectory.cpp.o" \
"CMakeFiles/src.dir/src/common/LegController.cpp.o" \
"CMakeFiles/src.dir/src/common/OrientationEstimator.cpp.o" \
"CMakeFiles/src.dir/src/common/PositionVelocityEstimator.cpp.o" \
"CMakeFiles/src.dir/src/interface/IOSDK.cpp.o" \
"CMakeFiles/src.dir/src/interface/KeyBoard.cpp.o" \
"CMakeFiles/src.dir/src/interface/WirelessHandle.cpp.o"

# External object files for target src
src_EXTERNAL_OBJECTS =

libsrc.a: CMakeFiles/src.dir/BalanceController/BalanceController.cpp.o
libsrc.a: CMakeFiles/src.dir/BalanceController/BalanceControllerWrapper.cpp.o
libsrc.a: CMakeFiles/src.dir/ConvexMPC/Gait.cpp.o
libsrc.a: CMakeFiles/src.dir/ConvexMPC/MPCLocoMotion.cpp.o
libsrc.a: CMakeFiles/src.dir/ConvexMPC/NominalMPC.cpp.o
libsrc.a: CMakeFiles/src.dir/ConvexMPC/RobotState.cpp.o
libsrc.a: CMakeFiles/src.dir/ConvexMPC/SolverMPC.cpp.o
libsrc.a: CMakeFiles/src.dir/ConvexMPC/StairMPC.cpp.o
libsrc.a: CMakeFiles/src.dir/ConvexMPC/convexMPC_interface.cpp.o
libsrc.a: CMakeFiles/src.dir/src/FSM/FSM.cpp.o
libsrc.a: CMakeFiles/src.dir/src/FSM/FSMState.cpp.o
libsrc.a: CMakeFiles/src.dir/src/FSM/FSMState_Climb.cpp.o
libsrc.a: CMakeFiles/src.dir/src/FSM/FSMState_PDStand.cpp.o
libsrc.a: CMakeFiles/src.dir/src/FSM/FSMState_Passive.cpp.o
libsrc.a: CMakeFiles/src.dir/src/FSM/FSMState_QPStand.cpp.o
libsrc.a: CMakeFiles/src.dir/src/FSM/FSMState_ThreeFoot.cpp.o
libsrc.a: CMakeFiles/src.dir/src/FSM/FSMState_Walking.cpp.o
libsrc.a: CMakeFiles/src.dir/src/common/DesiredCommand.cpp.o
libsrc.a: CMakeFiles/src.dir/src/common/FootSwingTrajectory.cpp.o
libsrc.a: CMakeFiles/src.dir/src/common/LegController.cpp.o
libsrc.a: CMakeFiles/src.dir/src/common/OrientationEstimator.cpp.o
libsrc.a: CMakeFiles/src.dir/src/common/PositionVelocityEstimator.cpp.o
libsrc.a: CMakeFiles/src.dir/src/interface/IOSDK.cpp.o
libsrc.a: CMakeFiles/src.dir/src/interface/KeyBoard.cpp.o
libsrc.a: CMakeFiles/src.dir/src/interface/WirelessHandle.cpp.o
libsrc.a: CMakeFiles/src.dir/build.make
libsrc.a: CMakeFiles/src.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/junchao/biped_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_26) "Linking CXX static library libsrc.a"
	$(CMAKE_COMMAND) -P CMakeFiles/src.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/src.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/src.dir/build: libsrc.a

.PHONY : CMakeFiles/src.dir/build

CMakeFiles/src.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/src.dir/cmake_clean.cmake
.PHONY : CMakeFiles/src.dir/clean

CMakeFiles/src.dir/depend:
	cd /home/junchao/biped_control/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/junchao/biped_control /home/junchao/biped_control /home/junchao/biped_control/build /home/junchao/biped_control/build /home/junchao/biped_control/build/CMakeFiles/src.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/src.dir/depend

