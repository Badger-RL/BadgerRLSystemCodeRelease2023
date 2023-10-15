
# Badger RL README

This is the Code Release to go with BadgerRL's participation in 2023 Robocup SPL as team BadgerBots. It is pre-loaded with all necessary policies to field a team exactly like the one we used at Robocup 2023. It is Configured to assume a team of 5 robots.

## Setup instructions

This codebase is compatible with both MacOS and Ubuntu 22.04, though the experience tends to be best on Ubuntu 22.04.  

To setup up this repository, first install the dependencies via apt as listed [here](https://wiki.b-human.de/coderelease2022/getting-started/), then run:


````
git clone --recursive git@github.com:Badger-RL/BadgerRLSystem2022.git

````

There are MacOS and Ubuntu specific instructions for generating and compiling the code [here](https://wiki.b-human.de/coderelease2022/getting-started/).


To run the simulator after compiling it(example provided for Ubuntu), you can run the following command from the top level directory of the project:


````
./Build/Linux/SimRobot/Develop/SimRobot
````

## Basic usage instructions

After opening SimRobot, Go to File, then open to select a scene from `Config/Scenes`. A good first scene to try is OneTeamFast.ros2. You can also try GameFast for a a full game.  

Once you open the Scene file, there will be a window titled scene graph, which shows `Console` and `Robocup`. Double click on each of these to open the visualization window and SimRobot internal console.  

To test out the team of robots in simulation, several commands are important:

To instruct the robots to prepare for a kickoff, type

````
gc ready
````
To tell the robots to stop moving pending a kickoff, type 
````
gc set
````
and finally, to tell the robots to start the countdown after which the the kickoff will start and the game will enter the playing mode, type

````
gc playing
````

## Reinforcement Learning-Derived Policies


Policies trained with https://github.com/Badger-RL/AbstractSimRelease2023 can be placed in `Config/Policies/AttackerPolicy` and `Config/Policies/GoalKeeperKickPolicy` to control the attacker/defenders and goalie respectively. Instructions for how to export a policy are contained in the README.md files in https://github.com/Badger-RL/AbstractSimRelease2023.


## Live Stream Physical Robot Image View
To live stream physical robot view, first select the build checkbox and deploy the robot with the Develop version and a non-negative magic number. 
  
  <img width="228" alt="image" src="https://user-images.githubusercontent.com/60803591/228946695-17b0436b-cc41-4352-ac9a-282511240e25.png"> <img width="203" alt="image" src="https://user-images.githubusercontent.com/60803591/228946805-e3fa2ec5-5091-477a-ae30-000253dece46.png">
  
  After deployment, open SimRobot, go to File, and select the RemoteRobot.ros2 scene from 'Config/Scenes'. 
  
  Select the corresponding wlan address of the robot, otherwise enter the command sc <Robot Name> <ip address> to connect to the robot remotely using the console. You can find the ip address in the folder Config/Robots/<Robot Name>. An image view should get automatically added to the Scene graph.
  
  <img width="133" alt="image" src="https://user-images.githubusercontent.com/60803591/228947589-c4133380-c5d0-4111-998d-eae37056096a.png">


## Add Image View in SimRobot(Live Stream SimRobot Perspective)

Image Views allows you to view what robots are seeing using their perception with annotations. It consists of upper and lower view. 

First open SimRobot. After opening SimRobot, go to File and select a scene from 'Config/Scenes'. The following scenes have been tested to work with adding Image View: Game.ros2, OneTeam, and ReplayRobot.ros2.

To Add an Image View, first select the robot(s) where you want to add image views by the following command:
  
  To select the robot with the same robot name, type
  ````
  robot <robot name>  
  ````
  To select all robots, type
  ````
  robot all
  ````
  To get all robot's name, type
  ````
  robot ? 
  ````
  
Then, Type the following command:
  To add Lower Image view:
  ````
  vi image Lower lowerImage
  vid lowerImage representation:BallPercept:image
  vid lowerImage representation:LinesPercept:image
  vid lowerImage representation:ObstaclesImagePercept:image
  ````
  To add Upper Image view:
  ````
  vi image Upper upperImage
  vid upperImage representation:BallPercept:image
  vid upperImage representation:LinesPercept:image
  vid upperImage representation:ObstaclesImagePercept:image
  ````
  
  Note: Sometimes if you copied and paste all four commands it wouldn't work. Try copying and pasteing commands line by line instead if that's the case.

# Log File Explanation
  
The B-Human current release comes with a flexible logging system, which may help with debugging processes. Please check out the previous section to live stream or use the default logger.

If, for some reason, we want to modify the information contained in the log file, we can modify the configuration of loggers.

First, we must travel to "Config/Scenarios" to pick one Scenario. Notably, each Scenario corresponds to different circumstances or roles and has individual logger configurations, respectively. For demonstration, we select the default Scenario. And we can find the "logger.cfg" file in the sub-folder. There are multiple configs that we can modify: "enabled", "path", "numOfBuffers", "sizeOfBuffer", "writePriority", "minFreeDriverSpace", and "representationPerThread".
  
````
// Is logging enabled?
enabled = true;

// The directory that will contain the log file.
path = "/home/nao/logging";

// The number of buffers allocated.
numOfBuffers = 12000;

// The size of each buffer in bytes.
sizeOfBuffer = 200000;

// The scheduling priority of the writer thread.
writePriority = -2;

// Logging will stop if less MB are available to the target device.
minFreeDriveSpace = 100;

// Representations to log per thread
representationsPerThread = [
  {
    thread = Upper;
    representations = [
      JPEGImage,

      BallPercept,
      BallSpots,
      BodyContour,
      CameraInfo,
      CameraMatrix,
      CirclePercept,
      FieldBoundary,
      FrameInfo,
      ImageCoordinateSystem,
      LinesPercept,
      ObstaclesFieldPercept,
      ObstaclesImagePercept,
      OdometryData,
      PenaltyMarkPercept,
    ];
  },
  {
    thread = Lower;
    representations = [
      JPEGImage,

      BallPercept,
      BallSpots,
      BodyContour,
      CameraInfo,
      CameraMatrix,
      CirclePercept,
      FieldBoundary,
      FrameInfo,
      ImageCoordinateSystem,
      LinesPercept,
      ObstaclesFieldPercept,
      ObstaclesImagePercept,
      OdometryData,
      PenaltyMarkPercept,
    ];
  },
  {
    thread = Cognition;
    representations = [
      ActivationGraph,
      AlternativeRobotPoseHypothesis,
      ArmMotionRequest,
      BallModel,
      BehaviorStatus,
      CameraCalibration,
      GameState,
      IMUCalibration,
      MotionRequest,
      ObstacleModel,
      OdometryData,
      RobotHealth,
      RobotPose,
      SelfLocalizationHypotheses,
      SideInformation,
      SkillRequest,
      StrategyStatus,
      TeammatesBallModel,
      TeamData,
    ];
  },
  {
    thread = Motion;
    representations = [
      FallDownState,
      FootOffset,
      FootSoleRotationCalibration,
      FootSupport,
      FrameInfo,
      FsrData,
      FsrSensorData,
      GroundContactState,
      InertialSensorData,
      InertialData,
      JointCalibration,
      JointPlay,
      JointRequest,
      JointSensorData,
      KeyStates,
      MotionInfo,
      OdometryData,
      OdometryDataPreview,
      SystemSensorData,
      WalkLearner,
      WalkStepData,
    ];
  },
  {
    thread = Audio;
    representations = [
      AudioData,
      FrameInfo,
      Whistle,
    ];
  }
];
````
And supposedly, many representations can be found in "Src/Representations". So theoretically, we can customize data being reported from loggers and thusly extract more critical data such as the TorsoMatrix (Src/Representations/Sensing/TorsoMatrix.cpp) or raw JPEGImage (Src/Representations/Infrastructure/JPEGImage.cpp). The format of log files and the validity of such aforementioned customization shall be further explored.

# Mac compiling for NAO details
When you are compiling the code to be deployed on the physical NAO robots, you must make a small change Src/Tools/RLConfig.h.
Navigate to the file and find the line (about line 20):

#define BUILD_MAC_NAO 2 // 1 for NAO, 0 for MAC, 2 for LINUX

When you are compiling for the NAOs on Mac change this flag to be 1. If you are compiling for MAC for SimRobot, change this to be 0, and for all other uses (likely Linux), leave at 2 or change to 2 if it is not. This should then compile with no problems but if it doesn't, reach out to Adam and he will help.

# B-Human Code Release statment
This product includes software developed by B-Human(http://www.b-human.de).

Please see License.md for the original BHuman License.

# Other Third Party Code
 
## https://github.com/kthohr/gcem
Apache2.0


## https://github.com/kthohr/stats
Apache2.0


## https://github.com/eteran/cpp-json
pp-json
Copyright (C) 2014-2023  Evan Teran
                         evan.teran@gmail.com

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
