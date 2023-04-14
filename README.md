
# Badger RL README

This is the BadgerRL internal fork of BHumanCodeRelease.

## Setup instructions

This codebase is compatible with both MacOS and Ubuntu 22.04, though the experience tends to be best on Ubuntu 22.04.  

To setup up this repository, first install the dependencies via apt as listed [here](https://wiki.b-human.de/coderelease2022/getting-started/), then run:


````
git clone --recursive https://github.com/Badger-RL/BadgerRLSystem2022.git

````

There are MacOS and Ubuntu specific instructions for generating and compiling the code [here](https://wiki.b-human.de/coderelease2022/getting-started/).


To run the simulator after compiling it(example provided for Ubuntu), you can run the following command from the top level directory of the project:


````
./Build/Linux/SimRobot/Develop/SimRobot
````

## Basic usage instructions

After opening SimRobot, Go to File, then open to select a scene from `Config/Scenes`. A good first scene to try is OneTeamFast.ros2. 

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


# B-Human Code Release README

This is the official 2022 B-Human code release. Documentation can be found in our [public wiki](https://wiki.b-human.de/coderelease2022/).

B-Human is a research project. We provide the software as is. We release it as a snapshot of our internal repository on a yearly basis. You can build on this software, but you have to make do with the documentation we released. You may report bugs or problems with newer releases of operating systems or the tools used by opening a GitHub issue. We might give hints on how to solve these problems if we also ran into them since the code release. Do not write emails to the team or any of its members directly. They will not be answered.

Before cloning this repository, you must read our [license terms](License.md) and the [instructions in our wiki](https://wiki.b-human.de/coderelease2022/getting-started/).

Previous code releases are tagged with "coderelease&lt;year&gt;", where &lt;year&gt; is the year in which the code was released (starting with 2013).
