# EXPROBLAB_Assignment1

## Introduction
The assignment is the realization of an algorithm to make the robot move in an environment and have a surveillance behaviour inside it. The invironment is composed of locations that are connected in a certain way depending on the choice of the user.
The goal is to build a Finite State Machine (FSM) that allows the robot to choose the behaviour to adopt depending on the situation. The robot is also provided a battery, periodically checked, that needs to be recharged after some time of using. <br>
The locations are divided into:
* room: location with one door;
* corridor: location with at least two door.

The entities that connect two locations are called _doors_ and the entity that moves in the environment is the robot, called _Robot1_.

## Software Architecture

## Installation and Running Procedure
### Run by `roslaunch` file
In order to run the program by the `roslaunch` file we first need to install the `xterm` tool by following these steps:
```bash
sudo apt-get update
sudo apt-get -y install xterm
```

Now to run the code we just type these on a terminal:
```bash
roslaunch EXPROBLAB_Assignment1 assignment.launch
```

### Run each single node manually
In order to run the code without using the `roslaunch`:
```bash
roscore &

# Terminal A: aRMOR Server
rosrun armor execute it.emarolab.armor.ARMORMainService

# Terminal B: Planner Server
rosrun EXPROBLAB_Assignment1 planner.py

# Terminal C: Controller Server
rosrun EXPROBLAB_Assignment1 controller.py

# Terminal D: FSM node
rosrun EXPROBLAB_Assignment1 fsm.py
```

## Video
The video below represents a part of the program during its normal execution.
As the video starts there are four sections in the window:
* top-left: it is the aRMOR Server window and it is used to make the query and communicate with the FSM program;
* top-right: it is the FSM node. Here the program shows its log messages and it is the one that has to be read to better understand the program execution. <br>
It shows us which state the FSM is into, what it is doing into ithe state and the changing action that provides the pass from one state to another;
* bottom-left: it is the planner server that computes the path from the start postion to the target one and publish the result;
* bottom-right: it is the controller server that simualtes the movement of the robot by changing its position through the points of the path.

The video begins with the starting of all the points, then the FSM enters in the _PlanPathToPostion_ so the planner publishes the result. Then it goes in the _GoToLocationToVisit_ state and controller server moves the robot. In the new location the robot waits for some instants, defined as a parameter, and then plan the reason the changes in the ontology: where it is and which lcoation it can now reach. <br>
After that, it plans again and starts moving randomly again. <br>
The robot moves among the corridors of the environment until an _urgent_ location is detected. At this point the robot visits it: in the video the urgent location is the R4 one. The robot then move to the corridor again since it is the only location reachable from that room in that particular ontology. <br>
The FSM is then brought to execute the _Recharge_ state, so the robot is firstly driven to this location and then it is recharged (as it can be read from the comments in the video). <br>
Just as a video demonstration the battery is set to need to be recharged after 15 seconds of execution so that the user can see how the robot behaves in case of battery low. <br>
The normal autonomy of the battery is set to one minute of program execution, after which the robot has to recharge to go on with its surveillance behaviour.

## Working Hypothesis and Environment

## Authors and Contacts
[Matteo Maragliano](https://github.com/mmatteo-hub) (S4636216) <br>
[S4636216@studenti.unige.it](mailto:S4636216@studenti.unige.it)
