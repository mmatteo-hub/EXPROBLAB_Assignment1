# EXPROBLAB_Assignment1

## Introduction
The assignment is the realization of an algorithm to make the robot move in an environment and have a surveillance behaviour to choose the way to move. The invironment is composed of locations that are connected in a certain way depending on the choice of the user.
The goal is to build a Finite State Machine (FSM) that allows the robot to choose the behaviour to adopt depending on the situation. The robot is also provided a battery, periodically checked, that needs to be recharged after some time of using. <br>
The locations are divided into:
* room: location with one door;
* corridor: location with at least two door.

The entities that connect two locations are called _doors_ and the entity that moves in the environment is the robot, called _Robot1_.

## Software Architecture
The main software, the one of the Finite State Machine is composed of four states:
* _Init State_;
* _Reasoner State_;
* _Move Random State_;
* _Recharge State_.

This structure can be seen by the following images:

<img 
    src="/images/less_depth.jpg"
    title="Less Depth in the FSM"
    width="50%" height="50%"><img 
    src="/images/more_depth.jpg" 
    title="More Depth in the FSM"
    width="50%" height="50%">

However, the _Move Random State_ represents a sub finite state machine, which means that it is composed in turn of other states, in particular:
* Plan Path To Location State;
* Go To Location To Visit State.

The graph above are taken from the automatic SMACH viewer and they could be a little bit confusing. In order to evaluate the entire graph correctly and see it clearly we provided one by drawing it and highlighting the transition in a better way. This graph can be seen in the following image:

<img
    src="/images/FSM.jpg"
    title="FSM"
    width="60%" height="60%">

### Software components
It follows the details of the software components used in the program, which are available in the `scripts` folder.

#### The `planner` Node
<img
	src="images/planner.jpg"
	title="planner node"
	width="50%" height="50%">
	
The `planner` node implements an action server called `motion/planner`. This is done by the means of the `SimpleActionServer` class based on the `Plan` action message. This action server requires a `start` and a `target` position passed as two fields of the goal. <br>
Given the goal parameters this component return a plan as a list of `via_points`, which are computed by spacing linearly the distance between the two _x_ and _y_ coordinates of the points. The number of `via_points` can be modified thanks to the parameter in the [`name_mapper.py`](utilities/EXPROBLAB_Assignment1/name_mapper.py) file. <br>
When a new `via_points` is generated, the updated plan is provided as `feedback`. When all the `via_points` have been generated, the plan is provided as `results``.

#### The `controller` Node
<img
	src="images/controller.jpg"
	title="controller node"
	width="50%" height="50%">
	
The `controller` node implements an action server named `motion/controller`. This is done by the means of the `SimpleActionServer` class based on the `Control` action message. This action server requires the `plan` given as a list of `via_points` by the planner.<br>
Given the plan the `controller` iterates for each planned `via_points` and waits to simulate the time spent to move the robot. <br>
Each time a `via_point` is reached the a `feedback` is provided. When the last `via_point` is reached, the action service provides a `result` by propagating the current robot position.

The program starts in the _Init State_ which initializes the ontology (the environmemt). Then this state is no longer executed. The program passes to the _Reasoner State_ which reasons the changes:
* the actual robot position;
* the timestamps representing the last time a location has been visited by the robot.

The FSM enter now the sub-FSM, which first computes a path from the actual location of the robot to a reachable location and then moves the robot to that location. <br>
The loop of _Reasoner_ - _Plan Path_ - _Go To Location_ is repeated in an infinite loop until the robot gets a stimulus:
* _battery low_: which means that the robot needs to be recharged. To do so, the robot is firstly driven to the dedicated location and then is recharged. At the end it starts again the loop;
* _urgent location_: after some time a location is not visited by the robot it becomes urgent and if the robot can reach it by its actual position it has to go there.
<br>

The code is also composed of an _Helper_ which is a class containing all the shared variables and the mutex to allow the FSM states access them correctly.

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
After that, it plans again and starts moving randomly again.

#### Surveillance Policy adopted
When the robot’s battery is not low, it should move among locations with this policy: 
* it has to stay mainly in the corridors: _E_, _C1_ and _C2_ in this case;
* if a reachable location has not been visited for some times, which means that it has become _urgent_, then is has to go there immediately.

In the video the urgent location is the _X_ one. The robot then move to the corridor again since it is the only location reachable from that room in this particular ontology. <br>
The FSM is then brought to execute the _Recharge_ state, so the robot is firstly driven to this location, _E_ and then it is recharged (as it can be read from the comments in the video). <br>
Just as a video demonstration the battery is set to need a recharge after 15 seconds of execution so that the user can see how the robot behaves in case of battery low. <br>
The normal autonomy of the battery is set to one minute of program execution, after which the robot has to recharge to go on with its surveillance behaviour.

## Working Hypothesis and Environment
### Environment
The ontology that we initialized in this assignment is the following:

<img 
    src="/images/map.jpg" 
    title="Ontology Map"
    width="25%" height="25%">

The environment used and initialized is supposed to be consinstent with the real one, so that the reasoner can alsways find a consinstent ontology to work on. <br>
Moreover, it is also assumed that all corridors, _E_, _C1_ and _C2_ are connected together. In this way the robot is able to perform its _surveillance policy_ correctly. <br>
Also for more difficult environment, so for bigger ontologies it is assumed that all the corridors are connected together and at least with the _recharging location_.

### Robot
The robot starts in the _Recharging Room_ which is in this case the _E_ one. From here it move randomly, which means that it checks the reachable location from its actual one and then chooses randomly among the list of possibilities. <br>
The robot is also assumed to have an autonomy of 60 seconds, after which it needs to be recharged. It is assumed that the robot needs to reach the proper location before being recharged and also this reaching transition is performed randomly at the beginning when the robot cannot reach the recharging location; as soon as it has the possibility to reach it, then the choice is imposed to this one.

### System Features
In order to build the system it was opted to build a robust code. In fact, when the robot has a low battery level its only goal is to reach the _E_ room: as it was more than an _urgent_ location. This means that the robot could move around for a high amount of time if it did not detect the recharing room immediately, but this behaviour would not affect the surveillance problem of the robot.

### System Limitation
One of the possible limitation of the code is the possibility of having the robot moving randomly for a long time before reaching the recharging room when needed which could be tricky from a physical point of view since the robot could not have all that amount of battery left.

### Possible technical improvements
A possible improvement could be the possibility to make the robot remember a path from each location to the recharging room so that in case of low battery it can reach the location without having to choose randomly and thus reducing the time wasting due to the random choice.

## Authors and Contacts
[Matteo Maragliano](https://github.com/mmatteo-hub) (S4636216) <br>
[S4636216@studenti.unige.it](mailto:S4636216@studenti.unige.it)
