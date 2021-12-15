# Ch9_MotionForceControl

## Problem Statement

Designing a PID controller for a single-link manipulator by writing a python script to simulate in Coppeliasim.

## Pre-requisites:

- Software: Coppeliasim (for simulation), Spyder (for python coding)
- Basic python syntax
- Understanding of the python remote API commands for coppeliasim (refer: [Remote API Python](https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm))

## PID Controller Solution

Open the "PID_Control_1joint.ttt" file. For this problem, ensure that there is a connection for CoppeilaSim and Spyder to communicate. This is done by adding the following command into the child script

`simRemoteApi.start(19999)`

To open a child script, right-click on any of the joints or links then “Add > Associated child script > Nonthreaded”. This will add a script-like logo next to the selected link or joint. double click that logo, and paste the above code line as the first line of the code without changing anything else.

Before running the code, the simulation in CoppeliaSim must be started. To do this, navigate to the play button at the top of the ribbon and click. The simulation will start, however nothing should happen since the software is waiting for a command from Spyder.

The code begins by extracting the object handle for the joint and its current position. It then propmts for the user to define the proportional, integral, and derivative gains. Once completeded, the script then asks for a target position of the robotic manipulator. An error value is calculated based on the current position of the manipulator and the target position. The script then calculates what the control law based on the gains defined by the user. There are three sections each pertaining to the proportional, integral, and derivative control. An angle is then calculated based on the control and the link's geometry. The script then moves the robotic manipulator to that new angle. A new error value is calculated and a control law is calcualted again. This process repeats until the error value is less than 1%.

After specifying the gains and target position, you can switch back over to CoppeliaSim and watch as the robot manipulator will move to the desired trajectory.

## Inverse Dyanmics Solution

This solution is very similar to the PID controller solution.
Open the "inverseDynamics_Control.ttt" file. For this problem, ensure that there is a connection for CoppeilaSim and Spyder to communicate. This is done by adding the following command into the child script

`simRemoteApi.start(19999)`

To open a child script, right-click on any of the joints or links then “Add > Associated child script > Nonthreaded”. This will add a script-like logo next to the selected link or joint. double click that logo, and paste the above code line as the first line of the code without changing anything else.

Before running the code, the simulation in CoppeliaSim must be started. To do this, navigate to the play button at the top of the ribbon and click. The simulation will start, however nothing should happen since the software is waiting for a command from Spyder.

The code begins by extracting the object handle for the joint and its current position. It then propmts for the user to define the proportional, integral, and derivative gains. Once completeded, the script then asks for a target position of the robotic manipulator. An error value is calculated based on the current position of the manipulator and the target position. The script then calculates what the control law based on the gains defined by the user. There are three sections each pertaining to the proportional, integral, and derivative control. There is an additional section pertaining to the inverse dynamics part - which is based of off the object's properties and geometry. A graviational force  and an acceleration error term are required to be calculated. The control law is then calculated using Equation (9.12) from the chapter. An angle is then calculated based on the control and the link's geometry. The script then moves the robotic manipulator to that new angle. A new error value is calculated and a control law is calcualted again. This process repeats until the error value is less than 1%.

After specifying the gains and target position, you can switch back over to CoppeliaSim and watch as the robot manipulator will move to the desired trajectory.

## PID and Inverse Dynamics Control using CoppeilaSim

To design these controllers in CoppeilaSim, the jointCallBack function must be used. When present in a dynamically abled joint, the physics engine will call upon the user specified control rather than the default control from the joint. Essentually, this function overwrites the defalut and allows for the user to customize the control loop by writing their own algorithms.

In both the "PID_Control_1joint.ttt" and "inverseDynamics_Control.ttt" files, open the child script. Paste the code found in the "PID_Control_CoppeiliaSim" and "inverseDynamics_CoppeliaSim" files, respectfully. You can then specify the control gains by changing the PID_P, PID_I, and PID_D variables. Close out of the script. Then double click the revolute joint and go to the "show dynamic properties dialong". Specify the target position and then run the simulation. The robotic manipulator will then move based on what is in the jointCallBack function.
