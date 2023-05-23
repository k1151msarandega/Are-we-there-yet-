# Are-we-there-yet-?
Usage

The program must be capable of utilizing a command line argument to specify the input file and output file.

./project2 inputFile outputFile
The program ensures the user has correctly provided the required command-line arguments and display the above usage statement if the provided arguments are incorrect.

Requirements Summary

Create a program that will simulate the behavior of a car, based on input files that describe its behavior. The input file will include the initial position and orientation of a vehicle, and a set of commands to make it move. The output of the program should be a time-series that describes the position and orientation of the vehicle, and records the command given at each time as well..

Data description and motivation
The data in the input file describes characteristics of the vehicle and its initial state. The end of the file contains an array of input values that describe the velocity and steering angle rate of change for the vehicle.

The goal of the project is to simulate a car-like robot's motion. A simple algorithm is applied to simulate the state evolution of the vehicle, based on kinematic motion.

Classes
My project uses at least the below classes, for which required features are included in their template definitions.

MathVector
Input (note: inherits from MathVector)
State: (note: inherits from MathVector)
Vehicle
The additional classes below are recommended, and files that match their names are part of the submission framework:

Project2: The main function should call only Project2 methods, except for checking arguments. Th Project2 class should provide methods to read in the file, create the Director, and the Vehicle, simulate it, and call the Director to get the necessary output information. If there are problems with the file, the Director, Vehicle, and simulation may not be created or take place.
Director: Utilizes the Vehicle and a vector of Input values to simulate the Vehicle's path. Once the simulation is finished, the Project2 can query a Director to get the necessary information to produce the output file.
The following details are valid for your implementations:

You are free to provide inline implementations if you choose
You are free to add more features to prototyped classes
You are free to edit header files to include other headers commonly available for C++ (or to change the way in which your headers are included)
You are free to define other classes; if you do, you should define them in the Project2 header file, and ensure their implementations are in the Project2 cpp file.
Additional descriptions for the required classes are below:

MathVector
This class should abstract a one dimensional math vector of double values, with the number of elements of the vector set on construction. It should provide a constructor that permits creating the vector from a string of comma separated numerical values. It should also provide a function to print those values (again, with comma separation) with fixed precision 3.

When reading in values into the vector, the return value should be true only if all values are successfully read in. The below are example failures that should be written to the standard out if reading in a value fails. Note that the index is listed (when failing), and the number of elements is listed at the end; i.e., element 3 is the 4th element of the input string.

If the object is has two elements, but a line has 3 values:

Unable to read line [1.0,0.2,-1] (more than 2 elements)
If a non-numeric value is passed in:

Unable to read element 3 of [0.1,0.0,0.0,0a.0] (expected 4 elements)
If not enough elements are provided:

Unable to read element 2 of [0.1,0.0,] (expected 4 elements)
or

Unable to read element 3 of [0.1,0.0,0.0,] (expected 4 elements)
or

Unable to read element 2 of [0.1,0.0,0.0] (expected 4 elements)
Note: the printout of the reading in error has brackets around the vector, but these brackets are only valid for printing the error messages.

Input
This class should inherit with public access to the MathVector class. It should provide methods to get and set values of v (input velocity) and deltaDot. Required method signatures are included in the prototype header provided, and other methods or features may be added at your discretion.

State
This class should inherit with public access to the MathVector class. It should provide methods to get and set values of x, y, delta, and theta. Required method signatures are included in the prototype header provided, and other methods or features may be added at your discretion.

Vehicle
This class simulates the behavior of a car-like robot. The Vehicle should maintain its own State, and when the update(.) function is called it returns a pointer to its state. It is the responsibility of the calling function to make a copy, if one is needed, of the State return value. When deleted, the Vehicle should ensure that any allocated memory is destroyed.

The equations of motion for the car-like robot are below:

x1(t+Dt) = x1(t) + Dt u1(t) cos(x3(t)) cos(x4(t))
x2(t+Dt) = x2(t) + Dt u1(t) cos(x3(t)) sin(x4(t))
x3(t+Dt) = x3(t) + Dt u2(t)
x4(t+Dt) = x4(t) + Dt u1(t) (1/L) sin(x3(t))
Where x1 is x position, x2 is y position, x3 is tire angle delta (in radians) and x4 is heading theta (in radians). L is the Vehicle's wheelbase. The variable u1 is the input velocity, and u2 is the tire angle rate of change.

For the entire simulation, assume a constant value of Delta t (Dt) as set in the inputFile parameters. The value for x3 (tire angle) must always be between [-0.5236,0.5236] radians. If a value is commanded outside this range, then x3 will be saturated using the above range. Likewise, heading should always be between [0, 2π)---it is up to you to determine how to wrap values of heading. Defined values are present for these ranges inside of State.h

Input File
An example input file is included below:

Wheelbase: 2.6
InitialPose: 1,0,0,0
Dt: 0.100
1,0
1,0
1,0
1,0
1,0
The format is given below, with variable types substituting for values. The Wheelbase, Initial Pose, and Dt should be able to be provided in any order, but must be read in prior to reading the Input vectors.

Wheelbase: double
InitialPose: double,double,double,double
Dt: double
double,double
double,double
...
These correspond to:

L (wheelbase in the Vehicle)
State0 (the initial state of the Vehicle)
Dt (the timestamp to use in the simulation)
1 or more lines of inputs
...
There are many ways to fail, so you should NOT simulate the Vehicle's path if any of these occur, and instead you should produce an output file that is empty.

Failure when reading the wheelbase, or Dt
Failure to read the Initial pose
Any incorrect line for an Input value
If the failure is in reading a line for State or Input, you should echo the error as per the above requirements; otherwise, you should not provide any output and instead you should just produce an empty file. If you read a value that does not match either wheelbase, Initial pose, or Dt as one of the first 3 lines, you should produce an empty file without any error messages.

Output File
The output file format should be of comma separated values with precision 3. The values for each timestep (in order) are: t, x, y, theta, delta, v, deltadot. In short, there are

time, state, input
time, state, input
time, state, input
time, state, input
...
time, state, input
time, state
For the last line, only state and time will be available. NO additional commas or zeros should be added for the final inputs (i.e., the state at time k+Dt is a function of x(k) and u(k), so there is no k+Dt input unless provided). An example output is given below:

0.000,1.000,0.000,0.000,0.000,1.000,0.000
0.100,1.100,0.000,0.000,0.000,1.000,0.000
0.200,1.200,0.000,0.000,0.000,1.000,0.000
0.300,1.300,0.000,0.000,0.000,1.000,0.000
0.400,1.400,0.000,0.000,0.000,1.000,0.000
0.500,1.500,0.000,0.000,0.000
Validation through visualization
You can validate your efforts through visualizing the outputs according to a MATLAB script posted to the resources of the PIazza webpage (included below as well). You need not do this validation, but it could provide interest to you as you contemplate providing your own inputs/outputs, and then validating whether the vehicle behaves as you expect it should.

% Author: Jonathan Sprinkle
% License: BSD
% Exempt from restrictions on copying per Academic Integrity
% Description: Should read in and plot the results of a test output from
% project2

function [inputs,outputs] = project2(filename)

if nargin < 1
%     filename='jms0.txt';
    error('Usage: project2(''filename'')');
end

data = csvread(filename);

% indices in the matrix for each variable
t=1; x=2; y=3; delta=4; theta=5; v=6; deltaDot=7;

ts = data(:,t);
xs = data(:,x);
ys = data(:,y);
deltas = data(:,delta);
thetas = data(:,theta);
vs = data(:,v);
deltaDots = data(:,deltaDot);

stateData = {ts, xs, ys, deltas, thetas};

animateCar(stateData);


end

% Author: Jonathan Sprinkle
% Copyright (c) 2008-2015, Arizona Board of Regents
% Reads in a csv formatted .txt file with format
% time,x1,x2,x3,x4
% where
%  time = time in s
%  x1   = x position (xpos) in meters
%  x2   = y position (ypos) in meters
%  x3   = tire angle (tireangle) in radians
%  x4   = heading (heading) in radians
% 
function data = animateCar( stateData )

data = stateData;

% put each cell into data that we can use
time = data{1};
xpos = data{2};
ypos = data{3};
tireangle = data{4};
heading = data{5};

% Extra stuff added by JMS to help visualize a vehicle...
% if running for a non-standard vehicle length, adjust L for visualization
L = 2.6187;
xtire = [ 0.4 0.4 -0.4 -0.4 ];
ytire = [ 0.15 -0.15 -0.15 0.15 ];

x = [ (L)+0.3 (L)+0.3 -(max(xtire)/2+0.4) -(max(xtire)/2+0.4) ];
y = [.75 -.75 -.75 .75];

xlrtire = xtire;
ylrtire = ytire + y(3);
xrrtire = xtire;
yrrtire = ytire + y(4);

% xlftire = xtire - max(xtire);
% ylftire = ytire + y(1);
% xrftire = xtire - max(xtire);
% yrftire = ytire + y(2);
figure
skip = 1;
% if moving too fast, comment out the two lines below
skip = 1/((max(time)-min(time))/length(time));
skip = ceil(skip/10); % show at tenth of a second intervals
for i=1:skip:length(xpos)
    plotPatch( time,xpos,ypos,tireangle,heading, i)
end

function plotPatch( time,xpos,ypos,tireangle,heading,i )

  clf
  hold on
  grid on
  x_tr = xpos(i) + cos(heading(i))*x + sin(heading(i))*y;
y_tr = ypos(i) + sin(heading(i))*x - cos(heading(i))*y;

% do the rear tires
  lrtire_x = xpos(i) + cos(heading(i))*xlrtire + sin(heading(i))*ylrtire;
lrtire_y = ypos(i) + sin(heading(i))*xlrtire - cos(heading(i))*ylrtire;
rrtire_x = xpos(i) + cos(heading(i))*xrrtire + sin(heading(i))*yrrtire;
rrtire_y = ypos(i) + sin(heading(i))*xrrtire - cos(heading(i))*yrrtire;

% rotate front tires by tire angle
xlftire = cos(-tireangle(i))*xtire + sin(-tireangle(i))*ytire;
ylftire = sin(-tireangle(i))*xtire - cos(-tireangle(i))*ytire;
xrftire = cos(-tireangle(i))*xtire + sin(-tireangle(i))*ytire;
yrftire = sin(-tireangle(i))*xtire - cos(-tireangle(i))*ytire;

% perform translation
xlftire = xlftire + L - max(xtire)/2;
ylftire = ylftire + y(1);
xrftire = xrftire + L - max(xtire)/2;
yrftire = yrftire + y(2);

% perform rigid body translation/rotation
lftire_x = xpos(i) + cos(heading(i))*(xlftire) + sin(heading(i))*ylftire;
lftire_y = ypos(i) + sin(heading(i))*(xlftire) - cos(heading(i))*ylftire;
rftire_x = xpos(i) + cos(heading(i))*(xrftire) + sin(heading(i))*yrftire;
rftire_y = ypos(i) + sin(heading(i))*(xrftire) - cos(heading(i))*yrftire;

patch(lrtire_x,lrtire_y,'k')
patch(rrtire_x,rrtire_y,'k')
patch(lftire_x,lftire_y,'k')
patch(rftire_x,rftire_y,'k')
patch(x_tr,y_tr,'w')
plot(xpos(1:i),ypos(1:i));
axis equal

% how much time should we wait before advancing to the next plot point?
if( i > 1 )
    timediff = time(i)-time(i-1);
    pause(timediff)
else
pause(0.1)
end
end

end
