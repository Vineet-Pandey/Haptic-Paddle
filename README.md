# Haptic-Paddle
This project holds the code to control the position and velocity of a haptic paddle
Background:
A closed-loop system uses feedback to drive the actual output towards a desired output. By
creating a feedback loop, the paddle system will be able to automatically and accurately track
reference inputs and to react against disturbances or user inputs.

Control Laws for Haptic Effects:
One aspect of controllers for force-feedback (or haptic feedback) that sets them apart from
regular position or velocity feedback controllers is the fact that they need to take into account a
person who will be physically interacting with the device through its handle. Therefore the
controller is not the only "actor" on the device, the human user provides additional inputs
(forces/ torques) to the device.

In speed and position controllers above, any input other than the controller's input to the device
is interpreted as unwanted disturbances, and the controllers are tuned so that these
disturbances are strongly/forcefully rejected. Consider a speed control scenario (which is
common in industry, such as in conveyors): regardless of the amount of materials being carried
on the conveyor belt, it has to maintain a consistent constant speed. Therefore materials being
placed on or taken off the belt are disturbances to the speed controller, and the controller does
its best to prevent these disturbances from changing the speed. Similar scenario can be
imagined for an industrial manipulator under position control: the goal is to have highly accurate
positioning and following of trajectories. Anything that comes in the way of the manipulator will
therefore be pushed away strongly, since the controller focuses solely on following the
reference trajectory. These controllers are well-suited for controlled, factory environments,
where human operators are not allowed to get in contact with these devices or their workspace,
since any physical contact between the human and the device will likely cause injury.

Haptic feedback controllers therefore need to be designed to take into account human user
interaction, and be safe for physical human-robot interaction. Consider a simple haptic effect
that allows the user to feel a virtual spring: this can be implemented using a position controller
with proportional only control, but the controller gain Kp should not be too high. In fact, Kp
becomes the parameter to control the stiffness of the virtual spring, while the reference position
input becomes the equilibrium position of the virtual spring. When the user pushes the handle
away from the equilibrium, it will resist and want to go back to the equilibrium, and the amount of
force pulling it back will be changing linearly with the position error, and be proportional to the
spring stiffness. Another haptic effect is "viscous friction" or "viscous damping". A control law
can be written to resist movements of the handle, and the resistance force can be proportional
to the velocity of handle movement. It is important that a positive damping is implemented (that
always resists motion, rather than amplify it), so that the device responds in a stable manner. A
virtual spring and some viscous damping can be combined to implement a most commonly used
haptic effect: a virtual wall.

Setup:
To use this code to observe the haptic paddle effect, the following equipments are required:
1) Haptic Paddle (with AVR microcontroller)
2) Atmel Studio software
3) AVR programmer


