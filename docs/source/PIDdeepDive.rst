PID Deep Dive
=============

.. note::
    this is a work in progress

In this section the process for developing the PID controller is detailed.
This includes deriving the kinematic relationships and creating trajectory
profiles.

Motivation
----------
A PID controller was choosen to be implemented due to...

Derivation
==========

Essential to any advanced PID controller, the kinematic relationships and
control laws were developed for a tri-omni wheel robot.

Kinematic Relationships
-----------------------

First, the robot's location and heading with respect to the course's frame
is defined as :math:`(x, y, \theta)`. Additionally, the robot's velocity
with respect to the course's frame is defined as :math:`(\dot{x}, \dot{y},
\dot{\theta})`. With respect to the robot's frame, the angles for
wheels 1, 2, and 3 are :math:`a_1=0^\circ`, :math:`a_2=120^\circ`,
and :math:`a_3=240^\circ` respectively.

Next, the translational velocity of each wheel, :math:`v_i` that make up
the robot's velocity can be split into its components:

:math:`v_i = v_{trans, i} + v_{rot}`

Looking at :math:`v_{trans, i}`, we can map the vector onto :math:`\dot{x}` and
:math:`\dot{y}` leading to the general equation for translation at each wheel:


:math:`v_{trans, i} = -\sin(\theta+a_i)\dot{x}+\cos(\theta+a_i)\dot{y}`

Now consider the case were the robot only performs a rotation. Each wheel
velocity, :math:`v_{rot}`, will have to be the following:

:math:`v_{rot}=R\dot{\theta}`

Where :math:`R` is the radius from the center of the robot to the wheels. Note
that each wheel will have the same equation as :math:`R` is the same for all
wheels.

