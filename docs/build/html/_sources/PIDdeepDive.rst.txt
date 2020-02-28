PID Deep Dive
=============

.. note::
    Looking for the function documentation? See :doc:`functions/func_PIDMoveTo`.

In this section the process for developing the PID controller is detailed.
This includes deriving the kinematic relationships and creating trajectory
profiles.

Motivation
----------
A PID controller was choosen to be implemented due to...

Derivation
==========

.. note::
   The derivation presented is a summary of the work by T.A. Baede, Motion control of an
   omnidirectional mobile robot (2006).

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

:math:`v_i = v_{trans, i} + v_{rot}\tag{1}\label{eq:1}`

Looking at :math:`v_{trans, i}`, we can map the vector onto :math:`\dot{x}` and
:math:`\dot{y}` leading to the general equation for translation at each wheel:


:math:`v_{trans, i} = -\sin(\theta+a_i)\dot{x}+\cos(\theta+a_i)\dot{y}\tag{2}`

Now consider the case were the robot only performs a rotation. Each wheel's
translational velocity, :math:`v_{rot}`, will have to be the following:

:math:`v_{rot}=R\dot{\theta}\tag{3}`

where :math:`R` is the radius from the center of the robot to the wheels. Note
that each wheel will have the same equation as :math:`R` is the same for all
wheels. The value of :math:`R` for our robot was measured to be 0.0869 meters.

Substituting the equations for :math:`v_{trans, i}` and :math:`v_{rot}` into
the equation :math:`\eqref{eq:1}`, the following equation is created:

:math:`v_i=-\sin(\theta+a_i)\dot{x}+\cos(\theta+a_i)+R\dot{\theta}\tag{4}\label{eq:4}`

This equation transforms the robot's velocities into the translational
velocity of each wheel. However, we can also relate the robot's velocities
into the angular velocity of each wheel as translational velocities are
directly related to angular velocities:

:math:`v_i=r\dot{\phi_i}\tag{5}\label{eq:5}`

where :math:`r` is the radius of the wheels, measured to be 0.027 meters.

Substituting :math:`\eqref{eq:5}` into :math:`\eqref{eq:4}` and rearranging to
solve for :math:`\dot{\phi}`, we get:

:math:`\dot{\phi}_i=\frac{1}{r}(-\sin(\theta+a_i)\dot{x}+\cos(\theta+a_i)+R\dot{\theta})\tag{6}\label{eq:6}`

Converting :math:`\eqref{eq:6}` from global coordinates to local coordinates is
out-of-scope of the documentation (see Baede's paper). However, the end result
is the following equation for each wheel's angular velocity given the robot's
velocities and steering in the local frame, defined as :math:`[x_L, y_L]`:

:math:`\dot{\phi}_i=(-\sin(\theta+a_i)\cos(\theta)\dot{x}_L+\cos(\theta+a_i)\cos(\theta)\dot{y}_L+R\dot{\theta})/r\tag{7}\label{eq:7}`

Control Laws
------------
When controlling a robot, we will have an idea of the movement it should
perform, called the reference movement. However, simply using
:math:`\eqref{eq:7}` to find the angular velocities of each wheel
and setting each motor to the respective percent values will not
suffice. This is because of effects such as friction and motor powers not
being equivalent or consistent. Thus, it is neccessary to produce control
laws to use for a feedback loop.

We can use :math:`\eqref{eq:7}` to produce reference angular velocities
of each wheel, :math:`\dot{\phi}_{ref,i}`, and through integration can also
create reference angular positions of each wheel, :math:`\phi_{ref,i}`.

It was choosen to use a position controller as converting from encoder
counts directly to angular postion is more likely to be more accurate
rather than having to estimate an average velocity during program execution.

-----

Control actions is calculated based on the difference between reference
angular positions and actual angular positions:

:math:`e=\phi_{ref,i}-\phi_i\tag{8}`

:math:`e` is the tracking error. Note that the units for :math:`e`,
:math:`\phi_{ref,i}`, and :math:`\phi_i` are radians.

For use in a program, the discrete form of the PID control law is
implemented:

:math:`u_k=K_pe_k+K_i\Delta T\sum_{j=1}^k e_j+\frac{K_d}
{\Delta T}(e_k-e_{k-1})\tag{9}`

+------------------+--------------------------+
| :math:`u`        | Control signal           |
+------------------+--------------------------+
| :math:`k`        | Control loop iteration   |
+------------------+--------------------------+
| :math:`K_p`      | Proportional constant    |
+------------------+--------------------------+
| :math:`K_i`      | Integral constant        |
+------------------+--------------------------+
| :math:`K_d`      | Derivative constant      |
+------------------+--------------------------+
| :math:`e`        | Tracking error           |
+------------------+--------------------------+
| :math:`\Delta T` | Time between interations |
+------------------+--------------------------+

Note that for the final control loop, the derivative term was not
implemented as it was found to be not neccessary.

Implementation
==============
With the mathematical concepts defined, the code implementation can
be constructed.

Trajectory profiles
-------------------
To represent the reference angular positions, pre-generated trajectory
profiles are generated for every movement. This is because the RPS has
a large delay that negatively impacts the accuracy of the error,
especially as the robot moves. To solve this, trajectory profiles are
generated in matlab, using the code shown in :doc:`mat_trajGen`. Then
the Proteus will read the profile file in the SD card, and store the
reference values in an 2D array, which will be compared to in real time. 
Each trajectory profile outputted has the same format:

.. code-block::
       :linenos:

        0.000000	0.000000	0.000000	0.000000	0.000000	0.000000
        0.051794	0.051794	0.103589	-1.035885	-1.035885	2.071770
        0.192379	0.192379	0.384757	-1.775803	-1.775803	3.551606
        0.392157	0.392157	0.784313	-2.219754	-2.219754	4.439508
        0.621531	0.621531	1.243062	-2.367738	-2.367738	4.735475

From left to right, the columns are wheel 1's, wheel 2's, and wheel 3's
refererence total angular displacement, and wheel 1's, wheel 2's, and wheel 3's
refererence angular velocity. Although we only use total angular displacement
to determine error, reference angular velocity is used to help decide which
direction to wheels should spin in response to error as encoder counts can only
increase.

PID function
------------
To be filled.
