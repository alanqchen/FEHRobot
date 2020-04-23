Performance Task 3
==================

.. warning::
    Looking for the raw code or download? `See the GitHub repository <https://github.com/alanqchen/FEHRobot>`_.

This is the code for the performance task 3. The goal was to move from the starting position,
go up the ramp, and flip the burger. For the bonus task, the goal was to flip any lever down.

Code
----
.. code-block:: c++
    :linenos:

    // to center of ramp
    PIMoveTo("start1.txt", 31, true);

    //up ramp
    PIMoveTo("mR34.txt", 31, false);

    //east to wall
    forward12(-25, 17);

    //west off wall
    //forward12(25, 1.5);
    forward12(25, .75);

    //rotate towards burger
    rotateCC(25, 22);
    //PIMoveTo("ss.txt", 6, false);

    arm_servo.SetDegree(0);

    //north towards burger
    forward31(25, 8.1);
    // back off from burger
    forward31(-25, .15);
    arm_servo.SetDegree(55);
    Sleep(1.5);
    arm_servo.SetDegree(0);
    Sleep(1.5);

    //south from burger
    forward31(-25, 4);

    //face ice cream
    rotateCC(-25, 23);

    arm_servo.SetDegree(70);

    //move diagonally to lever
    PIMoveTo("toLever.txt", 31, false);

    arm_servo.SetDegree(40);
    Sleep(1.5);
    arm_servo.SetDegree(70);
