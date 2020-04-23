Performance Task 1
==================

.. warning::
    Looking for the raw code or download? `See the GitHub repository <https://github.com/alanqchen/FEHRobot>`_.

This is the code for performance task 1. The goal was to move from the starting position to the
jukebox light and then press the correct button color. The bonus task was to move up the ramp
and back down. Note that the PI controller was not ready at the time, so basic movement using
encoder counts was used.


Code
----
.. code-block:: c++
    :linenos:

    moveForward(25, 18);
    allStop();
    // Read Cds Cell
    int cdsValue = getCdsColor();
    Sleep(1.5);
    // If blue
    if(cdsValue == 1)  {
        rotateCC(25, 106);
        Sleep(1.5);
        moveForward(25, 4);
    // If red
    } else if (cdsValue == 2) {
        moveForward(25, 3);
        Sleep(1.5);
        rotateCC(25, 107);
        Sleep(1.5);
        moveForward(25, 5.25);
    }
    Sleep(1.5);
    // Move backward from jukebox
    moveForward(-25, 6);
    jukebox_servo.SetDegree(5.0);
    Sleep(0.5);
    // Rotate to align heading to ramp starting position
    rotateCC(25, 90);

    Sleep(0.5);
    // Move forward to ramp starting position
    if(cdsValue == 1) {
        moveForward(25, 6);
    } else if(cdsValue == 2) {
        moveForward(25, 10);
    }
    Sleep(0.5);
    // Rotate towards ramp
    rotateCC(25, 90);
    Sleep(0.5);
    // Go up ramp
    moveForward(75, 30);
    Sleep(0.5);
    moveForward(-25, 30);
    Sleep(0.5);/
