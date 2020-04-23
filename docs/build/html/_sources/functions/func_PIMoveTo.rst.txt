PIMoveTo
================

.. note::
    Looking for a detailed description? See :doc:`/pidd/PIdeepDive`.

.. doxygenfunction:: PIMoveTo
   :project: FEHRobot


Examples
-------------------

.. code-block:: c++
   :linenos:

   PIMoveTo("upRamp.txt", 31, false);

Note that while ``size`` may be any number equal to or larger to the number of lines, it is more effecient for memory
to keep ``size`` equal to the number of lines. Additionally, the given file name must match the format shown in
:ref:`trajectory-profiles`.

----

.. code-block:: c++
   :linenos:

   PIMoveTo("start.txt", 21, true);

In this example, ``preload`` is true. Thus, it will read the file and populate the arrays, but will not start moving
until :doc:`/functions/func_getCdsColor` reports that light is detected.


Function Variables
------------------

+---------------+-------------------+----------------------------------------------------------------------------------+
| Type          | Name              | Description                                                                      |
+===============+===================+==================================================================================+
| ``int``       | ``countNew1``     | The current iteration encoder counts for motor 1                                 |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``int``       | ``countNew2``     | The current iteration encoder counts for motor 2                                 |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``int``       | ``countNew3``     | The current iteration encoder counts for motor 3                                 |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``int``       | ``countOld1``     | The previous iteration encoder counts for motor 1                                |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``int``       | ``countOld2``     | The previous iteration encoder counts for motor 2                                |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``int``       | ``countOld3``     | The previous iteration encoder counts for motor 3                                |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``float``     | ``displacement1`` | The total angular displacement for motor 1 in radians                            |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``float``     | ``displacement2`` | The total angular displacement for motor 2 in radians                            |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``float``     | ``displacement3`` | The total angular displacement for motor 3 in radians                            |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``float``     | ``refPos1``       | Temporary variable to read motor 1's reference angular position from file        |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``float``     | ``refPos2``       | Temporary variable to read motor 2's reference angular position from file        |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``float``     | ``refPos3``       | Temporary variable to read motor 3's reference angular position from file        |
+---------------+-------------------+----------------------------------------------------------------------------------+  
| ``float``     | ``refSpeed1``     | Temporary variable to read motor 1's reference angular velocity from file        |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``float``     | ``refSpeed2``     | Temporary variable to read motor 2's reference angular velocity from file        |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``float``     | ``refSpeed3``     | Temporary variable to read motor 3's reference angular velocity from file        |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``float[][]`` | ``pos_ref``       | Reference angular positions - Columns are each motor, rows are each value        |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``float[][]`` | ``vel_ref``       | Reference angular velocities - Columns are each motor, rows are each value       |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``float``     | ``phi1``          | Stores the actual current total displacement for motor 1 in radians              |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``float``     | ``phi2``          | Stores the actual current total displacement for motor 2 in radians              |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``float``     | ``phi3``          | Stores the actual current total displacement for motor 3 in radians              |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``float``     | ``motorSpeed1``   | Calculated motor 1's speed in rad/s to correct for error from PI controller      |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``float``     | ``motorSpeed2``   | Calculated motor 2's speed in rad/s to correct for error from PI controller      |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``float``     | ``motorSpeed3``   | Calculated motor 3's speed in rad/s to correct for error from PI controller      |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``float``     | ``errorTotal1``   | The total error error for motor 1 used by the integral term                      |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``float``     | ``errorTotal2``   | The total error error for motor 2 used by the integral term                      |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``float``     | ``errorTotal3``   | The total error error for motor 3 used by the integral term                      |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``float``     | ``Kp``            | The proportional constant                                                        |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``float``     | ``Ki``            | The integral constant                                                            |
+---------------+-------------------+----------------------------------------------------------------------------------+
