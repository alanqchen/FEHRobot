forwardX
================

.. doxygenfunction:: forwardX
   :project: FEHRobot


Examples
-------------------

.. code-block:: c++
   :linenos:

   forwardX(25.0, 4.0);

Moves in the positive local x direction 4 inches, with motor 3 given 25% power.

----

.. code-block:: c++
   :linenos:

   forwardX(-15.0, 4.0);

Moves in the negative local x direction 4 inches, with motor 3 given -15% power.

Function Variables
------------------

+---------------+-------------------+----------------------------------------------------------------------------------+
| Type          | Name              | Description                                                                      |
+===============+===================+==================================================================================+
| ``float``     | ``start``         | The start time of the function, used for timeout if the motors don't move.       |
+---------------+-------------------+----------------------------------------------------------------------------------+

