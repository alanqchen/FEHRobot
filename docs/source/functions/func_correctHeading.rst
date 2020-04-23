correctHeading
==============

.. doxygenfunction:: correctHeading
   :project: FEHRobot

Examples
-------------------

.. code-block:: c++
   :linenos:

   correctHeading(0.0, 25.0);

Corrects the robot's global heading to 0 degrees, setting the motors at 25%.

----

.. code-block:: c++
   :linenos:

   correctHeading(190.0, 14.0);

Corrects the robot's global heading to 190 degrees, setting the motors at 14%.

Function Variables
------------------

+---------------+-------------------+----------------------------------------------------------------------------------+
| Type          | Name              | Description                                                                      |
+===============+===================+==================================================================================+
| ``float``     | ``currHeading``   | Stores the current heading from RPS                                              |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``float``     | ``angle``         | Stores the absolute angle difference between the final and current heading       |
+---------------+-------------------+----------------------------------------------------------------------------------+
