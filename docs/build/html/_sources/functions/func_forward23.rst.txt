forward23
==========

.. doxygenfunction:: forward23
   :project: FEHRobot

Examples
-------------------

.. code-block:: c++
   :linenos:

   forward23(25.0, 4.0);

Moves in the forward 4 inches in the direction of motors 2 & 3.

----

.. code-block:: c++
   :linenos:

   forward23(-15.0, 4.0);

Moves in the backwards 4 inches, opposite of the direction of motors 2 & 3.

Function Variables
------------------

+---------------+-------------------+----------------------------------------------------------------------------------+
| Type          | Name              | Description                                                                      |
+===============+===================+==================================================================================+
| ``float``     | ``start``         | The start time of the function, used for timeout if the motors don't move.       |
+---------------+-------------------+----------------------------------------------------------------------------------+
