setRadSToPercent
================

.. doxygenfunction:: setRadSToPercent
   :project: FEHRobot

Example
-------------------

.. code-block:: c++
   :linenos:

   setRadSToPercent(2.0, -1.0, 3.0);

Sets motor 1 to the equivalent percent for 2 rad/s, motor 2 to the equivalent -1.0 rad/s, and motor 3 to the equivalent 3.0 rad/s.

Function Variables
------------------

.. note::
    This functions depends on global variables.

+---------------+-------------------+----------------------------------------------------------------------------------+
| Type          | Name              | Description                                                                      |
+===============+===================+==================================================================================+
| ``float``     | ``percent1``      | The raw calculated percent for motor 1                                           |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``float``     | ``percent2``      | The raw calculated percent for motor 2                                           |
+---------------+-------------------+----------------------------------------------------------------------------------+
| ``float``     | ``percent3``      | The raw calculated percent for motor 3                                           |
+---------------+-------------------+----------------------------------------------------------------------------------+
