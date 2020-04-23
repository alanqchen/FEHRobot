Main File
=========

.. warning::
    Looking for the raw code or download? `See the GitHub repository <https://github.com/alanqchen/FEHRobot>`_.

The robot's complete code is under one file, main.cpp. This is done so as to simplify development as
instances the given Proteus libraries could not be easily reused in multiple classes.

Representations
---------------
Alogrithm:

.. image:: /images/algorithm.png 
    :alt: Algorithm

Flowchart:

.. image:: /images/flowchart.png 
    :alt: Flowchart

.. _global-vars:

Global Variables & Constants
----------------------------

.. note::
    This does not include the motor, servo, and sensor definitions.


+---------------+----------------------------+-------------------------------------------------------------------------------------------------------------------+
| Type          | Name                       | Description                                                                                                       |
+===============+============================+===================================================================================================================+
| Constant      | ``MAX_RPM``                | The maximum RPM for IGWAN motors, used in :doc:`/functions/func_setRadSToPercent`                                 |
+---------------+----------------------------+-------------------------------------------------------------------------------------------------------------------+
| Constant      | ``ENCODER_RES``            | The encoder resolution, used in :doc:`/functions/func_countsToRadDisp`                                            |
+---------------+----------------------------+-------------------------------------------------------------------------------------------------------------------+
| Constant      | ``DELTA_T``                | The time between iterations in the PI control loop                                                                |
+---------------+----------------------------+-------------------------------------------------------------------------------------------------------------------+
| Constant      | ``COUNTS_PER_INCH``        | The number of encoder counts per inch, used in all forward functions                                              |
+---------------+----------------------------+-------------------------------------------------------------------------------------------------------------------+
| Constant      | ``RPS_HEAD_CORR_SPEED``    | The default correction percent speed for :doc:`/functions/func_correctHeading`                                    |
+---------------+----------------------------+-------------------------------------------------------------------------------------------------------------------+
| ``float``     | ``RPS_RAMP_START_X``       | Stores the X coordinate for the start of the ramp waypoint, set by :doc:`/functions/func_setupRPS`                |
+---------------+----------------------------+-------------------------------------------------------------------------------------------------------------------+
| ``float``     | ``RPS_RAMP_START_Y``       | Stores the Y coordinate for the start of the ramp waypoint, set by :doc:`/functions/func_setupRPS`                |
+---------------+----------------------------+-------------------------------------------------------------------------------------------------------------------+
| ``float``     | ``RPS_RAMP_START_HEADING`` | Stores the heading for the start of the ramp waypoint, set by :doc:`/functions/func_setupRPS`                     |
+---------------+----------------------------+-------------------------------------------------------------------------------------------------------------------+
| ``float``     | ``RPS_LEVERS_X``           | Stores the X coordinate for levers waypoint, set by :doc:`/functions/func_setupRPS`                               |
+---------------+----------------------------+-------------------------------------------------------------------------------------------------------------------+
| ``float``     | ``RPS_LEVERS_Y``           | Stores the Y coordinate for levers waypoint, set by :doc:`/functions/func_setupRPS`                               |
+---------------+----------------------------+-------------------------------------------------------------------------------------------------------------------+
| ``float``     | ``RPS_LEVERS_HEADING``     | Stores the heading for levers waypoint, set by :doc:`/functions/func_setupRPS`                                    |
+---------------+----------------------------+-------------------------------------------------------------------------------------------------------------------+
| ``float``     | ``RPS_BURGER_X``           | Stores the X coordinate for the burger flip waypoint, set by :doc:`/functions/func_setupRPS`                      |
+---------------+----------------------------+-------------------------------------------------------------------------------------------------------------------+
| ``float``     | ``RPS_BURGER_Y``           | Stores the Y coordinate for the burger flip waypoint, set by :doc:`/functions/func_setupRPS`                      |
+---------------+----------------------------+-------------------------------------------------------------------------------------------------------------------+
| ``float``     | ``FORWARD_TIME_OUT``       | The default timeout length for the corrections, in case the motors do not move                                    |
+---------------+----------------------------+-------------------------------------------------------------------------------------------------------------------+
| ``float``     | ``errorCurr1``             | Motor 1's current error from the :doc:`/functions/func_PIMoveTo`, used by :doc:`/functions/func_setRadSToPercent` |
+---------------+----------------------------+-------------------------------------------------------------------------------------------------------------------+
| ``float``     | ``errorCurr2``             | Motor 2's current error from the :doc:`/functions/func_PIMoveTo`, used by :doc:`/functions/func_setRadSToPercent` |
+---------------+----------------------------+-------------------------------------------------------------------------------------------------------------------+
| ``float``     | ``errorCurr3``             | Motor 3's current error from the :doc:`/functions/func_PIMoveTo`, used by :doc:`/functions/func_setRadSToPercent` |
+---------------+----------------------------+-------------------------------------------------------------------------------------------------------------------+
