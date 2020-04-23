getCdsColor
===========

.. doxygenfunction:: getCdsColor
   :project: FEHRobot

Examples
-------------------

.. code-block:: c++
   :linenos:

   while(getCdsColor(true) == 0);

Waits until the a light is detected, using the start light threshold.

----

.. code-block:: c++
   :linenos:

   int cdsValue = getCdsColor(false);

Gets the light color (or no light), using the jukebox light threshold.

Function Variables
------------------

No function variables are used.

