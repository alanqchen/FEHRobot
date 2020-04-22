Performance Task 2
==================

.. note::
   This is a work in progress

Code
----
.. code-block:: c++
   :linenos:

   arm_servo.SetDegree(65);
   PIMoveTo("start1.txt", 31, true);
   for(int i = 65; i >= 45; i-=2) {
      arm_servo.SetDegree(i);
      Sleep(10);
   }
   PIMoveTo("mR34.txt", 31, false);

   Sleep(0.5);
   rotateCC(-25, 90);
   Sleep(0.5);
   PIMoveTo("toSink.txt", 31, false);

   Sleep(0.5);

   sinkDump();
   rotateCC(-25, 152);

   Sleep(0.5);

   forward31(25, 24.0);
   forward31(-25, .1);
   rotateCC(-25, 30);

   arm_servo.SetDegree(90.0);
   jukebox_servo.SetDegree(170.0);
   Sleep(0.75);
   forward12(25, 5);
   jukebox_servo.SetDegree(160.0);
   Sleep(0.5);
   PIMoveTo("slideT.txt", 26, false);
   Sleep(0.5);
   forward12(-25, 23);

