PIDMoveTo
================

.. doxygenfunction:: PIDMoveTo
   :project: FEHRobot


Working Example
-------------------

.. code-block:: c++
   :linenos:

   int main() {
   }

----

.. _class_AnimatedSprite3D_method_play:

- void **play** **(** :ref:`String<class_String>` anim="" **)**

Plays the animation named ``anim``. If no ``anim`` is provided, the current animation is played.

----

.. _class_AnimatedSprite3D_method_stop:

- void **stop** **(** **)**

Stops the current animation (does not reset the frame counter).

Variables
-----------

+-------------------------+-------------------------------------------------------------------------------------------------+
| :ref:`bool<class_bool>` | :ref:`is_playing<class_AnimatedSprite3D_method_is_playing>` **(** **)** const                   |
+-------------------------+-------------------------------------------------------------------------------------------------+
| void                    | :ref:`play<class_AnimatedSprite3D_method_play>` **(** :ref:`String<class_String>` anim="" **)** |
+-------------------------+-------------------------------------------------------------------------------------------------+
| void                    | :ref:`stop<class_AnimatedSprite3D_method_stop>` **(** **)**                                     |
+-------------------------+-------------------------------------------------------------------------------------------------+



