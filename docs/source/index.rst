.. Team E3's Robot documentation master file, created by
   sphinx-quickstart on Tue Feb 25 01:20:58 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to Team E3's Robot's documentation!
===========================================

.. warning::
    Looking for the raw code or download? `See the GitHub repository <https://github.com/alanqchen/FEHRobot>`_.

This is the code documentation website, offering detailed descriptions of the code.
Here you can find detailed information of the PI controller and each function. Also
the code used for the performance task is shown. For easy navigation, refer to the
side/hamburger bar or the table of contents below.

In Main File, you'll find representations of the overall logic of the code and the
global variables/constants. In PI Deep Dive, you'll find a lengthy description of the
PI controller extensively used by the robot.
In Performance Tests, you'll find the code used to complete each
performance test. In Function Reference you'll find detailed documentation
for each function, including parameters and function variables. And in MATLAB code
you'll find the code used to support the PI controller.

.. warning::
    Again, if you would like to download any of the code, `visit the GitHub repository <https://github.com/alanqchen/FEHRobot>`_
    to ensure you get the latest version.

----

Table of Contents
=================

.. toctree::
   :maxdepth: 2

   Main Documentation <https://u.osu.edu/feh20e3/>

.. toctree::
   :maxdepth: 2

   general/mainFile

.. toctree::
   :maxdepth: 2
   :caption: PI Deep Dive

   pidd/PIdeepDive

.. toctree::
   :maxdepth: 2
   :caption: Performance tasks

   perfTasks/index

.. toctree::
   :maxdepth: 2
   :caption: Function reference

   functions/index

.. toctree::
   :maxdepth: 2
   :caption: MATLAB code

   matlabcode/mat_trajGen
   matlabcode/mat_errPlot

----
.. Indices and tables
.. ------------------
..
.. * :ref:`genindex`
.. * :ref:`modindex`
.. * :ref:`search`
