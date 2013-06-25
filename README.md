vme-nmpc
========

Author : Timothy A.V. Teatro <timothy.teatro@uoit.ca>  
Date   : April 2013  

The Non-linear model predictive control path optimizer and tracker for
Crosswing's VirtualME platform.

Documentation to be added later. For now, I'm working on having the
code self-documented. I will also be adding a paper that I have
written which explains most of the low-level mathematics.

Building
--------

Configuration is done with cmake. Reccomended build commands for
out-of-source building:

    $ cd build  
    $ cmake ../  
    $ make

Dependencies
------------
List of dependencies other than standard libraries:
 * None.

TODO
====

 * Rephrase the mathematics for the VirtualME platform. This means
   that I will modify the state and control vectors, the path
   predictor and the Lagrangian/Hamiltonian stuff.

 * Work on wighting the acceleratin as a function of speed. The robot
   will be able to change direction more quickly at lower speeds.

 * Create a command-line flag parser and --help/--usage documentation.
   Create flag -c for crippling the simulation for human display

 * Build in a timer to see how quickly the execution is to determine
   suitability for real time execution.

 * Create XML input initializer so that rapid testing can be done
   without recompiling the source.
