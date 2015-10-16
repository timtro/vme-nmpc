#vme-nmpc

Author : Timothy A.V. Teatro <timothy.teatro@uoit.ca>
Date   : April 2013

The Non-linear model predictive control path optimizer and tracker for
Crosswing's VirtualME platform.

Documentation to be added later. For now, I'm working on having the
code self-documented. I will also be adding a paper that I have
written which explains most of the low-level mathematics.

##Building

Configuration is done with cmake. Reccomended build commands for
out-of-source building:

    $ cd build
    $ cmake ../
    $ make

### Testing

Unit tests are provided for key modules and will be bult by default. run

    $ make test

to automatically run all tests. Users adding obstacle, model or minimizer types are encouraged to write a test module in ./test .

##Usage

##Dependencies

List of dependencies other than standard libraries:
 * Boost


##Code Format
Code formatting was done with `clang-format -style=LLVM`, and an attempt is made to conform to the [LLVM Coding Standards](http://llvm.org/docs/CodingStandards.html).

##TODO
