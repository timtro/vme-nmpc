# vme-nmpc

Author : Timothy A.V. Teatro <timothy.teatro@uoit.ca>
Date   : April 2013

The Non-linear model predictive control path optimizer and tracker for
Crosswing's VirtualME platform.

Documentation to be added later. For now, I'm working on having the
code self-documented. I will also be adding a paper that I have
written which explains most of the low-level mathematics.

## Building

Configuration is done with cmake. Reccomended build commands for
out-of-source building:

    $ cd build
    $ cmake ../
    $ make

### Testing

Unit tests are provided for key modules and will be bult by default. run

    $ make test

to automatically run all tests. Users adding obstacle, model or minimizer types are encouraged to write a test module in ./test .

## Usage

### Reading and Using the Code

Files and objects prefixed with VMe or vMe are specific to the VirtualMe Robot. For example, the NmpcModel class is an interface (that is, an abstract class wherein all methods are virtual) that is quite generic to the NMPC problem. However, VMeNmpcModel is a class that derives from the interface and contains data and methods specific to the virtualME robot. If you port this code to another type of device, files and classes prefixed with VMe are the files that must be rewritten.

Classes begin with Calital Letters, and appear in CammelCase. Source files defining those classes are named after the class they contain. Classes which exist only to support a more fundamental class may be defined in the same file(s) as the fundamental class.

Variables and functions are named in cammelCase, and begin with a lower case letter.

Source files conatining unit tests have the prefix `utest-`. Source files containing integration tests have the prefix `itest-`. Tests are supported by Phil Nash's [CATCH unit testing framework](https://github.com/philsquared/Catch).

Types defined either with `typedef` or `using` may have a prefix separated with an underscore. For example, up_TypeName is a `unique_ptr<TypeName>`. The type `fptype` is just `float` by default, but can be changed to double or alike. Types derived from `fptype`, such as fp_point2d have the `fp_` prefix.

## Dependencies

List of dependencies other than standard libraries:
 * Boost


##Code Format
Code formatting was done with `clang-format -style=LLVM`, and an attempt is made to conform to the [LLVM Coding Standards](http://llvm.org/docs/CodingStandards.html).

##TODO
* Rename VMeNmpcInitPkg to AggregatingInitPkg or something (much) better.
* Rename VMeNmpcInitPkg::bindToAggregator().
* Add minimizer logging to VMeNmpcEngine.
* Change InitPkg safety checks from bool to void if I'm not going to check the return values.
* Write utests for new VMeNmpcEngine initialization.