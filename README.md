# vme-nmpc

[![Build Status](https://travis-ci.org/timtro/vme-nmpc.svg?branch=master)](https://travis-ci.org/timtro/vme-nmpc)
[![Build Status](https://scan.coverity.com/projects/7887/badge.svg)](https://scan.coverity.com/projects/vme-nmpc)

The Non-linear model predictive controller and tracker for
CrossWing's virtualME platform.

The mathematical foundations for the software and a description of the algorithm are expounded in `Teatro, T.A.V.; Eklund, J.M.; Milman, R., "Nonlinear Model Predictive Control for Omnidirectional Robot Motion Planning and Tracking With Avoidance of Moving Obstacles," in Electrical and Computer Engineering, Canadian Journal of , vol.37, no.3, pp.151-156, Summer 2014
doi: 10.1109/CJECE.2014.2328973` which can be found by clicking [here](http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=6978004&queryText=CJECE%202014%202328973&newsearch=true).

A general paper introducing the architecture is currently in peer review. I will link to it here once it is either published, or rejected (in which case I'll link directly to the paper.)

## Building

Configuration is done with cmake. Recommended build commands for
out-of-source building:

    $ cd build
    $ cmake ..
    $ make

### Testing

Unit tests are provided for key modules and will be built by default. Run

    $ make test

to automatically run all tests. Users adding obstacle, model or minimizer types are encouraged to write a test module in ./test .

## Usage

See vme-nmpc.cpp for an example of constructing objects and binding them with the NMPC kernel. Here is a naive example using default classes:

    Daemon command_server(5111, commandHandler);
    vme.originate()

    AggregatorInitializer init(inputFileData);
    init.targets = &targets;
    init.obstacles = &obstacles;
    auto model = make_unique<MyModel>(init);
    auto minimizer = make_unique<MySdMinimizer>(init);
    auto planner = make_unique<MyPathPlanner>(init);
    auto logger = make_unique<MyLogger>(init, ...);
    auto kernel = make_unique<MyNmpcKernel>(init);

    auto executor = make_unique<VMeDefaultExecutor>(kernel.get());

    planner->set_stateEstimateRetriever([&vme]() {
      SeedPackage state;
      int q;
      std::tie(state.pose.x, state.pose.y, state.pose.th, q) = vme.q();
      state.pose.th = degToRad(state.pose.th);
      return state;
    });

    for (;;) {
      while (targets.empty()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
      do {
        std::this_thread::sleep_for(std::chrono::milliseconds(T));
        kernel->nmpc_step(planner->get_seed());
        executor->run(vme);
      } while (planner->is_continuing());
      vme.stop();
    }

### Reading and Using the Code

Files and objects prefixed with VMe or vMe are specific to the VirtualMe Robot and the prefix is generally used to distinguish those concrete entities from more general and abstract entities. For example, the NmpcModel class is an interface (that is, an abstract class wherein all methods are virtual) that is quite generic to the NMPC problem. However, VMeNmpcModel is a class that derives from the interface and contains data and methods specific to the virtualME robot. If you port this code to another type of device, files and classes prefixed with VMe are the files that must be rewritten.

Classes begin with Capital Letters, and appear in CammelCase. Source files defining those classes are named after the class they contain. Classes which exist only to support a more fundamental class may be defined in the same file(s) as the fundamental class.

Variables are named in cammelCase, and begin with a lower case letter.

Functions are named in keeping with the C++ Standard Library. That is, function names are lowercase (where appropriate) and spaced with underscores (`_`).

Source files containing unit tests have the prefix `utest-`. Source files containing integration tests have the prefix `itest-`. Tests are supported by Phil Nash's [CATCH unit testing framework](https://github.com/philsquared/Catch).

Types defined either with `typedef` or `using` may have a prefix separated with an underscore. For example, up_TypeName is a `unique_ptr<TypeName>`. The type `fptype` is just `float` by default, but can be changed to double or alike. Types derived from `fptype`, such as `fp_point2d` have the `fp_` prefix.

## Dependencies

List of dependencies other than standard libraries:
 * The current `InputFileData` class implementation relies on `Boost.PropertyTree` for parsing JSON input files and storing the data.
 * The current `ClArgs` class implementation relies on `getopt.h` from the GNU C library for parsing commandline options.

### (Included dependencies)

This codebase includes a copy of Phil Nash's [CATCH unit testing framework](https://github.com/philsquared/Catch). It also borrows from CrossWing's mathematical structures for object classes related to virtualME.

## Code Format

Code formatting was done with `clang-format -style=Google`.

Efforts are currently underway to ensure compliance with the [C++ Core Guidelines](https://github.com/isocpp/CppCoreGuidelines). Any deviations from those guidelines should be considered bugs unless appropriate documentation explains otherwise. Please report bugs.
