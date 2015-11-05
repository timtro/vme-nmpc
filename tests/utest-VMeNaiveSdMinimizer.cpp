#include "catch.hpp"

#include "../src/NmpcModels/VMeModel.hpp"
#include "../src/NmpcMinimizers/VMeNaiveSdMinimizer.hpp"

//TODO(Tim): Test that throw when iteration limit is reached.
//TODO(Tim): Test that steps are reducing gradient.
//TODO(Tim): Test that plain cases converge.