#TODO
* Tests to see if the aggregator throws should be in an aggregator test. The VMeModel unit test shouldn't need a FakeMinimizer.
* Abstract factory or factory method for obstacles.
* FIXME: ObstacleContainer takes pointer and makes unique_ptr which violates convention that ownership isn't passed if plain points are given.
* FIXME: ObstacleContainer and TargetContainer should both derive from a container interface.
* FIXME: Investigate simple_linux_js.py technique.
* FIXME: Model shouldn't know about targets and x^ref. Both of these should be the responsibility of the path planner. NmpcModel should have a set xref() method.