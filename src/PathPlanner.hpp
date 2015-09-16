
struct Target {
  pointR2 locus;
  float tolerance;
  decltype(locus.x)& x;
  decltype(locus.x)& y;

  Target(float x, float y, float tol)
    : locus{x,y}, tolerance{tol}, x{locus.x}, y{locus.y} {}
};

class TargetStack {
  void pushFinalTarget(Target);
  void pushCurrentTarget(Target);
  int numberOfTargets();
  Target& currentTarget();
  Target& finalTarget();
  void popFinalTarget();
  void clearTargetList();
  bool areTargetsRemaining();
};