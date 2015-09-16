void PathPlanner::pushFinalTarget(Target tgt) {
  targets.push_back(tgt);
}

void PathPlanner::pushCurrentTarget(Target tgt) {
  targets.push_front(tgt);
}

void PathPlanner::popFinalTarget() {
  targets.pop_back();
}

int PathPlanner::numberOfTargets() {
  return targets.size();
}

Target& PathPlanner::currentTarget() {
  return targets.front();
}

Target& PathPlanner::finalTarget() {
  return targets.back();
}

void PathPlanner::clearTargetList() {
  targets.clear();
}

bool PathPlanner::areTargetsRemaining() {
  return !targets.empty();
}