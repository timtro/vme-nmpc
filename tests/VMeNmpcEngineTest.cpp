#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include "../src/VirtualMeNmpcEngine.hpp"
#include "../src/NmpcModels/VirtualMeModel.hpp"
#include "../src/NmpcMinimizers/VirtualMeSDMinimizer.hpp"
#include "../src/VirtualMeCommand.hpp"

class FakeVirtualMeNmpcModel : public NmpcModel {
  int seedCount_;
  int forecastCount_;
  int setTrackingErrorsCount_;
  int computePathPotentialGradientCount_;
  int computeGradientCount_;
  std::string eventHistory_{};

  void recordEvent(char eventCode) { eventHistory_ += eventCode; }

 public:
  unsigned N = 0;

  FakeVirtualMeNmpcModel(unsigned N) : N{N} {}
  virtual ~FakeVirtualMeNmpcModel() = default;
  virtual void seed() { recordEvent('S'); }
  virtual void forecast() { recordEvent('F'); }
  virtual void setTrackingErrors(Point2R target) { recordEvent('E'); }
  virtual void computePathPotentialGradient(ObstacleStack& obstacles) {
    recordEvent('P');
  }
  virtual void computeGradient() { recordEvent('G'); }

  std::string eventHistory() { return eventHistory_; }
};
class FakeVirtualMeMinimizer : public NmpcMinimizer {};
class FakeExecutor : public Observer {
  VirtualMeNmpcEngine* observed_;

 public:
  CmdUP recievedCmd_;
  void update(Subject* sub) {
    if (sub == observed_) recievedCmd_ = observed_->nextCommand();
  }
};
struct standardTestSetup {
  FakeVirtualMeNmpcModel* mod{nullptr};
  FakeVirtualMeMinimizer* min{nullptr};
  VirtualMeNmpcEngine* eng{nullptr};

  standardTestSetup() {
    unsigned num_of_intervals{5};

    mod = new FakeVirtualMeNmpcModel{num_of_intervals};
    min = new FakeVirtualMeMinimizer{};
    eng = new VirtualMeNmpcEngine{*mod, *min};
  }
  ~standardTestSetup() {
    delete eng;
    delete min;
    delete mod;
  }
};

bool isStopCmd(VirtualMeCommand* cmd) { return dynamic_cast<VMeStop*>(cmd); }

TEST_CASE(
    "When the robot is on the target the controller shold return"
    " the command to stop.") {
  standardTestSetup test;
  test.eng->setTarget(Point2R{0, 0});
  auto cmd = test.eng->nextCommand();
  REQUIRE(isStopCmd(cmd.get()));
}

TEST_CASE("Same as previous test, but now with an observer") {
  standardTestSetup test;
  FakeExecutor exec;
  test.eng->attachObserver(&exec);
  test.eng->setTarget(Point2R{0, 0});
}

TEST_CASE(
    "Given a non-originated target, the controller should try to"
    " iterate on the min mock, and get convergence in iterations.") {}

TEST_CASE(
    "If I ask for more commands than are available from the current"
    " calculation, then start returning stop commanda.") {}