#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include "../src/NmpcModel.hpp"
#include "../src/NmpcMinimizer.hpp"
#include "../src/VirtualMeNmpcEngine.hpp"

class FakeVirtualMeNmpcModel : public NmpcModel {
  std::string eventHistory_{};

  void recordEvent(char eventCode) { eventHistory_ += eventCode; }

 public:
  unsigned N = 0;

  FakeVirtualMeNmpcModel(unsigned N) : N{N} {}
  virtual ~FakeVirtualMeNmpcModel() = default;
  virtual void seed(xyvth position) { recordEvent('S'); }
  virtual void seed(xyvth position, Point2R target) { recordEvent('S'); }
  virtual void forecast() { recordEvent('F'); }
  virtual void setTrackingErrors() { recordEvent('E'); }
  virtual void computePathPotentialGradient(ObstacleStack& obstacles) {
    recordEvent('P');
  }
  virtual void computeGradient() { recordEvent('G'); }
  virtual fptype distanceToTarget() {
    recordEvent('D');
    return 0;
  }

  std::string eventHistory() { return eventHistory_; }
};

class FakeVirtualMeMinimizer : public NmpcMinimizer {};

class FakeExecutor : public Observer {
  VirtualMeNmpcEngine* subject_ = nullptr;

 public:
  FakeExecutor(VirtualMeNmpcEngine* s) {
    subject_ = s;
    s->attachObserver(this);
  }
  // ~FakeExecutor()
  // {subject_->detachObserver(dynamic_cast<Subject*>(subject_))}
  upVirtualMeCommand recievedCmd_;
  void update(Subject* sub) {
    if (sub == dynamic_cast<Subject*>(subject_))
      recievedCmd_ = subject_->nextCommand();
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

TEST_CASE("Throw appropriately if we try to attach the same observer twice.") {
  standardTestSetup test;
  FakeExecutor exec(test.eng);
  //NB: the FakeExecutor's constructor does the first attach.
  REQUIRE_THROWS_AS(test.eng->attachObserver(&exec),
                    AttemptToAttachAlreadyAttachedObserver);
}

TEST_CASE(
    "When the robot is on the target the controller should return the command "
    "to stop.") {
  standardTestSetup test;
  FakeExecutor exec(test.eng);
  REQUIRE(exec.recievedCmd_.get() == nullptr);
  test.eng->seed(xyvth{1, 1, 0, 0}, Point2R{1, 1});
  REQUIRE(isStopCmd(exec.recievedCmd_.get()));
}

// TEST_CASE(
//     "If I ask for more commands than are available from the current "
//     "calculation, then start returning stop commanda.") {}