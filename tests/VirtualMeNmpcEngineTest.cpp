#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include "../src/NmpcModel.hpp"
#include "../src/NmpcMinimizer.hpp"
#include "../src/VirtualMeNmpcEngine.hpp"
#include "FakeVirtualMeModel.hpp"
#include "FakeVirtualMeMinimizer.hpp"

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
  FakeVirtualMeModel* mod{nullptr};
  FakeVirtualMeMinimizer* min{nullptr};
  VirtualMeNmpcEngine* eng{nullptr};

  standardTestSetup() {
    unsigned num_of_intervals{5};

    mod = new FakeVirtualMeModel{num_of_intervals};
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
bool isMoveCmd(VirtualMeCommand* cmd) { return dynamic_cast<VMeV*>(cmd); }

TEST_CASE("Throw appropriately if we try to attach the same observer twice.") {
  standardTestSetup test;
  FakeExecutor exec(test.eng);
  // NB: the FakeExecutor's constructor does the first attach.
  REQUIRE_THROWS_AS(test.eng->attachObserver(&exec),
                    AttemptToAttachAlreadyAttachedObserver);
}

TEST_CASE(
    "When the robot is on the target the controller should return the command "
    "to stop, and should have halted the model.") {
  standardTestSetup test;
  FakeExecutor exec(test.eng);
  REQUIRE(exec.recievedCmd_.get() == nullptr);
  test.eng->seed(xyvth{1, 1, 0, 0}, Point2R{1, 1});
  // Should have called (S)eed (D)istanceToTarget and (H)alt:
  REQUIRE(test.mod->eventHistory() == "SDH");
  REQUIRE(isStopCmd(exec.recievedCmd_.get()));
}

TEST_CASE(
    "When the robot and target are sufficiently separated, the engine should "
    "orchestrate the minimization of the cost function over the NMPC horizon "
    "and notify observers of success.") {
  standardTestSetup test;
  FakeExecutor exec(test.eng);
  REQUIRE(exec.recievedCmd_.get() == nullptr);
  test.eng->seed(xyvth{0, 0, 0, 0}, Point2R{5,5});
  REQUIRE(test.mod->eventHistory() == "SD");
  REQUIRE(test.min->eventHistory() == "O");
  // REQUIRE(isMoveCmd(exec.recievedCmd_.get()));
}

// TEST_CASE(
//     "If I ask for more commands than are available from the current "
//     "calculation, then start returning stop commanda.") {}