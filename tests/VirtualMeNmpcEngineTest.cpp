#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include "../src/NmpcModel.hpp"
#include "../src/NmpcMinimizer.hpp"
#include "../src/VirtualMeNmpcEngine.hpp"
#include "FakeVirtualMeModel.hpp"
#include "FakeVirtualMeMinimizer.hpp"

class FakeExecutor : public Observer {
  VirtualMeNmpcEngine* subjectEngine = nullptr;

 public:
  upVirtualMeCommand commandFromLastNotify;

  FakeExecutor(VirtualMeNmpcEngine* s) {
    subjectEngine = s;
    s->attachObserver(this);
  }
  ~FakeExecutor() { subjectEngine->detachObserver(this); }
  void update(Subject* s) {
    if (s == dynamic_cast<Subject*>(subjectEngine))
      commandFromLastNotify = subjectEngine->nextCommand();
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
bool isNullCmd(VirtualMeCommand* cmd) { return dynamic_cast<VMeNullCmd*>(cmd); }
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
  REQUIRE(exec.commandFromLastNotify.get() == nullptr);
  test.eng->seed(xyvth{1, 1, 0, 0}, Point2R{1, 1});
  // Should have called (S)eed (D)istanceToTarget and (H)alt:
  REQUIRE(test.mod->eventHistory() == "SDHC");
  REQUIRE(isNullCmd(exec.commandFromLastNotify.get()));
}

TEST_CASE(
    "When the robot and target are sufficiently separated, the engine should "
    "orchestrate the minimization of the cost function over the NMPC horizon "
    "and notify observers of success.") {
  standardTestSetup test;
  FakeExecutor exec(test.eng);
  REQUIRE(exec.commandFromLastNotify.get() == nullptr);
  test.eng->seed(xyvth{0, 0, 0, 0}, Point2R{5, 5});
  REQUIRE(test.mod->eventHistory() == "SDC");
  REQUIRE(test.min->eventHistory() == "O");
  REQUIRE(isMoveCmd(exec.commandFromLastNotify.get()));
}

// TEST_CASE(
//     "If I ask for more commands than are available from the current "
//     "calculation, then start returning stop commanda.") {}