#include "user_func.hpp"

int main(int argc, char const *argv[]) {
  MakeBitbotEverywhere everyone(
      "/home/dknt/Project/bitbot-ovinf/config/booster/booster_mj.xml",
      "/home/dknt/Project/bitbot-ovinf/config/booster/robot.yaml");
  everyone.WillMake();
  everyone.BeMaking();
  everyone.HaveMade();
  return 0;
}
