#include "user_func.hpp"

int main(int argc, char const *argv[]) {
  MakeBitbotEverywhere everyone(
      "/home/dknt/Project/bitbot-ovinf/config/hhfc/bitbot_hhfc.xml",
      "/home/dknt/Project/bitbot-ovinf/config/hhfc/robot.yaml");
  everyone.WillMake();
  everyone.BeMaking();
  everyone.HaveMade();
  return 0;
}
