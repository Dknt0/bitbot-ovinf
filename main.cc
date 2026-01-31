#include "user_func.hpp"

int main(int argc, char const *argv[]) {
  MakeBitbotEverywhere everyone(
      "/home/dknt/Project/bitbot-ovinf/config/efc/efc_mj.xml",
      "/home/dknt/Project/bitbot-ovinf/config/efc/robot.yaml");
  everyone.WillMake();
  everyone.BeMaking();
  everyone.HaveMade();
  return 0;
}
