#include "user_func.hpp"

int main(int argc, char const *argv[]) {
  MakeBitbotEverywhere everyone(
      "/home/dknt/Project/bitbot-ovinf/config/dex/dex_mj.xml",
      "/home/dknt/Project/bitbot-ovinf/config/dex/robot.yaml");
  everyone.WillMake();
  everyone.BeMaking();
  everyone.HaveMade();
  return 0;
}
