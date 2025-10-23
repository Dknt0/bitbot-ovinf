#include "user_func.hpp"

int main(int argc, char const *argv[]) {
  MakeBitbotEverywhere everyone(
      "/home/dknt/Project/bitbot-ovinf/config/fc2/bhr8_mujoco.xml",
      "/home/dknt/Project/bitbot-ovinf/config/fc2/bhr8_mujoco.yaml");
  everyone.WillMake();
  everyone.BeMaking();
  everyone.HaveMade();
  return 0;
}
