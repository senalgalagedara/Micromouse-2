#include "planner.h"

Primitive Planner::decide(uint8_t heading, uint8_t desiredDir){
  if(heading == desiredDir) return PRIM_FWD_1;
  uint8_t diff = (4 + desiredDir - heading) % 4;
  if(diff==1) return PRIM_TURN_R90;   // assumes heading: 0N,1E,2S,3W and right-hand turn when dir ahead is +1
  if(diff==3) return PRIM_TURN_L90;
  if(diff==2) return PRIM_TURN_180;
  return PRIM_FWD_1;
}
