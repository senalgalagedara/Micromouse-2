#include "floodfill.h"

// Small queue for BFS
static uint8_t qx[MAZE_W*MAZE_H];
static uint8_t qy[MAZE_W*MAZE_H];

static inline uint16_t baseCost(){ return 1; }

uint16_t FloodFill::weightTurn(uint8_t fromDir, uint8_t toDir, bool speedRun){
  if(fromDir==toDir) return 0;
  uint8_t diff = (4 + toDir - fromDir) % 4;
  if(diff==2) return speedRun ? 4 : 6; // 180
  if(diff==1 || diff==3) return speedRun ? 1 : 2; // 90
  return 0;
}

static bool isGoal(uint8_t x,uint8_t y){
  return (x==7||x==8) && (y==7||y==8); // center 4 cells
}

void FloodFill::compute(uint8_t goalMask){
  // Initialize distances to max
  for(uint8_t x=0;x<MAZE_W;x++){
    for(uint8_t y=0;y<MAZE_H;y++){
      Maze::setDist(x,y, 0x3FFF);
    }
  }
  // Seed goals
  int head=0, tail=0;
  for(uint8_t x=7;x<=8;x++){
    for(uint8_t y=7;y<=8;y++){
      Maze::setDist(x,y,0);
      qx[tail]=x; qy[tail]=y; tail++;
    }
  }
  // BFS flood
  while(head!=tail){
    uint8_t cx = qx[head];
    uint8_t cy = qy[head]; head++;
    uint16_t cd = Maze::getDist(cx,cy);
    // for each neighbor
    const int dx[4]={0,1,0,-1};
    const int dy[4]={1,0,-1,0};
    for(uint8_t dir=0;dir<4;dir++){
      int nx = cx + dx[dir];
      int ny = cy + dy[dir];
      if(nx<0||nx>=MAZE_W||ny<0||ny>=MAZE_H) continue;
      if(Maze::hasWall(cx,cy,dir)) continue;
      uint16_t nd = cd + baseCost();
      if(nd < Maze::getDist(nx,ny)){
        Maze::setDist(nx,ny, nd);
        qx[tail]=nx; qy[tail]=ny; tail++;
      }
    }
  }
}

// Choose the neighbor direction that minimizes distance+turn weight
uint8_t FloodFill::chooseDir(uint8_t x,uint8_t y,uint8_t heading){
  uint16_t best=0x7FFF; uint8_t bestDir=heading;
  const int dx[4]={0,1,0,-1};
  const int dy[4]={1,0,-1,0};
  for(uint8_t dir=0;dir<4;dir++){
    if(Maze::hasWall(x,y,dir)) continue;
    int nx = (int)x + dx[dir];
    int ny = (int)y + dy[dir];
    if(nx<0||nx>=MAZE_W||ny<0||ny>=MAZE_H) continue;
    uint16_t cand = Maze::getDist(nx,ny) + weightTurn(heading, dir, false);
    if(cand < best){ best=cand; bestDir=dir; }
  }
  return bestDir;
}
