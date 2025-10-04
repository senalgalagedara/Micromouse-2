#include "maze.h"

MazeCell Maze::cells[MAZE_W][MAZE_H];

void Maze::clear(){
  for(uint8_t x=0;x<MAZE_W;x++){
    for(uint8_t y=0;y<MAZE_H;y++){
      cells[x][y].bits = 0;
      cells[x][y].dist = 0xFFFF;
    }
  }
}

static inline uint8_t bitForDir(uint8_t dir){
  static const uint8_t map[4] = {1,2,4,8};
  return map[dir&3];
}

void Maze::setWall(uint8_t x,uint8_t y,uint8_t dir, bool present){
  if(x>=MAZE_W||y>=MAZE_H) return;
  if(present) cells[x][y].bits |= bitForDir(dir);
  else        cells[x][y].bits &= ~bitForDir(dir);
  // mirror to neighbor
  int nx=x, ny=y; uint8_t opp=0;
  if(dir==NORTH){ ny=y+1; opp=SOUTH; }
  else if(dir==SOUTH){ ny=y-1; opp=NORTH; }
  else if(dir==EAST){ nx=x+1; opp=WEST; }
  else { nx=x-1; opp=EAST; }
  if(nx>=0 && nx<MAZE_W && ny>=0 && ny<MAZE_H){
    if(present) cells[nx][ny].bits |= bitForDir(opp);
    else        cells[nx][ny].bits &= ~bitForDir(opp);
  }
}

bool Maze::hasWall(uint8_t x,uint8_t y,uint8_t dir){
  if(x>=MAZE_W||y>=MAZE_H) return true;
  return (cells[x][y].bits & bitForDir(dir)) != 0;
}

void Maze::setVisited(uint8_t x,uint8_t y){
  if(x>=MAZE_W||y>=MAZE_H) return;
  cells[x][y].bits |= 16;
}

bool Maze::isVisited(uint8_t x,uint8_t y){
  if(x>=MAZE_W||y>=MAZE_H) return false;
  return (cells[x][y].bits & 16) != 0;
}

uint16_t Maze::getDist(uint8_t x,uint8_t y){
  return (x<MAZE_W && y<MAZE_H) ? cells[x][y].dist : 0xFFFF;
}

void Maze::setDist(uint8_t x,uint8_t y,uint16_t d){
  if(x<MAZE_W && y<MAZE_H) cells[x][y].dist = d;
}
