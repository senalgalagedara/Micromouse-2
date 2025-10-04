#include "storage.h"
#include "config.h"

struct Header { uint32_t magic; uint16_t ver; uint16_t crc; };

static uint16_t crc16(const uint8_t* data, size_t len){
  uint16_t c=0xFFFF;
  for(size_t i=0;i<len;i++){ c ^= data[i]; for(int j=0;j<8;j++){ c = (c&1)? (c>>1)^0xA001 : (c>>1); } }
  return c;
}

void Storage::begin(){
  EEPROM.begin(EEPROM_BYTES);
}

void Storage::saveMaze(){
  Header h; h.magic = 0xA55AF11E; h.ver=1; h.crc=0;
  uint8_t *p = (uint8_t*)&Maze::cells[0][0];
  size_t sz = sizeof(Maze::cells);
  h.crc = crc16(p, sz);
  // Write header then data
  int addr=0;
  EEPROM.put(addr, h); addr += sizeof(h);
  for(size_t i=0;i<sz;i++){ EEPROM.write(addr++, p[i]); }
  EEPROM.commit();
}

void Storage::loadMaze(){
  Header hRead; int addr=0;
  EEPROM.get(addr, hRead); addr += sizeof(hRead);
  if(hRead.magic != 0xA55AF11E) return;
  size_t sz = sizeof(Maze::cells);
  uint8_t *p = (uint8_t*)&Maze::cells[0][0];
  for(size_t i=0;i<sz;i++){ p[i] = EEPROM.read(addr++); }
  uint16_t c = crc16(p, sz);
  if(c != hRead.crc){
    // invalid; clear
    Maze::clear();
  }
}
