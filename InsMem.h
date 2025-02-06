#ifndef INSMEM_H
#define INSMEM_H

#include <bitset>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Globals.h"

#define MemSize 1000

class InsMem {
 public:
  std::string id, ioDir;
  int instrCount;
  InsMem(std::string name, std::string ioDir);
  std::bitset<32> readInstr(std::bitset<32> ReadAddress);

 private:
  std::vector<std::bitset<8>> IMem;
};
#endif  // INSMEM_H
