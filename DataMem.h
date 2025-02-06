#ifndef DATAMEM_H
#define DATAMEM_H

#include <algorithm>
#include <bitset>
#include <cctype>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Globals.h"

#define MemSize 1000
class DataMem {
 public:
  std::string id, opFilePath, ioDir;
  DataMem(std::string name, std::string ioDir);
  std::bitset<32> readDataMem(std::bitset<32> Address);
  void writeDataMem(std::bitset<32> Address, std::bitset<32> WriteData);
  void outputDataMem();
  void printDataMem();

 private:
  std::vector<std::bitset<8>> DMem;
};

#endif  // DATAMEM_H
