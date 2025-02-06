#ifndef REGISTERFILE_H
#define REGISTERFILE_H

#include <bitset>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

class RegisterFile {
 public:
  std::string outputFile;
  RegisterFile(std::string ioDir);
  std::bitset<32> readRF(std::bitset<5> Reg_addr);
  void writeRF(std::bitset<5> Reg_addr, std::bitset<32> Wrt_reg_data);
  void outputRF(int cycle);

 private:
  std::vector<std::bitset<32>> Registers;
};

#endif  // REGISTERFILE_H
