#include "InsMem.h"

InsMem::InsMem(std::string name, std::string ioDir) : id{name}, ioDir{ioDir} {
  IMem.resize(MemSize);
  std::ifstream imem(ioDir + separator + "imem.txt");
  std::string line;
  int i = 0;
  if (imem.is_open()) {
    while (std::getline(imem, line)) {
      // only get the first 8 bits of the line
      line = line.substr(0, 8);
      IMem[i] = std::bitset<8>(line);
      i++;
    }
    // get the number of instructions
    instrCount = i / 4;
    std::cout << "Instruction Memory loaded with " << instrCount
              << " instructions." << std::endl;
  } else {
    std::cout << "Unable to open IMEM input file." << std::endl;
  }
  imem.close();
}

std::bitset<32> InsMem::readInstr(std::bitset<32> ReadAddress) {
  std::string instruction = "";
  for (int i = 0; i < 4; i++) {
    instruction += IMem[ReadAddress.to_ulong() + i].to_string();
  }
  return std::bitset<32>(instruction);
}
