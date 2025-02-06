#include "RegisterFile.h"

RegisterFile::RegisterFile(std::string ioDir)
    : outputFile{ioDir + "RFResult.txt"} {
  Registers.resize(32);
  Registers[0] = std::bitset<32>(0);  // x0 is always 0
}

std::bitset<32> RegisterFile::readRF(std::bitset<5> Reg_addr) {
  return Registers[Reg_addr.to_ulong()];
}

void RegisterFile::writeRF(std::bitset<5> Reg_addr,
                           std::bitset<32> Wrt_reg_data) {
  if (Reg_addr.to_ulong() != 0) {  // Register x0 is always 0 in RISC-V
    Registers[Reg_addr.to_ulong()] = Wrt_reg_data;
  }
}

void RegisterFile::outputRF(int cycle) {
  std::ofstream rfout;
  if (cycle == 0)
    rfout.open(outputFile, std::ios_base::trunc);
  else
    rfout.open(outputFile, std::ios_base::app);

  if (rfout.is_open()) {
    rfout << "State of RF after executing cycle:\t" << cycle << std::endl;
    for (int j = 0; j < 32; j++) {
      rfout << Registers[j] << std::endl;
    }
  } else {
    std::cout << "Unable to open RF output file." << std::endl;
  }
  rfout.close();
}
