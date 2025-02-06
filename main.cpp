#include <iomanip>  // For std::setprecision

#include "Core.h"
#include "DataMem.h"
#include "InsMem.h"

int main(int argc, char *argv[]) {
  std::string ioDir = "";

  // Handle arguments for I/O directory
  if (argc == 1) {
    std::cout << "Enter path containing the memory files: ";
    std::cin >> ioDir;
  } else if (argc > 2) {
    std::cout << "Invalid number of arguments. Machine stopped." << std::endl;
    return -1;
  } else {
    ioDir = argv[1];
    std::cout << "IO Directory: " << ioDir << std::endl;
  }

  // Initialize Instruction Memory and Data Memory for both cores
  InsMem imem = InsMem("Imem", ioDir);
  DataMem dmem_ss = DataMem("SS", ioDir);  // Single-stage core data memory
  DataMem dmem_fs = DataMem("FS", ioDir);  // Five-stage core data memory

  // Initialize single-stage core and pipelined core
  SingleStageCore SSCore(ioDir, imem, dmem_ss);
  FiveStageCore FSCore(ioDir, imem, dmem_fs);

  // Main execution loop: run both cores until halted
  while (true) {
    // Step through single-stage core
    if (!SSCore.halted) {
      SSCore.step();
    }

    // Step through five-stage pipelined core
    if (!FSCore.halted) {
      FSCore.step();
    }

    // Break when both cores have halted
    if (SSCore.halted && FSCore.halted) {
      break;
    }
  }

  // Output performance metrics for Single-Stage Core
  std::cout << std::endl;
  std::cout << "Single Stage Core Performance Metrics" << std::endl;
  std::cout << "Number of cycles taken: " << SSCore.cycle + 1 << std::endl;
  std::cout << std::fixed << std::setprecision(6);  // Set precision for output
  // get instruction count from the instruction memory
  int INSTRUCTION_COUNT = imem.instrCount;
  if (INSTRUCTION_COUNT > 0) {
    double cpi_ss = static_cast<double>(SSCore.cycle + 1) / INSTRUCTION_COUNT;
    double ipc_ss = static_cast<double>(INSTRUCTION_COUNT) / (SSCore.cycle + 1);
    std::cout << "Cycles per instruction: " << cpi_ss << std::endl;
    std::cout << "Instructions per cycle: " << ipc_ss << std::endl;
  } else {
    std::cout << "No instructions were executed in Single Stage Core."
              << std::endl;
  }

  // Output performance metrics for Five-Stage Core
  std::cout << std::endl;
  std::cout << "Five Stage Core Performance Metrics" << std::endl;
  std::cout << "Number of cycles taken: " << FSCore.cycle << std::endl;
  if (INSTRUCTION_COUNT > 0) {
    double cpi_fs = static_cast<double>(FSCore.cycle) / INSTRUCTION_COUNT;
    double ipc_fs = static_cast<double>(INSTRUCTION_COUNT) / (FSCore.cycle);
    std::cout << "Cycles per instruction: " << cpi_fs << std::endl;
    std::cout << "Instructions per cycle: " << ipc_fs << std::endl;
  } else {
    std::cout << "No instructions were executed in Five Stage Core."
              << std::endl;
  }
  // Dump data memories for both cores after completion
  dmem_ss.outputDataMem();  // Single-stage data memory
  dmem_fs.outputDataMem();  // Pipelined data memory
  std::cout << "Simulation complete." << std::endl;

  return 0;
}