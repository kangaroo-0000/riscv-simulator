#ifndef CORE_H
#define CORE_H

#include <bitset>
#include <fstream>
#include <iostream>
#include <string>

#include "DataMem.h"
#include "InsMem.h"
#include "RegisterFile.h"

enum ALUOps {
  ALU_ADD = 0,
  ALU_SUB,
  ALU_XOR,
  ALU_OR,
  ALU_AND,
  ALU_NOP,
  ALU_UNDEFINED
};

struct IFStruct {
  std::bitset<32> PC = 0;
  bool nop;
};

struct IDStruct {
  std::bitset<32> Instr = 0;
  bool nop;
};

struct EXStruct {
  std::bitset<32> Read_data1 = 0;
  std::bitset<32> Read_data2 = 0;
  std::bitset<16> Imm = 0;
  std::bitset<5> Rs = 0;
  std::bitset<5> Rt = 0;
  std::bitset<5> Wrt_reg_addr = 0;
  bool is_I_type = false;
  bool rd_mem = false;
  bool wrt_mem = false;
  int alu_op = ALU_UNDEFINED;
  bool wrt_enable = false;
  bool nop;
};

struct MEMStruct {
  std::bitset<32> ALUresult = 0;
  std::bitset<32> Store_data = 0;
  std::bitset<5> Rs = 0;
  std::bitset<5> Rt = 0;
  std::bitset<5> Wrt_reg_addr = 0;
  bool rd_mem = false;
  bool wrt_mem = false;
  bool wrt_enable = false;
  bool nop;
};

struct WBStruct {
  std::bitset<32> Wrt_data = 0;
  std::bitset<5> Rs = 0;
  std::bitset<5> Rt = 0;
  std::bitset<5> Wrt_reg_addr = 0;
  bool wrt_enable = false;
  bool nop;
};

struct stateStruct {
  IFStruct IF;
  IDStruct ID;
  EXStruct EX;
  MEMStruct MEM;
  WBStruct WB;
};

class Core {
 public:
  RegisterFile myRF;
  uint32_t cycle = 0;
  bool halted = false;
  std::string ioDir;
  stateStruct state, nextState;
  InsMem &ext_imem;
  DataMem &ext_dmem;

  Core(std::string ioDir, InsMem &imem, DataMem &dmem);
  virtual void step();
  virtual void printState();
};

class SingleStageCore : public Core {
 public:
  SingleStageCore(std::string ioDir, InsMem &imem, DataMem &dmem);
  void step();
  void printState(stateStruct state, int cycle);

 private:
  std::string opFilePath;
};

class FiveStageCore : public Core {
 public:
  FiveStageCore(std::string ioDir, InsMem &imem, DataMem &dmem);
  void step();
  void printState(stateStruct state, int cycle);

 private:
  std::string opFilePath;
};

#endif  // CORE_H
