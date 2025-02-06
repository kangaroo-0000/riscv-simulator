#include "Core.h"

int getALUControl(uint32_t opcode, uint32_t funct3, uint32_t funct7) {
  switch (opcode) {
    case 0b0110011:  // R-type instructions
      switch (funct3) {
        case 0b000:
          if (funct7 == 0b0000000)
            return ALU_ADD;  // ADD
          else if (funct7 == 0b0100000)
            return ALU_SUB;  // SUB
          break;
        case 0b100:
          return ALU_XOR;  // XOR
        case 0b110:
          return ALU_OR;  // OR
        case 0b111:
          return ALU_AND;  // AND
      }
      break;

    case 0b0010011:  // I-type arithmetic instructions
      switch (funct3) {
        case 0b000:
          return ALU_ADD;  // ADDI
        case 0b100:
          return ALU_XOR;  // XORI
        case 0b110:
          return ALU_OR;  // ORI
        case 0b111:
          return ALU_AND;  // ANDI
      }
      break;

    case 0b0000011:    // Load instructions (e.g., LW)
    case 0b0100011:    // Store instructions (e.g., SW)
      return ALU_ADD;  // Address calculation (rs1 + immediate)

    case 0b1100011:    // Branch instructions (e.g., BEQ, BNE)
      return ALU_SUB;  // Compare rs1 and rs2 by subtraction

    case 0b1101111:  // JAL instruction
      return ALU_NOP;

    default:
      return ALU_UNDEFINED;  // Unsupported opcode
  }
  return ALU_UNDEFINED;  // If no matching case is found
}

int32_t getITypeImm(std::bitset<32> instruction) {
  int32_t imm = (instruction.to_ulong() >> 20) & 0xFFF;  // Bits [31:20]
  if (imm & 0x800) {                                     // Check sign bit
    imm |= 0xFFFFF000;  // Sign-extend negative immediates
  }
  return imm;
}

int32_t getSTypeImm(std::bitset<32> instruction) {
  int32_t imm = ((instruction.to_ulong() >> 25) << 5) |
                ((instruction.to_ulong() >> 7) & 0x1F);
  if (imm & 0x800) {
    imm |= 0xFFFFF000;
  }
  return imm;
}

int32_t getBTypeImm(std::bitset<32> instruction) {
  int32_t imm = ((instruction.to_ulong() >> 31) << 12) |
                (((instruction.to_ulong() >> 25) & 0x3F) << 5) |
                (((instruction.to_ulong() >> 8) & 0x0F) << 1) |
                (((instruction.to_ulong() >> 7) & 0x01) << 11);
  if (imm & 0x1000) {
    imm |= 0xFFFFE000;
  }
  return imm;
}

int32_t getJTypeImm(std::bitset<32> instruction) {
  int32_t imm = ((instruction.to_ulong() >> 31) << 20) |
                (((instruction.to_ulong() >> 12) & 0xFF) << 12) |
                (((instruction.to_ulong() >> 20) & 0x01) << 11) |
                (((instruction.to_ulong() >> 21) & 0x3FF) << 1);
  if (imm & 0x100000) {
    imm |= 0xFFE00000;
  }
  return imm;
}

Core::Core(std::string ioDir, InsMem &imem, DataMem &dmem)
    : myRF(ioDir), ioDir{ioDir}, ext_imem{imem}, ext_dmem{dmem} {}

void Core::step() {}

void Core::printState() {}

SingleStageCore::SingleStageCore(std::string ioDir, InsMem &imem, DataMem &dmem)
    : Core(ioDir + separator + "SS_", imem, dmem),
      opFilePath(ioDir + separator + "StateResult_SS.txt") {}

void SingleStageCore::step() {
  std::cout << "SS Cycle: " << cycle << std::endl;
  if (state.IF.nop) {
    halted = true;
    myRF.outputRF(cycle);          // dump RF
    printState(nextState, cycle);  // print states after executing cycle
    return;
  }

  // 1. Instruction Fetch (IF)
  std::bitset<32> instruction = ext_imem.readInstr(state.IF.PC);

  // Decode the instruction fields
  uint32_t instr = instruction.to_ulong();
  uint32_t opcode = instr & 0x7F;          // opcode (bits 6-0)
  uint32_t funct3 = (instr >> 12) & 0x07;  // funct3 (bits 14-12)
  uint32_t funct7 = (instr >> 25) & 0x7F;  // funct7 (bits 31-25)
  uint32_t rs1 = (instr >> 15) & 0x1F;     // rs1 (bits 19-15)
  uint32_t rs2 = (instr >> 20) & 0x1F;     // rs2 (bits 24-20)
  uint32_t rd = (instr >> 7) & 0x1F;       // rd (bits 11-7)
  bool pc_updated = false;                 // flag to check if PC was updated

  std::cout << "opcode: " << std::bitset<7>(opcode) << std::endl;

  // 2. Instruction Decode and Execution (ID and EX)

  if (opcode == 0b0110011) {  // R-type (ADD, SUB, XOR, OR, AND)
    std::bitset<32> rs1_data = myRF.readRF(rs1);
    std::bitset<32> rs2_data = myRF.readRF(rs2);

    if (funct3 == 0b000) {  // ADD or SUB
      if (funct7 == 0b0000000) {
        // ADD: rd = rs1 + rs2
        nextState.WB.Wrt_data = rs1_data.to_ulong() + rs2_data.to_ulong();
      } else if (funct7 == 0b0100000) {
        // SUB: rd = rs1 - rs2
        nextState.WB.Wrt_data = rs1_data.to_ulong() - rs2_data.to_ulong();
      }
      nextState.WB.Wrt_reg_addr = rd;
      nextState.WB.wrt_enable = true;
    } else if (funct3 == 0b100) {  // XOR
      nextState.WB.Wrt_data = rs1_data.to_ulong() ^ rs2_data.to_ulong();
      nextState.WB.Wrt_reg_addr = rd;
      nextState.WB.wrt_enable = true;
    } else if (funct3 == 0b110) {  // OR
      nextState.WB.Wrt_data = rs1_data.to_ulong() | rs2_data.to_ulong();
      nextState.WB.Wrt_reg_addr = rd;
      nextState.WB.wrt_enable = true;
    } else if (funct3 == 0b111) {  // AND
      nextState.WB.Wrt_data = rs1_data.to_ulong() & rs2_data.to_ulong();
      nextState.WB.Wrt_reg_addr = rd;
      nextState.WB.wrt_enable = true;
    }
  } else if (opcode == 0b0010011) {  // I-type (ADDI, XORI, ORI, ANDI)
    std::bitset<32> rs1_data = myRF.readRF(rs1);
    int32_t imm = getITypeImm(instruction);

    if (funct3 == 0b000) {  // ADDI
      nextState.WB.Wrt_data = rs1_data.to_ulong() + imm;
    } else if (funct3 == 0b100) {  // XORI
      nextState.WB.Wrt_data = rs1_data.to_ulong() ^ imm;
    } else if (funct3 == 0b110) {  // ORI
      nextState.WB.Wrt_data = rs1_data.to_ulong() | imm;
    } else if (funct3 == 0b111) {  // ANDI
      nextState.WB.Wrt_data = rs1_data.to_ulong() & imm;
    }
    nextState.WB.Wrt_reg_addr = rd;
    nextState.WB.wrt_enable = true;
  } else if (opcode == 0b0000011 && funct3 == 0b000) {  // LW (I-type)
    std::bitset<32> rs1_data = myRF.readRF(rs1);
    int32_t imm = getITypeImm(instruction);
    uint32_t address = rs1_data.to_ulong() + imm;
    nextState.WB.Wrt_data = ext_dmem.readDataMem(address);
    nextState.WB.Wrt_reg_addr = rd;
    nextState.WB.wrt_enable = true;
  } else if (opcode == 0b0100011) {  // SW instruction (S-type)
    std::bitset<32> rs1_data = myRF.readRF(rs1);
    std::bitset<32> rs2_data = myRF.readRF(rs2);
    int32_t imm = getSTypeImm(instruction);
    uint32_t address = rs1_data.to_ulong() + imm;
    ext_dmem.writeDataMem(address, rs2_data);
  } else if (opcode == 0b1100011) {  // B-type (Branch: BEQ, BNE)
    std::bitset<32> rs1_data = myRF.readRF(rs1);
    std::bitset<32> rs2_data = myRF.readRF(rs2);
    int32_t imm = getBTypeImm(instruction);

    if (funct3 == 0b000) {  // BEQ
      if (rs1_data == rs2_data) {
        nextState.IF.PC = std::bitset<32>(state.IF.PC.to_ulong() + imm);
        pc_updated = true;
      }
    } else if (funct3 == 0b001) {  // BNE
      if (rs1_data != rs2_data) {
        nextState.IF.PC = std::bitset<32>(state.IF.PC.to_ulong() + imm);
        pc_updated = true;
      }
    }
  } else if (opcode == 0b1101111) {  // JAL instruction (J-type)
    int32_t imm = getJTypeImm(instruction);
    nextState.WB.Wrt_data = state.IF.PC.to_ulong() + 4;  // PC + 4 into rd
    nextState.IF.PC = std::bitset<32>(state.IF.PC.to_ulong() + imm);
    nextState.WB.Wrt_reg_addr = rd;
    nextState.WB.wrt_enable = true;
    pc_updated = true;
  } else if (instr == 0xFFFFFFFF) {  // HALT
    nextState.IF.nop = true;
    // halted = true;
    pc_updated = true;
  }

  // 3. Increment PC if no branch or jump was taken
  if (!pc_updated) {
    nextState.IF.PC = std::bitset<32>(state.IF.PC.to_ulong() + 4);
  }

  // 4. Writeback handled in EX section; store results in register file
  if (nextState.WB.wrt_enable && nextState.WB.Wrt_reg_addr != 0) {
    myRF.writeRF(nextState.WB.Wrt_reg_addr, nextState.WB.Wrt_data);
  }

  myRF.outputRF(cycle);          // dump RF
  printState(nextState, cycle);  // print states after executing cycle

  // Update the current state with the values calculated in this cycle
  state = nextState;
  cycle++;
}

void SingleStageCore::printState(stateStruct state, int cycle) {
  std::ofstream printstate;
  if (cycle == 0)
    printstate.open(opFilePath, std::ios_base::trunc);
  else
    printstate.open(opFilePath, std::ios_base::app);
  if (printstate.is_open()) {
    printstate << "State after executing cycle:\t" << cycle << std::endl;

    printstate << "IF.PC:\t" << state.IF.PC.to_ulong() << std::endl;
    printstate << "IF.nop:\t" << state.IF.nop << std::endl;
  } else
    std::cout << "Unable to open SS StateResult output file." << std::endl;
  printstate.close();
}

FiveStageCore::FiveStageCore(std::string ioDir, InsMem &imem, DataMem &dmem)
    : Core(ioDir + "/FS_", imem, dmem),
      opFilePath(ioDir + "/StateResult_FS.txt") {}

void FiveStageCore::step() {
  std::cout << "FS Cycle: " << cycle << std::endl;

  /* --------------------- WB stage --------------------- */
  // Write-back stage
  if (!state.WB.nop && state.WB.wrt_enable &&
      state.WB.Wrt_reg_addr.to_ulong() != 0) {
    myRF.writeRF(state.WB.Wrt_reg_addr,
                 state.WB.Wrt_data);  // Write data to register
    // Debug: Write-back information
    std::cout << "WB Stage: Writing " << state.WB.Wrt_data << " to register x"
              << state.WB.Wrt_reg_addr.to_ulong() << std::endl;
  } else if (!state.WB.nop) {
    // Debug: WB stage inactive
    std::cout << "WB Stage: No write-back performed." << std::endl;
  }

  /* --------------------- MEM stage --------------------- */
  // Memory access stage
  if (!state.MEM.nop) {
    // Debug: MEM stage active
    std::cout << "MEM Stage: Processing..." << std::endl;
    if (state.MEM.rd_mem) {  // Load word (LW)
      // Read data from memory at the calculated address
      nextState.WB.Wrt_data = ext_dmem.readDataMem(state.MEM.ALUresult);

      nextState.WB.wrt_enable = true;
      nextState.WB.Wrt_reg_addr = state.MEM.Wrt_reg_addr;

      // Debug: Memory read
      std::cout << "MEM Stage: Loaded data " << nextState.WB.Wrt_data
                << " from address " << state.MEM.ALUresult.to_ulong()
                << std::endl;
    } else if (state.MEM.wrt_mem) {  // Store word (SW)
      // Write data to memory at the calculated address
      ext_dmem.writeDataMem(state.MEM.ALUresult, state.MEM.Store_data);
      nextState.WB.wrt_enable = false;

      // Debug: Memory write
      std::cout << "MEM Stage: Stored data " << state.MEM.Store_data
                << " to address " << state.MEM.ALUresult.to_ulong()
                << std::endl;
    } else {  // No memory access
      nextState.WB.Wrt_data = state.MEM.ALUresult;
      nextState.WB.wrt_enable = state.MEM.wrt_enable;
      nextState.WB.Wrt_reg_addr = state.MEM.Wrt_reg_addr;

      // Debug: Passing ALU result to WB stage
      std::cout << "MEM Stage: Passing ALU result " << state.MEM.ALUresult
                << " to WB stage." << std::endl;
    }
    nextState.WB.nop = false;
  } else {
    nextState.WB.nop = true;  // Propagate nop if MEM is stalled
    // Debug: MEM stage inactive
    std::cout << "MEM Stage: NOP." << std::endl;
  }

  /* --------------------- EX stage --------------------- */
  // Execution stage
  if (!state.EX.nop) {
    // Debug: EX stage active
    std::cout << "EX Stage: Processing..." << std::endl;
    // print alu op1
    std::cout << "EX Stage: alu_op1: " << state.EX.Read_data1 << std::endl;
    // print alu op2
    std::cout << "EX Stage: alu_op2: " << state.EX.Read_data2 << std::endl;
    uint32_t alu_op1 = state.EX.Read_data1.to_ulong();
    uint32_t alu_op2;
    // Sign-extend immediate
    int32_t imm = static_cast<int16_t>(state.EX.Imm.to_ulong());

    // Forwarding logic for operand 2
    if (state.EX.is_I_type) {
      alu_op2 = imm;
    } else {
      alu_op2 = state.EX.Read_data2.to_ulong();
    }

    // Perform ALU operation based on alu_op
    uint32_t alu_result = 0;
    switch (state.EX.alu_op) {
      case ALU_ADD:
        alu_result = alu_op1 + alu_op2;
        // Debug: ALU operation
        std::cout << "EX Stage: ALU_ADD " << alu_op1 << " + " << alu_op2
                  << " = " << alu_result << std::endl;
        break;
      case ALU_SUB:
        alu_result = alu_op1 - alu_op2;
        // Debug: ALU operation
        std::cout << "EX Stage: ALU_SUB " << alu_op1 << " - " << alu_op2
                  << " = " << alu_result << std::endl;
        break;
      case ALU_XOR:
        alu_result = alu_op1 ^ alu_op2;
        // Debug: ALU operation
        std::cout << "EX Stage: ALU_XOR " << alu_op1 << " ^ " << alu_op2
                  << " = " << alu_result << std::endl;
        break;
      case ALU_OR:
        alu_result = alu_op1 | alu_op2;
        // Debug: ALU operation
        std::cout << "EX Stage: ALU_OR " << alu_op1 << " | " << alu_op2 << " = "
                  << alu_result << std::endl;
        break;
      case ALU_AND:
        alu_result = alu_op1 & alu_op2;
        // Debug: ALU operation
        std::cout << "EX Stage: ALU_AND " << alu_op1 << " & " << alu_op2
                  << " = " << alu_result << std::endl;
        break;
      case ALU_NOP:
        // No operation; ALU result remains zero
        // Debug: ALU NOP operation
        std::cout << "EX Stage: ALU_NOP operation." << std::endl;
        break;
      default:
        std::cerr << "Unsupported ALU operation in EX stage" << std::endl;
        halted = true;
        nextState.MEM.nop = true;
        break;
    }
    nextState.MEM.ALUresult = std::bitset<32>(alu_result);

    // Propagate control signals to the MEM stage
    nextState.MEM.Store_data = state.EX.Read_data2;  // Data to store in SW
    nextState.MEM.Wrt_reg_addr =
        state.EX.Wrt_reg_addr;  // Destination register address
    nextState.MEM.rd_mem =
        state.EX.rd_mem;  // Control signal to read from memory (LW)
    nextState.MEM.wrt_mem =
        state.EX.wrt_mem;  // Control signal to write to memory (SW)
    nextState.MEM.wrt_enable =
        state.EX.wrt_enable;  // Control signal to write back to register file
    nextState.MEM.nop = false;
  } else {
    nextState.MEM.nop = true;  // Propagate nop if EX is stalled
    // Debug: EX stage inactive
    std::cout << "EX Stage: NOP." << std::endl;
  }

  /* --------------------- ID stage --------------------- */
  // Instruction decode stage
  bool branch_taken = false;
  bool pc_stall = false;
  if (!state.ID.nop) {
    uint32_t instr = state.ID.Instr.to_ulong();
    // Debug: ID stage active
    std::cout << "ID Stage: Decoding instruction " << state.ID.Instr
              << std::endl;

    uint32_t opcode = instr & 0x7F;
    uint32_t funct3 = (instr >> 12) & 0x07;
    uint32_t funct7 = (instr >> 25) & 0x7F;
    uint32_t rs1_addr = (instr >> 15) & 0x1F;
    uint32_t rs2_addr = (instr >> 20) & 0x1F;
    uint32_t rd_addr = (instr >> 7) & 0x1F;

    // Read registers with forwarding if needed
    std::bitset<32> rs1_data, rs2_data;

    // Forwarding for rs1
    if (rs1_addr != 0 && state.EX.wrt_enable &&
        state.EX.Wrt_reg_addr.to_ulong() == rs1_addr) {
      rs1_data = nextState.MEM.ALUresult;
      // Debug: EX-ID forwarding for rs1
      std::cout << "ID Stage: EX-ID Forwarding for rs1 (x" << rs1_addr << ")"
                << std::endl;
    } else if (rs1_addr != 0 && state.MEM.wrt_enable &&
               state.MEM.Wrt_reg_addr.to_ulong() == rs1_addr) {
      rs1_data = state.MEM.ALUresult;
      // Debug: MEM-ID forwarding for rs1
      std::cout << "ID Stage: MEM-ID Forwarding for rs1 (x" << rs1_addr << ")"
                << std::endl;
    } else {
      rs1_data = myRF.readRF(rs1_addr);
    }

    // Forwarding for rs2
    if (rs2_addr != 0 && state.EX.wrt_enable &&
        state.EX.Wrt_reg_addr.to_ulong() == rs2_addr) {
      rs2_data = nextState.MEM.ALUresult;
      // Debug: EX-ID forwarding for rs2
      std::cout << "ID Stage: EX-ID Forwarding for rs2 (x" << rs2_addr << ")"
                << std::endl;
    } else if (rs2_addr != 0 && state.MEM.wrt_enable &&
               state.MEM.Wrt_reg_addr.to_ulong() == rs2_addr) {
      rs2_data = state.MEM.ALUresult;
      // Debug: MEM-ID forwarding for rs2
      std::cout << "ID Stage: MEM-ID Forwarding for rs2 (x" << rs2_addr << ")"
                << std::endl;
    } else {
      rs2_data = myRF.readRF(rs2_addr);
    }

    // Hazard detection for Load-Use hazard
    if ((state.EX.rd_mem) && (!state.EX.nop) &&
        ((state.EX.Wrt_reg_addr.to_ulong() == rs1_addr && rs1_addr != 0) ||
         (state.EX.Wrt_reg_addr.to_ulong() == rs2_addr && rs2_addr != 0))) {
      // Stall the pipeline
      pc_stall = true;
      nextState.ID = state.ID;  // Keep the instruction in ID stage
      nextState.EX.nop = true;  // Insert NOP in EX stag
      std::cout << "ID Stage: Stalling due to Load-Use hazard." << std::endl;
    } else {
      // Prepare EX stage
      nextState.EX.Read_data1 = rs1_data;
      nextState.EX.Read_data2 = rs2_data;
      nextState.EX.Rs = rs1_addr;
      nextState.EX.Rt = rs2_addr;
      nextState.EX.Wrt_reg_addr = rd_addr;

      // Initialize control signals
      nextState.EX.rd_mem = false;
      nextState.EX.wrt_mem = false;
      nextState.EX.wrt_enable = false;
      nextState.EX.is_I_type = false;
      nextState.EX.Imm = 0;
      nextState.EX.nop = false;

      // Determine ALU operation
      nextState.EX.alu_op = getALUControl(opcode, funct3, funct7);

      if (opcode == 0b0110011) {  // R-type instruction
        nextState.EX.wrt_enable = true;
        // Debug: Decoded R-type instruction
        std::cout << "ID Stage: Decoded R-type instruction." << std::endl;
      } else if (opcode == 0b0010011) {  // I-type arithmetic instruction
        nextState.EX.is_I_type = true;
        nextState.EX.wrt_enable = true;
        // Extract immediate
        int32_t imm_i = ((int32_t)instr) >> 20;  // Sign-extended immediate
        nextState.EX.Imm = imm_i;
        // Debug: Decoded I-type instruction
        std::cout << "ID Stage: Decoded I-type instruction with immediate "
                  << imm_i << std::endl;
      } else if (opcode == 0b0000011) {  // Load instructions (e.g., LW)
        nextState.EX.is_I_type = true;
        nextState.EX.rd_mem = true;
        nextState.EX.wrt_enable = true;
        // Extract immediate
        int32_t imm_i = ((int32_t)instr) >> 20;
        nextState.EX.Imm = imm_i;
        // Debug: Decoded Load instruction
        std::cout << "ID Stage: Decoded Load instruction." << std::endl;
      } else if (opcode == 0b0100011) {  // Store instructions (e.g., SW)
        nextState.EX.is_I_type = true;
        nextState.EX.wrt_mem = true;
        nextState.EX.wrt_enable = false;
        // Extract immediate (S-type)
        int32_t imm_s =
            (((int32_t)(instr & 0xFE000000)) >> 20) | ((instr >> 7) & 0x1F);
        nextState.EX.Imm = imm_s;
        // Debug: Decoded Store instruction
        std::cout << "ID Stage: Decoded Store instruction." << std::endl;
      } else if (opcode == 0b1100011) {  // Branch instructions (e.g., BEQ, BNE)
        // Branch instructions are handled here
        int32_t imm_b =
            (((instr >> 31) & 0x1) << 12) | (((instr >> 25) & 0x3F) << 5) |
            (((instr >> 8) & 0xF) << 1) | (((instr >> 7) & 0x1) << 11);
        if (imm_b & 0x1000) {  // Sign-extend
          imm_b |= 0xFFFFE000;
        }
        // Branch offsets are in multiples of 2 (shifted left by 1)
        imm_b <<= 1;

        bool take_branch = false;
        if (funct3 == 0b000) {  // BEQ
          take_branch = (rs1_data == rs2_data);
        } else if (funct3 == 0b001) {  // BNE
          take_branch = (rs1_data != rs2_data);
        }

        if (take_branch) {
          nextState.IF.PC = std::bitset<32>(state.IF.PC.to_ulong() + imm_b);
          branch_taken = true;
          // Debug: Branch taken
          std::cout << "ID Stage: Branch taken. Updating PC to "
                    << nextState.IF.PC << std::endl;
          // Set nop for the next ID stage to flush the fetched instruction
          nextState.ID.nop = true;
        } else {
          // Debug: Branch not taken
          std::cout << "ID Stage: Branch not taken. PC remains "
                    << nextState.IF.PC << std::endl;
        }
        // Insert NOP in EX stage since branch instructions don't proceed to EX
        nextState.EX.nop = true;
      } else if (opcode == 0b1101111) {  // JAL instruction
        // Handle JAL instruction
        int32_t imm_j =
            (((instr >> 31) & 0x1) << 20) | (((instr >> 12) & 0xFF) << 12) |
            (((instr >> 20) & 0x1) << 11) | (((instr >> 21) & 0x3FF) << 1);
        if (imm_j & 0x100000) {
          imm_j |= 0xFFE00000;
        }

        // Write PC + 4 to rd
        nextState.WB.Wrt_data = std::bitset<32>(state.IF.PC.to_ulong() + 4);
        nextState.WB.Wrt_reg_addr = rd_addr;
        nextState.WB.wrt_enable = true;
        nextState.WB.nop = false;

        // Update PC to jump target
        nextState.IF.PC = std::bitset<32>(state.IF.PC.to_ulong() + imm_j);
        branch_taken = true;
        // Debug: JAL instruction
        std::cout << "ID Stage: JAL instruction. Jumping to " << nextState.IF.PC
                  << std::endl;

        // Set nop for the next ID stage to flush the fetched instruction
        nextState.ID.nop = true;
        // Insert NOP in EX stage since JAL doesn't proceed to EX
        nextState.EX.nop = true;
      } else {
        // Invalid or unsupported instruction
        std::cerr << "Unsupported instruction with opcode: " << opcode
                  << std::endl;
        nextState.EX.nop = true;
      }

      if (!branch_taken && !pc_stall) {
        // Update PC for next instruction (assume branch not taken)
        nextState.IF.PC = std::bitset<32>(state.IF.PC.to_ulong() + 4);
        nextState.ID.nop = false;
      } else if (pc_stall) {
        // Keep PC the same during stall
        nextState.IF.PC = state.IF.PC;
      }
    }
  } else {
    nextState.EX.nop = true;  // Propagate nop if ID is stalled
    // Debug: ID stage inactive
    std::cout << "ID Stage: NOP." << std::endl;
  }

  /* --------------------- IF stage --------------------- */
  // Instruction fetch stage
  if (!state.IF.nop) {
    if (pc_stall) {
      // Stall the IF stage; do not fetch a new instruction
      nextState.IF = state.IF;  // Keep IF stage the same
      nextState.ID = state.ID;  // Keep ID stage the same
      // Debug: IF stage stalled
      std::cout << "IF Stage: Stalled due to hazard." << std::endl;
    } else {
      nextState.ID.Instr = ext_imem.readInstr(state.IF.PC);

      // Check if we have hit an invalid instruction or halt condition
      if (nextState.ID.Instr ==
          std::bitset<32>("11111111111111111111111111111111")) {
        // Stop fetching and let pipeline drain
        nextState.IF.nop = true;
        nextState.ID.nop = true;  // Ensure the halt instruction doesn't proceed
        // Debug: Halt instruction
        std::cout << "IF Stage: HALT instruction encountered." << std::endl;
      } else {
        // Debug: Instruction fetched
        std::cout << "IF Stage: Fetched instruction " << nextState.ID.Instr
                  << " at PC " << state.IF.PC << std::endl;
        if (!branch_taken) {
          nextState.ID.nop = false;
        }
      }
    }
  } else {
    nextState.ID.nop = true;
    // Debug: IF stage inactive
    std::cout << "IF Stage: NOP." << std::endl;
  }

  // Check if the pipeline is done
  halted = (state.IF.nop && state.ID.nop && state.EX.nop && state.MEM.nop &&
            state.WB.nop && nextState.IF.nop && nextState.ID.nop &&
            nextState.EX.nop && nextState.MEM.nop && nextState.WB.nop);

  // Output the state and register file
  myRF.outputRF(cycle);
  printState(nextState, cycle);

  // Update the current state with nextState
  state = nextState;
  cycle++;
}

void FiveStageCore::printState(stateStruct state, int cycle) {
  std::ofstream printstate;
  if (cycle == 0)
    printstate.open(opFilePath, std::ios_base::trunc);
  else
    printstate.open(opFilePath, std::ios_base::app);

  if (printstate.is_open()) {
    printstate << "State after executing cycle:\t" << cycle << std::endl;

    printstate << "IF.PC:\t" << state.IF.PC.to_ulong() << std::endl;
    printstate << "IF.nop:\t" << state.IF.nop << std::endl;

    printstate << "ID.Instr:\t" << state.ID.Instr << std::endl;
    printstate << "ID.nop:\t" << state.ID.nop << std::endl;

    printstate << "EX.Read_data1:\t" << state.EX.Read_data1 << std::endl;
    printstate << "EX.Read_data2:\t" << state.EX.Read_data2 << std::endl;
    printstate << "EX.Imm:\t" << state.EX.Imm << std::endl;
    printstate << "EX.Rs:\t" << state.EX.Rs << std::endl;
    printstate << "EX.Rt:\t" << state.EX.Rt << std::endl;
    printstate << "EX.Wrt_reg_addr:\t" << state.EX.Wrt_reg_addr << std::endl;
    printstate << "EX.is_I_type:\t" << state.EX.is_I_type << std::endl;
    printstate << "EX.rd_mem:\t" << state.EX.rd_mem << std::endl;
    printstate << "EX.wrt_mem:\t" << state.EX.wrt_mem << std::endl;
    printstate << "EX.alu_op:\t" << state.EX.alu_op << std::endl;
    printstate << "EX.wrt_enable:\t" << state.EX.wrt_enable << std::endl;
    printstate << "EX.nop:\t" << state.EX.nop << std::endl;

    printstate << "MEM.ALUresult:\t" << state.MEM.ALUresult << std::endl;
    printstate << "MEM.Store_data:\t" << state.MEM.Store_data << std::endl;
    printstate << "MEM.Rs:\t" << state.MEM.Rs << std::endl;
    printstate << "MEM.Rt:\t" << state.MEM.Rt << std::endl;
    printstate << "MEM.Wrt_reg_addr:\t" << state.MEM.Wrt_reg_addr << std::endl;
    printstate << "MEM.rd_mem:\t" << state.MEM.rd_mem << std::endl;
    printstate << "MEM.wrt_mem:\t" << state.MEM.wrt_mem << std::endl;
    printstate << "MEM.wrt_enable:\t" << state.MEM.wrt_enable << std::endl;
    printstate << "MEM.nop:\t" << state.MEM.nop << std::endl;

    printstate << "WB.Wrt_data:\t" << state.WB.Wrt_data << std::endl;
    printstate << "WB.Rs:\t" << state.WB.Rs << std::endl;
    printstate << "WB.Rt:\t" << state.WB.Rt << std::endl;
    printstate << "WB.Wrt_reg_addr:\t" << state.WB.Wrt_reg_addr << std::endl;
    printstate << "WB.wrt_enable:\t" << state.WB.wrt_enable << std::endl;
    printstate << "WB.nop:\t" << state.WB.nop << std::endl;
  } else {
    std::cout << "Unable to open FS StateResult output file." << std::endl;
  }

  printstate.close();
}
