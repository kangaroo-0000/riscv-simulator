# RISC-V Simulator

This project is a RISC-V simulator that can be compiled and executed to simulate the behavior of a RISC-V processor.

## Getting Started

### Prerequisites

Make sure you have `g++` installed on your system. You can check this by running:

```sh
g++ --version
```

### Building the Project
To compile the executable, simply run:
```sh
make
```

This will generate an executable called riscv_simulator.

### Running the Simulator
To execute the simulator, run:
```sh
./riscv_simulator
```

### Cleaning Up
To clean up the build files, run:
```sh
make clean
```
This will remove the object files, the executable and result files in input/.

### Project File Structure
The submissions/ folder contains solutions to the homework Tasks. The schematic.pdf contains schematics of both the single stage and five stage processor. Please refer to this file for Tasks (1) and (2). The PerformaceMetrics_Result.txt contains a comparison report in terms of average CPI, total execution cycles, and instructions per cycle for both these cores. Please refer to this file for Tasks (3) and (4). 
