# Compiler and Flags
CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra 

# Target executable
TARGET = riscv_simulator

# Source files and Object files
SRCS = main.cpp Core.cpp DataMem.cpp InsMem.cpp RegisterFile.cpp
OBJS = $(SRCS:.cpp=.o)

# Build target executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS)

# Build object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up build files
.PHONY: clean
clean:
	rm -f $(OBJS) $(TARGET) ./input/FS_RFResult.txt ./input/SS_RFResult.txt ./input/StateResult_SS.txt ./input/FS_DMEMResult.txt ./input/SS_DMEMResult.txt ./input/StateResult_FS.txt
