#include "DataMem.h"

DataMem::DataMem(std::string name, std::string ioDir) : id{name}, ioDir{ioDir} {
  DMem.resize(MemSize);
  opFilePath = ioDir + separator + name + "_DMEMResult.txt";
  std::ifstream dmemFile(ioDir + separator + "dmem.txt");
  std::string line;
  unsigned long i = 0;

  if (dmemFile.is_open()) {
    std::cout << "DMEM file opened successfully at "
              << ioDir + separator + "dmem.txt" << std::endl;

    while (std::getline(dmemFile, line)) {
      // Remove any leading/trailing whitespace
      line.erase(line.begin(),
                 std::find_if_not(line.begin(), line.end(),
                                  [](int ch) { return std::isspace(ch); }));
      line.erase(std::find_if_not(line.rbegin(), line.rend(),
                                  [](int ch) { return std::isspace(ch); })
                     .base(),
                 line.end());

      // Only process non-empty lines
      if (!line.empty()) {
        // Ensure the line has at least 8 characters
        if (line.length() >= 8) {
          line = line.substr(0, 8);
        } else {
          std::cerr << "Invalid line length at line " << i + 1 << ": '" << line
                    << "'" << std::endl;
          continue;  // Skip to the next line
        }

        // Check for invalid characters (only '0's and '1's)
        if (line.find_first_not_of("01") != std::string::npos) {
          std::cerr << "Invalid characters found in line " << i + 1 << ": '"
                    << line << "'" << std::endl;
          continue;  // Skip to the next line
        }

        // Ensure index i is within bounds
        if (i < DMem.size()) {
          DMem[i] = std::bitset<8>(line);
          // Debug: Output the value stored
          std::cout << "DMem[" << i << "] = " << DMem[i] << std::endl;
          i++;
        } else {
          std::cerr << "DMem index out of bounds at line " << i + 1
                    << std::endl;
          break;
        }
      }
    }
  } else {
    std::cerr << "Unable to open DMEM input file at " << ioDir + "/dmem.txt"
              << std::endl;
  }
  dmemFile.close();
}

std::bitset<32> DataMem::readDataMem(std::bitset<32> Address) {
  unsigned long baseAddress = Address.to_ulong();
  std::string data = "";

  // Check for address out of bounds
  if (baseAddress + 3 >= DMem.size()) {
    std::cerr << "Memory read access violation at address " << baseAddress
              << std::endl;
    return std::bitset<32>(0);  // Return zero or handle error as needed
  }

  // Read bytes in big-endian order (most significant byte first)
  for (int i = 0; i < 4; i++) {
    data += DMem[baseAddress + i].to_string();
  }

  // Debug: Output the data being read
  std::cout << "Data read from address " << baseAddress << ": " << data
            << std::endl;

  return std::bitset<32>(data);
}

void DataMem::writeDataMem(std::bitset<32> Address, std::bitset<32> WriteData) {
  unsigned long baseAddress = Address.to_ulong();
  std::string dataStr = WriteData.to_string();

  // Check for address out of bounds
  if (baseAddress + 3 >= DMem.size()) {
    std::cerr << "Memory write access violation at address " << baseAddress
              << std::endl;
    return;  // Handle error as needed
  }

  // Write bytes in big-endian order (most significant byte first)
  for (int i = 0; i < 4; i++) {
    std::string byteStr = dataStr.substr(i * 8, 8);
    DMem[baseAddress + i] = std::bitset<8>(byteStr);

    // Debug: Output the data being written
    std::cout << "Writing byte to address " << (baseAddress + i) << ": "
              << DMem[baseAddress + i] << std::endl;
  }
}

void DataMem::outputDataMem() {
  std::ofstream dmemout(opFilePath, std::ios_base::trunc);
  if (dmemout.is_open()) {
    for (size_t j = 0; j < DMem.size(); j++) {
      dmemout << DMem[j] << std::endl;
    }
    dmemout.close();
  } else {
    std::cerr << "Unable to open " << id << " DMEM result file at "
              << opFilePath << std::endl;
  }
}

// function to print the data memory
void DataMem::printDataMem() {
  for (int j = 0; j < MemSize; j++) {
    std::cout << DMem[j] << std::endl;
  }
}
