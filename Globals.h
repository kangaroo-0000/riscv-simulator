#ifndef GLOBALS_H
#define GLOBALS_H
#include <string>

#ifdef _WIN32
const std::string separator = "\\";
#else
const std::string separator = "/";
#endif

#endif  // GLOBALS_H