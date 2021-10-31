#include "utility.h"

std::string utility::str_between(const std::string& str, char pre, char post) {
  std::string s = str.substr(str.find(pre) + 1);
  return s.substr(0, s.find(post));
}

std::string utility::str_between(const std::string& str, const std::string& pre, const std::string& post) {
  std::string s = str.substr(str.find(pre) + pre.length());
  return s.substr(0, s.find(post));
}

std::string utility::str_after(const std::string& str, const std::string& pre) {
  return str.substr(str.find(pre) + pre.length());
}