#ifndef DPHPC_UTILITY_H
#define DPHPC_UTILITY_H

#include <string>

namespace utility {
  std::string str_between(const std::string&, char, char);
  std::string str_between(const std::string&, const std::string&, const std::string&);
  std::string str_after(const std::string&, const std::string&);
}

#endif //DPHPC_UTILITY_H
