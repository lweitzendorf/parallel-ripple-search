#ifndef DPHPC_FILEPARSER_H
#define DPHPC_FILEPARSER_H

#include "graph/WeightedGraph.h"
#include <string>

class FileParser {
protected:
  std::string file_path;
  static std::string str_between(const std::string &, char, char);
  static std::string str_between(const std::string &, const std::string &,
                                 const std::string &);
  static std::string str_after(const std::string &, const std::string &);

public:
  explicit FileParser(std::string);
  virtual bool build_graph(WeightedGraph &) = 0;
};

class BitMapParser : FileParser {
public:
  explicit BitMapParser(std::string file_path)
      : FileParser(std::move(file_path)){};
  bool build_graph(WeightedGraph &) override;
};

class DotParser : FileParser {
public:
  explicit DotParser(std::string file_path)
      : FileParser(std::move(file_path)){};
  bool build_graph(WeightedGraph &) override;
};

#endif // DPHPC_FILEPARSER_H
