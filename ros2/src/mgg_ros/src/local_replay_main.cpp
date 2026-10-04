#include "mgg_ros/local_replay.h"

#include <iostream>
#include <map>
#include <set>
#include <string>

int main(int argc, char** argv) {
  std::map<std::string, std::string> options;
  const std::set<std::string> names{
      "--manifest", "--component", "--output", "--local-params"};
  for (int i = 1; i < argc; i += 2) {
    if (!names.count(argv[i]) || i + 1 == argc ||
        !options.emplace(argv[i], argv[i + 1]).second) {
      std::cerr << "usage: mgg_local_replay --manifest FILE --component local "
                   "--output FILE --local-params FILE\n";
      return 2;
    }
  }
  if (options["--component"] != "local" || options["--manifest"].empty() ||
      options["--output"].empty()) {
    std::cerr << "mgg_local_replay requires --manifest FILE --component local --output FILE\n";
    return 2;
  }
  return mgg::runLocalReplay(options["--manifest"], options["--output"],
                             options["--local-params"]);
}
