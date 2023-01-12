#ifndef RL_CONFIG
#define RL_CONFIG


#include <stdio.h>
#include <string>
#include <unistd.h>
#include <fstream>
#include "Tools/json.h"
#include <libgen.h>         // dirname

#define PATH_MAX 4096

#ifdef BUILD_MAC_FLAG
#include <mach-o/dyld.h>
#endif

#ifndef BUILD_MAC_FLAG
#include <linux/limits.h>
#endif 
#include <mutex>

#define PATH_MAX 4096

//derived from https://stackoverflow.com/questions/23943239/how-to-get-path-to-current-exe-file-on-linux
static std::string getConfigDirectory()
{
#ifndef BUILD_MAC_FLAG
  char result[PATH_MAX];
  ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
  const char *path;
  if (count != -1)
  {
      path = dirname(result);
  }
  std::string output(path);

  #ifndef BUILD_NAO_FLAG
  output = output + "/../../../../Config/rl_config.json";
  #endif
  #ifdef BUILD_NAO_FLAG
  output = output + "/rl_config.json";
  #endif
#endif
  
#ifdef BUILD_MAC_FLAG
    char buf [PATH_MAX];
    uint32_t bufsize = PATH_MAX;
    if(!_NSGetExecutablePath(buf, &bufsize))
      puts(buf);
  
    std::string output_temp(buf);
  
    std::string output(output_temp);
  
    for (int i = output_temp.length(); i >= 0; i--) {
      if (output[i] == '/')
        break;
      output.erase(i);
    }
    
    #ifndef BUILD_NAO_FLAG
    output = output + "../../../../../../../Config/rl_config.json";
    #endif
    #ifdef BUILD_NAO_FLAG
    output = output + "/rl_config.json";
    #endif
#endif
  
 std::cout << "Config Directory: " << output << std::endl;
  return output;
}


namespace RLConfig
{
 static std::string configPath = getConfigDirectory();
 static std::ifstream configFile(configPath);
 static json::value configData = json::parse(configFile);
 static std::string mode = to_string(configData["mode"]);
 static bool train_mode = to_bool(configData["train_mode"]);
 static bool deterministic = to_bool(configData["deterministic"]);
 static bool referee_enabled = to_bool(configData["referee_enabled"]);
 static int action_steps = std::stoi(to_string(configData["action_steps"]));
 static int episode_length = std::stoi(to_string(configData["episode_length"]));
 static int batch_size = std::stoi(to_string(configData["batch_size"]));
 static int epoch_count = std::stoi(to_string(configData["epoch_count"]));
 static int seed = std::stoi(to_string(configData["seed"]));
 static bool normalization = to_bool(configData["normalization"]);
 static bool debug_print = to_bool(configData["debug_print"]);
 static bool visualization_mode = to_bool(configData["visualization_mode"]);

 extern std::mutex resetLock;
 extern bool resetting;
}

#endif
