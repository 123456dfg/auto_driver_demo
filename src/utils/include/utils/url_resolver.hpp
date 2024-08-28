#ifndef _URL_RESOLVER_H_
#define _URL_RESOLVER_H_

#include <filesystem>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <random>

namespace utils
{
    class URLResolver
    {
    private:

    public:
        static std::vector<std::string> getClassNameFromMetadata(const std::string &metadata_path);
        static std::string getPackageFileName(const std::string &url);
        URLResolver()=delete;
    };
    
} // namespace utils


#endif // _URL_RESOLVER_H_
