#include <filesystem>
#include "utils/url_resolver.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <iostream>

namespace utils
{
    std::string URLResolver::getPackageFileName(const std::string &url)
    {
        // Scan URL from after "package://" until next '/' and extract
        // package name.  The parseURL() already checked that it's present.
        size_t prefix_len = std::string("package://").length();
        size_t rest = url.find('/', prefix_len);
        std::string package(url.substr(prefix_len, rest - prefix_len));

        // Look up the ROS package path name.
        std::string pkg_path = ament_index_cpp::get_package_share_directory(package);
        if (pkg_path.empty())
        { // package not found?
            return pkg_path;
        }
        else
        {
            // Construct file name from package location and remainder of URL.
            return pkg_path + url.substr(rest);
        }
    }
    std::vector<std::string> URLResolver::getClassNameFromMetadata(const std::string &metadata_path)
    {
        std::ifstream check_file(metadata_path);

        if (!check_file.is_open())
        {
            std::cerr << "Unable to open file: " << metadata_path << std::endl;
            return {};
        }

        check_file.close();

        YAML::Node metadata = YAML::LoadFile(metadata_path);
        std::vector<std::string> class_names;

        if (!metadata["names"])
        {
            std::cerr << "ERROR: 'names' node not found in the YAML file" << std::endl;
            return {};
        }

        for (size_t i = 0; i < metadata["names"].size(); ++i)
        {
            std::string class_name = metadata["names"][std::to_string(i)].as<std::string>();
            class_names.push_back(class_name);
        }

        return class_names;
    }
    
} // namespace utils
