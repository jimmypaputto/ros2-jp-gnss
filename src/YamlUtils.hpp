/*
 * Jimmy Paputto 2026
 */

#ifndef JP_GNSS_HAT_YAML_UTILS_HPP_
#define JP_GNSS_HAT_YAML_UTILS_HPP_

#include <optional>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>


namespace JimmyPaputto::yaml
{

inline std::string joinKey(const std::string& prefix, const std::string& key)
{
    return prefix.empty() ? key : prefix + "." + key;
}

[[noreturn]] inline void fail(const std::string& key, const std::string& what)
{
    throw std::runtime_error("Invalid config key '" + key + "': " + what);
}

// Absent key -> nullopt. A present key must hold a mapping: a null value
// (e.g. all children commented out) is rejected instead of silently ignored.
inline std::optional<YAML::Node> optionalMap(
    const YAML::Node& parent, const std::string& key, const std::string& path)
{
    const YAML::Node node = parent[key];
    if (!node.IsDefined())
        return std::nullopt;
    if (!node.IsMap())
        fail(path, "expected a mapping (remove the key to omit the section)");
    return node;
}

template <typename T>
std::optional<T> optionalValue(
    const YAML::Node& parent, const std::string& key, const std::string& path)
{
    const YAML::Node node = parent[key];
    if (!node.IsDefined() || node.IsNull())
        return std::nullopt;
    if (!node.IsScalar())
        fail(path, "expected a scalar value");
    try
    {
        return node.as<T>();
    }
    catch (const YAML::Exception&)
    {
        fail(path, "cannot convert '" + node.Scalar() + "'");
    }
}

template <typename T>
T value(const YAML::Node& parent, const std::string& key,
    const std::string& path, const T& fallback)
{
    return optionalValue<T>(parent, key, path).value_or(fallback);
}

template <typename T>
T requiredValue(
    const YAML::Node& parent, const std::string& key, const std::string& path)
{
    const auto result = optionalValue<T>(parent, key, path);
    if (!result)
        fail(path, "required key is missing");
    return *result;
}

}  // JimmyPaputto::yaml

#endif  // JP_GNSS_HAT_YAML_UTILS_HPP_
