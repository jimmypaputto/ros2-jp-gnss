/*
 * Jimmy Paputto 2026
 */

#ifndef JP_GNSS_HAT_TOPICS_HPP_
#define JP_GNSS_HAT_TOPICS_HPP_

#include <string>

namespace JimmyPaputto
{

struct Topics
{
    std::string nodeName;
    std::string navigation;
    std::string navSatFix;
    std::string velocity;
    std::string timeReference;
    std::string rtkCorrections;
    std::string geofencingCfg;
    std::string getConfig;
    std::string setConfig;

    explicit Topics(const std::string& nodeId = "")
    {
        const auto gnss = nodeId.empty()
            ? std::string("/gnss")
            : "/gnss/" + nodeId;

        nodeName       = nodeId.empty() ? "jp_gnss_hat" : "jp_gnss_hat_" + nodeId;
        navigation     = gnss + "/navigation";
        navSatFix      = "/gps/fix";
        velocity       = "/gps/vel";
        timeReference  = "/gps/time_reference";
        geofencingCfg  = gnss + "/geofencing_cfg";
        getConfig      = gnss + "/get_config";
        setConfig      = gnss + "/set_config";

        rtkCorrections = "/gnss/rtk/corrections";
    }
};

}  // JimmyPaputto

#endif  // JP_GNSS_HAT_TOPICS_HPP_
