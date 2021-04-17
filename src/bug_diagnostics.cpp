#include <bug_diagnostics/bug_diagnostics.h>

namespace bug_diagnostics{
    BugDiagnostics::BugDiagnostics() : BugDiagnostics("Bug Catcher"){
    }

    BugDiagnostics::BugDiagnostics(std::string name) : name_(name){
        ros::NodeHandle nh;
        diagnostics.add(name_, this, &BugDiagnostics::updateDiag);
        diagnostics.setHardwareID("none");
        enabled_.store(true);
        enable_stat_time_.store(true);
        initialized_ = true;
    }

    template<> void BugDiagnostics::addStat(std::string key, std::string value){
        std::lock_guard<std::mutex> lg(data_mutex);

        if(enabled_.load()){
            if(initialized_){
                string_pair pair_value;
                pair_value.first = std::to_string(ros::Time::now().toSec());
                pair_value.second = std::string(value);
                if(stat_.find(key) == stat_.end())
                    stat_.insert(std::pair<std::string, string_pair>(key, pair_value));

                else
                    stat_[key] = pair_value;
            } else{
                ROS_ERROR_STREAM("BugDiagnostics is not initalized properly");
            }
        }
    }

    template<> void BugDiagnostics::addStat(std::string key, double value){
        // std::lock_guard<std::mutex> lg(data_mutex);
        std::string str_value = std::to_string(value);
        addStat(key, str_value);
    }

    template<> void BugDiagnostics::addStat(std::string key, const char value[]){
        // std::lock_guard<std::mutex> lg(data_mutex);
        std::string str_value = std::string(value);
        addStat(key, str_value);
    }

    template<> void BugDiagnostics::getStat(std::string key, std::string &out){
        std::lock_guard<std::mutex> lg(data_mutex);
        if(stat_.find(key) == stat_.end()){
            ROS_WARN_STREAM("The key has not been added");
            out = "";
        } else out = stat_[key].second;
        
    }

    template<> void BugDiagnostics::getStat(std::string key, double &out){
        std::string result;
        getStat(key, result);

        out = std::atof(result.c_str());
    }

    void BugDiagnostics::updateDiag(diagnostic_updater::DiagnosticStatusWrapper& stat){
        std::lock_guard<std::mutex> lg(data_mutex);
        std::map<std::string, string_pair>::iterator it;
        
        auto time = std::chrono::high_resolution_clock::now();
        double this_update = time.time_since_epoch().count();
        update_interval = (this_update - last_update)/1e+09;
        last_update = this_update;

        stat.add("Update interval", update_interval);
        for(it = stat_.begin(); it != stat_.end(); it++){
            if(enable_stat_time_)
                stat.add(it->first, it->second.first + " - " + it->second.second);
            else
                stat.add(it->first, it->second.second);
        }

        stat.summary(state, summary_msg_);
    }

    void BugDiagnostics::publish(){
        if(enabled_)
            diagnostics.force_update();
    }

    void BugDiagnostics::updateSummary(unsigned char lvl, std::string summary_msg){
        std::lock_guard<std::mutex> lg(data_mutex);

        switch(lvl){
            case(diagnostic_msgs::DiagnosticStatus::OK):
                state = lvl;
                break;
            case(diagnostic_msgs::DiagnosticStatus::WARN):
                state = lvl;
                break;
            case(diagnostic_msgs::DiagnosticStatus::ERROR):
                state = lvl;
                break;
        }

        summary_msg_ = summary_msg;
    }
}