#ifndef BUG_CATCHER
#define BUG_CATCHER

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <map>
#include <mutex>
#include <chrono>
#include <atomic>

namespace bug_diagnostics{
    class BugDiagnostics{
        public:
            BugDiagnostics();
            BugDiagnostics(std::string name);
            ~BugDiagnostics(){}
            
           /**
            * @brief Add a status to diagnostics, if status exists, then it will be updated
            * @param key name of the status
            * @param value value of the status, only support std::string and double
            */
            template<typename value_type>
            void addStat(std::string key, value_type value);

           /**
            * @brief Get status' value, if status does not exist, will return none
            * @param key name of the status
            * @param out the variable to pass the value to, only support std::string and double
            */  
            template<typename value_type>
            void getStat(std::string key, value_type &out);


            void updateDiag(diagnostic_updater::DiagnosticStatusWrapper& stat);

            void publish();

           /**
            * @brief Enable the diagnostics to be published
            */  
            void enable(){enabled_.store(true);}

           /**
            * @brief Disable the diagnostics to be published
            */ 
            void disable(){enabled_.store(false);}

           /**
            * @brief Enable the each stat's update time to be published
            */ 
            void enableStatusTimestamp(){enable_stat_time_.store(true);}
           
           /**
            * @brief Disable the each stat's update time to be published
            */ 
            void disableStatusTimestamp(){enable_stat_time_.store(false);}

           /**
            * @brief Update summary, should be update every publish cycle
            * @param lvl diagnostics status, please check diagnostics_msgs::DiagnosticsStatus
            * @param summary_msg summary message
            */ 
            void updateSummary(unsigned char lvl, std::string summary_msg);

        private:
            const std::string name_;
            std::mutex data_mutex;

            typedef std::pair<std::string, std::string> string_pair;
            
            std::map<std::string, string_pair> stat_;
            
            diagnostic_updater::Updater diagnostics;

            unsigned char state = diagnostic_msgs::DiagnosticStatus::OK;

            std::string summary_msg_ = "OK";

            double update_interval;
            double last_update;
            bool initialized_;

            std::atomic<bool> enabled_;
            std::atomic<bool> enable_stat_time_;


    };

    template<> void BugDiagnostics::addStat(std::string key, std::string value);
    template<> void BugDiagnostics::addStat(std::string key, double value);
    template<> void BugDiagnostics::addStat(std::string key, const char value[]);
    
    template<> void BugDiagnostics::getStat(std::string key, std::string &out);
    template<> void BugDiagnostics::getStat(std::string key, double &out);
    // template<> void BugDiagnostics::getStat(std::string key, int out);
    // template<> void BugDiagnostics::getStat(std::string key, float out);
}



#endif