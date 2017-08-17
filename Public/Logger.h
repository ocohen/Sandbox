#ifndef OC_LOGGER_H
#define OC_LOGGER_H

#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <sstream>

typedef std::string LoggerKey;

class Logger
{
public:
    void log(LoggerKey key, float val)
    {
        auto it = allEntries.find(key);
        if(it == allEntries.end())
        {
            allEntries.insert(std::pair<LoggerKey, std::vector<Entry>>(key, std::vector<Entry>()));
            it = allEntries.find(key);
        }
        
        std::vector<Entry>& entries = it->second;

        entries.push_back({curFrame, val});
    }

    void dumpLogs()
    {

        std::ostringstream stringStream;
        
        for(auto it = allEntries.begin(); it != allEntries.end(); ++it)
        {
            stringStream << it->first << std::endl;
            for(const Entry& entry : it->second)
            {
                stringStream << entry.frame << "," << entry.val << std::endl;
            }
            stringStream << std::endl;

            allEntries.empty();
        }
        OutputDebugStringA(stringStream.str().c_str());
    }

    void advance()
    {
        ++curFrame;
    }

    Logger()
    : curFrame(0)
    {
    }

private:
    struct Entry
    {
        int frame;
        float val;
    };

    std::map<LoggerKey, std::vector<Entry>> allEntries;
    int curFrame;
};

#endif
