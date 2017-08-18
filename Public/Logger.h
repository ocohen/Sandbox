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
    void log(LoggerKey key, LoggerKey subKey, float val)
    {
        auto it = allEntries.find(key);
        if(it == allEntries.end())
        {
            allEntries.insert(std::pair<LoggerKey, LoggingObject>(key, LoggingObject()));
            it = allEntries.find(key);
        }
        
        LoggingObject& logObj = it->second;
        std::vector<Entry>* entries = nullptr;
        for(SubKeyEntries& subKeyEntry : logObj.subKeyEntries)
        {
            if(subKeyEntry.key == subKey)
            {
                entries = &subKeyEntry.entries;
                break;
            }
        }

        if(entries == nullptr)
        {
            logObj.subKeyEntries.push_back({subKey, std::vector<Entry>()});
            entries = &logObj.subKeyEntries[logObj.subKeyEntries.size()-1].entries;
        }


        entries->push_back({curFrame, val});
    }

    void dumpLogs()
    {

        
        for(auto it = allEntries.begin(); it != allEntries.end(); ++it)
        {
            std::ostringstream stringStream;
            LoggingObject& obj = it->second;

            stringStream << std::endl;
            stringStream << std::endl << "#" << it->first;
            for (const SubKeyEntries& entries : obj.subKeyEntries)
            {
                stringStream << "\t" << entries.key;
            }
            stringStream << std::endl;

            int curRow = 0;
            int curFrame = 0;
            bool bProcessObj = true;
            while(bProcessObj)
            {
                bool bFirstEntry = true;
                for (const SubKeyEntries& subEntries : obj.subKeyEntries)
                {
                    if(curRow < subEntries.entries.size())
                    {
                        if(subEntries.entries[curRow].frame != curFrame)
                        {
                            curFrame++;
                            break;
                        }
                        else
                        {
                            if(bFirstEntry)
                            {
                                bFirstEntry = false;
                                stringStream << std::endl << curFrame;
                            }

                            stringStream << "\t" << subEntries.entries[curRow].val;
                        }
                    }
                    else
                    {
                        bProcessObj = false;
                        OutputDebugStringA(stringStream.str().c_str());

                        break;
                    }
                }

                if(!bFirstEntry)    //don't skip row if we haven't written any value
                {
                    ++curRow;
                }
                
            }

        }
        

        allEntries.empty();
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

    struct SubKeyEntries
    {
       LoggerKey key;
       std::vector<Entry> entries;
    };

    //We create one for every object we collect logs for
    struct LoggingObject
    {
        //for each sub key we have an array of entries
        std::vector<SubKeyEntries> subKeyEntries;    //we want to make sure keys are always iterated over in the same order
    };

    std::map<LoggerKey, LoggingObject> allEntries;
    int curFrame;
};

#endif
