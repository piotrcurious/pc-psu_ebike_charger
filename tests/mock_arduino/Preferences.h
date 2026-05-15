#ifndef PREFERENCES_H
#define PREFERENCES_H
#include <stdint.h>
#include <string>
#include <map>
class Preferences {
public:
    bool begin(const char* name, bool readOnly) { return true; }
    void putFloat(const char* key, float val) { _data[key] = val; }
    float getFloat(const char* key, float defaultVal) {
        if (_data.find(key) != _data.end()) return _data[key];
        return defaultVal;
    }
    void end() {}
private:
    std::map<std::string, float> _data;
};
#endif
