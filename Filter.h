#ifndef FILTER_H
#define FILTER_H

template <typename T>
class EMAFilter {
public:
    EMAFilter(float alpha) : _alpha(alpha), _initialized(false), _filteredValue(0) {}

    void update(T newValue) {
        if (!_initialized) {
            _filteredValue = (float)newValue;
            _initialized = true;
        } else {
            _filteredValue = (_alpha * (float)newValue) + ((1.0f - _alpha) * _filteredValue);
        }
    }

    T value() const {
        return (T)_filteredValue;
    }

    void reset() {
        _initialized = false;
    }

private:
    float _alpha;
    bool _initialized;
    float _filteredValue;
};

#endif
