#include "Event.h"

// basic constructors
Event::Event(Type t, std::vector<int32_t> p) : type(t), params(p) {}
Event::Event() {}
