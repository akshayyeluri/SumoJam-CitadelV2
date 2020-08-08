#pragma once

class State
{
public:
    virtual void setup() { }
    virtual void loop() = 0;
};
