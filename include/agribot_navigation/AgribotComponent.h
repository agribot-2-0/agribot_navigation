#pragma once

class AgribotComponent
{
public:
    virtual int configure() = 0;
    virtual bool startup() = 0;
    virtual bool update() = 0;
    virtual bool stop() = 0;
    virtual bool cleanup() = 0;
};