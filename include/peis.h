#ifndef PEIS_H
#define PEIS_H

#include <string>

extern "C"{
#include <peiskernel/peiskernel_mt.h>
#include <peiskernel/peiskernel.h>
}

class Peis
{
private:

public:
    enum class SubscribeBeforeReading
    {
        DoSubscribeBeforeReading,
	DoNotSubscribeBeforeReading
    };
    
    Peis(int argc, char **argv);
    
    // the callback function should be like "void callbackFunction(PeisTuple* t, void* arg)"
    PeisSubscriberHandle registerCallback(const int owner, const std::string& key, void *userData, PeisTupleCallback *callbackFunction);
    PeisSubscriberHandle registerCallbackAbstract(const int owner, const std::string& key, void* userData, PeisTupleCallback *callbackFunction);
    
    PeisSubscriberHandle subscribe(const int owner, const std::string& key);
    
    void unsubscribe(PeisSubscriberHandle h);
    
    void publish(const std::string key, const std::string value);
    
    void publishRemote(const int owner, const std::string key, const std::string value);
    
    std::string getTupleKey(const PeisTuple* t);
    
    // With Peis, you can only getTuple()s that you have subscribe()d beforehand. Using 
    // subscribeFirst, getTuple() can first subscribe(), then getTuple(), then unsubscribe().
    PeisTuple* getTuple(const int owner, const std::string& key, const SubscribeBeforeReading = SubscribeBeforeReading::DoNotSubscribeBeforeReading, const int timeoutMs = 1000);
    
    PeisTuple* cloneTuple(PeisTuple* t);
    void deleteTuple(PeisTuple* t);
    
    std::string toString(PeisTuple* t);
    
    int ownerId();
};

#endif