#include <peis.h>
#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>

Peis::Peis(int argc, char **argv)
{
    // Initialize PEIS
    peiskmt_initialize(&argc, argv);
}

PeisSubscriberHandle Peis::registerCallback(const int owner,const std::string& key, void *userData, PeisTupleCallback *callbackFunction)
{
    return peiskmt_registerTupleCallback(owner, key.c_str(), userData, callbackFunction);
}

PeisSubscriberHandle Peis::registerCallbackAbstract(const int owner, const std::string& key, void* userData, PeisTupleCallback *callbackFunction)
{
    PeisTuple proto;
    // prototype declaration
    peiskmt_initAbstractTuple(&proto);
    // we have to initialize the prototype
    proto.owner = owner;
    // we don’t know who is the creator, therefore we put -1
    peiskmt_setTupleName(&proto,key.c_str());
    // we provide the pattern we are looking for
    peiskmt_subscribeByAbstract(&proto); // subscription to the blackboard about this prototype
    //callback registration, we are not passing any initial argument
    return peiskmt_registerTupleCallbackByAbstract(&proto, userData, callbackFunction);
}

PeisSubscriberHandle Peis::subscribe(const int owner, const std::string& key)
{
    return peiskmt_subscribe(owner, key.c_str());
}

void Peis::unsubscribe(PeisSubscriberHandle h)
{
    peiskmt_unsubscribe(h);
}
    
void Peis::publish(const std::string key, const std::string value)
{
    peiskmt_setStringTuple(key.c_str(), value.c_str());
}
    
void Peis::publishRemote(const int owner, const std::string key, const std::string value)
{
    peiskmt_setRemoteStringTuple(owner, key.c_str(), value.c_str());
}
    
int Peis::ownerId()
{
    return peiskmt_peisid();
}

std::string Peis::getTupleKey(const PeisTuple* t)
{
    char buffer[500];
    peiskmt_getTupleName((PeisTuple*)t, buffer, 500);
    return std::string(buffer);
}

PeisTuple* Peis::getTuple(const int owner, const std::string& key, const Peis::SubscribeBeforeReading sbr, const int timeoutMs)
{
    //std::cout << "Peis::getTuple(): getting tuple: owner " << owner  << " key " << key << std::endl;

    // If we used the PEISK_BLOCKING flag, PEIS waits for the tuple to arrive. Unfortunately,
    // there's no timeout, so we could hang here forever. So, we build a timeout-wrapper.
    const boost::posix_time::ptime timeStart = boost::posix_time::microsec_clock::local_time();
    
    // First subscribe to the PeisTuple if told to do so.
    PeisSubscriberHandle h;
    if(sbr == Peis::SubscribeBeforeReading::DoSubscribeBeforeReading)
        h = subscribe(owner, key);
    
    PeisTuple* t = 0;
    
    do
    {
        //peisk_waitOneCycle(1000);
	//peisk_step();
	usleep(20000);
        t = peiskmt_getTuple(owner, key.c_str(), 0);
    } while(t == 0 && (boost::posix_time::microsec_clock::local_time() - timeStart).total_milliseconds() < timeoutMs);
    
    if(t == 0)
        std::cout << "Peis::getTuple(): got no tuple from owner " << owner << " after " << (boost::posix_time::microsec_clock::local_time() - timeStart).total_milliseconds() << " ms" << std::endl;
    
    if(sbr == Peis::SubscribeBeforeReading::DoSubscribeBeforeReading)
        unsubscribe(h);

    return t;
}

std::string Peis::toString(PeisTuple* t)
{
    char buffer[512];
    peiskmt_snprintTuple(buffer, 512, t);
    return std::string(buffer);
}

PeisTuple* Peis::cloneTuple(PeisTuple* t)
{
    return peiskmt_cloneTuple(t);
}

void Peis::deleteTuple(PeisTuple* t)
{
    peiskmt_freeTuple(t);
}
