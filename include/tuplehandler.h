#ifndef TUPLEHANDLER_H
#define TUPLEHANDLER_H

#include <string>

class PeisTuple;

// Every class that handles incoming PeisTuples must derive from TupleHandler. This
// makes sure that all virtual methods below are implemented.

class TupleHandler
{
public:
    // Return the PeisTuple-Key-Pattern you're interested in processing. For example, the MoveBaseActionClient
    // is interested in processing "Dustcart1.MoveTo.*.COMMAND". When you instantiate and register your
    // TupleHandler in PeisRos::registerTupleHandler(), a callback for PeisTuples matching your getPattern()
    // will be created and matching PeisTuples will be processed by your processTuple()-method (see below)
    virtual const std::string getPattern() = 0;
    
    // processTuple() will be called with incoming PeisTuples with keys matching your getPattern().
    virtual bool processTuple(PeisTuple* t) = 0;
};

#endif