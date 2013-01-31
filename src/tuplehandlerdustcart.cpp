#include <tuplehandlerdustcart.h>
#include <boost/algorithm/string.hpp>

TupleHandlerDustCart::TupleHandlerDustCart(const std::string& robotName, Peis* peis) :
    mRobotName(robotName),
    mPeis(peis)
{
    ROS_INFO("TupleHandlerDustCart::TupleHandlerDustCart(): listening for Peis actions for robot %s.", mRobotName.c_str());
    
    mRosPublisher = mRosNodeHandle.advertise<std_msgs::String>("command", 10);
    
    mRosSubDustCartStatus = mRosNodeHandle.subscribe("/status", 10, &TupleHandlerDustCart::onRosDustCartStatusChange, this);
}

TupleHandlerDustCart::~TupleHandlerDustCart(void)
{
    // nothing allocated in c'tor, so we're happy here.
}

const std::string TupleHandlerDustCart::getPattern()
{
    return mRobotName + ".action.*.COMMAND";
}

bool TupleHandlerDustCart::processTuple(PeisTuple* t)
{
    ROS_INFO("TupleHandlerDustCart::processTuple(): tuple: %s", mPeis->toString(t).c_str());

    // the FQTK is e.g. condo.MoveTo.14.COMMAND...
    const std::string fullyQualifiedTupleKey = mPeis->getTupleKey(t);
    
    // split the FQTK into a "stringlist"
    std::vector<std::string> tupleNames;
    boost::split(tupleNames, fullyQualifiedTupleKey, boost::is_any_of("."));
    
    //ROS_INFO("TupleHandlerDustCart::matchesTupleKey(): robotName %s, robotName %s, MoveTo: %s, COMMAND: %s, ON: %s", mRobotName.c_str(), tupleNames[0].c_str(), tupleNames[1].c_str(), tupleNames[3].c_str(), t->data);
    
    if(
        tupleNames.size() != 4
        || tupleNames[0] != mRobotName
        || tupleNames[1] != "action"
        || tupleNames[3] != "COMMAND"
        || std::string(t->data) != "ON"
    )
    {
        ROS_INFO("TupleHandlerDustCart::matchesTupleKey(): useless tuple received, ignoring");
        return false;
    }

    // create a prefix to append to ("condo.MoveTo.13.")
    mLastCommand.tuplePrefix = tupleNames[0] + "." + tupleNames[1] + "." + tupleNames[2] + ".";
    
    // Now that we know we're supposed to move (COMMAND=ON), lets
    // ask Peis for the  PARAMETER Tuple (containing the coordinates)
    //ROS_INFO("TupleHandlerDustCart::processTuple(): getting parameters-tuple with owner %d and key %s", t->owner, (mLastCommand.tuplePrefix + "PARAMETERS").c_str());
    PeisTuple* tupleActionParameters = mPeis->getTuple(t->owner, mLastCommand.tuplePrefix + "PARAMETERS", Peis::SubscribeBeforeReading::DoSubscribeBeforeReading);
    
    if(!tupleActionParameters)
    {
	ROS_ERROR("TupleHandlerDustCart::processTuple(): tuple %sPARAMETERS doesn't exist (or has different owner than the COMMAND-tuple (owner %d)). This is a BUG!", mLastCommand.tuplePrefix.c_str(), t->owner);
	return false;
    }
    else
    {
        ROS_INFO("TupleHandlerDustCart::processTuple(): got parameters-tuple: %s", mPeis->toString(tupleActionParameters).c_str());
    }

    mLastCommand.action = std::string(tupleActionParameters->data);
    mLastCommand.tupleOwner = t->owner;
    
    std_msgs::String msg;
    msg.data = std::string(tupleActionParameters->data);
    ROS_INFO("TupleHandlerDustCart::processTuple(): sending ROS command to %s: %s", mRobotName.c_str(), msg.data.c_str());
    mRosPublisher.publish(msg);
    return true;
}

void TupleHandlerDustCart::onRosDustCartStatusChange(const std_msgs::String::ConstPtr& msg)
{
    //ROS_INFO("PeisRos::onRosDustCartStatusChange(): status: %s", msg->data.c_str());
    
    // We're called with a status, e.g. "bin open", "bin close"
    if(msg->data == "bin is open" && mLastCommand.action == std::string("bin open"))
    {
	ROS_INFO("PeisRos::onRosDustCartStatusChange(): bin was opened successfully, setting %s -> COMPLETED", (mLastCommand.tuplePrefix + "STATE").c_str());
        mPeis->publishRemote(mLastCommand.tupleOwner, mLastCommand.tuplePrefix + "STATE", "COMPLETED");
	mLastCommand = LastCommand();
    }
    else if(msg->data == "bin is closed" && mLastCommand.action == std::string("bin close"))
    {
	ROS_INFO("PeisRos::onRosDustCartStatusChange(): bin was closed successfully, setting %s -> COMPLETED", (mLastCommand.tuplePrefix + "STATE").c_str());
        mPeis->publishRemote(mLastCommand.tupleOwner, mLastCommand.tuplePrefix + "STATE", "COMPLETED");
	mLastCommand = LastCommand();
    }
    else
    {
        //ROS_INFO("PeisRos::onRosDustCartStatusChange(): ignored ROS-status %s, waiting for answer to action %s", msg->data.c_str(), mLastCommand.action.c_str());
    }
}
