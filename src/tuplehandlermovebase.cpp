#include <tuplehandlermovebase.h>
#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>

#include "cognidrive_ros/LoadMap.h"

TupleHandlerMoveBase::TupleHandlerMoveBase(const std::string& robotName, Peis* peis) :
    mRobotName(robotName),
    mActionClientRos("move_base", true),
    mPeis(peis)
{
    ROS_INFO("TupleHandlerMoveBase::TupleHandlerMoveBase(): started action client, listening for Peis driving tasks for robot %s.", mRobotName.c_str());
}

TupleHandlerMoveBase::~TupleHandlerMoveBase(void)
{
    // nothing allocated in c'tor, so we're happy here.
}

const std::string TupleHandlerMoveBase::getPattern()
{
    return mRobotName + ".MoveTo.*.COMMAND";
}

bool TupleHandlerMoveBase::processTuple(PeisTuple* t)
{
    ROS_INFO("TupleHandlerMoveBase::processTuple(): tuple: %s", mPeis->toString(t).c_str());

    // the FQTK is e.g. condo.MoveTo.14.COMMAND...
    const std::string fullyQualifiedTupleKey = mPeis->getTupleKey(t);
    
    // split the FQTK into a "stringlist"
    std::vector<std::string> tupleNames;
    boost::split(tupleNames, fullyQualifiedTupleKey, boost::is_any_of("."));
    
    //ROS_INFO("TupleHandlerMoveBase::matchesTupleKey(): robotName %s, robotName %s, MoveTo: %s, COMMAND: %s, ON: %s", mRobotName.c_str(), tupleNames[0].c_str(), tupleNames[1].c_str(), tupleNames[3].c_str(), t->data);
    
    if(
        tupleNames.size() != 4
        || tupleNames[0] != mRobotName
        || tupleNames[1] != "MoveTo"
        || tupleNames[3] != "COMMAND"
        || std::string(t->data) != "ON"
    )
    {
        ROS_INFO("TupleHandlerMoveBase::matchesTupleKey(): useless tuple received, ignoring");
        return false;
    }

    // create a prefix to append to ("condo.MoveTo.13.")
    mLastCommand.tuplePrefix = tupleNames[0] + "." + tupleNames[1] + "." + tupleNames[2] + ".";
    
    // Now that we know we're supposed to move (COMMAND=ON), lets
    // ask Peis for the  PARAMETER Tuple (containing the coordinates)
    //ROS_INFO("TupleHandlerMoveBase::processTuple(): getting parameters-tuple with owner %d and key %s", t->owner, (mLastCommand.tuplePrefix + "PARAMETERS").c_str());
    PeisTuple* tupleMoveToParameters = mPeis->getTuple(t->owner, mLastCommand.tuplePrefix + "PARAMETERS", Peis::SubscribeBeforeReading::DoSubscribeBeforeReading);
    
    if(!tupleMoveToParameters)
    {
	ROS_ERROR("TupleHandlerMoveBase::processTuple(): tuple %sPARAMETERS doesn't exist (or has different owner than the COMMAND-tuple (owner %d)). This is a BUG!", mLastCommand.tuplePrefix.c_str(), t->owner);
	return false;
    }
    else
    {
        ROS_INFO("TupleHandlerMoveBase::processTuple(): got parameters-tuple: %s", mPeis->toString(tupleMoveToParameters).c_str());
    }
    
    // destinationString is "x y phi [map]", e.g. "1.23 14.54 0.32 [groundfloor]"
    const std::string destinationString(tupleMoveToParameters->data);
    std::vector<std::string> destinationCoordinates;

    std::string token;
    std::istringstream iss(destinationString);
    while(getline(iss, token, ' '))
        destinationCoordinates.push_back(token);

    if(destinationCoordinates.size() == 4)
    {
        std::string map = destinationCoordinates[3];
	destinationCoordinates.pop_back();
	
	if(map != mLastCommand.map)
	{
	    // the map changed. Tell ros->cognidrive_ros->mira to load the new map
	    ROS_INFO("TupleHandlerMoveBase::processTuple(): the map changed from \"%s\" to \"%s\", telling ROS about this!", mLastCommand.map.c_str(), map.c_str());
	    
	    ros::ServiceClient client = mRosNodeHandle.serviceClient<cognidrive_ros::LoadMap>("LoadMap");
	    cognidrive_ros::LoadMap srv;
	    srv.request.map = map;
	    if(client.call(srv))
	    {
		ROS_INFO("TupleHandlerMoveBase::processTuple(): map changed acknowledged with result: %s.", srv.response.result.c_str());
	    }
	    else
	    {
		ROS_ERROR("TupleHandlerMoveBase::processTuple(): failed to call service LoadMap - are both cognidrive_ros and MIRA running and in good mood?");
		return false;
	    }

	    mLastCommand.map = map;
	}
	else
	{
	    ROS_INFO("TupleHandlerMoveBase::processTuple(): MoveTo command contains unchanged map : %s.", map.c_str());
	}
    }
    
    // Set a navigation action
    geometry_msgs::PoseStamped goal;
    
    try
    {
	goal.pose.position.x = boost::lexical_cast<float>(destinationCoordinates[0]);
	goal.pose.position.y = boost::lexical_cast<float>(destinationCoordinates[1]);
	goal.pose.position.z = 0.0f;

	tf::Quaternion q = tf::createQuaternionFromRPY(0.0f, 0.0f, boost::lexical_cast<float>(destinationCoordinates[2]));
	tf::quaternionTFToMsg(q, goal.pose.orientation);
    }
    catch(boost::bad_lexical_cast &)
    {
        ROS_ERROR("TupleHandlerMoveBase::processTuple(): could not convert parameters %s to float/float/float/[map], ignoring tuple.", destinationString.c_str());
	return false;
    }
    
    mLastCommand.tupleOwner = t->owner;
    
    if(setGoal(goal))
    {
	// The confirmation that the goal is processed (STATE -> RUNNING) happens in the active callback below.	
	return true;
    }
    else
    {
        mPeis->publishRemote(mLastCommand.tupleOwner, mLastCommand.tuplePrefix + "STATE", "FAILED");
	return false;
    }
}

bool TupleHandlerMoveBase::setGoal(const geometry_msgs::PoseStamped& goal)
{
    double roll, pitch, yaw;
    tf::Quaternion q(goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    ROS_INFO("TupleHandlerMoveBase::setGoal(): setting move_base goal to X%.2f, Y%.2f, phi %.2f.", goal.pose.position.x, goal.pose.position.y, yaw);
    if(mActionClientRos.isServerConnected())
    {
        mGoal.target_pose = goal;
  
        mActionClientRos.sendGoal(
            mGoal,
            boost::bind(&TupleHandlerMoveBase::callbackDone, this, _1, _2),
            boost::bind(&TupleHandlerMoveBase::callbackActive, this),
            boost::bind(&TupleHandlerMoveBase::callbackFeedback, this, _1)
        );
	
	return true;
    }
    else
    {
        ROS_ERROR("TupleHandlerMoveBase::setGoal(): can't set move_base goal, action client not connected.");
	return false;
    }
}

// Called once when the goal completes
void TupleHandlerMoveBase::callbackDone(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
{
    if(state.toString() == "SUCCEEDED")
    {
        ROS_INFO("TupleHandlerMoveBase::callbackDone(): finished in state %s, telling Peis (STATE -> COMPLETED)", state.toString().c_str());
	mPeis->publishRemote(mLastCommand.tupleOwner, mLastCommand.tuplePrefix + "STATE", "COMPLETED");
    }
    else if(state.toString() == "ABORTED")
    {
        ROS_INFO("TupleHandlerMoveBase::callbackDone(): finished in state %s, telling Peis (STATE -> FAILED)", state.toString().c_str());
        mPeis->publishRemote(mLastCommand.tupleOwner, mLastCommand.tuplePrefix + "STATE", "FAILED");
    }
    
    // Even though we're done, don't reset mLastCommand - the map-field is still needed:
    // we load a new map if the next MoveTo has a differnet map - parameter.
}

// Called once when the goal becomes active
void TupleHandlerMoveBase::callbackActive()
{
    ROS_INFO("TupleHandlerMoveBase::callbackActive(): goal is active, telling Peis (STATE -> RUNNING)");
    mPeis->publishRemote(mLastCommand.tupleOwner, mLastCommand.tuplePrefix + "STATE", "RUNNING");
}

// Called every time feedback is received for the goal
void TupleHandlerMoveBase::callbackFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr &fb)
{
    ROS_INFO("TupleHandlerMoveBase::callbackFeedback()");
}
