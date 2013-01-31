#include <ros/ros.h>
#include <tf/tf.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <tuplehandler.h>
#include <peis.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseActionClientRos;

class TupleHandlerMoveBase : public TupleHandler
{
public:
    TupleHandlerMoveBase(const std::string& robotName, Peis* peis);
    virtual ~TupleHandlerMoveBase(void);
    
    // reimplemented form TupleHandler
    bool processTuple(PeisTuple* t);
    const std::string getPattern();
  
protected:
    struct LastCommand
    {
	// the COMMAND-Tuple came from owner 123, so we should send the STATE -> COMPLETE packet back to this owner in callbackDone.
	int tupleOwner; 
    
	// the COMMAND-Tuple's key was Dustcart1.MoveTo.14.*, so we should send the STATE -> COMPLETE packet back using this prefix in callbackDone.
	std::string tuplePrefix;
	
	// The map referenced in the MoveTo-Command. At the beginning, this field will be emty.
	std::string map;
	
	LastCommand()
	{
	    tupleOwner = 0;
	}
    };
    
    LastCommand mLastCommand;
    
    std::string mRobotName;
    ros::NodeHandle mRosNodeHandle;
    MoveBaseActionClientRos mActionClientRos;
    Peis* mPeis;
    move_base_msgs::MoveBaseGoal mGoal;
        
    // called internally from processTuple() after the tuple has passed sanity checks.
    bool setGoal(const geometry_msgs::PoseStamped& goal);

    // Called once when the goal completes
    void callbackDone(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result);

    // Called once when the goal becomes active
    void callbackActive();

    // Called every time feedback is received for the goal
    void callbackFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr &fb);
};