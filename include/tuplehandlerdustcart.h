#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <tuplehandler.h>
#include <peis.h>

class TupleHandlerDustCart : public TupleHandler
{
public:
    TupleHandlerDustCart(const std::string& robotName, Peis* peis);
    virtual ~TupleHandlerDustCart(void);
    
    // reimplemented form TupleHandler
    bool processTuple(PeisTuple* t);
    const std::string getPattern();
  
protected:
    ros::Subscriber mRosSubDustCartStatus;
    void onRosDustCartStatusChange(const std_msgs::String::ConstPtr& msg);

    struct LastCommand
    {
	// the COMMAND-Tuple came from owner 123, so we should send the STATE -> COMPLETE packet back to this owner in callbackDone.
	int tupleOwner; 
    
	// the COMMAND-Tuple's key was Dustcart1.MoveTo.14.*, so we should send the STATE -> COMPLETE packet back using this prefix in callbackDone.
	std::string tuplePrefix;
	
	std::string action;
	
	LastCommand()
	{
	    tupleOwner = 0;
	}
    };
    
    LastCommand mLastCommand;
    
    std::string mRobotName;
    Peis* mPeis;
    
    ros::NodeHandle mRosNodeHandle;
    ros::Publisher mRosPublisher;
};