#ifndef PEISROS_H
#define PEISROS_H

#include <peis.h>
#include <tuplehandlermovebase.h>
#include <tuplehandlerdustcart.h>

#include <boost/thread/mutex.hpp>
#include <algorithm>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <queue>

#include <nav_msgs/Odometry.h>

class PeisRos
{
private:
    std::vector<TupleHandler*> mTupleHandlers;
    boost::mutex mMutex;
    Peis mPeis;
    std::queue<std::pair<int, PeisTuple*> > mPeisTupleQueue;
    std::string mRobotName;
    ros::NodeHandle* mRosNodeHandle;
    tf::TransformListener* mRosTransformListener;
    tf::StampedTransform mLastTransform;

    void registerTupleHandlers();

    // necessary subscribers for ROS topics
    ros::Subscriber mRosSubExample;

    // This is an example for how to mae things happen when a ROS message comes in
    void onRosExample(const nav_msgs::Odometry::ConstPtr& msg);

public:
    PeisRos(int argc, char **argv);
    ~PeisRos();
    
    void newTupleArrived(const int tupleHandlerIndex, PeisTuple* t);
    
    int exec();
    static PeisRos* peisRosPoorMansSingleton;
};

#endif