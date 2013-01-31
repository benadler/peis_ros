#include <peisros.h>
#include <boost/algorithm/string/predicate.hpp>
#include <tclap/CmdLine.h>

PeisRos* PeisRos::peisRosPoorMansSingleton = 0;

void newTupleCallback(PeisTuple* t, void* arg)
{
    // Enqueue the Tuple for later processing outside of the peis callback.
    // Also store the TupleHandler-index in the mTupleHandlers-vector, so
    // we can pass the PeisTuple to the correct TupleHandler.
    PeisRos::peisRosPoorMansSingleton->newTupleArrived((int)arg, t);
}

PeisRos::PeisRos(int argc, char **argv) : mPeis(argc, argv)
{
    if(peisRosPoorMansSingleton != 0)
    {
        ROS_ERROR("PeisRos::PeisRos(): PeisRos was instantiated twice. This should never happen.");
	exit(-1);
    }
    
    peisRosPoorMansSingleton = this;
    
    TCLAP::CmdLine cmd("peis_ros connects PEIS to ROS.");
    
    TCLAP::ValueArg<std::string> switchRobotName("n","robotname","The name of the robot, e.g. \"condo\". This will listen to robotname.[].*.COMMAND",true,"condo","string");
    cmd.add(switchRobotName);
    cmd.parse(argc, argv);
    mRobotName = switchRobotName.getValue();

    mRosNodeHandle = new ros::NodeHandle;
    mRosTransformListener = new tf::TransformListener;
    
    // This is an example for how to react to incoming ROS messages IN THIS CLASS. For more elaborate
    // ROS handling, please create your own class and instantiate it here. See the ROS-parts of e.g.
    // TupleHandlerDustCart for how to do this.
    mRosSubExample = mRosNodeHandle->subscribe("/base_odometry/odom", 100, &PeisRos::onRosExample, this);
    
    registerTupleHandlers();
  
    // Subscribe to all of the TupleHandler's patterns (e.g. DustCart1.MoveTo.*.*)
    //
    // Note: We THOUGHT we could subscribe to e.g. mRobotName.* to catch PeisTuples like:
    // - mRobotName.OpenBin.COMMAND (but then we'd have to subscribe to *.*.*)
    // - mRobotName.MoveTo.13.COMMAND (but then we'd have to subscribe to *.*.*.*)
    // so then we thought we could subscribe to EVERYTHING (leave the key empty) and
    // filter ourselves. This, we were told, potentially causes too much peis traffic.
    // Thus, we subscribe to all 
    for(unsigned int i=0;i<mTupleHandlers.size();i++)
    {
        const std::string tupleHandlerPattern = mTupleHandlers[i]->getPattern();
	ROS_INFO("PeisRos::PeisRos(): subscribing to all PeisTuples with key %s...", tupleHandlerPattern.c_str());
        mPeis.registerCallbackAbstract(-1, tupleHandlerPattern, (void*)i, &newTupleCallback);
    }
}

void PeisRos::registerTupleHandlers()
{
    // Create and register a TupleHandler (one that handles the MoveTo tuples) 
    mTupleHandlers.push_back(new TupleHandlerMoveBase(mRobotName, &mPeis));
    
    // Here, you can add your own TupleHandlers!
    mTupleHandlers.push_back(new TupleHandlerDustCart(mRobotName, &mPeis));
}

PeisRos::~PeisRos()
{
    std::cout << "PeisRos::~PeisRos(): deleting members." << std::endl;

    delete mRosNodeHandle;
    delete mRosTransformListener;
    
    // delete all TupleHandlers
    for(unsigned int i = 0;i < mTupleHandlers.size(); i++)
        delete mTupleHandlers[i];

    mTupleHandlers.clear();
    
    std::cout << "PeisRos::~PeisRos(): finished." << std::endl;
    fflush(stdout);
}

void PeisRos::newTupleArrived(const int tupleHandlerIndex, PeisTuple* t)
{
    mMutex.lock();
    ROS_INFO("PeisRos::newTupleArrived(): tuplehandler %d will soon process tuple: %s", tupleHandlerIndex, mPeis.toString(t).c_str());
    mPeisTupleQueue.push(std::make_pair(tupleHandlerIndex, mPeis.cloneTuple(t)));
    mMutex.unlock();
}

void PeisRos::onRosExample(const nav_msgs::Odometry::ConstPtr& msg)
{
    // This method is called whenever a ros-message (e.g. an odometry) comes in.
    // You're free to process it here and mPeis.publish() some PeisTuple as a reply.
    //ROS_INFO("PeisRos::onRosExample()");
}

int PeisRos::exec()
{
    // do the locomotion
    ROS_INFO("PeisRos::exec(): starting event loop...");
    
    ros::Rate rateLoop(10.0);
    
    while(ros::ok())
    {
        ros::spinOnce();

	// process all queued PeisTuples using all registered TupleHandlers
	while(true)
	{
	    mMutex.lock();
	    if(mPeisTupleQueue.empty())
	    {
		mMutex.unlock();
		break;
	    }

	    const int tupleHandlerIndex = mPeisTupleQueue.front().first;
	    PeisTuple* tuple = mPeisTupleQueue.front().second;
	    mPeisTupleQueue.pop();
	    mMutex.unlock();
	    
	    // Just in case someone deleted a registered TupleHandler
	    if(mTupleHandlers[tupleHandlerIndex] != 0)
	    {	    
		if(!mTupleHandlers[tupleHandlerIndex]->processTuple(tuple))
		{
		    ROS_ERROR("PeisRos::exec(): TupleHandler %d failed to process PeisTuple %s", tupleHandlerIndex, mPeis.toString(tuple).c_str());
		}
	    }

	    mPeis.deleteTuple(tuple);
	}
	
	rateLoop.sleep();
    }

    ROS_INFO("PeisRos::exec(): quit ROS event loop...");

    return 0;
}