#include <peisros.h>

int main(int argc, char **argv)
{
    // ros init
    ros::init(argc, argv, "peis_ros");
    
    PeisRos p(argc, argv);
    
    return p.exec();
}