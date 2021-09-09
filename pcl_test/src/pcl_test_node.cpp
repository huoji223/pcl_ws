#include "pcl_test_core.h"

int main(int argc, char **argv)//main函数，节点入口
{
    ros::init(argc, argv, "pcl_test");//初始化节点，第三个参数为节点名
    
/*argc和argv用于处理remapping参数，使用这种形式后，在命令行中使用参数就无效了。如果还想在命令行中处理，需要在ros::init之后调用ros::removeROSArgs()
参考网址：https://www.dazhuanlan.com/2019/12/14/5df3c1ef11183/
*/

    ros::NodeHandle nh;//启动节点

    PclTestCore core(nh);
    // core.Spin();
    return 0;
}
