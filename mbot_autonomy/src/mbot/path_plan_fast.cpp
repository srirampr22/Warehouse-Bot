#include <utils/lcm_config.h>
#include <mbot/mbot_channels.h>
#include "mbot_lcm_msgs/path2D_t.hpp"
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>

int main(int argc, char** argv)
{
    int numTimes = 9;

    if(argc > 1)
    {
        numTimes = std::atoi(argv[1]);
    }

    std::cout << "Commanding robot to drive around maze.\n";

    mbot_lcm_msgs::path2D_t path;
    path.path.resize(numTimes);

    mbot_lcm_msgs::pose2D_t nextPose;

    nextPose.x = 0.0f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    path.path[0] = nextPose;

    nextPose.x = 0.61f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    path.path[1] = nextPose;

    nextPose.x = 0.61f;
    nextPose.y = 0.61f;
    nextPose.theta = 0.0f;
    path.path[2] = nextPose;

    nextPose.x = 1.22f;
    nextPose.y = 0.61f;
    nextPose.theta = 0.0f;
    path.path[3] = nextPose;

    nextPose.x = 1.22f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    path.path[4] = nextPose;

    nextPose.x = 1.83f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    path.path[5] = nextPose;

    nextPose.x = 1.83f;
    nextPose.y = 1.22f;
    nextPose.theta = 0.0f;
    path.path[6] = nextPose;

    nextPose.x = 0.0f;
    nextPose.y = 1.22f;
    nextPose.theta = 0.0f;
    path.path[7] = nextPose;

    nextPose.x = 0.0f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    path.path[8] = nextPose;

    // nextPose.x = 0.0f;
    // nextPose.y = 0.0f;
    // nextPose.theta = 0.0f;
    // path.path[0] = nextPose;

    // nextPose.x = 0.30f;
    // nextPose.y = 0.0f;
    // nextPose.theta = 0.0f;
    // path.path[1] = nextPose;

    // nextPose.x = 0.37f;
    // nextPose.y = 0.01f;
    // nextPose.theta = 0.0f;
    // path.path[2] = nextPose;

    // nextPose.x = 0.44f;
    // nextPose.y = 0.03f;
    // nextPose.theta = 0.0f;
    // path.path[3] = nextPose;

    // nextPose.x = 0.49f;
    // nextPose.y = 0.07f;
    // nextPose.theta = 0.0f;
    // path.path[4] = nextPose;

    // nextPose.x = 0.54f;
    // nextPose.y = 0.11f;
    // nextPose.theta = 0.0f;
    // path.path[5] = nextPose;

    // nextPose.x = 0.58f;
    // nextPose.y = 0.17f;
    // nextPose.theta = 0.0f;
    // path.path[6] = nextPose;

    // nextPose.x = 0.60f;
    // nextPose.y = 0.24f;
    // nextPose.theta = 0.0f;
    // path.path[7] = nextPose;

    // nextPose.x = 0.61f;
    // nextPose.y = 0.30f;
    // nextPose.theta = 0.0f;
    // path.path[8] = nextPose;

    // nextPose.x = 0.62f;
    // nextPose.y = 0.37f;
    // nextPose.theta = 0.0f;
    // path.path[9] = nextPose;

    // nextPose.x = 0.64f;
    // nextPose.y = 0.44f;
    // nextPose.theta = 0.0f;
    // path.path[10] = nextPose;

    // nextPose.x = 0.68f;
    // nextPose.y = 0.49f;
    // nextPose.theta = 0.0f;
    // path.path[11] = nextPose;

    // nextPose.x = 0.72f;
    // nextPose.y = 0.54f;
    // nextPose.theta = 0.0f;
    // path.path[12] = nextPose;

    // nextPose.x = 0.78f;
    // nextPose.y = 0.58f;
    // nextPose.theta = 0.0f;
    // path.path[13] = nextPose;

    // nextPose.x = 0.85f;
    // nextPose.y = 0.60f;
    // nextPose.theta = 0.0f;
    // path.path[14] = nextPose;

    // nextPose.x = 0.91f;
    // nextPose.y = 0.61f;
    // nextPose.theta = 0.0f;
    // path.path[15] = nextPose;

    // nextPose.x = 0.98f;
    // nextPose.y = 0.60f;
    // nextPose.theta = 0.0f;
    // path.path[16] = nextPose;

    // nextPose.x = 1.05f;
    // nextPose.y = 0.58f;
    // nextPose.theta = 0.0f;
    // path.path[17] = nextPose;

    // nextPose.x = 1.10f;
    // nextPose.y = 0.54f;
    // nextPose.theta = 0.0f;
    // path.path[18] = nextPose;

    // nextPose.x = 1.15f;
    // nextPose.y = 0.49f;
    // nextPose.theta = 0.0f;
    // path.path[19] = nextPose;

    // nextPose.x = 1.19f;
    // nextPose.y = 0.44f;
    // nextPose.theta = 0.0f;
    // path.path[20] = nextPose;

    // nextPose.x = 1.21f;
    // nextPose.y = 0.37f;
    // nextPose.theta = 0.0f;
    // path.path[21] = nextPose;

    // nextPose.x = 1.22f;
    // nextPose.y = 0.30f;
    // nextPose.theta = 0.0f;
    // path.path[22] = nextPose;

    // nextPose.x = 1.23f;
    // nextPose.y = 0.24f;
    // nextPose.theta = 0.0f;
    // path.path[23] = nextPose;

    // nextPose.x = 1.25f;
    // nextPose.y = 0.17f;
    // nextPose.theta = 0.0f;
    // path.path[24] = nextPose;

    // nextPose.x = 1.28f;
    // nextPose.y = 0.11f;
    // nextPose.theta = 0.0f;
    // path.path[25] = nextPose;

    // nextPose.x = 1.33f;
    // nextPose.y = 0.07f;
    // nextPose.theta = 0.0f;
    // path.path[26] = nextPose;

    // nextPose.x = 1.39f;
    // nextPose.y = 0.03f;
    // nextPose.theta = 0.0f;
    // path.path[27] = nextPose;

    // nextPose.x = 1.46f;
    // nextPose.y = 0.01f;
    // nextPose.theta = 0.0f;
    // path.path[28] = nextPose;

    // nextPose.x = 1.52f;
    // nextPose.y = 0.00f;
    // nextPose.theta = 0.0f;
    // path.path[29] = nextPose;

    // nextPose.x = 1.83f;
    // nextPose.y = 0.00f;
    // nextPose.theta = 0.0f;
    // path.path[30] = nextPose;

    // nextPose.x = 1.83f;
    // nextPose.y = 0.91f;
    // nextPose.theta = 0.0f;
    // path.path[31] = nextPose;

    // nextPose.x = 1.82f;
    // nextPose.y = 0.98f;
    // nextPose.theta = 0.0f;
    // path.path[32] = nextPose;

    // nextPose.x = 1.80f;
    // nextPose.y = 1.05f;
    // nextPose.theta = 0.0f;
    // path.path[33] = nextPose;

    // nextPose.x = 1.71f;
    // nextPose.y = 1.15f;
    // nextPose.theta = 0.0f;
    // path.path[34] = nextPose;

    // nextPose.x = 1.66f;
    // nextPose.y = 1.19f;
    // nextPose.theta = 0.0f;
    // path.path[35] = nextPose;

    // nextPose.x = 1.59f;
    // nextPose.y = 1.21f;
    // nextPose.theta = 0.0f;
    // path.path[36] = nextPose;

    // nextPose.x = 1.52f;
    // nextPose.y = 1.22f;
    // nextPose.theta = 0.0f;
    // path.path[37] = nextPose;

    // nextPose.x = 0.30f;
    // nextPose.y = 1.22f;
    // nextPose.theta = 0.0f;
    // path.path[38] = nextPose;

    // nextPose.x = 0.24f;
    // nextPose.y = 1.21f;
    // nextPose.theta = 0.0f;
    // path.path[39] = nextPose;

    // nextPose.x = 0.17f;
    // nextPose.y = 1.19f;
    // nextPose.theta = 0.0f;
    // path.path[40] = nextPose;

    // nextPose.x = 0.11f;
    // nextPose.y = 1.15f;
    // nextPose.theta = 0.0f;
    // path.path[41] = nextPose;

    // nextPose.x = 0.07f;
    // nextPose.y = 1.10f;
    // nextPose.theta = 0.0f;
    // path.path[42] = nextPose;

    // nextPose.x = 0.03f;
    // nextPose.y = 1.05f;
    // nextPose.theta = 0.0f;
    // path.path[43] = nextPose;

    // nextPose.x = 0.01f;
    // nextPose.y = 0.98f;
    // nextPose.theta = 0.0f;
    // path.path[44] = nextPose;

    // nextPose.x = 0.00f;
    // nextPose.y = 0.91f;
    // nextPose.theta = 0.0f;
    // path.path[45] = nextPose;

    // nextPose.x = 0.00f;
    // nextPose.y = 0.00f;
    // nextPose.theta = 0.0f;
    // path.path[46] = nextPose;

    
//     Return to original heading after completing all circuits
//    nextPose.theta = 0.0f;
//    path.path.push_back(nextPose);

    // nextPose.x = 0.0f;
    // nextPose.y = 0.0f;
    // nextPose.theta = 0.0f;
    // path.path.insert(path.path.begin(), nextPose);

    path.path_length = path.path.size();

    lcm::LCM lcmInstance(MULTICAST_URL);
	std::cout << "publish to: " << CONTROLLER_PATH_CHANNEL << std::endl;
    lcmInstance.publish(CONTROLLER_PATH_CHANNEL, &path);
    sleep(1);

    return 0;
}
