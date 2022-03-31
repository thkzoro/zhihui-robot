#include "robot.h"


int Robot::do_init()
{
    /********* Need to adjust parameters for specific hardware *********/
    joint[ANY] = JointStatus_t{0, -180, 180, 90};
    // Head
    joint[1] = JointStatus_t{2, 70, 95, 0, -15, 15, true};
    // Left arm roll
    joint[2] = JointStatus_t{4, -9, 3, 0, 0, 30, false};
    // Left arm pitch
    joint[3] = JointStatus_t{6, -16, 117, 0, -20, 180, false};
    // Right arm roll
    joint[4] = JointStatus_t{8, 133, 141, 0, 0, 30, true};
    // Right arm pitch
    joint[5] = JointStatus_t{10, 15, 150, 0, -20, 180, true};
    // Body
    joint[6] = JointStatus_t{12, 0, 180, 0, -90, 90, false};
    /********* Need to adjust parameters for specific hardware *********/

    m_pLcd = new RoundLcd();
    m_pLcd->Init();


}