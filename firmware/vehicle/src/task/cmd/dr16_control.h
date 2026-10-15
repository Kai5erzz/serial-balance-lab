#ifndef DR16_CONTROL_H
#define DR16_CONTROL_H

#include "rm_module.h"
#include "robot.h"

void dr16_control_update(const Remote_Info_Typedef *remote,
                         const struct chassis_fdb_msg *chassis_fdb,
                         const struct gimbal_fdb_msg *gimbal_fdb,
                         const struct shoot_fdb_msg *shoot_fdb,
                         struct chassis_cmd_msg *chassis_cmd,
                         struct gimbal_cmd_msg *gimbal_cmd,
                         struct shoot_cmd_msg *shoot_cmd);

#endif
