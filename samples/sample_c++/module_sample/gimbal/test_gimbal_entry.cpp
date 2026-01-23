/**
 ********************************************************************
 * @file    test_gimbal_entry.cpp
 * @version V2.0.0
 * @date    2023/3/28
 * @brief   云台管理器测试程序入口文件
 *
 * @copyright (c) 2018-2023 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI's authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <stdexcept>
#include "test_gimbal_entry.hpp"
#include "dji_logger.h"
#include "utils/util_misc.h"
#include "dji_gimbal.h"
#include "dji_gimbal_manager.h"
#include <iostream>
#include "dji_aircraft_info.h"
#include "dji_fc_subscription.h"
#include <math.h>

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
// 云台管理器测试选项枚举
typedef enum
{
    E_DJI_TEST_GIMBAL_MANAGER_SAMPLE_SELECT_ROTATE_GIMBAL_ON_FREE_MODE,          // 0: 自由模式下旋转云台
    E_DJI_TEST_GIMBAL_MANAGER_SAMPLE_SELECT_ROTATE_GIMBAL_ON_YAW_FOLLOW_MODE,    // 1: YAW跟随模式下旋转云台
    E_DJI_TEST_GIMBAL_MANAGER_SAMPLE_SELECT_SET_PITCH_RANGE_EXTENSION_MODE,      // 2: 设置俯仰角范围扩展模式
    E_DJI_TEST_GIMBAL_MANAGER_SAMPLE_SELECT_SET_CONTROLLER_MAX_SPEED_PERCENTAGE, // 3: 设置控制器最大速度百分比
    E_DJI_TEST_GIMBAL_MANAGER_SAMPLE_SELECT_SET_CONTROLLER_SMOOTH_FACTOR,        // 4: 设置控制器平滑因子
    E_DJI_TEST_GIMBAL_MANAGER_SAMPLE_SELECT_RESET_GIMBAL_SETTINGS,               // 5: 重置云台设置
    E_DJI_TEST_GIMBAL_MANAGER_SAMPLE_SELECT_ROTATE_GIMBAL_BY_KEYBOARD,           // 6: 键盘控制旋转云台并读取回传值
    E_DJI_TEST_GIMBAL_MANAGER_SAMPLE_NEW_FEATURE_1,                              // --------------------------------------7: 新功能1 (待定)
    E_DJI_TEST_GIMBAL_MANAGER_SAMPLE_SELECT_QUIT,                                // q: 退出
} E_DjiTestGimbalManagerSampleSelect;

/* Private values -------------------------------------------------------------*/
// 云台管理器测试选项菜单列表
static const char *s_gimbalManagerSampleList[] = {
    [E_DJI_TEST_GIMBAL_MANAGER_SAMPLE_SELECT_ROTATE_GIMBAL_ON_FREE_MODE] =
        "| [0] 云台管理器示例 - 自由模式下旋转云台                                          |",
    [E_DJI_TEST_GIMBAL_MANAGER_SAMPLE_SELECT_ROTATE_GIMBAL_ON_YAW_FOLLOW_MODE] =
        "| [1] 云台管理器示例 - YAW跟随模式下旋转云台                                      |",
    [E_DJI_TEST_GIMBAL_MANAGER_SAMPLE_SELECT_SET_PITCH_RANGE_EXTENSION_MODE] =
        "| [2] 云台管理器示例 - 设置俯仰角范围扩展模式                                      |",
    [E_DJI_TEST_GIMBAL_MANAGER_SAMPLE_SELECT_SET_CONTROLLER_MAX_SPEED_PERCENTAGE] =
        "| [3] 云台管理器示例 - 设置控制器最大速度百分比                                    |",
    [E_DJI_TEST_GIMBAL_MANAGER_SAMPLE_SELECT_SET_CONTROLLER_SMOOTH_FACTOR] =
        "| [4] 云台管理器示例 - 设置控制器平滑因子                                          |",
    [E_DJI_TEST_GIMBAL_MANAGER_SAMPLE_SELECT_RESET_GIMBAL_SETTINGS] =
        "| [5] 云台管理器示例 - 重置云台设置                                                |",
    [E_DJI_TEST_GIMBAL_MANAGER_SAMPLE_SELECT_ROTATE_GIMBAL_BY_KEYBOARD] =
        "| [6] 云台管理器示例 - 通过键盘输入旋转云台并读取回传值                            |",
    [E_DJI_TEST_GIMBAL_MANAGER_SAMPLE_NEW_FEATURE_1] =
        "| [7] 云台管理示例 - 新的测试选项功能                                            |",
    [E_DJI_TEST_GIMBAL_MANAGER_SAMPLE_SELECT_QUIT] =
        "| [q] 云台管理器示例 - 退出                                                        |",
};

/* Private functions declaration ---------------------------------------------*/
// 显示测试选项列表
void DjiTest_GimbalManagerShowSampleSelectList(const char **SampleList, uint8_t size);

/* Exported functions definition ---------------------------------------------*/
/**
 * @brief 云台管理器测试主入口函数
 * @note 提供交互式菜单，允许用户选择不同的云台测试功能
 * @retval None
 */
void DjiUser_RunGimbalManagerSample(void)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler(); // 获取操作系统抽象层句柄
    char inputTestCase;                                           // 用户输入的测试用例选项
    char mountPosition;                                           // 用户输入的云台挂载位置
    E_DjiMountPosition gimbalMountPosition;                       // 云台挂载位置枚举
    T_DjiReturnCode returnCode;                                   // 函数返回码

// 开始标签，用于返回到菜单选择
start:
    osalHandler->TaskSleepMs(100); // 延时100ms，确保输出显示完整

    // 显示云台挂载位置选择菜单
    std::cout
        << "| 可用的挂载位置:                                                                          |"
        << std::endl;
    std::cout
        << "| [1 ~ 4] 选择1~4号云台挂载位置                                                              |"
        << std::endl;
    std::cout
        << "| [q] 退出                                                                                 |"
        << std::endl;

    std::cin >> mountPosition; // 读取用户输入的挂载位置

    // 如果用户选择退出
    if (mountPosition == 'q')
    {
        return;
    }

    // 验证挂载位置输入是否有效（1-4）
    if (mountPosition > '4' || mountPosition < '1')
    {
        USER_LOG_ERROR("输入的挂载位置无效");
        goto start; // 输入无效，返回重新输入
    }

    // 将字符转换为枚举值（'1'->1, '2'->2, 等）
    gimbalMountPosition = E_DjiMountPosition(mountPosition - '0');

    osalHandler->TaskSleepMs(100); // 延时100ms

    // 显示测试功能选择菜单
    std::cout
        << "| 可用命令:                                                                                  |"
        << std::endl;
    DjiTest_GimbalManagerShowSampleSelectList(s_gimbalManagerSampleList,
                                              UTIL_ARRAY_SIZE(s_gimbalManagerSampleList));

    std::cin >> inputTestCase; // 读取用户输入的测试选项

    // 根据用户选择执行相应的测试功能
    switch (inputTestCase)
    {
    case '0': // 自由模式下旋转云台
        DjiTest_GimbalManagerRunSample(gimbalMountPosition, DJI_GIMBAL_MODE_FREE);
        goto start; // 执行完成后返回菜单
    case '1':       // YAW跟随模式下旋转云台
        DjiTest_GimbalManagerRunSample(gimbalMountPosition, DJI_GIMBAL_MODE_YAW_FOLLOW);
        goto start;
    case '2':
    {                       // 设置俯仰角范围扩展模式
        int32_t enableFlag; // 启用标志（0:禁用, 1:启用）

        osalHandler->TaskSleepMs(10); // 延时10ms
        printf("输入启用标志(0:禁用, 1:启用): ");
        scanf("%d", &enableFlag); // 读取用户输入的启用标志

        // 初始化云台管理器
        returnCode = DjiGimbalManager_Init();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("初始化云台管理器失败，错误码: 0x%08X", returnCode);
            return;
        }

        // 设置俯仰角范围扩展模式
        returnCode = DjiGimbalManager_SetPitchRangeExtensionEnabled(gimbalMountPosition, (bool)enableFlag);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("设置失败!");
        }

        USER_LOG_INFO("成功设置云台俯仰角范围扩展模式为 %d!", enableFlag);

        goto start; // 返回菜单
        break;
    }
    case '3':
    {                       // 设置控制器最大速度百分比
        int32_t percentage; // 速度百分比（0-100）

        osalHandler->TaskSleepMs(10);
        printf("输入YAW轴最大速度百分比: ");
        scanf("%d", &percentage); // 读取YAW轴速度百分比

        // 初始化云台管理器
        returnCode = DjiGimbalManager_Init();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("初始化云台管理器失败，错误码: 0x%08X", returnCode);
            return;
        }

        // 设置YAW轴最大速度百分比
        returnCode = DjiGimbalManager_SetControllerMaxSpeedPercentage(gimbalMountPosition,
                                                                      DJI_GIMBAL_AXIS_YAW,
                                                                      (uint8_t)percentage);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("设置失败!");
        }

        USER_LOG_INFO("成功设置YAW轴最大速度百分比为 %d!", percentage);

        osalHandler->TaskSleepMs(10);
        printf("输入PITCH轴最大速度百分比: ");
        scanf("%d", &percentage); // 读取PITCH轴速度百分比

        // 设置PITCH轴最大速度百分比
        returnCode = DjiGimbalManager_SetControllerMaxSpeedPercentage(gimbalMountPosition,
                                                                      DJI_GIMBAL_AXIS_PITCH,
                                                                      (uint8_t)percentage);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("设置失败!");
        }

        USER_LOG_INFO("成功设置PITCH轴最大速度百分比为 %d!", percentage);

        goto start; // 返回菜单
        break;
    }
    case '4':
    {                   // 设置控制器平滑因子
        int32_t factor; // 平滑因子值

        osalHandler->TaskSleepMs(10);
        printf("输入YAW轴平滑因子: ");
        scanf("%d", &factor); // 读取YAW轴平滑因子

        // 初始化云台管理器
        returnCode = DjiGimbalManager_Init();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("初始化云台管理器失败，错误码: 0x%08X", returnCode);
            return;
        }

        // 设置YAW轴平滑因子
        returnCode = DjiGimbalManager_SetControllerSmoothFactor(gimbalMountPosition,
                                                                DJI_GIMBAL_AXIS_YAW,
                                                                (uint8_t)factor);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("设置失败!");
        }

        USER_LOG_INFO("成功设置YAW轴平滑因子为 %d!", factor);

        osalHandler->TaskSleepMs(10);
        printf("输入PITCH轴平滑因子: ");
        scanf("%d", &factor); // 读取PITCH轴平滑因子

        // 设置PITCH轴平滑因子
        returnCode = DjiGimbalManager_SetControllerSmoothFactor(gimbalMountPosition,
                                                                DJI_GIMBAL_AXIS_PITCH,
                                                                (uint8_t)factor);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("设置失败!");
        }

        USER_LOG_INFO("成功设置PITCH轴平滑因子为 %d!", factor);

        goto start; // 返回菜单
        break;
    }
    case '5':
    { // 重置云台设置

        // 初始化云台管理器
        returnCode = DjiGimbalManager_Init();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("初始化云台管理器失败，错误码: 0x%08X", returnCode);
            return;
        }

        // 恢复云台出厂设置
        returnCode = DjiGimbalManager_RestoreFactorySettings(gimbalMountPosition);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("重置失败!");
        }

        USER_LOG_INFO("成功重置云台出厂设置!");

        goto start; // 返回菜单
        break;
    }
    case '6':
    {                                                       // 键盘控制旋转云台并读取回传值
        uint32_t gimbalMode;                                // 云台模式
        uint32_t rotateMode;                                // 旋转模式
        dji_f32_t pitch, roll, yaw;                         // 俯仰、横滚、偏航角度
        T_DjiGimbalManagerRotation rotation;                // 云台旋转参数结构体
        T_DjiAircraftInfoBaseInfo baseInfo;                 // 飞机基本信息
        E_DjiAircraftSeries aircraftSeries;                 // 飞机系列
        E_DjiFcSubscriptionTopic topicOfPayloadGimablAngle; // 云台角度订阅主题

        // 初始化云台管理器
        returnCode = DjiGimbalManager_Init();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("初始化云台管理器失败，错误码: 0x%08X", returnCode);
            return;
        }

        // 获取飞机基本信息
        returnCode = DjiAircraftInfo_GetBaseInfo(&baseInfo);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("获取飞机基本信息失败，错误码: 0x%08X", returnCode);
            goto end; // 跳转到清理退出
        }

        aircraftSeries = baseInfo.aircraftSeries; // 获取飞机系列

        // 根据飞机系列订阅不同的数据主题
        if (aircraftSeries == DJI_AIRCRAFT_SERIES_M300 || aircraftSeries == DJI_AIRCRAFT_SERIES_M350)
        {
            // M300/M350系列需要订阅IMU姿态导航数据和四元数数据
            returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_IMU_ATTI_NAVI_DATA_WITH_TIMESTAMP,
                                                          DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                                          NULL);
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
            {
                USER_LOG_ERROR("订阅主题 %d 失败，错误码: %d",
                               DJI_FC_SUBSCRIPTION_TOPIC_IMU_ATTI_NAVI_DATA_WITH_TIMESTAMP, returnCode);
                goto end;
            }

            returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                                          DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                                          NULL);
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
            {
                USER_LOG_ERROR("订阅主题 %d 失败，错误码: %d",
                               DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, returnCode);
                goto end;
            }
        }

        // 根据飞机系列订阅云台角度数据
        if (aircraftSeries == DJI_AIRCRAFT_SERIES_M300 || aircraftSeries == DJI_AIRCRAFT_SERIES_M350)
        {
            // M300/M350系列订阅三云台数据主题
            returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_THREE_GIMBAL_DATA,
                                                          DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                                          NULL);
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
            {
                USER_LOG_ERROR("订阅主题 %d 失败，错误码: %d",
                               DJI_FC_SUBSCRIPTION_TOPIC_THREE_GIMBAL_DATA, returnCode);
                goto end;
            }
            USER_LOG_INFO("成功订阅主题 DJI_FC_SUBSCRIPTION_TOPIC_THREE_GIMBAL_DATA.");
        }
        else if (aircraftSeries == DJI_AIRCRAFT_SERIES_M30 || aircraftSeries == DJI_AIRCRAFT_SERIES_M3 ||
                 aircraftSeries == DJI_AIRCRAFT_SERIES_M3D)
        {
            // M30/M3/M3D系列订阅云台角度主题
            topicOfPayloadGimablAngle = DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES;
            returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
                                                          DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                                          NULL);
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
            {
                USER_LOG_ERROR("订阅主题 %d 失败，错误码: %d",
                               DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES, returnCode);
                goto end;
            }
            USER_LOG_INFO("成功订阅主题 DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES.");
        }
        else if (aircraftSeries == DJI_AIRCRAFT_SERIES_M400)
        {
            // M400系列根据挂载位置选择不同的云台角度主题
            topicOfPayloadGimablAngle = gimbalMountPosition == 1 ? DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES_ON_POS_NO1 : gimbalMountPosition == 2 ? DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES_ON_POS_NO2
                                                                                                                    : gimbalMountPosition == 3   ? DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES_ON_POS_NO3
                                                                                                                    : gimbalMountPosition == 4   ? DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES_ON_POS_NO4
                                                                                                                                                 : DJI_FC_SUBSCRIPTION_TOPIC_TOTAL_NUMBER;
            returnCode = DjiFcSubscription_SubscribeTopic(topicOfPayloadGimablAngle,
                                                          DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                                          NULL);
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
            {
                USER_LOG_ERROR("订阅主题 %d 失败，错误码: %d",
                               DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES, returnCode);
                goto end;
            }
            USER_LOG_INFO("成功订阅主题 DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES_ON_POS_NO%d.", gimbalMountPosition);
        }

        // 进入键盘控制循环
        while (1)
        {
            T_DjiFcSubscriptionQuaternion quat;                             // 四元数数据结构
            T_DjiFcSubscriptionThreeGimbalData threeGimbalData = {0};       // 三云台数据（M300/M350）
            T_DjiFcSubscriptionGimbalAngles gimbalAngles = {0};             // 云台角度数据（其他机型）
            T_DjiDataTimestamp timestamp = {0};                             // 数据时间戳
            dji_f32_t nPitch, nRoll, nYaw;                                  // 飞机姿态角
            dji_f32_t qPitch, qRoll, qYaw;                                  // 云台四元数转换的角度
            dji_f32_t yawOffset = 0;                                        // 偏航角偏移量
            T_DjiFcSubscriptionImuAttiNaviDataWithTimestamp naviData = {0}; // 导航数据

            osalHandler->TaskSleepMs(5); // 延时5ms

            // 显示输入提示
            printf("云台模式: 0:自由模式, 1:FPV模式, 2:YAW跟随模式, 3:退出示例\n");
            printf("旋转模式: 0:相对角度, 1:绝对角度\n");
            printf("请输入云台模式、旋转模式、俯仰角、横滚角、偏航角(绝对角度模式下范围为0~360度): ");

            // 读取用户输入
            scanf("%d", &gimbalMode);
            if (gimbalMode == 3)
            { // 如果输入3则退出循环
                break;
            }

            scanf("%d %f %f %f", &rotateMode, &pitch, &roll, &yaw);

            printf("云台模式 %d, 旋转模式 %d, 俯仰角 %f, 横滚角 %f, 偏航角 %f\n",
                   gimbalMode, rotateMode, pitch, roll, yaw);

            // 设置云台模式
            returnCode = DjiGimbalManager_SetMode(gimbalMountPosition, (E_DjiGimbalMode)gimbalMode);
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
            {
                USER_LOG_ERROR("DjiGimbalManager_SetMode 返回错误码: 0x%08X", returnCode);
                goto end;
            }

            // 设置旋转参数
            rotation.rotationMode = (E_DjiGimbalRotationMode)rotateMode;
            rotation.pitch = pitch;
            rotation.roll = roll;
            rotation.yaw = yaw;
            rotation.time = 0.5; // 旋转时间固定为0.5秒

            // M300/M350系列特殊处理：计算偏航角偏移
            if (aircraftSeries == DJI_AIRCRAFT_SERIES_M300 || aircraftSeries == DJI_AIRCRAFT_SERIES_M350)
            {
                osalHandler->TaskSleepMs(20); // 延时20ms

                // 获取IMU姿态导航数据
                returnCode = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_IMU_ATTI_NAVI_DATA_WITH_TIMESTAMP,
                                                                     (uint8_t *)&naviData,
                                                                     sizeof(T_DjiFcSubscriptionImuAttiNaviDataWithTimestamp),
                                                                     &timestamp);
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                {
                    USER_LOG_ERROR("获取IMU姿态导航数据失败，错误码: 0x%08X", returnCode);
                }

                // 从四元数计算飞机偏航角
                // 公式: yaw = atan2(2*(q1*q2 + q0*q3), 1 - 2*(q2? + q3?)) * 180/π
                nYaw = (dji_f64_t)atan2f(2 * naviData.q[1] * naviData.q[2] + 2 * naviData.q[0] * naviData.q[3],
                                         -2 * naviData.q[2] * naviData.q[2] - 2 * naviData.q[3] * naviData.q[3] + 1) *
                       57.3;

                // 获取云台四元数数据
                returnCode = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                                                     (uint8_t *)&quat,
                                                                     sizeof(T_DjiFcSubscriptionQuaternion),
                                                                     &timestamp);
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                {
                    USER_LOG_ERROR("获取四元数数据失败，错误码: 0x%08X", returnCode);
                    goto end;
                }

                // 从云台四元数计算偏航角
                qYaw = (dji_f64_t)atan2f(2 * quat.q1 * quat.q2 + 2 * quat.q0 * quat.q3,
                                         -2 * quat.q2 * quat.q2 - 2 * quat.q3 * quat.q3 + 1) *
                       57.3;

                // 计算偏航角偏移量（飞机偏航角 - 云台偏航角）
                yawOffset = nYaw - qYaw;

                // 如果是绝对角度模式，需要补偿偏移量
                if (rotation.rotationMode == DJI_GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE)
                {
                    rotation.yaw += yawOffset;
                }
            }

            // 执行云台旋转
            returnCode = DjiGimbalManager_Rotate(gimbalMountPosition, rotation);

            // 检查是否为限位错误
            if (returnCode == DJI_ERROR_GIMBAL_MODULE_CODE_PITCH_REACH_POSITIVE_LIMIT ||
                returnCode == DJI_ERROR_GIMBAL_MODULE_CODE_PITCH_REACH_NEGATIVE_LIMIT ||
                returnCode == DJI_ERROR_GIMBAL_MODULE_CODE_ROLL_REACH_POSITIVE_LIMIT ||
                returnCode == DJI_ERROR_GIMBAL_MODULE_CODE_ROLL_REACH_NEGATIVE_LIMIT ||
                returnCode == DJI_ERROR_GIMBAL_MODULE_CODE_YAW_REACH_POSITIVE_LIMIT ||
                returnCode == DJI_ERROR_GIMBAL_MODULE_CODE_YAW_REACH_NEGATIVE_LIMIT)
            {
                USER_LOG_WARN("已达到限位!"); // 警告日志，不是错误
            }
            else if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
            {
                USER_LOG_ERROR("云台旋转失败，错误码: 0x%08X", returnCode);
                goto end;
            }

            osalHandler->TaskSleepMs(2000); // 等待2秒，让云台完成旋转

            // 读取云台实际角度并显示
            if (aircraftSeries == DJI_AIRCRAFT_SERIES_M300 || aircraftSeries == DJI_AIRCRAFT_SERIES_M350)
            {
                // M300/M350系列：从三云台数据中读取
                returnCode = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_THREE_GIMBAL_DATA,
                                                                     (uint8_t *)&threeGimbalData,
                                                                     sizeof(T_DjiFcSubscriptionThreeGimbalData),
                                                                     &timestamp);
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                {
                    USER_LOG_ERROR("获取三云台数据失败，错误码: 0x%08X", returnCode);
                    goto end;
                }

                // 将负角度转换为0-360度范围
                if (threeGimbalData.anglesData[0].yaw < 0)
                {
                    threeGimbalData.anglesData[0].yaw = 360 + threeGimbalData.anglesData[0].yaw;
                }

                // 显示读取的云台角度
                USER_LOG_INFO("读取云台角度(俯仰, 横滚, 偏航): p=%.4f r=%.4f y=%.4f",
                              threeGimbalData.anglesData[gimbalMountPosition - 1].pitch,
                              threeGimbalData.anglesData[gimbalMountPosition - 1].roll,
                              threeGimbalData.anglesData[gimbalMountPosition - 1].yaw);
            }
            else if (aircraftSeries == DJI_AIRCRAFT_SERIES_M30 || aircraftSeries == DJI_AIRCRAFT_SERIES_M3 ||
                     aircraftSeries == DJI_AIRCRAFT_SERIES_M3D || aircraftSeries == DJI_AIRCRAFT_SERIES_M400)
            {
                // 其他系列：从云台角度数据中读取
                returnCode = DjiFcSubscription_GetLatestValueOfTopic(topicOfPayloadGimablAngle,
                                                                     (uint8_t *)&gimbalAngles,
                                                                     sizeof(T_DjiFcSubscriptionGimbalAngles),
                                                                     &timestamp);
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                {
                    USER_LOG_ERROR("获取云台角度数据失败，错误码: 0x%08X", returnCode);
                    goto end;
                }

                // 将负角度转换为0-360度范围
                if (gimbalAngles.z < 0)
                {
                    gimbalAngles.z = 360 + gimbalAngles.z;
                }

                // 显示读取的云台角度
                USER_LOG_INFO("读取云台角度(俯仰, 横滚, 偏航): p=%.4f r=%.4f y=%.4f",
                              gimbalAngles.x, gimbalAngles.y, gimbalAngles.z);
            }
        }

        break;
    }
    case '7':
    {
        // 1.初始化云台管理器
        returnCode = DjiGimbalManager_Init();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("初始化云台管理器失败，错误码: 0x%08X", returnCode);
            return;
        }

        // 2.订阅云台数据
        returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_THREE_GIMBAL_DATA,
                                                      DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                                      NULL);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("订阅主题 %d 失败，错误码: %d",
                           DJI_FC_SUBSCRIPTION_TOPIC_THREE_GIMBAL_DATA, returnCode);
            return;
        }

        // 3.获取云台数据
        T_DjiFcSubscriptionThreeGimbalData threeGimbalData = {0};
        T_DjiDataTimestamp timestamp = {0};
        returnCode = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_THREE_GIMBAL_DATA, // 订阅主题
                                                             (uint8_t *)&threeGimbalData,                 // 订阅数据地址
                                                             sizeof(T_DjiFcSubscriptionThreeGimbalData),
                                                             &timestamp); // 时间戳
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("获取云台数据失败，错误码: 0x%08X", returnCode);
            return;
        }

        // 4.显示云台数据
        USER_LOG_INFO("云台1角度(俯仰, 横滚, 偏航): p=%.4f r=%.4f y=%.4f",
                      threeGimbalData.anglesData[0].pitch,
                      threeGimbalData.anglesData[0].roll,
                      threeGimbalData.anglesData[0].yaw); //

        // 5.反初始化云台管理器
        returnCode = DjiGimbalManager_Deinit();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("反初始化云台管理器失败，错误码: 0x%08X", returnCode);
        }
        goto start; // 返回主菜单
        break;
    }
    case 'q': // 退出程序
        break;
    default: // 无效输入
        USER_LOG_ERROR("输入的命令无效");
        goto start; // 返回重新输入
    }

// 清理退出标签
end:
    // 反初始化云台管理器
    returnCode = DjiGimbalManager_Deinit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("反初始化云台管理器失败，错误码: 0x%08X", returnCode);
        return;
    }

    USER_LOG_INFO("云台示例程序结束");

    return;
}

/* Private functions definition-----------------------------------------------*/
/**
 * @brief 显示测试选项列表
 * @param SampleList 选项字符串数组
 * @param size 数组大小
 * @retval None
 */
void DjiTest_GimbalManagerShowSampleSelectList(const char **SampleList, uint8_t size)
{
    uint8_t i = 0;

    // 遍历并显示所有选项
    for (i = 0; i < size; i++)
    {
        std::cout << SampleList[i] << std::endl;
    }
}
/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/