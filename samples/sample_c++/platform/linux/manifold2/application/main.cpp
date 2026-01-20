/**
 ********************************************************************
 * @file    main.cpp
 * @brief   PSDK C++示例程序主入口文件 - Manifold2平台
 * @details 本程序提供了PSDK各功能模块的交互式测试菜单，
 *          用户可以通过输入命令来测试不同的功能模块
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law. Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI's authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves right to pursue
 * legal actions for you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
// 实时视频流相关头文件
#include <liveview/test_liveview_entry.hpp>
// 感知系统相关头文件
#include <perception/test_perception_entry.hpp>
#include <perception/test_lidar_entry.hpp>
#include <perception/test_radar_entry.hpp>
// 飞行控制相关头文件
#include <flight_control/test_flight_control.h>
#include <flight_controller/test_flight_controller_entry.h>
// 云台控制相关头文件
#include <gimbal/test_gimbal_entry.hpp>
#include <gimbal_emu/test_payload_gimbal_emu.h>
// 相机相关头文件
#include <camera_emu/test_payload_cam_emu_media.h>
#include <camera_emu/test_payload_cam_emu_base.h>
#include "camera_manager/test_camera_manager_entry.h"
// 应用程序类头文件
#include "application.hpp"
// 飞控数据订阅头文件
#include "fc_subscription/test_fc_subscription.h"
// 日志系统头文件
#include <dji_logger.h>
// 小部件系统头文件
#include "widget/test_widget.h"
#include "widget/test_widget_speaker.h"
// 电源管理头文件
#include <power_management/test_power_management.h>
// 数据传输头文件
#include "data_transmission/test_data_transmission.h"
// 定位系统头文件
#include <positioning/test_positioning.h>
// 健康管理系统头文件
#include <hms_manager/hms_manager_entry.h>

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private values -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/

/* Exported functions definition ---------------------------------------------*/
/**
 * @brief 主函数 - PSDK C++示例程序入口
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出码（本程序为循环运行，正常情况下不会退出）
 *
 * @note 本程序提供了一个交互式菜单，用户可以通过输入命令来测试不同的PSDK功能模块
 *       程序会持续运行，等待用户输入命令
 */
int main(int argc, char **argv)
{
    // 创建Application对象，负责系统初始化和SDK启动
    Application application(argc, argv);
    // 用户输入字符变量，用于接收菜单选择
    char inputChar;
    // 获取OSAL（操作系统抽象层）处理器，用于跨平台操作
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    // SDK返回码变量
    T_DjiReturnCode returnCode;
    // 高功率申请处理器（用于E-Port等需要高功率的场景）
    T_DjiTestApplyHighPowerHandler applyHighPowerHandler;

// 程序主循环标签，用于实现菜单的循环显示
start:
    // 显示功能菜单
    std::cout
        << "\n"
        << "| Available commands:                                                                              |\n"
        << "| [0] Fc subscribe sample - subscribe quaternion and gps data                                      |\n"
        << "|     飞控订阅示例 - 订阅四元数和GPS数据                                                          |\n"
        << "| [1] Flight controller sample - you can control flying by PSDK                                    |\n"
        << "|     飞行控制示例 - 通过PSDK控制飞行                                                            |\n"
        << "| [2] Hms info manager sample - get health manger system info by language                          |\n"
        << "|     健康管理系统示例 - 获取健康管理系统信息（支持多语言）                                      |\n"
        << "| [a] Gimbal manager sample - you can control gimbal by PSDK                                       |\n"
        << "|     云台管理示例 - 通过PSDK控制云台                                                            |\n"
        << "| [c] Camera stream view sample - display camera video stream                                  |\n"
        << "|     相机视频流示例 - 显示相机视频流                                                            |\n"
        << "| [d] Stereo vision view sample - display stereo image                                         |\n"
        << "|     双目视觉示例 - 显示双目图像                                                                |\n"
        << "| [e] Run camera manager sample - you can test camera's functions interactively                    |\n"
        << "|     相机管理示例 - 交互式测试相机功能                                                          |\n"
        << "| [f] Start rtk positioning sample - you can receive rtk rtcm data when rtk signal is ok           |\n"
        << "|     RTK定位示例 - 在RTK信号正常时接收RTK RTCM数据                                            |\n"
        << "| [g] Request Lidar data sample - Request Lidar data and store point cloud data as pcd files   |\n"
        << "|     激光雷达数据示例 - 请求激光雷达数据并将点云数据存储为PCD文件                             |\n"
        << "| [h] Request Radar data sample - Request radar data                                               |\n"
        << "|     毫米波雷达数据示例 - 请求雷达数据                                                          |\n"
        << std::endl;

    // 获取用户输入
    std::cin >> inputChar;
    // 根据用户输入执行对应的功能模块
    switch (inputChar)
    {
    case '0':
        // 运行飞控数据订阅示例
        DjiTest_FcSubscriptionRunSample();
        break;
    case '1':
        // 运行飞行控制示例
        DjiUser_RunFlightControllerSample();
        break;
    case '2':
        // 运行健康管理系统示例
        DjiUser_RunHmsManagerSample();
        break;
    case 'a':
        // 运行云台管理示例
        DjiUser_RunGimbalManagerSample();
        break;
    case 'c':
        // 运行相机视频流显示示例
        DjiUser_RunCameraStreamViewSample();
        break;
    case 'd':
        // 运行双目视觉显示示例
        DjiUser_RunStereoVisionViewSample();
        break;
    case 'e':
        // 运行相机管理示例
        DjiUser_RunCameraManagerSample();
        break;
    case 'f':
        // 启动RTK定位服务
        returnCode = DjiTest_PositioningStartService();
        // 检查初始化是否成功
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("rtk positioning sample init error");
            break;
        }

        USER_LOG_INFO("Start rtk positioning sample successfully");
        break;
    case 'g':
        // 运行激光雷达数据订阅示例
        DjiUser_RunLidarDataSubscriptionSample();
        break;
    case 'h':
        // 运行毫米波雷达数据订阅示例
        DjiUser_RunRadarDataSubscriptionSample();
        break;
    default:
        // 无效输入，直接返回菜单
        break;
    }

    // 等待2秒后重新显示菜单
    osalHandler->TaskSleepMs(2000);

    // 跳转到程序开始处，实现循环菜单
    goto start;
}

/* Private functions definition-----------------------------------------------*/

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
