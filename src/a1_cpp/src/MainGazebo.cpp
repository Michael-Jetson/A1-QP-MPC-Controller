//
// Created by zixin on 11/1/21.
//
// stl
#include <iostream>
#include <iomanip>
#include <memory>
#include <thread>
#include <chrono>

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>

// control parameters
#include "A1Params.h"
// A1 control
#include "GazeboA1ROS.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "gazebo_a1_qp_ctrl");
    ros::NodeHandle nh;

    // change ros logger
    // 将默认日志级设置为 Debug，确保在控制循环中输出细节
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        // 让修改马上生效
        ros::console::notifyLoggerLevelsChanged();
    }

    // 使用ROS的仿真时间，否则控制器无法基于准确的时间步执行
    // make sure the ROS infra using sim time, otherwise the controller cannot run with correct time steps
    // 这是因为使用仿真时间的话，ros::Time::now() 与 Gazebo 时钟才可以同步
    std::string use_sim_time;
    if (ros::param::get("/use_sim_time", use_sim_time)) {
        if (use_sim_time != "true") {
            std::cout << "ROS must set use_sim_time in order to use this program!" << std::endl;
            return -1;
        }
    }

    // 创建A1控制器的智能指针
    std::unique_ptr<GazeboA1ROS> a1 = std::make_unique<GazeboA1ROS>(nh);

    // 声明原子bool变量，用于线程之间的同步，并且可以保证线程安全
    std::atomic<bool> control_execute{};
    // 初始值为true，使用memory_order_release保证写操作对其他线程可见
    control_execute.store(true, std::memory_order_release);

    // Thread 1: compute desired ground forces
    // 线程1：计算期望的地面作用力
    std::cout << "Enter thread 1: compute desired ground forces" << std::endl;
    // 开启第一个线程，并且引用外部所有变量，实现变量的线程间共享，并且避免全局变量
    std::thread compute_foot_forces_grf_thread([&]() {
        // prepare variables to monitor time and control the while loop
        //记录下循环开始时间，用来求累计运行时间 elapsed
        ros::Time start = ros::Time::now();
        //上一次循环时间
        ros::Time prev = ros::Time::now();
        //当前时间
        ros::Time now = ros::Time::now();  // bool res = app.exec();
        //时间增量，初始值为0
        ros::Duration dt(0);

        while (control_execute.load(std::memory_order_acquire) && ros::ok()) {
//            auto t1 = std::chrono::high_resolution_clock::now();
            //线程休眠，控制循环频率，若 GRF_UPDATE_FREQUENCY 设置为2.5，则控制周期为2.5ms、频率为400Hz
            //但是此处没有时间补偿，只是固定时间休眠，如果算上MPC计算用时，控制频率会有一定程度下降
            ros::Duration(GRF_UPDATE_FREQUENCY / 1000).sleep();

            // get t and dt
            // 更新时间变量
            now = ros::Time::now();
            dt = now - prev;
            prev = now;
            //累计运行时间
            ros::Duration elapsed = now - start;

            // 记录 MPC 求解前后高精度时间
            auto t1 = std::chrono::high_resolution_clock::now();

            // 核心算法：计算期望的地面作用力
            bool running = a1->update_foot_forces_grf(dt.toSec());

            auto t2 = std::chrono::high_resolution_clock::now();
            // 高精度的 MPC 求解所用时间
            std::chrono::duration<double, std::milli> ms_double = t2 - t1;
            std::cout << "MPC solution is updated in " << ms_double.count() << "ms" << std::endl;

            // 终止循环条件
            if (!running) {
                std::cout << "Thread 1 loop is terminated because of errors." << std::endl;
                ros::shutdown();
                std::terminate();
                break;
            }
        }
    });

    // Thread 2: update robot states, compute desired swing legs forces, compute desired joint torques, and send commands
    std::cout << "Enter thread 2: update robot states, compute desired swing legs forces, compute desired joint torques, and send commands"
              << std::endl;
    std::thread main_thread([&]() {
        // prepare variables to monitor time and control the while loop
        // 时间记录
        ros::Time start = ros::Time::now();
        ros::Time prev = ros::Time::now();
        ros::Time now = ros::Time::now();  // bool res = app.exec();
        ros::Duration dt(0);

        while (control_execute.load(std::memory_order_acquire) && ros::ok()) {
            auto t3 = std::chrono::high_resolution_clock::now();

            // 频率控制
            ros::Duration(MAIN_UPDATE_FREQUENCY / 1000).sleep();

            // get t and dt
            // 更新时间
            now = ros::Time::now();
            dt = now - prev;
            prev = now;
            ros::Duration elapsed = now - start;

            // compute desired ground forces
            bool main_update_running = a1->main_update(elapsed.toSec(), dt.toSec());
            // 将扭矩/命令发布出去
            bool send_cmd_running = a1->send_cmd();

            auto t4 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> ms_double = t4 - t3;
            // std::cout << "Thread 2 is updated in " << ms_double.count() << "ms" << std::endl;

            // 终止循环条件
            if (!main_update_running || !send_cmd_running) {
                std::cout << "Thread 2 loop is terminated because of errors." << std::endl;
                ros::shutdown();
                std::terminate();
                break;
            }
        }
    });
    // 让多达 12 条内部线程并行处理 ROS 回调（IMU、关节、joy 等消息），保证控制线程不会阻塞在回调上
    ros::AsyncSpinner spinner(12);
    spinner.start();
    // 主线程等待两个工作线程 join()，确保程序在退出前正确回收资源
    compute_foot_forces_grf_thread.join();
    main_thread.join();

    return 0;
}
