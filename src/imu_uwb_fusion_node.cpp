#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <iomanip>

#include <fstream>
#include <iostream>
// #include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <yaml-cpp/yaml.h>

#include <mutex>
#include <thread>
#include <vector>
#include <deque>

#include "imu_uwb_fusion.h"
#include "imu_uwb_fusion/UwbMsg.h"
#include "imu_uwb_fusion/LinktrackNodeframe2.h"

using namespace std;

int imu_freq = 100; // imu frequency is location frequency

nav_msgs::Path path; //result path

deque<sensor_msgs::ImuConstPtr> imu_buffer;
deque<imu_uwb_fusion::UwbMsg::ConstPtr> uwb_buffer;
deque<geometry_msgs::PoseStamped> vicon_buffer;

mutex imu_mtx, uwb_mtx, vicon_mtx;
std::string pose_path, imu_path;

Fusion::ImuUwbFusion imu_uwb_fuser; // fuser object
ros::Publisher traj_puber;
ros::Publisher result_puber;
bool initialized = false;
Fusion::ImuData<double> last_uwb_imu;     //last interpolated imu data at uwb time
Fusion::ImuData<double> last_imu;         //last imu data for predict
Fusion::State<double> last_updated_state; //last updated state by uwb

Fusion::ImuData<double> fromImuMsg(const sensor_msgs::Imu &msg)
{
    Fusion::ImuData<double> imu_data;
    imu_data.stamp = msg.header.stamp.toSec();
    imu_data.gyr[0] = msg.angular_velocity.x;
    imu_data.gyr[1] = msg.angular_velocity.y;
    imu_data.gyr[2] = msg.angular_velocity.z;
    imu_data.acc[0] = msg.linear_acceleration.x;
    imu_data.acc[1] = msg.linear_acceleration.y;
    imu_data.acc[2] = msg.linear_acceleration.z;

    return move(imu_data);
}

Fusion::UwbData<double> fromUwbMsg(const imu_uwb_fusion::UwbMsg &msg)
{
    Fusion::UwbData<double> uwb_data;
    uwb_data.data[0] = msg.pos_x;
    uwb_data.data[1] = msg.pos_y;
    uwb_data.data[2] = msg.pos_z;
    // uwb_data.data[2] = 0.2;
    uwb_data.cov.setIdentity();
    // uwb_data.cov(0, 0) = 0.00002; // need to fix it
    // uwb_data.cov(1, 1) = 0.00002;
    // uwb_data.cov(2, 2) = 0.00002;
    uwb_data.cov(0, 0) = 1e-4; // need to fix it
    uwb_data.cov(1, 1) = 1e-4;
    uwb_data.cov(2, 2) = 1e-5;
    return move(uwb_data);
}

void interpolateImuData(const sensor_msgs::ImuConstPtr &first_data,
                        const sensor_msgs::ImuConstPtr &second_data,
                        double cur_stamp, sensor_msgs::Imu &inter_data)
{
    double first_stamp = first_data->header.stamp.toSec();
    double second_stamp = second_data->header.stamp.toSec();
    double scale = (cur_stamp - first_stamp) / (second_stamp - first_stamp);
    inter_data = *first_data;
    inter_data.angular_velocity.x = scale * (second_data->angular_velocity.x -
                                             first_data->angular_velocity.x) +
                                    first_data->angular_velocity.x;
    inter_data.angular_velocity.y = scale * (second_data->angular_velocity.y -
                                             first_data->angular_velocity.y) +
                                    first_data->angular_velocity.y;
    inter_data.angular_velocity.z = scale * (second_data->angular_velocity.z -
                                             first_data->angular_velocity.z) +
                                    first_data->angular_velocity.z;
    inter_data.linear_acceleration.x =
        scale * (second_data->linear_acceleration.x -
                 first_data->linear_acceleration.x) +
        first_data->linear_acceleration.x;
    inter_data.linear_acceleration.y =
        scale * (second_data->linear_acceleration.y -
                 first_data->linear_acceleration.y) +
        first_data->linear_acceleration.y;
    inter_data.linear_acceleration.z =
        scale * (second_data->linear_acceleration.z -
                 first_data->linear_acceleration.z) +
        first_data->linear_acceleration.z;
}

void pubResult()
{
    // if (traj_puber.getNumSubscribers() != 0)
    // {
    Fusion::State<double> result = imu_uwb_fuser.getNominalState();
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    // pose.header.stamp = uwb_buffer[0]->header.stamp;
    pose.header.stamp = ros::Time(result.stamp);
    pose.pose.position.x = result.p[0];
    pose.pose.position.y = result.p[1];
    pose.pose.position.z = result.p[2];
    pose.pose.orientation.w = result.q.w();
    pose.pose.orientation.x = result.q.x();
    pose.pose.orientation.y = result.q.y();
    pose.pose.orientation.z = result.q.z();
    path.poses.push_back(pose);
    path.header = pose.header;
    traj_puber.publish(path);
    result_puber.publish(pose);

    std::ofstream file(pose_path, std::ios::app);
    if (file.is_open())
    {
        file << std::setprecision(19) << pose.header.stamp.toSec() << " " << pose.pose.position.x << " "
             << pose.pose.position.y << " " << pose.pose.position.z << " "
             << pose.pose.orientation.x << " " << pose.pose.orientation.y << " "
             << pose.pose.orientation.z << " " << pose.pose.orientation.w
             << std::endl;
    }
    else
    {
        std::cerr << "Cannot open: " << pose_path << std::endl;
    }

    std::cout << pose.header.stamp << " " << pose.pose.position.x << " "
              << pose.pose.position.y << " " << pose.pose.position.z << " "
              << pose.pose.orientation.x << " " << pose.pose.orientation.y << " "
              << pose.pose.orientation.z << " " << pose.pose.orientation.w << std::endl;
    // }
}

bool TEST()
{
    unique_lock<mutex> imu_lock(imu_mtx);
    unique_lock<mutex> uwb_lock(uwb_mtx);

    // if no data, no need update state
    if (!imu_buffer.size() && !uwb_buffer.size())
    {
        ROS_INFO_THROTTLE(10, "wait for uwb or imu msg ......");
        imu_lock.unlock();
        uwb_lock.unlock();

        return false;
    }
    // init correlative param
    if (!initialized && !imu_buffer.empty() && !uwb_buffer.empty())
    {
        // use imu datas at start to initial imu pose
        vector<Fusion::ImuData<double>> imu_datas;
        for (auto &imu_msg : imu_buffer)
        {
            imu_datas.push_back(fromImuMsg(*imu_msg));
        }
        imu_uwb_fuser.imuInit(imu_datas);
        // set reference ll for convert ll to enu frame
        imu_uwb_fuser.cfgRefUwb(uwb_buffer[0]->pos_x, uwb_buffer[0]->pos_y,
                                uwb_buffer[0]->pos_z);

        // interpolate first imu data at first uwb time
        auto iter = imu_buffer.begin();
        for (; iter != imu_buffer.end(); iter++)
        {
            if ((*iter)->header.stamp > uwb_buffer[0]->header.stamp)
                break;
        }
        // cant find imu data before or after gps data
        if (imu_buffer.begin() == iter || imu_buffer.end() == iter)
        {
            if (imu_buffer.begin() == iter)
                // uwb_buffer.erase(uwb_buffer.begin());
                uwb_buffer.pop_front();
            // no imu data before first gps data, cant
            // interpolate at gps stamp
            imu_lock.unlock();
            uwb_lock.unlock();
            return true;
        }
        sensor_msgs::Imu inter_imu;
        double cur_stamp = uwb_buffer[0]->header.stamp.toSec();
        interpolateImuData(*(iter - 1), *iter, cur_stamp, inter_imu);

        // record last gps frame time and interpolated imu data
        last_uwb_imu = fromImuMsg(inter_imu);
        last_uwb_imu.stamp = cur_stamp;
        last_imu = last_uwb_imu;
        last_updated_state = imu_uwb_fuser.getState();

        // delete old imu datas and gps data
        imu_buffer.erase(imu_buffer.begin(), iter);
        // uwb_buffer.erase(uwb_buffer.begin());
        uwb_buffer.pop_front();

        imu_lock.unlock();
        uwb_lock.unlock();

        initialized = true;
        return true;
    }

    // use imu predict location for increase locate frequency
    // actual state no change
    for (auto &imu_msg : imu_buffer)
    {
        Fusion::ImuData<double> cur_imu = fromImuMsg(*imu_msg);
        if (cur_imu.stamp >= last_imu.stamp)
        {
            imu_uwb_fuser.updateNominalState(last_imu, cur_imu);
            last_imu = cur_imu;
        }
        else
        {
            ROS_INFO(".initialized...%f, %f", cur_imu.stamp, last_imu.stamp);
        }
    }

    // use uwb data to update state
    if (uwb_buffer.size() != 0)
    {
        // if (uwb_buffer.front()->status.status != 2)
        // {
        //     cout << "gps data is bad !!!" << endl;
        //     gps_buffer.erase(gps_buffer.begin());
        //     imu_lock.unlock();
        //     gps_lock.unlock();
        //     loop_rate.sleep();
        //     continue;
        // }

        // recover to last updated state for imu predict again
        imu_uwb_fuser.recoverState(last_updated_state);

        // collect imu datas during two neighbor gps frames
        vector<Fusion::ImuData<double>> imu_datas(0);
        // search first imu data after gps data
        auto iter = imu_buffer.begin();
        for (; iter != imu_buffer.end(); iter++)
        {
            if ((*iter)->header.stamp > uwb_buffer[0]->header.stamp)
            {
                break;
            }
        }
        if (imu_buffer.end() == iter)
        {
            ROS_WARN("no imu data after first gps data, wait for new imu data");
            imu_lock.unlock();
            uwb_lock.unlock();
            return false;
        }
        assert(imu_buffer.begin() != iter);
        // add last gps_imu data (interpolated data at gps time)
        imu_datas.push_back(last_uwb_imu);
        // add imu data between last gps_imu data and current gps_imu data
        for (auto tmp_iter = imu_buffer.begin(); tmp_iter != iter; tmp_iter++)
            imu_datas.push_back(fromImuMsg(*(*tmp_iter)));
        // add current gps_imu data
        sensor_msgs::Imu inter_imu;
        double cur_stamp = uwb_buffer[0]->header.stamp.toSec();

        if (iter == imu_buffer.begin())
        {
            iter++;
        }
        interpolateImuData(*(iter - 1), *iter, cur_stamp, inter_imu);
        Fusion::ImuData<double> cur_uwb_imu = fromImuMsg(inter_imu);
        cur_uwb_imu.stamp = cur_stamp;
        imu_datas.push_back(cur_uwb_imu);

        // generate uwb data
        Fusion::UwbData<double> uwb_data = fromUwbMsg(*uwb_buffer[0]);
        if (!uwb_buffer.empty())
        {
            uwb_data.stamp = uwb_buffer[0]->header.stamp.toSec();
        }
        // update state (core)
        imu_uwb_fuser.uwbUpdate(uwb_data, imu_datas);

        // update last data
        last_uwb_imu = cur_uwb_imu;
        last_imu = last_uwb_imu;

        // delete old data
        uwb_buffer.erase(uwb_buffer.begin());
        imu_buffer.erase(imu_buffer.begin(), iter);

        // update last state
        last_updated_state = imu_uwb_fuser.getState();
    }

    // publish result
    pubResult();

    imu_lock.unlock();
    uwb_lock.unlock();
    return true;
}

void imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
    unique_lock<mutex> lock(imu_mtx);
    std::ofstream file(imu_path, std::ios::app);
    if (file.is_open())
    {
        file << std::setprecision(19) << msg->header.stamp.toSec() << " " << msg->linear_acceleration.x << " " << msg->linear_acceleration.y << " " << msg->linear_acceleration.z << " "
             << msg->angular_velocity.x << " " << msg->angular_velocity.y << " " << msg->angular_velocity.z << " " << 1
             << std::endl;
    }
    else
    {
        std::cerr << "Cannot open: " << pose_path << std::endl;
    }
    imu_buffer.push_back(msg);
    lock.unlock();
    // Calculate the time cost of TEST() function
    // auto start_time = std::chrono::high_resolution_clock::now();

    // // Call the TEST() function
    // TEST();

    // // Calculate the time cost
    // auto end_time = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    // double time_cost = duration.count() / 1000.0; // Convert to milliseconds

    // // Print the time cost
    // std::cout << "Time cost of TEST(): " << time_cost << " ms" << std::endl;
}

void uwbCallback(const imu_uwb_fusion::LinktrackNodeframe2::ConstPtr &msg)
{
    // unique_lock<mutex> lock(uwb_mtx);
    // uwb_buffer.push_back(msg);
    imu_uwb_fusion::LinktrackNodeframe2 tmp_in = *msg;
    imu_uwb_fusion::UwbMsg::Ptr output(new imu_uwb_fusion::UwbMsg);
    double min_duration = 1e18;
    double duration = 0.0;
    double input_stamp = tmp_in.local_time * 1e-6;

    // Find the vicon pose with the closest timestamp
    geometry_msgs::PoseStamped closest_pose;
    auto iter = vicon_buffer.begin();
    if (!vicon_buffer.empty())
    {
        closest_pose = vicon_buffer.front();
        // min_duration = std::abs(input.header.stamp.toSec() - closest_pose->header.stamp.toSec());
        for (auto it = vicon_buffer.begin(); it != vicon_buffer.end(); ++it)
        {
            duration = std::abs(input_stamp - it->header.stamp.toSec());
            if (duration < min_duration)
            {
                min_duration = duration;
                closest_pose = *it;
                iter = it;
            }
        }
        vicon_buffer.erase(iter);
    }
    else
    {
        ROS_WARN("vicon buffer is empty");
    }

    uint32_t seconds = tmp_in.local_time * 1e-6;
    uint32_t nanoseconds = (tmp_in.local_time % 1000000) * 1000;
    output->header.stamp = ros::Time(seconds, nanoseconds);
    output->pos_x = tmp_in.pos_3d.at(0);
    output->pos_y = tmp_in.pos_3d.at(1);
    output->pos_z = closest_pose.pose.position.z;

    uwb_buffer.push_back(output);

    auto start_time = std::chrono::high_resolution_clock::now();
    TEST();
    auto end_time = std::chrono::high_resolution_clock::now();
    auto dura_ = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    double time_cost = dura_.count() / 1000.0; // Convert to milliseconds
    std::cout << "Time cost of TEST(): " << time_cost << " ms" << std::endl;
    // lock.unlock();
}

void viconCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    unique_lock<mutex> lock(vicon_mtx);
    vicon_buffer.push_back(*msg);
    lock.unlock();
}

void processThread()
{
    std::cout << "processThread" << std::endl;
    ros::Rate loop_rate(imu_freq);
    while (ros::ok())
    {
        std::cout << "ros::ok(): " << ros::ok() << imu_buffer.size() << " "
                  << uwb_buffer.size() << std::endl;

        unique_lock<mutex> imu_lock(imu_mtx);
        unique_lock<mutex> uwb_lock(uwb_mtx);

        // if no data, no need update state
        if (!imu_buffer.size() && !uwb_buffer.size())
        {
            ROS_INFO_THROTTLE(10, "wait for uwb or imu msg ......");
            imu_lock.unlock();
            uwb_lock.unlock();
            loop_rate.sleep();
            // continue;
        }
        std::cout << "initialized: " << initialized << std::endl;
        // init correlative param
        if (!initialized)
        {
            // wait for enough sensor data to init
            if (!imu_buffer.size() || !uwb_buffer.size())
            {
                ROS_INFO_THROTTLE(10, "wait for uwb or imu msg ......");
                imu_lock.unlock();
                uwb_lock.unlock();
                loop_rate.sleep();
                continue;
            }

            // use imu datas at start to initial imu pose
            vector<Fusion::ImuData<double>> imu_datas;
            for (auto &imu_msg : imu_buffer)
            {
                imu_datas.push_back(fromImuMsg(*imu_msg));
            }
            imu_uwb_fuser.imuInit(imu_datas);

            // set reference ll for convert ll to enu frame
            imu_uwb_fuser.cfgRefUwb(uwb_buffer[0]->pos_x, uwb_buffer[0]->pos_y,
                                    uwb_buffer[0]->pos_z);

            // interpolate first imu data at first uwb time
            auto iter = imu_buffer.begin();
            for (; iter != imu_buffer.end(); iter++)
            {
                if ((*iter)->header.stamp > uwb_buffer[0]->header.stamp)
                    break;
            }
            if (imu_buffer.begin() == iter ||
                imu_buffer.end() ==
                    iter) // cant find imu data before or after gps data
            {
                if (imu_buffer.begin() == iter)
                    uwb_buffer.erase(
                        uwb_buffer.begin()); // no imu data before first gps data, cant
                                             // interpolate at gps stamp
                imu_lock.unlock();
                uwb_lock.unlock();
                loop_rate.sleep();
                continue;
            }
            sensor_msgs::Imu inter_imu;
            double cur_stamp = uwb_buffer[0]->header.stamp.toSec();
            interpolateImuData(*(iter - 1), *iter, cur_stamp, inter_imu);

            // record last gps frame time and interpolated imu data
            last_uwb_imu = fromImuMsg(inter_imu);
            last_uwb_imu.stamp = cur_stamp;
            last_imu = last_uwb_imu;
            last_updated_state = imu_uwb_fuser.getState();

            // delete old imu datas and gps data
            imu_buffer.erase(imu_buffer.begin(), iter);
            uwb_buffer.erase(uwb_buffer.begin());

            imu_lock.unlock();
            uwb_lock.unlock();
            loop_rate.sleep();

            initialized = true;

            continue;
        }

        // use imu predict location for increase locate frequency
        // actual state no change
        for (auto &imu_msg : imu_buffer)
        {
            Fusion::ImuData<double> cur_imu = fromImuMsg(*imu_msg);
            if (cur_imu.stamp > last_imu.stamp)
            {
                imu_uwb_fuser.updateNominalState(last_imu, cur_imu);
                last_imu = cur_imu;
            }
        }

        // use uwb data to update state
        if (uwb_buffer.size() != 0)
        {
            // if (uwb_buffer.front()->status.status != 2)
            // {
            //     cout << "gps data is bad !!!" << endl;
            //     gps_buffer.erase(gps_buffer.begin());
            //     imu_lock.unlock();
            //     gps_lock.unlock();
            //     loop_rate.sleep();
            //     continue;
            // }

            // recover to last updated state for imu predict again
            imu_uwb_fuser.recoverState(last_updated_state);

            // collect imu datas during two neighbor gps frames
            vector<Fusion::ImuData<double>> imu_datas(0);
            // search first imu data after gps data
            auto iter = imu_buffer.begin();
            for (; iter != imu_buffer.end(); iter++)
            {
                if ((*iter)->header.stamp > uwb_buffer[0]->header.stamp)
                    break;
            }
            if (imu_buffer.end() ==
                iter) // no imu data after first gps data, wait for new imu data
            {
                imu_lock.unlock();
                uwb_lock.unlock();
                loop_rate.sleep();
                continue;
            }
            assert(imu_buffer.begin() != iter);
            // add last gps_imu data (interpolated data at gps time)
            imu_datas.push_back(last_uwb_imu);
            // add imu data between last gps_imu data and current gps_imu data
            for (auto tmp_iter = imu_buffer.begin(); tmp_iter != iter; tmp_iter++)
                imu_datas.push_back(fromImuMsg(*(*tmp_iter)));
            // add current gps_imu data
            sensor_msgs::Imu inter_imu;
            double cur_stamp = uwb_buffer[0]->header.stamp.toSec();
            interpolateImuData(*(iter - 1), *iter, cur_stamp, inter_imu);
            Fusion::ImuData<double> cur_uwb_imu = fromImuMsg(inter_imu);
            cur_uwb_imu.stamp = cur_stamp;
            imu_datas.push_back(cur_uwb_imu);

            // generate uwb data
            Fusion::UwbData<double> uwb_data = fromUwbMsg(*uwb_buffer[0]);

            // update state (core)
            imu_uwb_fuser.uwbUpdate(uwb_data, imu_datas);

            // update last data
            last_uwb_imu = cur_uwb_imu;
            last_imu = last_uwb_imu;

            // delete old data
            uwb_buffer.erase(uwb_buffer.begin());
            imu_buffer.erase(imu_buffer.begin(), iter);

            // update last state
            last_updated_state = imu_uwb_fuser.getState();
        }

        // publish result
        pubResult();

        imu_lock.unlock();
        uwb_lock.unlock();
        loop_rate.sleep();
        continue;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_uwb_fusion_node");
    ros::NodeHandle nh;

    pose_path = "/home/mint/ws_fusion_uwb/res/fusion_pose.txt";
    imu_path = "/home/mint/ws_fusion_uwb/res/imu.txt";
    if (std::ifstream(pose_path))
    {
        std::remove(pose_path.c_str());
    }
    if (std::ifstream(imu_path))
    {
        std::remove(imu_path.c_str());
    }
    // load imu param and config fuser
    double sigma_an, sigma_wn, sigma_aw, sigma_ww;
    // if (!nh.getParam("sigma_an", sigma_an) ||
    //     !nh.getParam("sigma_wn", sigma_wn) ||
    //     !nh.getParam("sigma_aw", sigma_aw) ||
    //     !nh.getParam("sigma_ww", sigma_ww)) {
    //   cout << "please config imu param !!!" << endl;
    //   return 0;
    // }
    sigma_an = 3.5848651612538265e+04;
    sigma_an = 3.5848651612538265e+01;
    sigma_wn = 5.0319853834530663e-00;
    sigma_aw = 1.4189758078282432e-03;
    sigma_ww = 1.3487170893986536e-05;
    // sigma_an = 1e-1;
    // sigma_wn = 1e-1;
    // sigma_aw = 1e-1;
    // sigma_ww = 1e-1;
    imu_uwb_fuser.cfgImuVar(sigma_an, sigma_wn, sigma_aw, sigma_ww);

    // load imu freqency param
    if (!nh.getParam("imu_freq", imu_freq))
    {
        cout << "no config imu_freq param, use default: " << " " << imu_freq << endl;
    }

    ros::Subscriber imu_sub =
        nh.subscribe<sensor_msgs::Imu>("/livox/imu", 1000, imuCallback);
    ros::Subscriber uwb_sub =
        nh.subscribe<imu_uwb_fusion::LinktrackNodeframe2>("/nlink_linktrack_nodeframe2", 100, uwbCallback);
    ros::Subscriber vicon_sub =
        nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/uwb_ngx_tag/pose", 100, viconCallback);

    traj_puber = nh.advertise<nav_msgs::Path>("traj", 1);
    result_puber = nh.advertise<geometry_msgs::PoseStamped>("result", 1);

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}
