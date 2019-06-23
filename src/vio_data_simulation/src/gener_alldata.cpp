#include <ros/ros.h> 
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include "tf/transform_listener.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include "nav_msgs/Path.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <fstream>

#include "imu.h"
#include "utilities.h"

class Generate_AllanData
{
    public:
    // IMU model
    Param params;
    IMU imuGen;

    ros::NodeHandle m_n;
    ros::Publisher m_IMU_pub;
    ros::Publisher m_RealOdom_pub;
    ros::Publisher m_VioOdom_pub;
    ros::Publisher m_RealPath_pub;
    ros::Publisher m_eulerPath_pub;
    ros::Publisher m_midPath_pub;
    std::vector<Eigen::Vector3d> m_RealOdomPosition;
    std::vector<Eigen::Quaterniond> m_RealOdomRotation;

    std::vector<Eigen::Vector3d> m_VioOdomPosition;
    std::vector<Eigen::Quaterniond> m_VioOdomRotation;

    tf::TransformBroadcaster RealOdomTf_broadcaster;

    public:

    Generate_AllanData():imuGen(params)
    {
        m_IMU_pub = m_n.advertise<sensor_msgs::Imu>("/imu", 10);
        m_RealOdom_pub = m_n.advertise<nav_msgs::Odometry>("/real_odom", 10);
        m_VioOdom_pub = m_n.advertise<nav_msgs::Odometry>("/vio_odom", 10);
        m_RealPath_pub = m_n.advertise<nav_msgs::Path>("/real_path", 10);
        m_eulerPath_pub = m_n.advertise<nav_msgs::Path>("/euler_path", 10);
        m_midPath_pub = m_n.advertise<nav_msgs::Path>("/mid_path", 10);
    }

    void PublishPath(ros::Publisher& puber, std::vector<Eigen::Vector3d> position, std::vector<Eigen::Quaterniond> rotation)
    {
        nav_msgs::Path path_msg;
        geometry_msgs::PoseStamped pose;

        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "/vio_odom_link";
        
        for(int i = 0; i < position.size();i++)
        {
            Eigen::Quaterniond Q_from_R = rotation[i];
            Eigen::Vector3d traj_node = position[i];
            pose.pose.position.x = traj_node(0);
            pose.pose.position.y = traj_node(1);
            pose.pose.position.z = traj_node(2);
            pose.pose.orientation.x = Q_from_R.x();
            pose.pose.orientation.y = Q_from_R.y();
            pose.pose.orientation.z = Q_from_R.z();
            pose.pose.orientation.w = Q_from_R.w();
            path_msg.poses.push_back(pose);
        }

        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "/vio_odom_link";
        puber.publish(path_msg);
    }

    void publishOdomData(ros::Publisher& puber, Eigen::Vector3d tempCameraPose, Eigen::Quaterniond tem_q)
    {
        static Eigen::Vector3d LastCameraPose;
        geometry_msgs::TransformStamped OdomTf;
        geometry_msgs::Quaternion odom_quat;

        odom_quat.x = tem_q.x();
        odom_quat.y = tem_q.y();
        odom_quat.z = tem_q.z();
        odom_quat.w = tem_q.w();

        ros::Time current_time = ros::Time::now();
        OdomTf.header.stamp = current_time;
        OdomTf.header.frame_id = "vio_odom_link";
        OdomTf.child_frame_id = "imu_link";
        //set the relative pose
        OdomTf.transform.translation.x = tempCameraPose(0);
        OdomTf.transform.translation.y = tempCameraPose(1);
        OdomTf.transform.translation.z = tempCameraPose(2);
        OdomTf.transform.rotation = odom_quat;
        //send the transform
        RealOdomTf_broadcaster.sendTransform(OdomTf);

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "vio_odom_link";
        odom.child_frame_id = "imu_link";
        //set the position
        odom.pose.pose.position.x = tempCameraPose(0);
        odom.pose.pose.position.y = tempCameraPose(1);
        odom.pose.pose.position.z = tempCameraPose(2);
        odom.pose.pose.orientation = odom_quat;
        //set the velocity
        odom.twist.twist.linear.x = tempCameraPose(0) - LastCameraPose(0);
        odom.twist.twist.linear.y = tempCameraPose(1) - LastCameraPose(1);
        odom.twist.twist.linear.z = tempCameraPose(2) - LastCameraPose(2);
        odom.twist.twist.angular.x = 0.0;
        LastCameraPose = tempCameraPose;
        //publish the message
        // m_RealOdom_pub.publish(odom);
        puber.publish(odom);
    }

    void publishImuData(ros::Publisher& puber, Eigen::Vector3d acc, Eigen::Vector3d gyr, Eigen::Quaterniond tem_q)
    {
        // imu data
        sensor_msgs::Imu imu_data;

        imu_data.header.stamp = ros::Time::now();
        imu_data.header.frame_id = "imu_link";
        //四元数位姿
        imu_data.orientation.x = tem_q.x();
        imu_data.orientation.y = tem_q.y();
        imu_data.orientation.z = tem_q.z();
        imu_data.orientation.w = tem_q.w();
        //线加速度
        imu_data.linear_acceleration.x = acc(0); 
        imu_data.linear_acceleration.y = acc(1);
        imu_data.linear_acceleration.z = acc(2);
        //角速度
        imu_data.angular_velocity.x = gyr(0); 
        imu_data.angular_velocity.y = gyr(1); 
        imu_data.angular_velocity.z = gyr(2);
        //publish the message
        // m_IMU_pub.publish(imu_data);
        puber.publish(imu_data);
    }

    void generate_imu_rosbag_data(char flag = 0)
    {
        rosbag::Bag bag;
        bag.open("/home/suyun/shenlan_course/shenlan_vio/course2/course-code/vio-data-simulation-ws/src/vio_data_simulation/data/imu.bag", rosbag::bagmode::Write);

        // ros::Time::init();
        double begin =ros::Time::now().toSec();
        std::cout << " start generate data, please waiting..."<<std::endl;

        char bar[102]={0};
        const char symbol[4] = {'|','/','-','\\'};

        MotionData data;
        
        for (double t = params.t_start; t<params.t_end;) 
        {
            if( (int)t % params.imu_frequency == 0)
            {
                int i = (int)( (t - params.t_start) / (params.t_end - params.t_start) * 100);
                bar[i] = '#';
                printf("[%s][%d%%][%c]\r", bar, i, symbol[i%4]);
                fflush(stdout);
            }	

            // create imu data && add imu noise
            if(flag == 0)  
                data = imuGen.MotionModel(0);
            else           
                data = imuGen.MotionModel(t);
            MotionData data_noise = data;
            imuGen.addIMUnoise(data_noise);
    
            // R to q
            Eigen::Quaterniond q(data_noise.Rwb);

            // imu data
            sensor_msgs::Imu imu_data;
            ros::Time time_now(begin+t);

            imu_data.header.stamp = time_now;
            imu_data.header.frame_id = "imu_link";
            //四元数位姿
            imu_data.orientation.x = q.x();
            imu_data.orientation.y = q.y();
            imu_data.orientation.z = q.z();
            imu_data.orientation.w = q.w();
            //线加速度
            imu_data.linear_acceleration.x = data_noise.imu_acc(0); 
            imu_data.linear_acceleration.y = data_noise.imu_acc(1);
            imu_data.linear_acceleration.z = data_noise.imu_acc(2);
            //角速度
            imu_data.angular_velocity.x = data_noise.imu_gyro(0); 
            imu_data.angular_velocity.y = data_noise.imu_gyro(1); 
            imu_data.angular_velocity.z = data_noise.imu_gyro(2);

            bag.write("imu", time_now, imu_data);

            t += 1.0/params.imu_frequency;
        }
        bag.close();
        std::cout << " finish generate imu data ..."<<std::endl;
    }

    void generate_real_time_data(void)
    {
        std::cout << " start generate real time imu and odom data, please visual data in rviz ... "<<std::endl;
        double t = 0.0;
        static int count  = 0;
        MotionData data;
        ros::Rate loop_rate(200);
        while(ros::ok())
        {
            // create imu data && add imu noise
            data = imuGen.MotionModel(t);
            MotionData data_noise = data;
            imuGen.addIMUnoise(data_noise);
            // R to q
            Eigen::Quaterniond tem_q(data_noise.Rwb);

            // save path
            m_RealOdomPosition.push_back(data_noise.twb);
            m_RealOdomRotation.push_back(tem_q);

            //publish the message
            publishImuData(m_IMU_pub, data_noise.imu_acc, data_noise.imu_gyro, tem_q);

            publishOdomData(m_RealOdom_pub, data_noise.twb, tem_q);

            pre_integration_euler(data_noise);

            pre_integration_mid(data_noise);

            if(count++ % 20 == 0)
            {
                PublishPath(m_RealPath_pub, m_RealOdomPosition, m_RealOdomRotation);
                
                std::cout << " pub data : " << count << std::endl;
            }
            
            t += 1.0/params.imu_frequency;
            loop_rate.sleep();  
        }
        std::cout << " stop generate real time imu and odom data ... "<<std::endl;
    }

    void pre_integration_euler(MotionData temp_pose)
    {
        static double dt = params.imu_timestep;
        static Eigen::Vector3d Pwb = imuGen.init_twb_;              // position :    from  imu measurements
        static Eigen::Quaterniond Qwb(imuGen.init_Rwb_);            // quaterniond:  from imu measurements
        static Eigen::Vector3d Vw = imuGen.init_velocity_;          // velocity  :   from imu measurements
        static Eigen::Vector3d gw(0,0,-9.81);    // ENU frame
        static std::vector<Eigen::Vector3d> eulerPosition;
        static std::vector<Eigen::Quaterniond> eulerRotation;
        static int count = 0;

        Eigen::Quaterniond dq;
        Eigen::Vector3d dtheta_half =  temp_pose.imu_gyro * dt /2.0;
        dq.w() = 1;
        dq.x() = dtheta_half.x();
        dq.y() = dtheta_half.y();
        dq.z() = dtheta_half.z();
    
        //　imu 动力学模型　euler integration 参考svo预积分论文
        Eigen::Vector3d acc_w = Qwb.normalized() * (temp_pose.imu_acc) + gw; 
        Qwb = Qwb.normalized() * dq.normalized();
        Vw = Vw + acc_w * dt;
        Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;

        // save path
        eulerPosition.push_back(Pwb);
        eulerRotation.push_back(Qwb);

        if(count++ % 20 == 0)
        {
            PublishPath(m_eulerPath_pub, eulerPosition, eulerRotation);
            std::cout << " euler-integration pub path : " << count << std::endl;
        }
    }

    void pre_integration_mid(MotionData temp_pose)
    {
        static double dt = params.imu_timestep;
        static Eigen::Vector3d Pwb = imuGen.init_twb_;              // position :    from  imu measurements
        static Eigen::Quaterniond Qwb(imuGen.init_Rwb_);            // quaterniond:  from imu measurements
        static Eigen::Vector3d Vw = imuGen.init_velocity_;          // velocity  :   from imu measurements
        static Eigen::Vector3d gw(0,0,-9.81);    // ENU frame
        static Eigen::Vector3d last_acc = Eigen::Vector3d::Zero() ;
        static Eigen::Vector3d last_gyr = Eigen::Vector3d::Zero() ;
        static std::vector<Eigen::Vector3d> midPosition;
        static std::vector<Eigen::Quaterniond> midRotation; 
        static int count = 0;
        static bool first_flag = true;
        if(first_flag == true)
        {
            first_flag = false;
            last_acc = Qwb.normalized() * (temp_pose.imu_acc) + gw;
            last_gyr = temp_pose.imu_gyro;
        }
        Eigen::Quaterniond dq;
        Eigen::Vector3d dtheta_half =  (temp_pose.imu_gyro + last_gyr) / 2.0 * dt / 2.0;
        dq.w() = 1;
        dq.x() = dtheta_half.x();
        dq.y() = dtheta_half.y();
        dq.z() = dtheta_half.z();
        //　imu 动力学模型　mid-integration
        Eigen::Vector3d acc_k = Qwb.normalized() * (temp_pose.imu_acc) + gw;
        Eigen::Vector3d acc_mid = (acc_k + last_acc) / 2.0;  
        Qwb = Qwb.normalized() * dq.normalized();
        Vw = Vw + acc_mid * dt;
        Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_mid;

        last_acc = acc_k;
        last_gyr = temp_pose.imu_gyro;
        // save path
        midPosition.push_back(Pwb);
        midRotation.push_back(Qwb);
        // pub path
        if(count++ % 20 == 0)
        {
            PublishPath(m_midPath_pub, midPosition, midRotation);
            std::cout << " mid-integration pub path : " << count << std::endl;
        }
    }

    
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Generate_AllanData");     

    Generate_AllanData Gener_AllData;

    // you can choose to generate bag data or real time data

    // Gener_AllData.generate_imu_rosbag_data();
    Gener_AllData.generate_real_time_data();

    ros::spin(); 

    std::cout << " main END "<<std::endl;

    return 0;
}


