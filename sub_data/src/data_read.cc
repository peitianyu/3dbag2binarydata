#include "utility.h"
#include "lio_sam/cloud_info.h"

#include <fstream>
#include <sstream>
#include <iostream>

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))

struct OusterPointXYZIRT
{
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, t, t)(uint16_t, reflectivity, reflectivity)(uint8_t, ring, ring)(uint16_t, noise, noise)(uint32_t, range, range))

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

const int queueLength = 2000;

class ImageProjection : public ParamServer
{
private:
    std::mutex imuLock;
    std::mutex odoLock;

    ros::Subscriber subLaserCloud;
    ros::Publisher pubLaserCloud;

    ros::Publisher pubExtractedCloud;
    ros::Publisher pubLaserCloudInfo;

    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    sensor_msgs::PointCloud2 currentCloudMsg;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<PointType>::Ptr fullCloud;
    pcl::PointCloud<PointType>::Ptr extractedCloud;

    int deskewFlag;
    cv::Mat rangeMat;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    lio_sam::cloud_info cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::Header cloudHeader;

    vector<int> columnIdnCountVec;
    ros::Subscriber subGPS;

    std::ofstream m_ofs_imu;
    std::ofstream m_ofs_cloud;
    std::ofstream m_ofs_gps;

public:
    ImageProjection()
    {
        subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        subGPS = nh.subscribe<nav_msgs::Odometry>(gpsTopic, 200, &ImageProjection::gpsHandler, this, ros::TransportHints().tcpNoDelay());

        m_ofs_imu.open("/root/3d_laser_slam/data/imu.sensor", std::ios::binary);
        m_ofs_gps.open("/root/3d_laser_slam/data/gps.sensor", std::ios::binary);
        m_ofs_cloud.open("/root/3d_laser_slam/data/cloud.sensor", std::ios::binary);

        // 初始化
        allocateMemory();
        // 重置参数
        resetParameters();
    }

    /**
     * 初始化
     */
    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN * Horizon_SCAN);

        cloudInfo.startRingIndex.assign(N_SCAN, 0);
        cloudInfo.endRingIndex.assign(N_SCAN, 0);

        cloudInfo.pointColInd.assign(N_SCAN * Horizon_SCAN, 0);
        cloudInfo.pointRange.assign(N_SCAN * Horizon_SCAN, 0);

        resetParameters();
    }

    /**
     * 重置参数，接收每帧lidar数据都要重置这些参数
     */
    void resetParameters()
    {
        laserCloudIn->clear();
        extractedCloud->clear();

        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }
    }

    ~ImageProjection()
    {
        m_ofs_imu.close();
        m_ofs_gps.close();
        m_ofs_cloud.close();
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr &imuMsg)
    {
        std::cout<<"imuHandler"<<std::endl;
        uint64_t time_stamp = uint64_t(imuMsg->header.stamp.toSec() * 1e6);
        Eigen::Vector3d acc(imuMsg->linear_acceleration.x, imuMsg->linear_acceleration.y, imuMsg->linear_acceleration.z);

        Eigen::Vector3d gyr(imuMsg->angular_velocity.x, imuMsg->angular_velocity.y, imuMsg->angular_velocity.z);

        Eigen::Quaterniond q_from(imuMsg->orientation.w, imuMsg->orientation.x, imuMsg->orientation.y, imuMsg->orientation.z);
        Eigen::Vector3d rot = Quaterniond2Rot(q_from);

        m_ofs_imu.write(reinterpret_cast<const char *>(&time_stamp), sizeof(uint64_t));

        m_ofs_imu.write(reinterpret_cast<const char *>(&acc(0)), sizeof(double));
        m_ofs_imu.write(reinterpret_cast<const char *>(&acc(1)), sizeof(double));
        m_ofs_imu.write(reinterpret_cast<const char *>(&acc(2)), sizeof(double));

        m_ofs_imu.write(reinterpret_cast<const char *>(&gyr(0)), sizeof(double));
        m_ofs_imu.write(reinterpret_cast<const char *>(&gyr(1)), sizeof(double));
        m_ofs_imu.write(reinterpret_cast<const char *>(&gyr(2)), sizeof(double));

        m_ofs_imu.write(reinterpret_cast<const char *>(&rot(0)), sizeof(double));
        m_ofs_imu.write(reinterpret_cast<const char *>(&rot(1)), sizeof(double));
        m_ofs_imu.write(reinterpret_cast<const char *>(&rot(2)), sizeof(double));
    }

    Eigen::Vector3d Quaterniond2Rot(const Eigen::Quaterniond &in_q)
    {
        Eigen::Vector3d rot;
        Eigen::Quaterniond q = in_q.normalized();
        if (q.w() < 0)
            q = Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z());

        Eigen::Vector3d r_v(q.x(), q.y(), q.z());
        double norm = r_v.norm();
        if (norm < 1e-7)
        {
            rot = 2 * r_v;
        }
        else
        {
            double th = 2 * atan2(norm, q.w());
            rot = r_v / norm * th;
        }
        return rot;
    }

    void gpsHandler(const nav_msgs::Odometry::ConstPtr &gpsMsg)
    {
        std::cout<<"gpsHandler"<<std::endl;
        uint64_t time_stamp = uint64_t(gpsMsg->header.stamp.toSec() * 1e6);
        double x = gpsMsg->pose.pose.position.x;
        double y = gpsMsg->pose.pose.position.y;
        double z = gpsMsg->pose.pose.position.z;

        m_ofs_gps.write(reinterpret_cast<const char *>(&time_stamp), sizeof(uint64_t));

        m_ofs_gps.write(reinterpret_cast<const char *>(&x), sizeof(double));
        m_ofs_gps.write(reinterpret_cast<const char *>(&y), sizeof(double));
        m_ofs_gps.write(reinterpret_cast<const char *>(&z), sizeof(double));
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {
        std::cout << "cloudHandler" << std::endl;
        if (!cachePointCloud(laserCloudMsg))
            return;
    }

    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 2)
            return false;

        // convert cloud
        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();
        if (sensor == SensorType::VELODYNE || sensor == SensorType::LIVOX)
        {
            pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
        }
        else if (sensor == SensorType::OUSTER)
        {
            // Convert to Velodyne format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
            laserCloudIn->points.resize(tmpOusterCloudIn->size());
            laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
            for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
            {
                auto &src = tmpOusterCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.t * 1e-9f;
            }
        }
        else
        {
            ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
            ros::shutdown();
        }

        // get timestamp
        cloudHeader = currentCloudMsg.header;
        timeScanCur = cloudHeader.stamp.toSec();
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;

        uint64_t time_stamp = uint64_t(cloudHeader.stamp.toSec() * 1e6);
        uint64_t scan_time = uint64_t(laserCloudIn->points.back().time * 1e6);
        uint64_t point_size = laserCloudIn->points.size();

        m_ofs_cloud.write(reinterpret_cast<const char *>(&time_stamp), sizeof(uint64_t));
        m_ofs_cloud.write(reinterpret_cast<const char *>(&scan_time), sizeof(uint64_t));
        m_ofs_cloud.write(reinterpret_cast<const char *>(&point_size), sizeof(uint64_t));
        for (auto &p : laserCloudIn->points)
        {
            double x = p.x;
            double y = p.y;
            double z = p.z;
            m_ofs_cloud.write(reinterpret_cast<const char *>(&x), sizeof(double));
            m_ofs_cloud.write(reinterpret_cast<const char *>(&y), sizeof(double));
            m_ofs_cloud.write(reinterpret_cast<const char *>(&z), sizeof(double));
        }

        return true;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lio_sam");

    ImageProjection IP;

    ROS_INFO("\033[1;32m----> Image Projection Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

    return 0;
}