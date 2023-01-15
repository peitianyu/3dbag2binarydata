使用方法:

1. 修改保存文件位置: 
m_ofs_imu.open("/root/3d_laser_slam/data/imu.sensor", std::ios::binary);
m_ofs_gps.open("/root/3d_laser_slam/data/gps.sensor", std::ios::binary);
m_ofs_cloud.open("/root/3d_laser_slam/data/cloud.sensor", std::ios::binary);
2. 运行文件
catkin_make
roslaunch sub_data run.launch
