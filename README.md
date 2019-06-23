# imu_data_simulation
This is a ros package that can generate imu data based on the imu error model and test the imu euler and mid integretion algorithm based on the imu motion model, and visualize the trajectory in rviz.

1.计算IMU方差
通过IMU误差模型，利用程序增加加速度计和陀螺仪的高斯白噪声和bias的噪声，生成静止状态下的IMU数据，然后通过程序计算方差。加速度计和陀螺仪Allan方差曲线如下图所示：
![acc_allan](https://github.com/robosu12/imu_data_simulation/blob/master/picture/acc_allan.jpg)
![gyro_allan](https://github.com/robosu12/imu_data_simulation/blob/master/picture/gyro_allan.jpg)
程序中计算得到的是离散时间的噪声，连续时间到离散时间需要除以根号下采样频率。
总结：从标定结果可以看到，经过转换，加速度计和陀螺仪的高斯白噪声测量值和设定值基本接近，量级在百分之一左右；而加速度计和陀螺仪的bias噪声标定结果与设定结果相差较大，这是由于噪声的量级太小，在万分之一， 所以标定结果会有较大误差；所以这也说明在实际VIO应用中，需要将 IMU的bias加入到后端优化中进行实时估计。

2.进行IMU积分计算运动轨迹
程序为一个ROS的工作空间，本人在贺一加老师提供的代码基础上，将生成IMU数据的代码封装成一个类，可以将生成的IMU数据实时进行积分，并且添加了IMU，PATH，ODOMETRY等topic的发布代码，可以在rviz中实时可视化IMU姿态，真实轨迹，积分轨迹，并且提供了已经配置好的rviz文件。

程序运行流程：

a.	cd  vio-data-simulation-ws

b.	catkin_make

c.	source  devel/setup.bash

d.	roscore

e.	rviz  -d  rviz/vio_simulation.rviz

f.	rosrun  vio_data_simulation vio_data_simulation_node


两次运行结果如下图所示，其中绿色线为真实轨迹，红色线为欧拉积分轨迹，白色线为中值积分轨迹：
![imu_integration_1](https://github.com/robosu12/imu_data_simulation/blob/master/picture/imu_integretion_in_euler_and_mid1.png)
![imu_integration_2](https://github.com/robosu12/imu_data_simulation/blob/master/picture/imu_integretion_in_euler_and_mid2.png)

总结：可以看出，刚开始积分得到的轨迹和实际轨迹很接近，但是很快就飘走了。实际这是很正常的，而且这样的结果已经比较理想，因为仿真产生的IMU数据只考虑了噪声，并没有考虑到地球自转，加速度计与陀螺仪的耦合影响等其它因素的影响；在实际应用中，对于低端IMU进行纯积分，得到的轨迹几秒钟之后立刻就发散了，所以将视觉和IMU，甚至更多传感器融合到一起，取长补短，得到一个鲁棒的SLAM算法是目前研究的热点。

