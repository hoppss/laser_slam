#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>

class LidarMotionCalibrator {
 public:
  LidarMotionCalibrator( tf::TransformListener* tf) {
    tf_ = tf;
    beams_length_ = 0;
    initialized_ = false;
    scan_sub_ = nh_.subscribe("/scan", 10, &LidarMotionCalibrator::ScanCallBack, this);
    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_calib", 10);
  }

  ~LidarMotionCalibrator() {
    if (tf_ != NULL)
      delete tf_;
  }
  void init(const sensor_msgs::LaserScan& msg){
    beams_length_ = msg.ranges.size();
    angle_increment_ = msg.angle_increment;
    initialized_ = true;
  }

  void ScanCallBack(const sensor_msgs::LaserScanPtr& scan_msg) {
    if (!initialized_) init(*scan_msg);
    sensor_msgs::LaserScan laserScanMsg = *scan_msg;

    ros::Time l_startTime, l_endTime, r_startTime, r_endTime;
    l_startTime = scan_msg->header.stamp;
    r_startTime = l_startTime + ros::Duration(laserScanMsg.time_increment * 1020);

    double time_inc = laserScanMsg.time_increment;

    tf::Stamped<tf::Pose> visualPose;               //laser pose in odom frame
    if (!getLaserPose(visualPose, l_startTime, tf_)) {
      ROS_WARN("Not visualPose robot base,Can not Calib");
      return;
    }
    double visualYaw = tf::getYaw(visualPose.getRotation());

    std::vector<double> left_ranges, left_angles, right_ranges, right_angles;
    //得到最终点的时间, 左边区域
    int beamNum = laserScanMsg.ranges.size()/2;
    l_endTime = l_startTime + ros::Duration(laserScanMsg.time_increment * beamNum);
    r_endTime = r_startTime + ros::Duration(laserScanMsg.time_increment * beamNum);
    ROS_INFO_ONCE("laser callback beamNum is %d", beamNum);

    // 将数据复制出来
    for (int i = beamNum; i < laserScanMsg.ranges.size(); ++i){
      double lidar_dist = laserScanMsg.ranges[i];
      double lidar_angle = laserScanMsg.angle_increment * (i - beamNum)  /*+ laserScanMsg.angle_min*/;

      if (lidar_dist < 0.05 || std::isnan(lidar_dist) || std::isinf(lidar_dist))
        lidar_dist = 0.0;

      left_ranges.push_back(lidar_dist);
      left_angles.push_back(lidar_angle);
    }

    for (int i = 0; i < beamNum; ++i){
      double lidar_dist = laserScanMsg.ranges[i];
      double lidar_angle = laserScanMsg.angle_increment * i + laserScanMsg.angle_min;

      if (lidar_dist < 0.05 || std::isnan(lidar_dist) || std::isinf(lidar_dist))
        lidar_dist = 0.0;

      right_ranges.push_back(lidar_dist);
      right_angles.push_back(lidar_angle);
    }

      //进行矫正
    ROS_INFO("before CALIB, left %ld, right %ld", left_ranges.size(), right_ranges.size());
    Lidar_Calibration(visualPose, \
                    left_ranges, \
                    left_angles, \
                    l_startTime, \
                    l_endTime, \
                    right_ranges, \
                    right_angles, \
                    r_startTime, \
                    r_endTime, \
                    tf_, \
                    time_inc);

      ROS_INFO("AFTER CALIB, left %ld, right %ld", left_ranges.size(), right_ranges.size());

      for (int i = 0; i < laserScanMsg.ranges.size() / 2; ++i){
        laserScanMsg.ranges[i] = right_ranges[i];
      }
      for (int i = laserScanMsg.ranges.size() / 2; i < laserScanMsg.ranges.size(); ++i) {
        laserScanMsg.ranges[i] = left_ranges[i - laserScanMsg.ranges.size() / 2];
      }
      laserScanMsg.header.frame_id = "laser_calib";
      scan_pub_.publish(laserScanMsg);
  }

  /**
 * @name getLaserPose()
 * @brief 得到机器人在里程计坐标系中的位姿tf::Pose
 *        得到dt时刻激光雷达在odom坐标系的位姿
 * @param odom_pos  机器人的位姿
 * @param dt        dt时刻， startScanTime
 * @param tf_
*/
  bool getLaserPose(tf::Stamped<tf::Pose>& odom_pose,
                    ros::Time dt,
                    tf::TransformListener* tf_) {
    odom_pose.setIdentity();

    tf::Stamped<tf::Pose> robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = "laser";  //必须设置，因为要获得base_laser 在odom下的pose
    robot_pose.stamp_ = dt;               //设置为ros::Time()表示返回最近的转换关系

    // get the global pose of the robot
    try {
      //Block until a transform is possible or it times out.
      //当时间使用now()或者某个时刻，需要结合waitForTransform使用；等到标记时刻dt后首个有效的变换
      // 0.15s 的时间可以修改
      //默认认为odom->laser 频率高，误差忽略
      if (!tf_->waitForTransform("/odom", "/laser", dt, ros::Duration(0.15))) {
        ROS_ERROR("LidarMotion-Can not Wait Transform()");
        return false;
      }
      //等到有效变化后，完成矩阵变换
      tf_->transformPose("/odom", robot_pose, odom_pose);  //odom:target frame;    //odom_pose 输出
    } catch (tf::LookupException& ex) {
      ROS_ERROR("LidarMotion: No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    } catch (tf::ConnectivityException& ex) {
      ROS_ERROR("LidarMotion: Connectivity Error looking up looking up robot pose: %s\n", ex.what());
      return false;
    } catch (tf::ExtrapolationException& ex) {
      ROS_ERROR("LidarMotion: Extrapolation Error looking up looking up robot pose: %s\n", ex.what());
      return false;
    }

    return true;
  }

  /**
 * @brief Lidar_MotionCalibration
 *        激光雷达运动畸变去除分段函数;
 *        在此分段函数中，认为机器人是匀速运动；
 * @param frame_base_pose       标定完毕之后的基准坐标系,雷达startTime 时的位姿
 * @param frame_start_pose      本分段第一个激光点对应的位姿
 * @param frame_end_pose        本分段最后一个激光点对应的位姿
 * @param ranges                激光数据－－距离
 * @param angles                激光数据－－角度
 * @param startIndex            本分段第一个激光点在激光帧中的下标
 * @param beam_number           本分段的激光点数量
 */
  void Lidar_MotionCalibration(
      tf::Stamped<tf::Pose> frame_base_pose,
      tf::Stamped<tf::Pose> frame_start_pose,
      tf::Stamped<tf::Pose> frame_end_pose,
      std::vector<double>& ranges,
      std::vector<double>& angles,
      int startIndex,
      int& beam_number) {
    //TODO
    //每个位姿进行线性插值时的步长
    double beam_step = 1.0 / (beam_number - 1);

    //机器人的起始角度 和 最终角度
    tf::Quaternion start_angle_q = frame_start_pose.getRotation();
    tf::Quaternion end_angle_q = frame_end_pose.getRotation();

    //转换到弧度
    double start_angle_r = tf::getYaw(start_angle_q);
    double base_angle_r = tf::getYaw(frame_base_pose.getRotation());

    //机器人的起始位姿
    tf::Vector3 start_pos = frame_start_pose.getOrigin();
    start_pos.setZ(0);

    //最终位姿
    tf::Vector3 end_pos = frame_end_pose.getOrigin();
    end_pos.setZ(0);

    //基础坐标系
    tf::Vector3 base_pos = frame_base_pose.getOrigin();
    base_pos.setZ(0);

    double mid_angle;
    tf::Vector3 mid_pos;
    tf::Vector3 mid_point;

    double lidar_angle, lidar_dist;
    //插值计算出来每个激光点 对应 激光在odom中的位姿
    for (int i = 0; i < beam_number; i++) {
      //角度插值, 四元数插值 - 球面线性插值
      mid_angle = tf::getYaw(start_angle_q.slerp(end_angle_q, beam_step * i));

      //线性插值, 分别对x,y取线性插值，因为z无关，所以前面把z改为0
      mid_pos = start_pos.lerp(end_pos, beam_step * i);

      //得到激光点在odom 坐标系中的坐标 根据
      double tmp_angle;

      //如果激光雷达不等于无穷,则需要进行矫正.
      if (tfFuzzyZero(ranges[startIndex + i]) == false) {
        //计算对应的激光点在odom坐标系中的坐标

        //得到这帧激光束距离和夹角
        lidar_dist = ranges[startIndex + i];
        lidar_angle = angles[startIndex + i];

        //激光雷达坐标系下的坐标
        double laser_x, laser_y;
        laser_x = lidar_dist * cos(lidar_angle);
        laser_y = lidar_dist * sin(lidar_angle);

        //激光距离值  真实odom下位姿
        double odom_x, odom_y;
        odom_x = laser_x * cos(mid_angle) - laser_y * sin(mid_angle) + mid_pos.x();
        odom_y = laser_x * sin(mid_angle) + laser_y * cos(mid_angle) + mid_pos.y();

        //转换到类型中去
        mid_point.setValue(odom_x, odom_y, 0);

        //把在odom坐标系中的激光数据点 转换到 基础坐标系,（所有点云转换到开始第一个点时odom位姿）
        double x0, y0, a0, s, c;
        x0 = base_pos.x();
        y0 = base_pos.y();
        a0 = base_angle_r;
        s = sin(a0);
        c = cos(a0);
        /*
                * 把base转换到odom 为[c -s x0;
                *                   s c y0;
                *                   0 0 1]
                * 把odom转换到base为 [c s -x0*c-y0*s;
                *               -s c x0*s - y0*c;
                *                0 0 1]代数余子式取逆
                */
        double tmp_x, tmp_y;  //base 坐标系下第一个点scanTime 坐标系下 ranges，angles
        tmp_x = mid_point.x() * c + mid_point.y() * s - x0 * c - y0 * s;
        tmp_y = -mid_point.x() * s + mid_point.y() * c + x0 * s - y0 * c;
        mid_point.setValue(tmp_x, tmp_y, 0);

        //然后计算以起始坐标为起点的 dist angle
        //T_odom2base * p_base2point = T_odom2mid * p_mid2point
        //base 指startScan时odom
        //mid  值插补后激光点生成时的里程计，
        //p_mid2point 即激光原始数据得到的坐标， = 右边的值即为上面mid_point(odom_x, odom_y)
        //p_base2point 即初始激光扫描时刻，矫正后的坐标
        double dx, dy;
        dx = (mid_point.x());
        dy = (mid_point.y());
        lidar_dist = sqrt(dx * dx + dy * dy);
        lidar_angle = atan2(dy, dx);

        //激光雷达被矫正
        ranges[startIndex + i] = lidar_dist;
        angles[startIndex + i] = lidar_angle;
      }
      //如果等于无穷,则随便计算一下角度
      else {
        //激光角度
        lidar_angle = angles[startIndex + i];

        //里程计坐标系的角度
        tmp_angle = mid_angle + lidar_angle;
        tmp_angle = tfNormalizeAngle(tmp_angle);

        //如果数据非法 则只需要设置角度就可以了。把角度换算成start_pos坐标系内的角度
        lidar_angle = tfNormalizeAngle(tmp_angle - start_angle_r);

        angles[startIndex + i] = lidar_angle;
      }
      //end of TODO
    }
  }
  //激光雷达数据　分段线性进行插值　分段时间5ms??   分段的目的是认为雷达在短时间是匀速运动
  //这5ms 是基于odom >= 200 hz
  //分段函数用来二次曲线近似
  //首先每5ms odom 之间插值； 然后每个5ms的点云激光点之间插值
  //这里会调用Lidar_MotionCalibration()
  /**
 * @name Lidar_Calibration()
 * @brief 激光雷达数据　分段线性进行差值　分段的周期为5ms
 * @param ranges 激光束的距离值集合
 * @param angle　激光束的角度值集合
 * @param startTime　第一束激光的时间戳
 * @param endTime　最后一束激光的时间戳
 * @param *tf_
*/
  void Lidar_Calibration(tf::Stamped<tf::Pose>& frame_base_pose,
                         std::vector<double>& l_ranges,
                         std::vector<double>& l_angles,
                         ros::Time l_startTime,
                         ros::Time l_endTime,
                         std::vector<double>& r_ranges,
                         std::vector<double>& r_angles,
                         ros::Time r_startTime,
                         ros::Time r_endTime,
                         tf::TransformListener* tf_,
                         double time_inc) {
    //统计激光束的数量
    int beamNumber = l_ranges.size();
    if (beamNumber != l_angles.size()) {
      ROS_ERROR("Error: left_ranges not match to the angles");
      return;
    }

    // 5ms来进行分段, * 1000 us -> ms
    int interpolation_time_duration = 5 * 1000;

    tf::Stamped<tf::Pose> frame_start_pose;
    tf::Stamped<tf::Pose> frame_mid_pose;
    tf::Stamped<tf::Pose> frame_end_pose;

    //起始时间 us
    double l_start_time = l_startTime.toSec() * 1000 * 1000;
    double l_end_time = l_endTime.toSec() * 1000 * 1000;
    // double time_inc = (l_end_time - l_start_time) / beamNumber;  // 每束激光数据的时间间隔

    //当前插值的段的起始索引
    int start_index = 0;

    // ROS_INFO("get start pose");

    if (!getLaserPose(frame_start_pose, ros::Time(l_start_time / 1000000.0), tf_))  // 除以1000000 只是转换成s， e6
    {
      ROS_WARN("Not Start Left Pose,Can not Calib");
      return;
    }

    if (!getLaserPose(frame_end_pose, ros::Time(l_end_time / 1000000.0), tf_)) {
      ROS_WARN("Not End Left Pose, Can not Calib");
      return;
    }

    int cnt = 0;

    for (int i = 0; i < beamNumber; i++) {
      //分段线性,时间段的大小为interpolation_time_duration
      double mid_time = l_start_time + time_inc * (i - start_index);
      if (mid_time - l_start_time > interpolation_time_duration || (i == beamNumber - 1)) {//找到每个间隔了5ms的点和index
        cnt++;  //计数，有多少个中间点

        //得到起点和终点的位姿
        //终点的位姿
        if (!getLaserPose(frame_mid_pose, ros::Time(mid_time / 1000000.0), tf_)) {
          ROS_ERROR("left Mid %d Pose Error", cnt);
          return;
        }

        //对当前的起点和终点进行插值
        //interpolation_time_duration中间有多少个点.
        int interp_count = i - start_index + 1;

        Lidar_MotionCalibration(frame_base_pose,
                                frame_start_pose,
                                frame_mid_pose,  //用于计算相对位姿，这期间认为是匀速运动
                                l_ranges,
                                l_angles,
                                start_index,  //用于说明ranges中畸变矫正哪些点
                                interp_count);

        //更新时间
        l_start_time = mid_time;
        start_index = i;
        frame_start_pose = frame_mid_pose;  //找下一个中间点
      }
    }



    //right area calibraion
    //统计激光束的数量
    beamNumber = r_ranges.size();
    if (beamNumber != r_angles.size()) {
      ROS_ERROR("Error: right_ranges not match to the angles");
      return;
    }
    //起始时间 us
    double r_start_time = r_startTime.toSec() * 1000 * 1000;
    double r_end_time = r_endTime.toSec() * 1000 * 1000;
    // double time_inc = (l_end_time - l_start_time) / beamNumber;  // 每束激光数据的时间间隔

    //当前插值的段的起始索引
    start_index = 0;
    cnt = 0;

    if (!getLaserPose(frame_start_pose, ros::Time(r_start_time / 1000000.0), tf_))  // 除以1000000 只是转换成s， e6
    {
      ROS_WARN("Not Start Right Pose,Can not Calib");
      return;
    }

    if (!getLaserPose(frame_end_pose, ros::Time(r_end_time / 1000000.0), tf_)) {
      ROS_WARN("Not End Right Pose, Can not Calib");
      return;
    }

    for (int i = 0; i < beamNumber; i++) {
      //分段线性,时间段的大小为interpolation_time_duration
      double mid_time = r_start_time + time_inc * (i - start_index);
      if (mid_time - r_start_time > interpolation_time_duration || (i == beamNumber - 1))  //找到每个间隔了5ms的点和index
      {
        cnt++;  //计数，有多少个中间点

        //得到起点和终点的位姿
        //终点的位姿
        if (!getLaserPose(frame_mid_pose, ros::Time(mid_time / 1000000.0), tf_)) {
          ROS_ERROR("Right Mid %d Pose Error", cnt);
          return;
        }

        //对当前的起点和终点进行插值
        //interpolation_time_duration中间有多少个点.
        int interp_count = i - start_index + 1;

        Lidar_MotionCalibration(frame_base_pose,
                                frame_start_pose,
                                frame_mid_pose,  //用于计算相对位姿，这期间认为是匀速运动
                                r_ranges,
                                r_angles,
                                start_index,  //用于说明ranges中畸变矫正哪些点
                                interp_count);

        //更新时间
        r_start_time = mid_time;
        start_index = i;
        frame_start_pose = frame_mid_pose;  //找下一个中间点
      }
    }
  }

 private:
  tf::TransformListener* tf_;
  ros::NodeHandle nh_;
  ros::Subscriber scan_sub_;
  ros::Publisher scan_pub_;
  size_t beams_length_;
  double angle_increment_;
  bool initialized_;
};



int main(int argc,char ** argv)
{
    ros::init(argc,argv,"LidarMotionCalib");

    tf::TransformListener tf(ros::Duration(10.0));

    LidarMotionCalibrator tmpLidarMotionCalib(&tf);

    ros::spin();
    return 0;
}

