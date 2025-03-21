#include "myagv_odometry/myagv_odometry_node.hpp"


const unsigned char header[2] = { 0xfe, 0xfe };

boost::asio::io_service iosev;

boost::asio::serial_port sp(iosev, "/dev/ttyAMA2");

std::array<double, 36> odom_pose_covariance = {
    {1e-9, 0, 0, 0, 0, 0,
    0, 1e-3, 1e-9, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e-9} };
std::array<double, 36> odom_twist_covariance = {
    {1e-9, 0, 0, 0, 0, 0,
    0, 1e-3, 1e-9, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e-9} };
volatile float twoKp = twoKpDef;											
volatile float twoKi = twoKiDef;											
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;

MyAGVOdometry::MyAGVOdometry() : Node("myagv_odometry_node") {
  x = 0.0;
  y = 0.0;
  theta = 0.0;

  vx = 0.0;
  vy = 0.0;
  vtheta = 0.0;

  linearX = 0.0;
  linearY = 0.0;
  angularZ = 0.0;
  direction_flag = true;
  cmd_vel_received_ = false;
  timeout_duration_ = 0.5;
  last_cmd_time_ = this->get_clock()->now();
  init();
}

MyAGVOdometry::~MyAGVOdometry() {
  ;
}

bool MyAGVOdometry::init() {
  sp.set_option(boost::asio::serial_port::baud_rate(115200));
  sp.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
  sp.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
  sp.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
  sp.set_option(boost::asio::serial_port::character_size(8));
  Gyroscope_Xdata_Offset = 0.0f; 
  Gyroscope_Ydata_Offset = 0.0f; 
  Gyroscope_Zdata_Offset = 0.0f;
  Offest_Count = 0;

  lastTime = this->now();

  // Declare and initialize parameters
  this->declare_parameter<float>("x_speed", 0.2);
  this->declare_parameter<float>("y_speed", 0.2);
  this->declare_parameter<float>("w_speed", 0.75);

  // Get parameter values
  this->get_parameter("x_speed", x_max_speed_);
  this->get_parameter("y_speed", y_max_speed_);
  this->get_parameter("w_speed", angular_max_speed_);

  // Log the parameter values
  RCLCPP_INFO(this->get_logger(), "x_speed: %f, y_speed: %f, z_speed: %f", x_max_speed_, y_max_speed_, angular_max_speed_);

  pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);
  pub_imu_raw = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
  pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  pub_voltage = this->create_publisher<std_msgs::msg::Float32>("Voltage", 10);
  odomBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&MyAGVOdometry::cmdCallback,this, std::placeholders::_1));
  // Set a timer to periodically call `execute`
  timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&MyAGVOdometry::run, this));
  restore();
  return true;
}

void MyAGVOdometry::restore()
{
    // Clear serial port buffer by reading at least 1 byte
    boost::asio::streambuf clear_buffer; 
    boost::asio::read(sp, clear_buffer, boost::asio::transfer_at_least(1));
    
    // Pause for 100 milliseconds
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // Motor Stall Recovery
    unsigned char cmd[6] = {0xfe, 0xfe, 0x01, 0x00, 0x01, 0x02};
    RCLCPP_INFO(this->get_logger(), "Sending data: ");
    std::ostringstream oss;
    for (int i = 0; i < 6; ++i) 
    {
        oss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(cmd[i]) << " ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    // Write command data to the serial port
    boost::asio::write(sp, boost::asio::buffer(cmd));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    linearX = 0.0;
    linearY = 0.0;
    angularZ = 0.0;
    return;
}

void MyAGVOdometry::restoreRun()
{
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  ask_for_input();
  /* std::string res;
  RCLCPP_INFO(this->get_logger(), "If you want to restore run, please input 1, then press Enter.");

  while (res != "1") {
      //std::cin >> res;
      std::getline(std::cin, res);
      RCLCPP_INFO(this->get_logger(), "Press Enter");
      RCLCPP_INFO(this->get_logger(), "Input received: %s", res);
      std::this_thread::sleep_for(std::chrono::seconds(5));
  }
  restore();
  RCLCPP_INFO(this->get_logger(), "Restore run initiated!");*/
  return; 
}
void MyAGVOdometry::ask_for_input() {
        std::thread([this]() {
            std::string user_input;
            while (rclcpp::ok()) {
                std::cout << "Enter 1 to stop asking and continue ... ";
                std::getline(std::cin, user_input);

                if (user_input == "1") {
                    RCLCPP_INFO(this->get_logger(), "User entered 1. Restoring...");
                    return; // Stop the input loop
                } else {
                    RCLCPP_INFO(this->get_logger(), "Invalid input. Try again.");
                }
            }
        }).detach();
    }

bool MyAGVOdometry::readSpeed()
{
  int count = 0;
  //unsigned char checkSum;
  unsigned char buf_header[1] = {0};
  unsigned char buf[TOTAL_RECEIVE_SIZE] = {0};

  size_t ret;
  boost::system::error_code er2;
  bool header_found = false;
  while (!header_found) {
      ++count;
      ret = boost::asio::read(sp, boost::asio::buffer(buf_header), er2);
      if (ret != 1) {
          continue;
      }
      if (buf_header[0] != header[0]) {
          continue;
      }
      bool header_2_found = false;
      while (!header_2_found) {
          ret = boost::asio::read(sp, boost::asio::buffer(buf_header), er2);
          if (ret != 1) {
              continue;
          }
          if (buf_header[0] != header[0]) {
              continue;
          }
          header_2_found = true;
      }
      header_found = true;
  }

  ret = boost::asio::read(sp, boost::asio::buffer(buf), er2);  // ready break
if ((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4]) {
      int wheel_num = 0;
      for (int i = 0; i < 4; ++i) {
          if (buf[i] == 1) {
              wheel_num = i+1;
              RCLCPP_ERROR(this->get_logger(),"ERROR %d wheel current > 2000", wheel_num);
          }
      }
      restoreRun();
      return false;
  }
  if (ret != TOTAL_RECEIVE_SIZE) {
      RCLCPP_ERROR(this->get_logger(),"Read error %zu",ret);
      return false;
  }

  int index = 0;
  int check = 0;//ilter time older than imu message buffer
  for (int i = 0; i < (TOTAL_RECEIVE_SIZE-1); ++i)
      check += buf[index + i];
  if (check % 256 != buf[index + (TOTAL_RECEIVE_SIZE-1)])
{
  RCLCPP_ERROR(this->get_logger(),"Error:Serial port verification failed! check:%d -- %d ",check,buf[index+(TOTAL_RECEIVE_SIZE-1)]);	
    return false;
}

  vx = (static_cast<double>(buf[index]) - 128.0) * 0.01;
  vy = (static_cast<double>(buf[index + 1]) - 128.0) * 0.01;
  vtheta = (static_cast<double>(buf[index + 2]) - 128.0) * 0.01;

  imu_data.linear_acceleration.x = ((buf[index + 3] + buf[index + 4] * 256 ) - 10000) * 0.001;
  imu_data.linear_acceleration.y = ((buf[index + 5] + buf[index + 6] * 256 ) - 10000) * 0.001;
  imu_data.linear_acceleration.z = ((buf[index + 7] + buf[index + 8] * 256 ) - 10000) * 0.001;

  imu_data.angular_velocity.x  = ((buf[index + 9] + buf[index + 10] * 256 ) - 10000) * 0.1;
  imu_data.angular_velocity.y = ((buf[index + 11] + buf[index + 12] * 256 ) - 10000) * 0.1;
  imu_data.angular_velocity.z = ((buf[index + 13] + buf[index + 14] * 256 ) - 10000) * 0.1;

  Battery_voltage = (float)buf[index + 16] / 10.0f;
  Backup_Battery_voltage = (float)buf[index + 17] / 10.0f;
  return true;
}
void MyAGVOdometry::writeSpeed(double movex, double movey, double rot)
{
    if (movex > x_max_speed_) movex = x_max_speed_;
    if (movex < -x_max_speed_) movex = -x_max_speed_;
    if (movey > y_max_speed_) movey = y_max_speed_;
    if (movey < -y_max_speed_) movey = -y_max_speed_;
    if (rot > angular_max_speed_) rot = angular_max_speed_;
    if (rot < -angular_max_speed_) rot = -angular_max_speed_;

    unsigned char x_send = static_cast<signed char>(movex * 100) + 128;
    unsigned char y_send = static_cast<signed char>(movey * 100) + 128;
    unsigned char rot_send = static_cast<signed char>(rot * 100) + 128;
    unsigned char check = x_send + y_send + rot_send;

    char buf[8] = { 0 };
    buf[0] = header[0];
    buf[1] = header[1];
    buf[2] = x_send;
    buf[3] = y_send;
    buf[4] = rot_send;
    buf[5] = check;

    boost::asio::write(sp, boost::asio::buffer(buf));
}

float MyAGVOdometry::invSqrt(float number)
{
	volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );

	return y;
}

void MyAGVOdometry::accelerometerOffset(float gx, float gy, float gz)
{
	Gyroscope_Xdata_Offset += gx; 
  Gyroscope_Ydata_Offset += gy; 
  Gyroscope_Zdata_Offset += gz;

  if (Offest_Count == OFFSET_COUNT)
  {
    Gyroscope_Xdata_Offset = Gyroscope_Xdata_Offset / OFFSET_COUNT;
    Gyroscope_Ydata_Offset = Gyroscope_Ydata_Offset / OFFSET_COUNT;
    Gyroscope_Zdata_Offset = Gyroscope_Zdata_Offset / OFFSET_COUNT;
  }
}

/* void MyAGVOdometry::MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;
	
		
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;				
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;				
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

	
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	gx *= (0.5f * (1.0f / sampleFreq));		
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	imu_data.orientation.w = q0;
	imu_data.orientation.x = q1;
	imu_data.orientation.y = q2;
	imu_data.orientation.z = q3;
} */
void MyAGVOdometry::Publish_Voltage()
{
    std_msgs::msg::Float32 voltage_msg;
    voltage_msg.data = Battery_voltage;
    pub_voltage->publish(voltage_msg);
}

void MyAGVOdometry::publisherImuSensor()
{
	sensor_msgs::msg::Imu ImuSensor;

	ImuSensor.header.stamp = this->now(); 
	ImuSensor.header.frame_id = "imu";

	ImuSensor.orientation.x = 0.0; 
	ImuSensor.orientation.y = 0.0; 
	ImuSensor.orientation.z = imu_data.orientation.z;
	ImuSensor.orientation.w = imu_data.orientation.w;

	ImuSensor.orientation_covariance[0] = 1e6;
	ImuSensor.orientation_covariance[4] = 1e6;
	ImuSensor.orientation_covariance[8] = 1e-6;

	ImuSensor.angular_velocity.x = 0.0;		
	ImuSensor.angular_velocity.y = 0.0;		
	ImuSensor.angular_velocity.z = imu_data.angular_velocity.z;

	ImuSensor.angular_velocity_covariance[0] = 1e6;
	ImuSensor.angular_velocity_covariance[4] = 1e6;
	ImuSensor.angular_velocity_covariance[8] = 1e-6;

	ImuSensor.linear_acceleration.x = 0; 
	ImuSensor.linear_acceleration.y = 0; 
	ImuSensor.linear_acceleration.z = 0;  

	pub_imu->publish(ImuSensor); 
}
void MyAGVOdometry::publisherImuSensorRaw()
{
	sensor_msgs::msg::Imu ImuSensorRaw;

	ImuSensorRaw.header.stamp = this->now(); 
	ImuSensorRaw.header.frame_id = "imu_link";

	ImuSensorRaw.orientation.x = 0.0; 
	ImuSensorRaw.orientation.y = 0.0; 
	ImuSensorRaw.orientation.z = 0.0;
	ImuSensorRaw.orientation.w = 1.0;

	ImuSensorRaw.angular_velocity.x = imu_data.angular_velocity.x * DEG_TO_RAD;		
	ImuSensorRaw.angular_velocity.y = imu_data.angular_velocity.y * DEG_TO_RAD;		
	ImuSensorRaw.angular_velocity.z = imu_data.angular_velocity.z * DEG_TO_RAD;

	ImuSensorRaw.linear_acceleration.x = imu_data.linear_acceleration.x; 
	ImuSensorRaw.linear_acceleration.y = imu_data.linear_acceleration.y; 
	ImuSensorRaw.linear_acceleration.z = imu_data.linear_acceleration.z;  

	pub_imu_raw->publish(ImuSensorRaw); 
}
void MyAGVOdometry::publisherOdom()
{   
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = this->now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    geometry_msgs::msg::Quaternion odom_quat;
    odom_quat = createQuaternionMsgFromYaw(theta);

    odom_trans.transform.translation.x = x; 
    odom_trans.transform.translation.y = y; 

    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //odomBroadcaster->sendTransform(odom_trans);
    
    nav_msgs::msg::Odometry msgl;
    msgl.header.stamp = this->now();
    msgl.header.frame_id = "odom";
    
    msgl.pose.pose.position.x = x;
    msgl.pose.pose.position.y = y;
    msgl.pose.pose.position.z = 0.0;
    msgl.pose.pose.orientation = odom_quat;
    msgl.pose.covariance = odom_pose_covariance;

    msgl.child_frame_id = "base_footprint";
    msgl.twist.twist.linear.x = vx;
    msgl.twist.twist.linear.y = vy;
    msgl.twist.twist.angular.z = vtheta;
    msgl.twist.covariance = odom_twist_covariance;

    pub_odom->publish(msgl);
}

void MyAGVOdometry::execute(double linearX, double linearY, double angularZ)
{   
    currentTime = this->now();    
    double dt = (currentTime - lastTime).seconds();
    sampleFreq = 1.0f/dt;
    if (true ==  readSpeed()) 
    {   
        double delta_x = (vx * cos(theta) - vy * sin(theta)) * dt;
        double delta_y = (vx * sin(theta) + vy * cos(theta)) * dt;
        double delta_th = vtheta * dt;
        if(true == direction_flag) {
          x += delta_x * CORRECTION_FACTOR_FX;
        } else {
          x += delta_x * CORRECTION_FACTOR_RX;
        }
        
        y += delta_y;
        theta += delta_th;
        theta = atan2(sin(theta), cos(theta));
        if (Offest_Count < OFFSET_COUNT)
        {
            Offest_Count++;
            accelerometerOffset(imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z);
        }
        else
        {
            Offest_Count = OFFSET_COUNT;
            imu_data.angular_velocity.x = imu_data.angular_velocity.x - Gyroscope_Xdata_Offset;
            imu_data.angular_velocity.y = imu_data.angular_velocity.y - Gyroscope_Ydata_Offset;
            imu_data.angular_velocity.z = imu_data.angular_velocity.z - Gyroscope_Zdata_Offset;
            //MahonyAHRSupdateIMU(0.0, 0.0, imu_data.angular_velocity.z, 0.0, 0.0, imu_data.linear_acceleration.z);
            writeSpeed(linearX, linearY, angularZ);
            //linearX = 0.0;
            //linearY = 0.0;
            //angularZ = 0.0;
            publisherOdom();
            //publisherImuSensor();
            Publish_Voltage();
            publisherImuSensorRaw();

        }
    } 
    lastTime = currentTime;
}

void MyAGVOdometry::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_vel_received_ = true;
  last_cmd_time_ = this->get_clock()->now();
  linearX = msg->linear.x;
  linearY = msg->linear.y;
  angularZ = msg->angular.z;
  if(linearX >= 0){
    direction_flag = true;
  } else {
    direction_flag = false;
  }
}
void MyAGVOdometry::run() {
  checkCmdVelTimeout();
  execute(linearX, linearY, angularZ);
}
geometry_msgs::msg::Quaternion MyAGVOdometry::createQuaternionMsgFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

void MyAGVOdometry::setZeroVelocity() {
  linearX = 0.0;
  linearY = 0.0;
  angularZ = 0.0;
}

void MyAGVOdometry::checkCmdVelTimeout()
{
  if(cmd_vel_received_ == true) {
    auto current_time = this->get_clock()->now();
    double time_diff = (current_time - last_cmd_time_).seconds();

    if ((time_diff > timeout_duration_))
    {
        RCLCPP_WARN(this->get_logger(), "No cmd_vel message received. Setting velocity to zero.");
        setZeroVelocity();
        cmd_vel_received_ = false;
    }
  }
    
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyAGVOdometry>();
  try {
    rclcpp::spin(node);
  } catch (...) {
    // Handle exceptions here
    RCLCPP_ERROR(node->get_logger(), "Exception occurred");
  }

  rclcpp::shutdown();

  return 0;
}
