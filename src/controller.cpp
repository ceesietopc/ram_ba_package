#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Empty.h>
#include <math.h> 
#include <tf/transform_datatypes.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <cmath>

class Control
{
private:
  ros::NodeHandle node_handle_;
  ros::Subscriber sim_subscriber_;
  ros::Subscriber setpoint_subscriber_;
  ros::Subscriber odom_subscriber_;
  ros::Publisher velocity_publisher_;


  /* ADDITIONAL PUBLISHER FOR GRAPHING PURPOSES */
  ros::Publisher p_publisher_;
  ros::Publisher d_publisher_;
  ros::Publisher x_publisher_;
  ros::Publisher yaw_publisher_;
  ros::Publisher errorx_publisher_; 
  ros::Publisher errordx_publisher_;
  ros::Publisher foaw_publisher_;
  ros::Publisher velocity_damping_publisher_;
  /* END ADDITIONAL PUBLISHERS */

  /* Memory */
  std::vector <double> errorsx_; // Previous errors in x
  std::vector <double> errorsy_; // Previous errors in y
  std::vector <double> errorsz_; // Previous errors in z
  std::vector <double> times_; // Corresponding times
  std::vector <geometry_msgs::Twist>  speeds_; // speeds

  /* Filter parameters */
  // FOAW
  double uncertainty_band_; // Max band around pose to find line fitting
  int error_memory_; // Max errors stored

  // LOW PASS
  double T_, dt_;

  // STATE ESTIMATION
  double K_;

  // Messages
  geometry_msgs::Twist velocity_;
  std_msgs::Empty empty_;

  // Control parameters
  bool hovermode_;
  bool velocity_damping_;
  double hover_treshold_;

  struct 
  {
    double p_z;
    double d_z;
    double p_translational;
    double d_translational;
    double p_rotational;
    double d_rotational;
    double velocity;
  } Gains;

  struct
  {
    double x;
    double y;
    double z;
    double yaw;
  } Setpoint; // Check launch file for additional description of setpoint

  struct 
  {
    double x_translational;
    double y_translational;
    double z_translational;
    double z_rotational;
  } Error;

  struct 
  {
    double x_translational;
    double y_translational;
    double z_translational;
    double z_rotational;
  } ErrorDot;

  // general variables
  bool simulation;
  double previous_publish_time_;
  int freq_;
  double yaw_; // Screw quaternions

public:
  Control()
  {
    ROS_INFO("Initializing controller");
    ros::NodeHandle params("~");

    // If simulation is true, get data feedback from simulation and run simulationCallback. Otherwise, use OptiTrack
    simulation = false;
    params.getParam("simulation",simulation);
    if(simulation) {
      sim_subscriber_ = node_handle_.subscribe("gazebo/model_states", 1, &Control::simulationCallback, this); 
      ROS_INFO("SIMULATION MODE"); 
    }
    else { 
      odom_subscriber_ = node_handle_.subscribe("filtered_state", 1, &Control::odomCallback, this);  
      ROS_INFO("CONTROL MODE");
    }

    // Velocity publisher
    velocity_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("cmd_vel_controller", 1);

    /* ADDITIONAL PUBLISHER FOR GRAPHING PURPOSES */
    d_publisher_ = node_handle_.advertise<std_msgs::Float32>("control_d",1);
    p_publisher_ = node_handle_.advertise<std_msgs::Float32>("control_p",1);
    x_publisher_ = node_handle_.advertise<std_msgs::Float32>("tot_x",1);
    errorx_publisher_ = node_handle_.advertise<std_msgs::Float32>("error_x",1);
    errordx_publisher_ = node_handle_.advertise<std_msgs::Float32>("errord_x",1);
    yaw_publisher_ = node_handle_.advertise<std_msgs::Float32>("yaw",1);
    foaw_publisher_ = node_handle_.advertise<std_msgs::Int32>("foaw_d",1);
    velocity_damping_publisher_ = node_handle_.advertise<std_msgs::Float32>("velocity_damping",1);
    /* END ADDITIOANL PUBLISHERS */

    // Subscriber for setpoint changes
    setpoint_subscriber_ = node_handle_.subscribe("setpoint", 1, &Control::setpointCallback, this);

    // Initial values
    previous_publish_time_ = 0;
    freq_ = 50;
    params.getParam("publish_rate", freq_);
    
    // Hover mode
    hovermode_ = false;
    hover_treshold_ = 0.1;
    params.getParam("hovermode",hovermode_);
    params.getParam("hover_treshold",hover_treshold_);
    
    // Velocity damping
    velocity_damping_ = false;
    params.getParam("velocity_damping", velocity_damping_);
    

    /* Filtering */
    // FOAW
    error_memory_ = 15;
    uncertainty_band_ = 0.01;
    params.getParam("error_memory", error_memory_);
    params.getParam("uncertainty_band", uncertainty_band_);

    // STATE ESTIMATION
    K_ = 70;
    params.getParam("K", K_);

    // LOWPASS
    T_ = 0.1; //sec
    dt_ = 0.03; //ms

    
    // Initial values for setpoint
    params.getParam("setpoint_x", Setpoint.x);
    params.getParam("setpoint_y", Setpoint.y);
    params.getParam("setpoint_z", Setpoint.z);
    params.getParam("setpoint_yaw", Setpoint.yaw);
    // Controller gains
    params.getParam("gain_p_translational", Gains.p_translational);
    params.getParam("gain_d_translational", Gains.d_translational);
    params.getParam("gain_p_z", Gains.p_z);
    params.getParam("gain_d_z", Gains.d_z);
    params.getParam("gain_p_rotational", Gains.p_rotational);
    params.getParam("gain_d_rotational", Gains.d_rotational);
    params.getParam("gain_velocity", Gains.velocity);
  }

  ~Control()
  {
    // Publish empty msg as last message when closed
    velocity_ = geometry_msgs::Twist();
    velocity_publisher_.publish(velocity_);
  }

  void setpointCallback(const geometry_msgs::Pose setpoint)
  {
    // Coordinate frame: absolute world frame.
    // Since a pose is used as input, we have to deal with the quaternion.
    //
    tf::Quaternion q(setpoint.orientation.x, setpoint.orientation.y, setpoint.orientation.z, setpoint.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    Setpoint.x = setpoint.position.x;
    Setpoint.y = setpoint.position.y;
    Setpoint.z = setpoint.position.z;
    Setpoint.yaw = yaw;
  }

  void odomCallback(const nav_msgs::Odometry odom)
  {
    // Coordinate frame: absolute world frame.
    // Set time when message comes in (used for derivative)
    double current_time;
    current_time = ros::Time::now().toSec();

    // Prepare variables for error filtering
    double ex, ez, ey, eyaw;

    // Calculate current rotations based on sensor data. This is important, because it is used in all position error calculations. Filter yaw!
    tf::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    yaw_ = yaw;
    
    std_msgs::Float32 yaw_msg;
    yaw_msg.data = yaw_;
    yaw_publisher_.publish(yaw_msg);

    // Calculate errors. Positive error -> positive action needed
    ex = (Setpoint.x - odom.pose.pose.position.x);
    ey = (Setpoint.y - odom.pose.pose.position.y);
    ez = (Setpoint.z - odom.pose.pose.position.z);
    eyaw = Setpoint.yaw - yaw_;

    errorsx_.push_back(ex);
    errorsy_.push_back(ey);
    errorsz_.push_back(ez);
    times_.push_back(current_time);

    // Do some memory management 
    if(times_.size() > error_memory_) 
    {
      errorsx_.erase(errorsx_.begin());
      errorsy_.erase(errorsy_.begin());
      errorsz_.erase(errorsz_.begin());
      times_.erase(times_.begin());
    }

    // Store speeds for velocity damping
    speeds_.push_back(odom.twist.twist);
    if(speeds_.size() > 15)
    {
      speeds_.erase(speeds_.begin());
    }

    // Save definite error 
    Error.x_translational = ex; 
    Error.y_translational = ey;
    Error.z_translational = ez;
    Error.z_rotational = eyaw;

    ErrorDot.x_translational = fofw(errorsx_, times_, 10);
    ErrorDot.y_translational = fofw(errorsy_, times_, 10);
    ErrorDot.z_translational = fofw(errorsz_, times_, 10);

    std_msgs::Float32 errord_msg;
    errord_msg.data = ErrorDot.x_translational;
    errordx_publisher_.publish(errord_msg);

    ROS_INFO("%s", hovermode_ ? "true" : "false");
    if(hovermode_)
    {
      if(std::abs(Error.x_translational) < hover_treshold_ && std::abs(Error.y_translational) < hover_treshold_ && std::abs(Error.z_translational) < hover_treshold_)
      {
        ROS_INFO("HOVERING!");
        Error.x_translational = 0;
        Error.y_translational = 0;
        Error.z_translational = 0;
        Error.z_rotational = 0;

        ErrorDot.x_translational = 0;
        ErrorDot.y_translational = 0;
        ErrorDot.z_translational = 0;
        ErrorDot.z_rotational = 0;
      }
    }

    // Publish once every x time
    if(current_time - previous_publish_time_ > 1/freq_)
    {
      publishControl();
      previous_publish_time_ = current_time;  
    }
    
  }
  
  void simulationCallback(const gazebo_msgs::ModelStates pose)
  {
    // This function is called whenever a message on the simulation channel is found
    
    // Calculate roll pitch and yaw from quaternion data
    tf::Quaternion q(pose.pose[11].orientation.x, pose.pose[11].orientation.y, pose.pose[11].orientation.z, pose.pose[11].orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // Rotation error
    
    yaw_ = yaw;
    Error.z_rotational = Setpoint.yaw - yaw;

    std_msgs::Float32 yaw_msg;
    yaw_msg.data = yaw_;
    yaw_publisher_.publish(yaw_msg);

    // Position error
    Error.x_translational = -(Setpoint.x - pose.pose[11].position.x);
    Error.y_translational = -(Setpoint.y - pose.pose[11].position.y);
    Error.z_translational = Setpoint.z - pose.pose[11].position.z;

    publishControl();
    
    //ROS_INFO("Setpoint %f %f %f", Setpoint.x, Setpoint.y, Setpoint.z);
    ROS_INFO("Error %f %f %f %f", Error.x_translational, Error.y_translational, Error.z_translational, yaw);

  } 

  void publishControl()
  {
    // This function relates all errors in absolute world frame to body actions.
    // Positive errors means that we need positive action in that direction

    std_msgs::Float32 errorx_msg;
    errorx_msg.data = Error.x_translational;
    errorx_publisher_.publish(errorx_msg);

    // P - action
    double px, py, pz, pyaw;
    px = Error.x_translational * Gains.p_translational;
    py = Error.y_translational * Gains.p_translational;
    pz = Error.z_translational * Gains.p_z;
    pyaw = Error.z_rotational * Gains.p_rotational;

    std_msgs::Float32 p_action;
    p_action.data = px;
    p_publisher_.publish(p_action);
    //ROS_INFO("P-action %f", Error.x_translational);

    // D - action
    double dx, dy, dz, dyaw;
    dx = ErrorDot.x_translational * Gains.d_translational;
    dy = ErrorDot.y_translational * Gains.d_translational;
    dz = ErrorDot.z_translational * Gains.d_z;
    dyaw = ErrorDot.z_rotational * Gains.d_rotational;

    std_msgs::Float32 d_action;
    d_action.data = dx;
    d_publisher_.publish(d_action);

    double vx, vy, vz;
    vx = 0; vy = 0; vz = 0;
    // Make it go through honey! 
    if(velocity_damping_)
    {
      vx = speeds_.back().linear.x * Gains.velocity;
      vy = speeds_.back().linear.y * Gains.velocity;
      vz = speeds_.back().linear.z * Gains.velocity;
    }

    std_msgs::Float32 vel_action;
    vel_action.data = vx;
    velocity_damping_publisher_.publish(vel_action);

    // Total required actions in world frame
    double wx, wy, wz, wyaw;
    wx = px + dx + vx;
    wy = py + dy + vy;
    wz = pz + dz + vz;
    wyaw = pyaw + dyaw;

    // Transformation to the body fixed frame
    // This differs between simulation and real life
    double qx, qy, qz, qyaw;
    if(simulation)
    {
      qx = wx*-cos(yaw_) + wy*-sin(yaw_);
      qy = wx*sin(yaw_) + wy*-cos(yaw_);
    }
    else
    {
      qx = wx*-cos(yaw_) + wy*-sin(yaw_);
      qy = wx*sin(yaw_) + wy*-cos(yaw_);
    }
    qz = wz;
    qyaw = wyaw;

    // Sum actions (this is still in the absolute world frame)
    velocity_.linear.x = qx;
    velocity_.linear.y = qy;
    velocity_.linear.z = qz;
    velocity_.angular.z = qyaw;

    // LIMIT output to 1
    if(velocity_.linear.x > 1) { velocity_.linear.x = 1;}
    if(velocity_.linear.y > 1) { velocity_.linear.y = 1;}
    if(velocity_.linear.z > 1) { velocity_.linear.z = 1;}
    if(velocity_.linear.x < -1) { velocity_.linear.x = -1;}
    if(velocity_.linear.y < -1) { velocity_.linear.y = -1;}
    if(velocity_.linear.z < -1) { velocity_.linear.z = -1;}

    if(velocity_.angular.x > 1) { velocity_.angular.x = 1;}
    if(velocity_.angular.y > 1) { velocity_.angular.y = 1;}
    if(velocity_.angular.z > 1) { velocity_.angular.z = 1;}
    if(velocity_.angular.x < -1) { velocity_.angular.x = -1;}
    if(velocity_.angular.y < -1) { velocity_.angular.y = -1;}
    if(velocity_.angular.z < -1) { velocity_.angular.z = -1;}

    std_msgs::Float32 tot_x;
    tot_x.data = velocity_.linear.x;
    x_publisher_.publish(tot_x);
    //ROS_INFO("before publish");
    velocity_publisher_.publish(velocity_);
  }

  // Filter functions
  double lowPassFilter(double x, double y0, double dt, double T) // Extremely simple filter 
  {
     double res = y0 + (x - y0) * (dt_/(dt_+T_));
     return res;
  }

  double stateEstimationFilter(double x, double y0, double K)
  {
      /// TODO

      /*double period;
      period =  current_time - prev_time_;
      if(prev_time_ != 0 && period > 0)
      {   
          ErrorDot.x_translational = K_*(Error.x_translational - PrevError.x_translational);
          ErrorDot.y_translational = K_*(Error.y_translational - PrevError.y_translational);
          ErrorDot.z_translational = K_*(Error.z_translational - PrevError.z_translational);
          ErrorDot.z_rotational = K_*(Error.z_rotational - PrevError.z_rotational);

          PrevError.x_translational += period*K_*(Error.x_translational - PrevError.x_translational);
          PrevError.y_translational += period*K_*(Error.y_translational - PrevError.y_translational);
          PrevError.z_translational += period*K_*(Error.z_translational - PrevError.z_translational);
          PrevError.z_rotational += period*K_*(Error.z_rotational - PrevError.z_rotational);
      }*/
      return x;
  }

  double foaw(std::vector<double> memory, double band, int max)
  {

    /* FOAW approach on error */
    if(times_.size() > 1)
    {
      // We can calculate a speed!
      // We are never going to use more poses than pose_memory, so check for that
      /* TODO
    
      
      // At this point, we know the final number of poses available
      int n;
      n = times_.size();
      // FOAW outer loop
      // In this loop, we check if, given a certain beginning of the window, all points fit the line.
      // Loops over a set of possible steps back.
      int samplesBack;
      
      bool optimalSampleReached;
      double windowSpeed_x, windowSpeed_y, windowSpeed_z;
      optimalSampleReached = false; 
      samplesBack = 1;
      while(!optimalSampleReached)
      {
        windowSpeed_x = (errorsx_[n-1] - errorsx_[n-1-samplesBack])/(times_[n-1] - times_[n-1-samplesBack]);
        windowSpeed_y = (errorsy_[n-1] - errorsy_[n-1-samplesBack])/(times_[n-1] - times_[n-1-samplesBack]);
        windowSpeed_z = (errorsz_[n-1] - errorsz_[n-1-samplesBack])/(times_[n-1] - times_[n-1-samplesBack]);
        // At this point, we have a line through the two poins we know (at last and last - steps back). We do not have to check those. Check all points inbetween
        bool allInBand;
        int i;
        allInBand = true;
        for(i = n-1-samplesBack+1; i < n-1; i++)
        {
          if((std::abs(errorsx_[n-1-samplesBack] + windowSpeed_x*(times_[i]-times_[n-1-samplesBack])) > std::abs(errorsx_[i])+uncertainty_band_)
            && (std::abs(errorsy_[n-1-samplesBack] + windowSpeed_y*(times_[i]-times_[n-1-samplesBack])) > std::abs(errorsy_[i])+uncertainty_band_)
            && (std::abs(errorsz_[n-1-samplesBack] + windowSpeed_z*(times_[i]-times_[n-1-samplesBack])) > std::abs(errorsz_[i])+uncertainty_band_))
          {
            // If this happens, we can state that we cannot use this frame. End loop
            allInBand = false;
            break;
          }
        }

        // this many steps back still worked. Awesome! Try one more if possible
        if(allInBand && n > samplesBack + 1)
        {
          samplesBack++;
          if(samplesBack > error_memory_)
          {
            optimalSampleReached = true;
          }
        }
        else
        {
          // Too bad, we have to use this many steps back. Exit loop!
          optimalSampleReached = true;
        }
      }
      std_msgs::Int32 fb;
    fb.data = samplesBack+1;
    foaw_publisher_.publish(fb);
      */
      return memory.back();
    }
  }

  double fofw(std::vector<double> memory, std::vector<double> times, int samplesBack)
  {
    int n = times.size();
    if(n > samplesBack)
    {
      return (memory[n-1] - memory[n-1-samplesBack])/(times[n-1]-times[n-1-samplesBack]);
    }
    else if(n > 1)
    {
      return (memory[n-1] - memory[0])/(times[n-1]-times[0]);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");
  ROS_INFO("Before 1 loop");
  Control control;
  ROS_INFO("Before 2 loop");
  ros::spin();
  return 0;
}

