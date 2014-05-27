	void poseCallback(const geometry_msgs::Pose pose)
	{
		// set time
		double current_time;
		current_time = ros::Time::now().toSec();

		// add pose to poses vector, time to times vector. Sometimes however, the last pose is already in the vector. I have no idea why. Do not do anything than
		if(poses_.size() > 0)
		{
			if(current_time != times_.back())
			{
				int n;
				poses_.push_back(pose);
				times_.push_back(current_time);
				n = poses_.size();

				// if poses vector is too large, remove first one
				if(n >= pose_memory_)
				{
					poses_.erase(poses_.begin());
					times_.erase(times_.begin());
					n = pose_memory_;
				}

				// prepare msg
				nav_msgs::Odometry odom;
				odom.header.stamp = ros::Time::now();
				odom.header.frame_id = "filtered_state";

				/* Uncomment lines below to filter pose with standard lowpass filter */
				//pose_.position.x = lowPassFilter(pose.position.x, pose_.position.x, dt_, T_);
				//pose_.position.y = lowPassFilter(pose.position.y, pose_.position.y, dt_, T_);
				//pose_.position.z = lowPassFilter(pose.position.z, pose_.position.z, dt_, T_);
				/* Uncomment lines below to NOT filter pose with standard lowpass filter */
				pose_.position.x = pose.position.x;
				pose_.position.y = pose.position.y;
				pose_.position.z = pose.position.z;
				// Letś just neglect rotation.
				pose_.orientation.x = pose.orientation.x;
				pose_.orientation.y = pose.orientation.y;
				pose_.orientation.z = pose.orientation.z;
				pose_.orientation.w = pose.orientation.w;
				odom.pose.pose = pose_;
				
				/* Start First Order Adaptive Windowing Approach */
				// Set maximum steps back. This can be a limit from the size of the array (beginning)
				int max_back, back, final_back;
				max_back = std::min(n-1,pose_memory_);

				// Prepare speed variables
				double velx, vely, velz, period;

				// Define default first window
				geometry_msgs::Pose window_begin;
				back = 1; // So, the difference between this one and the previous sample
				final_back = 1;
				while(back <= max_back)
				{
					window_begin = poses_[n-back-1];
					period = current_time - times_[n-back-1];
					if(period > 0.000000)
					{
						// Calculate slope of the line
						velx = (pose.position.x - window_begin.position.x)/period;
						vely = (pose.position.y - window_begin.position.y)/period;
						velz = (pose.position.z - window_begin.position.z)/period;

						// Check if slope is not too far from the points within the window
						bool in_band;
						int i;
						double xval, accT;
						in_band = true;
						
						xval = poses_[n-back-1].position.x;
						for(i=n-back;i<n-1;i++)
						{
							if(!in_band) {break;}
							double predicted_position;
							predicted_position = xval + velx*(times_[i]-times_[n-back-1]);
							if(std::abs(predicted_position) > std::abs(poses_[i].position.x+uncertainty_band_))
							{
								// Deviation too large. 
								in_band = false;
							} 
						}
						if(!in_band) {break;}
					}
					else
					{
						ROS_INFO("Period: %f when comparing using index %i with corresponding time %f and current_time %f",period,n-back-1,times_[n-back-1],current_time);
						int k;
						for(k=0;k<n;k++)
						{
							ROS_INFO("Pose %f at time %f",poses_[k].position.x, times_[k]);
						}
					}	
					// For this many step back everything is fine. This is always the first time.
					final_back = back;
					back = back + 1;
					
					
				}
				std_msgs::Int32 fb;
				fb.data = final_back;
				foaw_publisher_.publish(fb);

				vel_.linear.x = (pose.position.x - poses_[n-final_back-1].position.x)/(current_time-times_[n-final_back-1]);
				vel_.linear.y = (pose.position.y - poses_[n-final_back-1].position.y)/(current_time-times_[n-final_back-1]);
				vel_.linear.z = (pose.position.z - poses_[n-final_back-1].position.z)/(current_time-times_[n-final_back-1]);

				odom.twist.twist = vel_;

				// Get previous pose and calculate differences, if previous pose is not all zero
				if(prev_time_ != 0)
				{
					// Calculate angles
					tf::Quaternion prev_q(prev_pose_.orientation.x, prev_pose_.orientation.y, prev_pose_.orientation.z, prev_pose_.orientation.w);
		    		tf::Matrix3x3 prev_m(prev_q);
		    		double prev_roll, prev_pitch, prev_yaw;
		    		prev_m.getRPY(prev_roll, prev_pitch, prev_yaw);

		    		tf::Quaternion q(pose_.orientation.x, pose_.orientation.y, pose_.orientation.z, pose_.orientation.w);
					tf::Matrix3x3 m(q);
					double roll, pitch, yaw;
					m.getRPY(roll, pitch, yaw);

					std_msgs::Float32 pubpitch;
					pubpitch.data = roll;
					pitch_publisher_.publish(pubpitch);

					// Uncomment this for other filter methods
					/*

		    		double velx, vely, velz, velyaw, period;
		    		period = current_time - prev_time_;
		    		if(period > 0)
		    		{
		    			//Discrete derivative approach 
		    			//velx = (pose_.position.x - prev_pose_.position.x)/period;
						//vely = (pose_.position.y - prev_pose_.position.y)/period;
						//velz = (pose_.position.z - prev_pose_.position.z)/period;
						//velyaw = (yaw - prev_yaw)/period;

						//vel_.linear.x = lowPassFilter(velx, vel_.linear.x, dt_, T_);
						//vel_.linear.y = lowPassFilter(vely, vel_.linear.y, dt_, T_);
						//vel_.linear.z = lowPassFilter(velz, vel_.linear.z, dt_, T_);
						//vel_.angular.z = lowPassFilter(velyaw, vel_.angular.z, dt_, T_);

						//State estimation filter approach
		    			vel_.linear.x = K_*(pose_.position.x - prev_pose_.position.x);
		    			vel_.linear.y = K_*(pose_.position.y - prev_pose_.position.y);
		    			vel_.linear.z = K_*(pose_.position.z - prev_pose_.position.z);
		    			vel_.angular.z = K_*(yaw - prev_yaw_);

						prev_pose_.position.x += period*K_*(pose_.position.x - prev_pose_.position.x);
						prev_pose_.position.y += period*K_*(pose_.position.y - prev_pose_.position.y);
						prev_pose_.position.z += period*K_*(pose_.position.z - prev_pose_.position.z);
						prev_yaw_ += period*rate_yaw_;

						odom.twist.twist = vel_;
		    		}
		    		*/
				}
				vel_publisher_.publish(vel_);
				pose_publisher_.publish(pose_);
				odom_publisher_.publish(odom);

				prev_pose_ = pose_;
				prev_time_ = current_time;
			}
		}
		else
		{
			poses_.push_back(pose);
			times_.push_back(current_time);
		}
	}
