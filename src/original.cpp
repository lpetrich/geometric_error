/* lpetrich /**/

#include "ros/rate.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h" 
#include "std_msgs/UInt32.h" 
#include "geometric_error/Error.h"
#include "geometric_error/ErrorInfo.h"
#include "geometric_error/TrackPoint.h"
#include "geometric_error/TrackedPoints.h"
#include <sstream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include <stdlib.h>
#include <cstring>
#include <unistd.h>

double MAX = 1.0e7;
bool CALCULATE, STEREO, TASKS_SET, RESET;
int count1, count2;
// coordinates from trackers [top_left, top_right, bot_right, bot_left, centroid, ... ]
std::vector<int> task_ids;
std::vector<std::vector<int> > trackers_cam1, trackers_cam2;
// to hold end effector position
geometric_error::TrackPoint eef_pt;
std_msgs::Bool reset;

class Error {
	ros::Publisher pub_reset;
	ros::Publisher pub_eef;
	ros::Publisher pub_err;
	ros::Publisher pub_err2;
    public:
        Error(ros::NodeHandle n) {
        	std::cout << "Initializing error calculation class and publishers.";
            pub_reset = n.advertise<std_msgs::Bool>("/error_reset", 10);
            pub_eef = n.advertise<geometric_error::TrackedPoints>("/eef_pos", 10);
            pub_err = n.advertise<geometric_error::ErrorInfo>("/cam1/ErrorInfo", 10);
            if (STEREO) {
            	pub_err2 = n.advertise<geometric_error::ErrorInfo>("/cam2/ErrorInfo", 10);
            }
        }

        ~Error() {
        	//TODO
        }

        bool get_error(int cam) {
        	geometric_error::ErrorInfo error_msg;

			if (cam == 2) {
				while (trackers_cam2.size() == 0) { continue; }
			} else {
				while (trackers_cam1.size() == 0) { continue; }
			}

			int k = 0;
        	for (int i = 0; i < task_ids.size(); i++) {
        		std::vector<std::vector<int> > task_coords;
				Eigen::VectorXd e;
				double err;
				switch (task_ids[i]) {
	                case 0:
	                	task_coords.resize(2);
	                	for (int j = k; j < k + 2; j++) {
	                		if (cam == 2){
	                			task_coords.push_back(trackers_cam2[j]);
	                		} else {
	                			task_coords.push_back(trackers_cam1[j]);
	                		}
	                	}
	                	// end effector posiion
    					eef_pt.x = task_coords[1][0];
						eef_pt.y = task_coords[1][1];
	                    e = point_to_point(task_coords);
	                    // add to error message, 2 dimensions -- x & y
            			error_msg.error.push_back(e(0));
						error_msg.error.push_back(e(1));
						error_msg.error_dim.push_back(2);
	                    k += 2;
	                    err = e.norm();
	                    break;
	                case 1:
	                	task_coords.resize(3);
	                	for (int j = k; j < k + 3; j++) {
	                		if (cam == 2){
	                			task_coords.push_back(trackers_cam2[j]);
	                		} else {
	                			task_coords.push_back(trackers_cam1[j]);
	                			// for (int a = 0; a < trackers_cam1[j].size(); a++)
	                			// 	std::cout << trackers_cam1[j][a] << " ";
	                			// std::cout << "\n";
	                		}
	                	}
    					eef_pt.x = task_coords[2][0];
						eef_pt.y = task_coords[2][1];
	                    err = point_to_line(task_coords);
	                    error_msg.error.push_back(err);
						error_msg.error_dim.push_back(1);
	                    k += 3;                   
	                    break;
	                case 2:
	                	task_coords.resize(4);
	                	for (int j = k; j < k + 4; j++) {
	                		if (cam == 2){
	                			task_coords.push_back(trackers_cam2[j]);
	                		} else {
	                			task_coords.push_back(trackers_cam1[j]);
	                		}
	                	}
    					eef_pt.x = task_coords[2][0];
						eef_pt.y = task_coords[2][1];	                    
						err = line_to_line(task_coords);
	                    error_msg.error.push_back(err);
						error_msg.error_dim.push_back(1);
	                    k += 4;
	                    break;
	                case 3:
	                	task_coords.resize(4);
   	                	for (int j = k; j < k + 4; j++) {
	                		if (cam == 2){
	                			task_coords.push_back(trackers_cam2[j]);
	                		} else {
	                			task_coords.push_back(trackers_cam1[j]);
	                		}
	                	}
    					eef_pt.x = task_coords[2][0];
						eef_pt.y = task_coords[2][1];	
	                    err = parallel_lines(task_coords);
	                    error_msg.error.push_back(err);
						error_msg.error_dim.push_back(1);
	                    k += 4;
	                    break;
	                case 4:
	                	task_coords.resize(8);
   	                	for (int j = k; j < k + 8; j++) {
	                		if (cam == 2){
	                			task_coords.push_back(trackers_cam2[j]);
	                		} else {
	                			task_coords.push_back(trackers_cam1[j]);
	                		}
	                	}
    					eef_pt.x = task_coords[5][0];
						eef_pt.y = task_coords[5][1];	
	                    err = points_to_conic(task_coords);
	                    error_msg.error.push_back(err);
						error_msg.error_dim.push_back(1);
	                    k += 8;
	                    break;
	                case 5:
	                	err = point_to_plane();
	                    break;
	                default:
	                    ROS_WARN_STREAM("INVALID TASK ID");
	                    break;
	            }
	            if (fabs(err) > MAX) {
	            	std::cout << fabs(err) << "\n";
	            	ROS_WARN("LOST TRACKER, RESETTING EVERYTHING");
	            	if (reset_all(true)) { return true; }
	            	else { return false; }
	            } 
	        }
            if (cam == 2) {
			    if (pub_err2.getNumSubscribers() != 0) {
					pub_err2.publish(error_msg);
			    }
            } else {
			    if (pub_err.getNumSubscribers() != 0) {
					pub_err.publish(error_msg);
			    }
            }
	        return true;
	    }

        Eigen::VectorXd point_to_point(std::vector<std::vector<int> > coords)
		{
			Eigen::Vector2d robot(2);
		  	Eigen::Vector2d target(2);
		  	Eigen::Vector2d result(2);
  			target(0) = coords[0][0];
  			target(1) = coords[0][1];
		  	robot(0) = coords[1][0];
  			robot(1) = coords[1][1];
  			result =  target - robot;
		  	return result.cwiseAbs(); 
		}

        double point_to_line(std::vector<std::vector<int> > coords)
		{
  			Eigen::Vector3d robot, p1, p2;
  			robot << coords[0][0], coords[0][1], 1;
  			p1 << coords[1][0], coords[1][1], 1;
  			p2 << coords[2][0], coords[2][1], 1;
  			double result;
  			result = p1.cross(p2).dot(robot);
		  	return result; 
		}

        double line_to_line(std::vector<std::vector<int> > coords)
		{
  			Eigen::Vector3d r1, r2, p1, p2;
  			r1 << coords[0][0], coords[0][1], 1;
  			r2 << coords[1][0], coords[1][1], 1;
  			p1 << coords[2][0], coords[2][1], 1;
  			p2 << coords[3][0], coords[3][1], 1;
  			Eigen::Vector3d line = p1.cross(p2);  
  			double result;
  			result = line.dot(r1) + line.dot(r2); 
		  	return result; 
		}

        double parallel_lines(std::vector<std::vector<int> > coords)
		{
  			Eigen::Vector3d r1, r2, p1, p2, l1, l2, intersection;
  			r1 << coords[0][0], coords[0][1], 1;
  			r2 << coords[1][0], coords[1][1], 1;
  			p1 << coords[2][0], coords[2][1], 1;
  			p2 << coords[3][0], coords[3][1], 1;
  			double result;
  			l1 = r1.cross(r2);
  			l2 = p1.cross(p2);
  			intersection = l1.cross(l2);
  			result = intersection(2);
		  	return result; 
		}

        double points_to_conic(std::vector<std::vector<int> > coords)
		{
			// uses code from visp
			// Eigen::MatrixXd A(points_.size() - 1, 5);
			// Eigen::VectorXd b(points_.size() - 1);
			// Eigen::Vector3d p;
			// p << normX(points_[0]->coord.x), normY(points_[0]->coord.y), 1;
			  
			// // A = (y^2 2xy 2x 2y 1)   x = (K0 K1 K2 K3 K4)^T  b = (-x^2 )
			// for(int i = 1; i < points_.size(); i++) {
			//     b(i - 1) = -(normX(points_[i]->coord.x) * normX(points_[i]->coord.x));
			//     A(i - 1, 0) = normY(points_[i]->coord.y) * normY(points_[i]->coord.y);
			//     A(i - 1, 1) = 2 * normX(points_[i]->coord.x) * normY(points_[i]->coord.y);
			//     A(i - 1, 2) = 2 * normX(points_[i]->coord.x);
			//     A(i - 1, 3) = 2 * normY(points_[i]->coord.y);
			//     A(i - 1, 4) = 1;
			// }
			  
			// // Solve Ax = b, least squares minimization.  
			// Eigen::JacobiSVD<Eigen::MatrixXd> svd = A.jacobiSvd(Eigen::ComputeThinU | 
			//                                                       Eigen::ComputeThinV);
			                                                      
			// e = svd.solve(b); // the ellipse parameters are stored as a class variable. */

			double result;
			// // result(0) = (p(0) * p(0)) + (e(0) * p(1) * p(1)) + (2 * e(1) * p(0) * p(1)) + (2 * e(2) * p(0)) + (2 * e(3) * p(1)) + e(4);  
			  
			std::cout << "Ellipse Error" << std::endl;
		  	return result; 

		}

        double point_to_plane()
		{
  			Eigen::Vector3d p, p0, vec1, vec2, n, n_unit;

  			// p << points_[0]->coord.x, points_[0]->coord.y, 1;
  			// p0 << points_[2]->coord.x, points_[2]->coord.y, 1;
  			// vec1 << points_[2]->coord.x - points_[1]->coord.x, points_[2]->coord.y - points_[1]->coord.y, 1;
  			// vec2 << points_[2]->coord.x - points_[3]->coord.x, points_[2]->coord.y - points_[3]->coord.y, 1;

  			double result;
  			// n = (vec1).cross(vec2);
  			// n_unit = (1/n.norm()) * n;
  			// double  d = (-(n(0) * p0(0)) - (n(1) * p0(1)) - (n(2) * p0(2)));// / n.norm();  
  			// //result(0) = n_unit.dot(p) + d;
  			// result(0) = (n(0) * p(0)) + (n(1) * p(1)) + (n(0) * p(2)) + d;
  			//std::cout << "Plane: " << result(0) << std::endl;
		  	return result; 
		}

		void publish_error() {
		    // Publish coordinate of designated end-effector point (for Jacobian update)
		    if (pub_eef.getNumSubscribers() != 0) {
			    geometric_error::TrackedPoints eef_msg;
				eef_msg.points.push_back(eef_pt);
				pub_eef.publish(eef_msg);
		    }
		    // Publish the error.
			get_error(1);
			if (STEREO) {
				get_error(2);		  
			}
		}

		bool reset_all(bool pub) {
			if (pub) {
				reset.data = true;
				pub_reset.publish(reset);
			}
			CALCULATE = false;
			TASKS_SET = false;
			task_ids.clear();
			for (int i = 0; i < task_ids.size(); i++) {
				trackers_cam1[i].clear();
				if (STEREO) {
					trackers_cam2[i].clear();
				}
			}
			trackers_cam1.clear(); 
			trackers_cam2.clear();
			count1 = 0;
			count2 = 0;
			usleep(2000);
			return true;
		}
};

class Callbacks {
	Error *e;
    public:
        Callbacks() {
            ros::NodeHandle n;
    		e = new Error(n);
            std::cout << "Initializing subscribers.\n";
        }

        ~Callbacks() {
        	//TODO
        }

        void callback_trackers_cam1(const std_msgs::String::ConstPtr& msg) {
        	if (CALCULATE && TASKS_SET) {
        		while (count1 <= 15) {
        			count1++;
        			continue;
        		}
        		count1 = 16;
        		trackers_cam1.clear();
        		std::string m = msg->data;
        		std::string s = ";";
				int i = 0;
				int j = 0;
				double d = 0.0;
				std::istringstream iss(m);
				do {
					std::vector<int> temp;
					std::string subs;
					iss >> subs;
					i = iss.peek();
					if (i != -1) {
						// std::cout << subs << " subs\n";
						if (subs != s){
							d = std::atof(subs.c_str());
							temp.push_back(d);
							j++;
							if (j == 2) {
								trackers_cam1.push_back(temp);
								j = 0;
							}
						}
					}
				} while (iss);
			}
        }

        void handleTask1(const std::vector<int> v) {
			std::vector<int> p;
			for (int i = 0; i < v.size(); i++) {
				if (i % 2 == 0) {
					p.push_back(v[i]);
				} else {
					p.push_back(v[i]);
					trackers_cam1.push_back(p);
				}
			}
		}

        void handleTask2(const std::vector<int> v) {
			std::vector<int> p2;
			for (int i = 0; i < v.size(); i++) {
				if (i % 2 == 0) {
					p2.push_back(v[i]);
				} else {
					p2.push_back(v[i]);
					trackers_cam2.push_back(p2);
				}
			}
		}

        void callback_trackers_cam2(const std_msgs::String::ConstPtr& msg) {
        	if (CALCULATE && TASKS_SET) {
        		while (count1 <= 15) { continue; }
        		trackers_cam2.clear();
        		std::string m2 = msg->data;
        		std::string s2 = ";";
				std::vector<int> temp2;
				int i2;
				double d2;
				std::istringstream iss2(m2);
				do {
					std::string subs2;
					iss2 >> subs2;
					i2 = iss2.peek();
					if (i2 != -1) {
						if (subs2 == s2){
							handleTask2(temp2);
							temp2.clear();
						} else {
							d2 = std::atof(subs2.c_str());
							temp2.push_back(d2);
						}
					}
				} while (iss2);
			}
        }

        void callback_task_ids(const std_msgs::String::ConstPtr& msg) {
        	if (!TASKS_SET) {
	        	std::stringstream ss;
	        	ss << msg->data.c_str();
	        	std::istringstream iss(ss.str());
	        	std::vector<std::string> c;
	        	std::vector<int> temp;
	        	std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter(c));
				for (int i = 0; i < c.size(); i++) {
					std::istringstream iss(c[i]);
					int ID;
					iss >> ID;
					temp.push_back(ID);
				}
				task_ids = temp;
				trackers_cam1.resize(task_ids.size());
				if (STEREO) {
					trackers_cam2.resize(task_ids.size());
				}
				// for (int i = 0; i < task_ids.size(); i++)
	   //  			std::cout << task_ids[i] << " ";
	  	// 		std::cout << '\n';
				count1 = 0;
				count2 = 0;
	  			TASKS_SET = true;
	        }
        }

        void callback_calculate(const std_msgs::UInt32::ConstPtr& msg) {
        	int c = msg->data;
        	if (c == 1) { CALCULATE = true; }
        	else { e->reset_all(false); }
        }

        void loop() {
        	ros::Rate r(10);
        	e->reset_all(false);
            while (ros::ok()) {
            	if (CALCULATE && TASKS_SET && count1 >= 15) {
	          		e->publish_error();
	                // std::cout << "\nPress c to confirm: \n";
	                // std::string line;
	                // std::getline(std::cin, line);
	                // std::stringstream ss;
	                // switch (line[0]) {
	                //     case 'c':
	                //         ROS_INFO_STREAM("Confirmed. Calculating Error.");
	                //         e->publish_error();
	                //         break;
	                //     default:
	                //     std::cout << line[0];
	                //         ROS_INFO_STREAM("WHAAAAAAAAT IS HAPPENING");
	                // }
            	}
                r.sleep();
        	}
        }
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "error_control");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    Callbacks c;
	// initialize global variables
	STEREO = false;
	CALCULATE = false;
	TASKS_SET = false;
	task_ids.clear();
	trackers_cam1.clear(); 
	trackers_cam2.clear();
	count1 = 0;
	count2 = 0;

	ros::master::V_TopicInfo master_topics;
	ros::master::V_TopicInfo::iterator it;
	ros::master::getTopics(master_topics);
	for (it = master_topics.begin(); it != master_topics.end(); it++) {
		const ros::master::TopicInfo& info = *it;
		std::string topic_name = info.name;
		std::string prefix = "/cam2/camera";
		if(topic_name.substr(0, prefix.size()) == prefix) {
			STEREO = true;
		}
	}
	if (STEREO) {
		std::cout << "Stereo vision activated.\n";
	}

    ros::Subscriber sub_m = n.subscribe("/task_ids", 100, &Callbacks::callback_task_ids, &c);
    ros::Subscriber sub_c = n.subscribe("/calculate", 100, &Callbacks::callback_calculate, &c);
    ros::Subscriber sub_t1 = n.subscribe("/cam1/centers", 100, &Callbacks::callback_trackers_cam1, &c);
    ros::Subscriber sub_t2 = n.subscribe("/cam2/centers", 100, &Callbacks::callback_trackers_cam2, &c);
    c.loop();

    return 0;
}