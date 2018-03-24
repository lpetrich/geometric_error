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

int MODE = 0;
int MODE2 = 0;
bool CALCULATE = false;
bool STEREO = false;
// coordinates from trackers [top_left, top_right, bot_right, bot_left, centroid, ... ]
std::vector<double> CORNERS;
std::vector<double> CORNERS2;
std::vector<double> CENTROID;
std::vector<double> CENTROID2;

class Error {
	ros::Publisher pub_err;
	ros::Publisher pub_eef;
    public:
        Error(ros::NodeHandle n) {
            pub_err = n.advertise<std_msgs::String>("/interface1/ErrorInfo", 10);
            pub_eef = n.advertise<std_msgs::String>("/eef_pos", 10);
        }

        ~Error() {
        	//TODO
        }

        Eigen::VectorXd get_error() {
			Eigen::VectorXd err;
			switch (MODE) {
                case 0:
                    return point_to_point();
                    break;
                case 1:
                    return point_to_line();
                    break;
                case 2:
                    return line_to_line();
                    break;
                case 3:
                    return parallel_lines();
                    break;
                case 4:
                    return points_to_conic();
                    break;
                case 5:
                	return point_to_plane();
                	break;
                default:
                    ROS_INFO_STREAM("WHAAAAAAAAT IS HAPPENING");
            }
        }
        Eigen::VectorXd point_to_point()
		{
			Eigen::VectorXd robot(2);
		  	Eigen::VectorXd target(2);
		  	Eigen::VectorXd result(2);
  			target(0) = CENTROID[0];
  			target(1) = CENTROID[1];
		  	robot(0) = CENTROID[2];
  			robot(1) = CENTROID[3];
  
  			result =  target - robot;
		  	ROS_INFO_STREAM("point to point error");
		  	return result.cwiseAbs(); 
		}
        Eigen::VectorXd point_to_line()
		{
  			Eigen::Vector3d robot, p1, p2;
  			robot << CENTROID[2], CENTROID[3], 1;
  			p1 << CORNERS[2], CORNERS[3], 1;
  			p2 << CORNERS[4], CORNERS[5], 1;
  
  			Eigen::VectorXd result(1);
  			result(0) = p1.cross(p2).dot(robot);
 			// for (int j = 0; j < CORNERS.size(); j++){
			// 	std::cout << CORNERS[j] << "\n";
			// }
			// std::cout << "\nCENTROIDS:\n " << CENTROID[0] << ", " << CENTROID[1]  << "\n";
			// std::cout << CENTROID[2] << ", " << CENTROID[3]  << "\n";
			// std::cout << "CORNERS:\n " << CORNERS[0] << ", " << CORNERS[1] << ", " << CORNERS[2] << ", " << CORNERS[3] << "\n";
			// std::cout << CORNERS[4] << ", " << CORNERS[5] << ", " << CORNERS[6] << ", " << CORNERS[7] << "\n";
			// std::cout << "P1:\n " << p1[0] << ", " << p2[1]  << "\n";
			// std::cout << "P2:\n " << p2[0] << ", " << p2[1]  << "\n";
			// std::cout << "ROBOT:\n " << robot[0] << ", " << robot[1]  << "\n";
		  	ROS_INFO_STREAM("point to line error");
		  	return result; 
		}
        Eigen::VectorXd line_to_line()
		{
  			Eigen::Vector3d r1, r2, p1, p2;
  			r1 << CORNERS[10], CORNERS[11], 1;
  			r2 << CORNERS[12], CORNERS[13], 1;
  			p1 << CORNERS[2], CORNERS[3], 1;
  			p2 << CORNERS[4], CORNERS[5], 1;
  			Eigen::Vector3d line = p1.cross(p2);
  			std::cout << "Line: " << line << std::endl;
  
  			Eigen::VectorXd result(1);
  			result(0) = line.dot(r1) + line.dot(r2);
 			// for (int j = 0; j < CORNERS.size(); j++){
			// 	std::cout << CORNERS[j] << "\n";
			// }
			// std::cout << "P1:\n " << p1[0] << ", " << p2[1]  << "\n";
			// std::cout << "P2:\n " << p2[0] << ", " << p2[1]  << "\n";			
			// std::cout << "R1:\n " << r1[0] << ", " << r2[1]  << "\n";
			// std::cout << "R2:\n " << r2[0] << ", " << r2[1]  << "\n";  
		  	ROS_INFO_STREAM("line to line error");
		  	return result; 
		}
        Eigen::VectorXd parallel_lines()
		{
  			Eigen::Vector3d r1, r2, p1, p2, l1, l2, intersection;

  			r1 << CORNERS[10], CORNERS[11], 1;
  			r2 << CORNERS[12], CORNERS[13], 1;
  			p1 << CORNERS[3], CORNERS[4], 1;
  			p2 << CORNERS[5], CORNERS[6], 1;

  			Eigen::VectorXd result(1);
  			l1 = r1.cross(r2);
  			l2 = p1.cross(p2);
  			intersection = l1.cross(l2);
  			result(0) = intersection(2);
  
		  	ROS_INFO_STREAM("par");
		  	return result; 
		}
        Eigen::VectorXd points_to_conic()
		{
			// // uses code from visp
			// //std::cout << "Ellipse Error begin" << std::endl;
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

			Eigen::VectorXd result(1);
			// result(0) = (p(0) * p(0)) + (e(0) * p(1) * p(1)) + (2 * e(1) * p(0) * p(1)) + (2 * e(2) * p(0)) + (2 * e(3) * p(1)) + e(4);  
			  
			//std::cout << "Ellipse Error end" << std::endl;
		  	return result; 

		}
        Eigen::VectorXd point_to_plane()
		{
  			Eigen::Vector3d p, p0, vec1, vec2, n, n_unit;

  			// p << points_[0]->coord.x, points_[0]->coord.y, 1;
  			// p0 << points_[2]->coord.x, points_[2]->coord.y, 1;
  			// vec1 << points_[2]->coord.x - points_[1]->coord.x, points_[2]->coord.y - points_[1]->coord.y, 1;
  			// vec2 << points_[2]->coord.x - points_[3]->coord.x, points_[2]->coord.y - points_[3]->coord.y, 1;

  			Eigen::VectorXd result(1);
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
	      geometric_error::TrackPoint pt;
	      geometric_error::TrackedPoints pt_msg;
	      pt.x = CENTROID[2];
	      pt.y = CENTROID[3];
	      pt_msg.points.push_back(pt);
	      pub_eef.publish(pt_msg);
	    }
	    // Publish the error.
	    geometric_error::ErrorInfo error_msg;
	    Eigen::VectorXd error = get_error();
		std::cout << "\nCURRENT ERROR:\n";
			for (int j = 0; j < error.size(); j++){
			std::cout << error[j] << " ";
		}
	 	if (STEREO) {
			error_msg.error.push_back(error(0));
			error_msg.error.push_back(error(1));
			error_msg.error_dim.push_back(2);
		} else {
			error_msg.error.push_back(error(0));
			error_msg.error_dim.push_back(1);
	 	}
	    if (pub_err.getNumSubscribers() != 0) {
	      pub_err.publish(error_msg);
	    }
	}
};

class Calculator {
	Error *e;
    public:
        Calculator() {
            ros::NodeHandle n;
    		e = new Error(n);
            ROS_INFO_STREAM("Initializing subscribers and publishers");
        }

        ~Calculator() {
        	//TODO
        }

        void callback_tracker1(const std_msgs::String::ConstPtr& msg) {
        	if (CALCULATE) {
	        	std::stringstream ss;
	        	ss << msg->data.c_str();
	        	// std::string C = ss.str();
	         	// std::cout << "\nCoord: \n" << C << ">>> ";
	        	std::istringstream iss(ss.str());
	        	std::vector<std::string> c;
	        	std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter(c));
	        	std::vector<double> temp1;
	        	std::vector<double> temp2;
				for (int i = 0; i < c.size(); i++) {
					std::istringstream iss(c[i]);
					double d;
					iss >> d;
					if (i == 8 || i == 9 || i == 18 || i == 19){
						temp2.push_back(d);
					} else {
						temp1.push_back(d);
					}
				}
				CORNERS = temp1;
				CENTROID = temp2;
			}
        }

        void callback_mode1(const std_msgs::UInt32::ConstPtr& msg) {
        	MODE = msg->data;
        }

        void callback_tracker2(const std_msgs::String::ConstPtr& msg) {
        	if (CALCULATE) {
	        	std::stringstream ss;
	        	ss << msg->data.c_str();
	        	std::istringstream iss(ss.str());
	        	std::vector<std::string> c;
	        	std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter(c));
	        	std::vector<double> temp1;
	        	std::vector<double> temp2;
				for (int i = 0; i < c.size(); i++) {
					std::istringstream iss(c[i]);
					double d;
					iss >> d;
					if (i == 8 || i == 9 || i == 18 || i == 19){
						temp2.push_back(d);
					} else {
						temp1.push_back(d);
					}
				}
				CORNERS2 = temp1;
				CENTROID2 = temp2;
			}
        }

        void callback_mode2(const std_msgs::UInt32::ConstPtr& msg) {
        	MODE2 = msg->data;
         	// std::cout << "\nMODE2: \n" << MODE2; 
        }
        void callback_calculate(const std_msgs::Bool::ConstPtr& msg) {
        	CALCULATE = msg->data;
        	// std::cout << "\nCALCULATE: " << CALCULATE;
        }

        void loop() {
        	ROS_INFO_STREAM("YAAAAAY");
            std::string line;
            while (ros::ok()) {
            	if (CALCULATE) {
            		e->publish_error();
	                // std::cout << "\nPress c to confirm: \n";
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
        	}
        }
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "VS_Control");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    Calculator c;
    ros::Subscriber sub_t1 = n.subscribe("/tracker1/node1/patch_tracker", 100, &Calculator::callback_tracker1, &c);
    ros::Subscriber sub_t2 = n.subscribe("/tracker2/node2/patch_tracker", 100, &Calculator::callback_tracker2, &c);
    ros::Subscriber sub_m1 = n.subscribe("/node1/mode", 100, &Calculator::callback_mode1, &c);
    ros::Subscriber sub_m2 = n.subscribe("/node2/mode", 100, &Calculator::callback_mode2, &c);
    ros::Subscriber sub_c = n.subscribe("/calculate", 100, &Calculator::callback_calculate, &c);
    c.loop();
    return 0;
}