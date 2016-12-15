#ifndef ASV_CTRL_SB_MPC_NODE
#define ASV_CTRL_SB_MPC_NODE

#include "asv_msgs/StateArray.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"

#include <vector>

class simulationBasedMpcNode
{
	public:
	/// Constructor
	simulationBasedMpcNode();
	/// Destructor
	~simulationBasedMpcNode();
	
	/**
	 * Initializes the Velocity Obstacle node
	 *
	 * @param cmd_pub
	 * @param mk_pub
	 * @param obstacle_sub
	 * @param og_sub
	 * @param asv_sub
	 * @param cmd_sub
	 * @param vo
	 */	
	void initialize(ros::Publisher *cmd_pub,
					ros::Publisher *os_pub,
					ros::Subscriber *obstacle_sub,
					ros::Subscriber *og_sub,
					ros::Subscriber *asv_sub,
					ros::Subscriber *cmd_sub,
					simulationBasedMpc *sb_mpc);
	
	/**
	 * Start the node. Enters a "never ending" while loop.
	 */
	void start();	
	
	void asvCallback(const nav_msgs::Odometry::ConstPtr &msg);
	void obstacleCallback(const asv_msgs::StateArray::ConstPtr &msg);
	void cmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
	void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
	
	private:
	
	geometry_msgs::Twist cmd_vel_;
	asv_msgs::Offset os_;
	
	simulationBasedMpc *sb_mpc_;
	
	nav_msgs::OccupancyGrid map_;
	std::vector<asv_msgs::State> obstacles_;
	Eigen::Vector3d asv_pose_;
	Eigen::Vector3d asv_twist_;
	

	double u_d_;
	double psi_d_;
	double u_os_;
	double psi_os_;
	
	// ROS API
	ros::Publisher *cmd_pub_;
	ros::Publisher *os_pub_;
	
	ros::Subscriber *obstacle_sub_;
	ros::Subscriber *og_sub_;
	ros::Subscriber *asv_sub_;
	ros::Subscriber *cmd_sub_;
};

#endif
