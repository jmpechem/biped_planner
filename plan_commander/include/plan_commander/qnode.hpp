/**
 * @file /include/plan_commander/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef plan_commander_QNODE_HPP_
#define plan_commander_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#include "planner_msgs/Mapbuilder.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace plan_commander {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }	

	void send_transition(std::string str,unsigned int id_,float value1, float value2, float value3);
	void pose_callback(const planner_msgs::Mapbuilder::ConstPtr &msg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void poseInfoUpdated();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    QStringListModel logging_model;
        bool isConnected;
        ros::Publisher state_publisher;
        ros::Subscriber pose_sub;
public:
        planner_msgs::Mapbuilder pose_recv_msg;
};

}  // namespace plan_commander

#endif /* plan_commander_QNODE_HPP_ */
