#include <QtGui/QApplication>
#include "mainwindow.h"
#include <ros/ros.h>


int main(int argc, char *argv[]) {
	ros::init(argc, argv, "shakey_quickstart_gui");
	ros::NodeHandle nh;

	QApplication a(argc, argv);
	MainWindow w;
	w.show();
	ros::Subscriber sub = nh.subscribe("move_base_simple/goal", 1000,
			&MainWindow::add_pose, &w);
	ros::WallRate rate(100);
	while (ros::ok() && w.isVisible()) {
		ros::spinOnce();

		a.processEvents();

		rate.sleep();
	}
	return 0;
}
