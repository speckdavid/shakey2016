#include <QtGui/QApplication>
#include "mainwindow.h"
#include <ros/ros.h>


int main(int argc, char *argv[]) {
	ros::init(argc, argv, "shakey_quickstart_gui");

	QApplication a(argc, argv);
	MainWindow w;
	w.show();
	ros::WallRate rate(100);
	while (ros::ok() && w.isVisible()) {
		ros::spinOnce();
		w.vis_pub.publish(w.poses_marker);
		a.processEvents();

		rate.sleep();
	}
	return 0;
}
