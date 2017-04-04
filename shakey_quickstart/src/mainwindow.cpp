#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QModelIndex>
#include <QtGui/QApplication>
#include <QFileDialog>
#include <QDir>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "shakey_utils/geometryPoses.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

// Get Output of system calls
string GetStdoutFromCommand(string cmd) {

	string data;
	FILE * stream;
	const int max_buffer = 256;
	char buffer[max_buffer];
	cmd.append(" 2>&1");

	stream = popen(cmd.c_str(), "r");
	if (stream) {
		while (!feof(stream))
			if (fgets(buffer, max_buffer, stream) != NULL)
				data.append(buffer);
		pclose(stream);
	}
	return data;
}

MainWindow::MainWindow(QWidget *parent) :
		QMainWindow(parent), ui(new Ui::MainWindow) {
	ui->setupUi(this);
}

MainWindow::~MainWindow() {
	delete ui;
}

void MainWindow::on_pushButton_createFolder_clicked() {
	QString path = QString::fromStdString(this->shakey_path);
	path.append(ui->worldName->text());
	if(QDir(path).exists() || path.length() < 0) {
		//TODO: Add some useful content
	} else {
		QDir().mkdir(path);

	}
	cout << "created folder: " << path.toUtf8().constData() << endl;

}

void MainWindow::on_pushButton_clicked() {
	if (ui->ListPoses->currentIndex().isValid()) {
		QListWidgetItem* item = ui->ListPoses->currentItem();
		delete item;
	}
}

void MainWindow::on_mapUpdateButton_clicked() {
	system("cd /home/david/Pictures/ && rosrun map_server map_saver");
	QString filename = "/home/david/Pictures/double_push.png";
	QImage image(filename);
	ui->currentMap->setPixmap(
			QPixmap::fromImage(image).scaled(100, 100, Qt::KeepAspectRatio));
	//QString dir = QFileDialog::getExistingDirectory(this, tr("Choose a world directory"),
	//                                            "/home",
	//                                          QFileDialog::ShowDirsOnly
	//                                        | QFileDialog::DontResolveSymlinks);
	//std::string utf8_text = dir.toUtf8().constData();
	//std::cout << utf8_text << std::endl;
}

void MainWindow::add_pose(geometry_msgs::PoseStampedConstPtr pose) {
	std::stringstream ss;
	ss << pose->header.stamp.sec << " " << pose->header.stamp.nsec << " ";
	ss << pose->header.frame_id << " ";
	ss << pose->pose.position.x << " ";
	ss << pose->pose.position.y << " ";
	ss << pose->pose.position.z << " ";
	ss << pose->pose.orientation.x << " ";
	ss << pose->pose.orientation.y << " ";
	ss << pose->pose.orientation.z << " ";
	ss << pose->pose.orientation.w;
	ui->ListPoses->addItem(QString::fromStdString(ss.str()));
}

void add_to_list(geometry_msgs::PoseStampedConstPtr pose) {
	//ROS_INFO("I heard: [%s]", msg->data.c_str());
	std::cout << "hallo" << std::endl;
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "shakey_quickstart_gui");
	ros::NodeHandle nh;

	QApplication a(argc, argv);
	MainWindow w;
	w.show();
	w.shakey_path = GetStdoutFromCommand("rospack find shakey_planning_server");
	w.shakey_path = w.shakey_path.substr(0,
			w.shakey_path.length() - 23);
	std::cout << w.shakey_path << std::endl;

	ros::Subscriber sub = nh.subscribe("move_base_simple/goal", 1000,
			&MainWindow::add_pose, &w);
	ros::WallRate rate(5);
	while (ros::ok()) {
		ros::spinOnce();

		a.processEvents();

		rate.sleep();
	}
	return 0;
}
