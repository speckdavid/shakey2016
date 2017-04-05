#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QModelIndex>
#include <QtGui/QApplication>
#include <QFileDialog>
#include <QDir>
#include <QTimer>
#include <QFileInfo>
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
	mapping = false;
	cropFlipper = true;
	myTimer = new QTimer(this);
	myTimer->setInterval(5000);
	myTimer->start();
	connect(myTimer, SIGNAL(timeout()), this, SLOT(updateMap()));
	shakey_path = GetStdoutFromCommand("rospack find shakey_planning_server");
	shakey_path = shakey_path.substr(0, shakey_path.length() - 23);
	project_folder = "";
	std::cout << shakey_path << std::endl;
	num_searchLocs = 0;
	num_doorways = 0;
}

MainWindow::~MainWindow() {
	delete ui;
}

void MainWindow::updateMap() {
	if (mapping) {
		std::string cmd = "cd " + this->shakey_path + "/" + this->project_folder;
		if (cropFlipper) {
			QFileInfo check_file(QString::fromStdString(cmd.substr(3, cmd.length()) + "/full.yaml"));
			if (check_file.exists() && check_file.isFile()) {
				system((cmd + "&& rosrun map_server crop_map full.yaml map.yaml &").c_str());
			}
			QString filename = QString::fromStdString(this->shakey_path + "/" + this->project_folder + "/map.pgm");
			QImage image(filename);
			ui->currentMap->setPixmap(
					QPixmap::fromImage(image).scaled(ui->currentMap->size(),	Qt::KeepAspectRatio));
			cropFlipper = false;
		} else {
			system((cmd + "&& rosrun map_server map_saver -f full &").c_str());
			cropFlipper = true;
		}
	}
}

void MainWindow::on_pushButton_createFolder_clicked() {
	QString path = QString::fromStdString(this->shakey_path);
	path.append(ui->worldName->text());
	if (path.length() < 0) {
		ui->worldName->setStyleSheet(
						"QLineEdit { background: rgb(255, 0, 0); selection-background-color: rgb(255, 0, 0); }");
	} else {
		if (!QDir(path).exists())
			QDir().mkdir(path);
		ui->worldName->setStyleSheet(
						"QLineEdit { background: rgb(255, 255, 255); selection-background-color: rgb(255, 255, 255); }");

	}
	this->project_folder = ui->worldName->text().toUtf8().constData();;
	std::cout << "created folder: " << path.toUtf8().constData() << std::endl;

}

void MainWindow::on_pushButton_clicked() {
	if (!ui->ListPoses->selectedItems().isEmpty()) {
		QList<QListWidgetItem*> items = ui->ListPoses->selectedItems();
		for (int i = 0; i < items.size(); i++)
			delete items.at(i);
		ui->ListPoses->clearSelection();
		ui->ListPoses->clearFocus();

	}
}

void MainWindow::on_mapUpdateButton_clicked() {
	if (this->project_folder.length() == 0) {
		ui->worldName->setStyleSheet(
				"QLineEdit { background: rgb(255, 0, 0); selection-background-color: rgb(255, 0, 0); }");
		return;
	}
	QString cur_str = ui->mapUpdateButton->text();
	if (!this->mapping) {
		this->mapping = true;
		this->cropFlipper = false;
		ui->mapUpdateButton->setText("Stop Mapping");
		ui->currentMap->setText("Processing ...");
		system("roslaunch shakey_executable shakey_mapping.launch &");
	} else {
		this->mapping = false;
		system("rosnode kill /slam_gmapping &");
		ui->mapUpdateButton->setText("Start Mapping");
		QFileInfo check_file(QString::fromStdString(this->shakey_path + "/" + this->project_folder + "/full.yaml"));
		if (check_file.exists() && check_file.isFile()) {
			ui->lineEdit_XMap->setText(QChar(0x2713));
		}

	}

	//system("cd /home/david/Pictures/ && rosrun map_server map_saver");
	//QString filename = "/home/david/Pictures/double_push.png";
	//QImage image(filename);
	//ui->currentMap->setPixmap(
	//		QPixmap::fromImage(image).scaled(100, 100, Qt::KeepAspectRatio));
	//QString dir = QFileDialog::getExistingDirectory(this, tr("Choose a world directory"),
	//                                            "/home",
	//                                          QFileDialog::ShowDirsOnly
	//                                        | QFileDialog::DontResolveSymlinks);
	//std::string utf8_text = dir.toUtf8().constData();
	//std::cout << utf8_text << std::endl;
}

void MainWindow::add_pose(geometry_msgs::PoseStampedConstPtr pose) {
	std::stringstream ss;
	QString loc_type = ui->comboBox_LocType->currentText();
	QString room = ui->comboBox_Room->currentText();

	// Is Search Locations
	if (!QString::compare(loc_type, ui->comboBox_LocType->itemText(0), Qt::CaseInsensitive)) {
		ss << "search_location_loc" << this->num_searchLocs << "_room" <<
				QString(room.at(room.length() -1)).toUtf8().constData() << " ";
		this->num_searchLocs++;

	// Is Doorway
	} else if (!QString::compare(loc_type, ui->comboBox_LocType->itemText(1), Qt::CaseInsensitive)) {
		ss << "doorway_" << static_cast<int>(this->num_doorways) << "_room" <<
				QString(room.at(room.length() -1)).toUtf8().constData() << " ";
		this->num_doorways += 0.5;
	} else {
		cout << "ComboBox (wrong assignments)!";
		return;
	}
	ss << pose->header.stamp.sec << " " << pose->header.stamp.nsec << " ";
	ss << pose->header.frame_id << " ";
	ss << pose->pose.position.x << " ";
	ss << pose->pose.position.y << " ";
	ss << pose->pose.position.z << " ";
	ss << pose->pose.orientation.x << " ";
	ss << pose->pose.orientation.y << " ";
	ss << pose->pose.orientation.z << " ";
	ss << pose->pose.orientation.w;
	QListWidgetItem *cur = new QListWidgetItem();
	cur->setText(QString::fromStdString(ss.str()));
	cur->setFlags (cur->flags () | Qt::ItemIsEditable);
	ui->ListPoses->addItem(cur);
}
