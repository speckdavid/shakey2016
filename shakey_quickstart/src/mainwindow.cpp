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
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <fstream>

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

void replaceInFile(string in_file, string out_file, string strReplace, string strNew) {
	ifstream in(in_file.c_str());
	ofstream out(out_file.c_str());
	if (!in || !out)
	{
	    cerr << "Could not open file " << in_file << " or " << out_file << "\n";
	    return;
	}
	string line;
	size_t len = strReplace.length();
	while (getline(in, line))
	{
	    while (true)
	    {
	        size_t pos = line.find(strReplace);
	        if (pos != string::npos)
	            line.replace(pos, len, strNew);
	        else
	            break;
	    }
	    out << line << '\n';
	}
}

MainWindow::MainWindow(QWidget *parent) :
		QMainWindow(parent), ui(new Ui::MainWindow) {
	ui->setupUi(this);
	mapping = false;
	cropFlipper = false;
	myTimer = new QTimer(this);
	myTimer->setInterval(2000);
	myTimer->start();
	connect(myTimer, SIGNAL(timeout()), this, SLOT(updateMap()));
	shakey_path = GetStdoutFromCommand("rospack find shakey_planning_server");
	shakey_path = shakey_path.substr(0, shakey_path.length() - 23);
	project_folder = "";
	std::cout << shakey_path << std::endl;
	num_searchLocs = 0;
	num_goalLocs = 0;
	num_doorways = 0;
	updateTopics();
	vis_pub = nh.advertise<visualization_msgs::MarkerArray>("poses_marker", 0);
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
	if (path.length() < 0 || ui->worldName->text().contains("/") == std::string::npos) {
		ui->worldName->setStyleSheet(
						"QLineEdit { background: rgb(255, 0, 0); selection-background-color: rgb(255, 0, 0); }");
	} else {

		this->project_folder = ui->worldName->text().toUtf8().constData();;
		// Create new world package
		if (!QDir(path).exists()) {
			//QDir().mkdir(path);
			std::string cmd = "cd "  + this->shakey_path +
					" && catkin_create_pkg " + ui->worldName->text().toUtf8().constData();
			system(cmd.c_str());
			std::cout << "created folder: " << path.toUtf8().constData() << std::endl;
		} else if (QFileInfo(path + "/map.yaml").exists()) {
			QString filename = path + "/map.pgm";
			QImage image(filename);
			ui->currentMap->setPixmap(
					QPixmap::fromImage(image).scaled(ui->currentMap->size(),	Qt::KeepAspectRatio));
			ui->lineEdit_XMap->setText(QChar(0x2713));
			if (!QString::compare(QString(QChar(0x2713)), ui->lineEdit_XLocations->text(), Qt::CaseInsensitive)) {
				ui->pushButton_Done->setEnabled(true);
			}

		}

		// Create evaluation folder
		if (!QDir(path + "/eval").exists()) {
			QDir().mkdir(path + "/eval");
		}

		// Create sceenrun folder
		if (!QDir(path + "/screenrun").exists()) {
			QDir().mkdir(path + "/eval");
			QString screen_cp = QString::fromStdString(this->shakey_path +
					"shakey_quickstart/screenrun_template");
			system("mkdir " + (path + "/screenrun").toUtf8());
			system("cp -R " + (screen_cp + "/config.yaml").toUtf8() +
					" " + path.toUtf8() + "/screenrun/config.yaml");
			system("cp -R " + (screen_cp + "/screenrun.lau").toUtf8() +
					" " + path.toUtf8() + "/screenrun/screenrun.launch");

			// Replace map_specific part in launch file
			std::string old_file_name = (screen_cp + "/screenrun.lau").toUtf8().constData();
			std::string new_file_name = (path + "/screenrun/screenrun.launch").toUtf8().constData();
			replaceInFile(old_file_name, new_file_name,
					"dummy", ui->worldName->text().toUtf8().constData());

			// Replace map_specific part in config file
			old_file_name = (screen_cp + "/config.yaml").toUtf8().constData();
			new_file_name = (path + "/screenrun/config.yaml").toUtf8().constData();
			replaceInFile(old_file_name, new_file_name,
					"dummy", ui->worldName->text().toUtf8().constData());
		}

		// Create config files in shakey_planning_server
		QString config_path = QString::fromStdString(this->shakey_path + "shakey_planning_server/config/planning");
		if (QDir(config_path).exists() && !QDir(config_path + "/" + ui->worldName->text()).exists()) {
			QString config_cp = QString::fromStdString(this->shakey_path +
					"shakey_quickstart/config_template");
			config_path.append("/" + ui->worldName->text());
			// Copy config files in shakey_planning_server
			system("cp -R " + config_cp.toUtf8() +
					" " + config_path.toUtf8());
			FILE * planer_param = fopen(config_path.toUtf8() + "/shakey_planner_params.yaml", "a");
			fputs("eval/eval_current_dir: " + path.toUtf8() + "/eval\n", planer_param);
			fputs( "eval/eval_dir: " + path.toUtf8() + "/eval", planer_param);
			fclose(planer_param);
			std::cout << config_path.toUtf8().constData() << "/shakey_planner_params.yaml" << std::endl;
		}

		// Read current locations
		QFile inputFile(config_path + "/" + ui->worldName->text() + "/shakey_locations.dat");
		cout << inputFile.fileName().toUtf8().constData() << endl;
		ui->ListPoses->clear();
		if (inputFile.open(QIODevice::ReadOnly)) {

			cout << "Read existing location data..." <<endl;
		    QTextStream in(&inputFile);
		    while (!in.atEnd())
		    {
		    	QString line = in.readLine();
		    	if (line.length() <= 0)
		    		continue;
		    	QListWidgetItem *cur = new QListWidgetItem();
		    	cur->setText(line);
		    	cur->setFlags (cur->flags () | Qt::ItemIsEditable);
		    	ui->ListPoses->addItem(cur);
		    	QStringList loc_data = line.split("_");
		    	if (!QString::compare(loc_data[0], "search", Qt::CaseInsensitive)) {
		    		int cur_num = loc_data[2][3].digitValue();
		    		this->num_searchLocs = cur_num >= this->num_searchLocs ? cur_num + 1 : this->num_searchLocs;
		    	}
		    	else if (!QString::compare(loc_data[0], "object", Qt::CaseInsensitive)) {
		    		int cur_num = loc_data[2][3].digitValue();
		    		this->num_goalLocs = cur_num >= this->num_goalLocs ? cur_num + 1 : this->num_goalLocs;
		    	}
		    	else if (!QString::compare(loc_data[0], "doorway", Qt::CaseInsensitive)) {
		    		int cur_num = loc_data[1][0].digitValue();
		    		this->num_doorways = cur_num >= this->num_doorways ? cur_num + 1 : this->num_doorways;
		    	}
		    }
		    inputFile.close();
			ui->lineEdit_XLocations->setText(QChar(0x2713));
			if (!QString::compare(QString(QChar(0x2713)), ui->lineEdit_XMap->text(), Qt::CaseInsensitive)) {
				ui->pushButton_Done->setEnabled(true);
			}
		    this->updateMarker();
		}
		ui->worldName->setStyleSheet(
						"QLineEdit { background: rgb(255, 255, 255); selection-background-color: rgb(255, 255, 255); }");

	}

}

void MainWindow::on_delPoseButton_clicked() {
	if (!ui->ListPoses->selectedItems().isEmpty()) {
		QList<QListWidgetItem*> items = ui->ListPoses->selectedItems();
		for (int i = 0; i < items.size(); i++)
			delete items.at(i);
		ui->ListPoses->clearSelection();
		ui->ListPoses->clearFocus();
	}
	this->updateMarker();
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

		// Set Checkmark and check if final button should be enabled
		if (check_file.exists() && check_file.isFile()) {
			ui->lineEdit_XMap->setText(QChar(0x2713));
			if (!QString::compare(QString(QChar(0x2713)), ui->lineEdit_XLocations->text(), Qt::CaseInsensitive)) {
				ui->pushButton_Done->setEnabled(true);
			}
		}

	}
}

void MainWindow::on_pushButton_SaveLocs_clicked() {
	if (this->project_folder.length() == 0) {
		ui->worldName->setStyleSheet(
				"QLineEdit { background: rgb(255, 0, 0); selection-background-color: rgb(255, 0, 0); }");
		return;
	}
	QString loc_path = QString::fromStdString(this->shakey_path +
			"shakey_planning_server/config/planning/" +
			this->project_folder);
	if (QDir(loc_path).exists()) {
		FILE * loc_file = fopen((loc_path + "/shakey_locations.dat").toUtf8(), "w");
		for (int i = 0; i < ui->ListPoses->count(); i ++) {
			fputs(ui->ListPoses->item(i)->text().toUtf8() + "\n", loc_file);
		}
		fclose(loc_file);
		ui->lineEdit_XLocations->setText(QChar(0x2713));
		if (!QString::compare(QString(QChar(0x2713)), ui->lineEdit_XMap->text(), Qt::CaseInsensitive)) {
			ui->pushButton_Done->setEnabled(true);
		}
		std::cout << "saved poses: " << loc_path.toUtf8().constData() << std::endl;
	}
}

void MainWindow::on_pushButton_Done_clicked() {
	this->setVisible(false);
}


void MainWindow::on_comboBox_Topic_currentIndexChanged(const QString &arg1) {
	if (ui->comboBox_Topic->count() == 0) return;
	if (sub_topic.length() == 0) return;
	if (QString::compare(arg1,QString::fromStdString(sub_topic),Qt::CaseInsensitive) != 0) {
		sub.shutdown();
		sub = nh.subscribe(arg1.toUtf8().constData(), 1000,
				&MainWindow::add_pose, this);
		sub_topic = arg1.toUtf8().constData();
		cout << "Subscribtion to: " << sub_topic << endl;
	}
}

void MainWindow::updateTopics() {
	std::stringstream topic_choices(GetStdoutFromCommand("rostopic find geometry_msgs/PoseStamped"));
	std::string line;
	sub_topic = "/move_base_simple/goal";
	if (topic_choices != NULL)
	{
		bool empty_topics = true;
		while(std::getline(topic_choices,line,'\n')){
			if (empty_topics && line.length() > 0) {
				ui->comboBox_Topic->clear();
				empty_topics = false;
			}
			QString cur_topic = QString::fromStdString(line);
			if (line.size() > 0)
				ui->comboBox_Topic->addItem(QString::fromStdString(line));
		}
		sub_topic = "dummy";
		on_comboBox_Topic_currentIndexChanged(QString::fromStdString("/move_base_simple/goal"));
		ui->comboBox_Topic->setCurrentIndex(ui->comboBox_Topic->findText(QString::fromStdString(sub_topic)));
	}
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

	// Is Object Goal Location
	} else if (!QString::compare(loc_type, ui->comboBox_LocType->itemText(1), Qt::CaseInsensitive)) {
		ss << "object_location_loc" << this->num_goalLocs << "_room" <<
				QString(room.at(room.length() -1)).toUtf8().constData() << " ";
		this->num_goalLocs++;

	// Is Doorway
	} else if (!QString::compare(loc_type, ui->comboBox_LocType->itemText(2), Qt::CaseInsensitive)) {
		ss << "doorway_" << static_cast<int>(this->num_doorways) << "_room" <<
				QString(room.at(room.length() -1)).toUtf8().constData() << " ";
		this->num_doorways += 0.5;
	} else {
		cout << "ComboBox (wrong assignments)!";
		return;
	}
	ss << pose->header.stamp.sec << " " << pose->header.stamp.nsec << " ";
	ss << "/" << pose->header.frame_id << " ";
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
	this->updateMarker();
}

void MainWindow::on_ListPoses_itemChanged(QListWidgetItem* item) {
	this->updateMarker();
}

void MainWindow::updateMarker() {
	// Delete all markers
	for (int i = 0; i < poses_marker.markers.size(); i++) {
		poses_marker.markers[i].action = visualization_msgs::Marker::DELETE;
	}
	vis_pub.publish(poses_marker);
	std::map<int, std::vector<visualization_msgs::Marker> > doorways;
	// Create new markers
	poses_marker.markers.resize(ui->ListPoses->count());
	for (int i = 0; i < ui->ListPoses->count(); i++) {
		QString cur_loc = ui->ListPoses->item(i)->text();
		QStringList loc_name = cur_loc.split("_");
		QStringList loc_data = cur_loc.split(" ");
		visualization_msgs::Marker marker;
		marker.id = i;
		marker.header.stamp.sec = loc_data[1].toInt();
		marker.header.stamp.nsec = loc_data[2].toInt();
		marker.header.frame_id = loc_data[3].toUtf8().constData();
		marker.pose.position.x = loc_data[4].toDouble();
		marker.pose.position.y = loc_data[5].toDouble();
		marker.pose.position.z = loc_data[6].toDouble();
		marker.pose.orientation.x = loc_data[7].toDouble();
		marker.pose.orientation.y = loc_data[8].toDouble();
		marker.pose.orientation.z = loc_data[9].toDouble();
		marker.pose.orientation.w = loc_data[10].toDouble();
		marker.color.a = 1;
		marker.scale.x = 1;
		marker.scale.y = 0.15;
		marker.scale.z = 0.15;
		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::ADD;
		if (!QString::compare(loc_name[0], "search", Qt::CaseInsensitive)) {
			marker.color.b = 1;
		} else if (!QString::compare(loc_name[0], "object",
				Qt::CaseInsensitive)) {
			marker.color.g = 1;
			marker.scale.x = marker.scale.y = 0.5;
			marker.type = visualization_msgs::Marker::CUBE;
		} else if (!QString::compare(loc_name[0], "doorway",
				Qt::CaseInsensitive)) {
			marker.color.r = 1;
			// Save doorway info
			int doorway_num = loc_name[1][loc_name[1].size() -1].toLatin1();
			if (!doorways.count(doorway_num)) {
				doorways[doorway_num] = std::vector<visualization_msgs::Marker>();
			}
			doorways[doorway_num].push_back(marker);
		}
		poses_marker.markers[i] = marker;
	}

	// Add doorway lines (line markers)
	int arrow_marker_num = poses_marker.markers.size();
	poses_marker.markers.resize(arrow_marker_num + doorways.size());
	std::map<int, std::vector<visualization_msgs::Marker> >::iterator it;
	int cur_id = arrow_marker_num;
	for ( it = doorways.begin(); it != doorways.end(); it++ )
	{
		if (it->second.size() != 2) {
			continue;
		}
		visualization_msgs::Marker marker;
		marker.id = cur_id;
		marker.header = it->second.at(0).header;
		marker.color.r = marker.color.a = 1;
		marker.type = visualization_msgs::Marker::LINE_STRIP;
		marker.action = visualization_msgs::Marker::ADD;
		marker.scale.x = 0.05;
		for (int i = 0; i < it->second.size(); i++) {
			marker.points.push_back(it->second.at(i).pose.position);
		}
		poses_marker.markers[cur_id] = marker;
		cur_id++;
	}



}
