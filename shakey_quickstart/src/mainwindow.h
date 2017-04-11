#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <geometry_msgs/PoseStamped.h>
#include <stdlib.h>
#include <ros/ros.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void add_pose(geometry_msgs::PoseStampedConstPtr pose);
    std::string shakey_path;
    std::string project_folder;
    bool mapping;
    bool cropFlipper;
    int num_searchLocs;
    int num_goalLocs;
    double num_doorways;
    ros::Subscriber sub;
    ros::NodeHandle nh;
    std::string sub_topic;
    
private slots:
    void on_delPoseButton_clicked();
    void on_mapUpdateButton_clicked();
    void on_pushButton_createFolder_clicked();
    void on_pushButton_SaveLocs_clicked();
    void on_pushButton_Done_clicked();
    void on_comboBox_Topic_currentIndexChanged(const QString &arg1);
    void updateMap();
    void updateTopics();

private:
    Ui::MainWindow *ui;
    QTimer *myTimer;
};

#endif // MAINWINDOW_H
