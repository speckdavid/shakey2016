#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <geometry_msgs/PoseStamped.h>
#include <stdlib.h>

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
    
private slots:
    void on_pushButton_clicked();
    void on_mapUpdateButton_clicked();
    void on_pushButton_createFolder_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
