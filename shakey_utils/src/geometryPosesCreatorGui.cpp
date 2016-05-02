#include "geometryPosesCreatorGui.h"
#include <QString>
#include <QFileDialog>
#include <QTextDocument>
#include <sstream>
#include <iomanip>
#include <QMessageBox>
#include <fstream>

GeometryPosesCreatorGui::GeometryPosesCreatorGui(const std::string & ff, const std::string & tf) : fixed_frame(ff), target_frame(tf)
{
   setupUi(this);

   _statusBar = new QStatusBar(this);
   this->layout()->addWidget(_statusBar);
}

GeometryPosesCreatorGui::~GeometryPosesCreatorGui()
{
}


void GeometryPosesCreatorGui::showMessage(QString msg)
{
   _statusBar->showMessage(msg, 2000);
}

void GeometryPosesCreatorGui::on_savePoseBtn_clicked()
{
    tf::StampedTransform transform;
    try{
        tf_listener.lookupTransform(fixed_frame, target_frame,
                ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
        QMessageBox::critical(this, "Error in getting transform", QString(ex.what()));
        return;
    }
    ROS_ASSERT(fixed_frame == transform.frame_id_);

    GeometryPoses::NamedPose np;
    std::stringstream ss;
    ss << "pose" << std::fixed << std::setfill('0') << std::setw(3) << poses.size();
    np.first = ss.str();
    np.second.header.stamp = transform.stamp_;
    np.second.header.frame_id = transform.frame_id_;
    np.second.pose.position.x = transform.getOrigin().x();
    np.second.pose.position.y = transform.getOrigin().y();
    np.second.pose.position.z = transform.getOrigin().z();
    if(writeYawOnlyCk->isChecked()) {
        ROS_INFO("Using yaw only");
        double yaw = tf::getYaw(transform.getRotation());
        tf::Quaternion quat = tf::createQuaternionFromYaw(yaw);
        transform.setRotation(quat);
    }
    np.second.pose.orientation.x = transform.getRotation().x();
    np.second.pose.orientation.y = transform.getRotation().y();
    np.second.pose.orientation.z = transform.getRotation().z();
    np.second.pose.orientation.w = transform.getRotation().w();

    poses.push_back(np);
    QMessageBox::information(this, "Pose save", "Saved pose");
}

void GeometryPosesCreatorGui::on_writeFileBtn_clicked()
{
    QString fn = QFileDialog::getSaveFileName(this, "Select poses file");
    if(fn.length() < 1)
        return;

    std::ofstream of(qPrintable(fn));
    if(!of.good())
        return;

    for(std::vector<GeometryPoses::NamedPose>::iterator it = poses.begin(); it != poses.end(); it++) {
        GeometryPoses::NamedPose & np = *it;
        of << np.first << " ";
        of << np.second.header.stamp.sec << " " << np.second.header.stamp.nsec << " ";
        of << np.second.header.frame_id << " ";
        of << np.second.pose.position.x << " ";
        of << np.second.pose.position.y << " ";
        of << np.second.pose.position.z << " ";
        of << np.second.pose.orientation.x << " ";
        of << np.second.pose.orientation.y << " ";
        of << np.second.pose.orientation.z << " ";
        of << np.second.pose.orientation.w << std::endl;
    }

    if(of.good()) {
        QMessageBox::information(this, "Write file", "File written");
    }

    of.close();
}
