#include <QDialog>
#include <QString>
#include <QStatusBar>
#include "ui_geometryPosesCreatorGui.h"
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "shakey_utils/geometryPoses.h"
#include <vector>

class GeometryPosesCreatorGui : public QDialog, protected Ui::mainDia
{
   Q_OBJECT

   public:
      GeometryPosesCreatorGui(const std::string & ff, const std::string & tf);
      ~GeometryPosesCreatorGui();

      void showMessage(QString msg);

   protected Q_SLOTS:
      void on_savePoseBtn_clicked();
      void on_writeFileBtn_clicked();

   protected:
      ros::NodeHandle nh;
      tf::TransformListener tf_listener;

      std::string fixed_frame;
      std::string target_frame;

      std::vector<GeometryPoses::NamedPose> poses;

      QStatusBar* _statusBar;
};
