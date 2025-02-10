#include "ad_mode_interface.h"
#include <boost/thread.hpp>
#include <rviz/config.h>
#include <ros/package.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSignalMapper>

#include <autohyu_msgs/ADModeInput.h>

namespace jsk_rviz_plugins
{

  ModeSwitchInterface::ModeSwitchInterface(QWidget* parent)
    : rviz::Panel(parent)
  {
    layout_ = new QVBoxLayout;

    ready_check_ = new QCheckBox("Ready");
    layout_->addWidget(ready_check_);

    lateral_check_ = new QCheckBox("Lateral Auto");
    layout_->addWidget(lateral_check_);
    lateral_check_->setEnabled(false);  // Disabled initially

    longitudinal_check_ = new QCheckBox("Longitudinal Auto");
    layout_->addWidget(longitudinal_check_);
    longitudinal_check_->setEnabled(false);  // Disabled initially

    manual_check_ = new QCheckBox("Manual");
    layout_->addWidget(manual_check_);

    connect(ready_check_, SIGNAL(stateChanged(int)), this, SLOT(updateModes()));
    connect(lateral_check_, SIGNAL(stateChanged(int)), this, SLOT(updateModes()));
    connect(longitudinal_check_, SIGNAL(stateChanged(int)), this, SLOT(updateModes()));
    connect(manual_check_, SIGNAL(stateChanged(int)), this, SLOT(updateModes()));

    setLayout(layout_);
  }

  void ModeSwitchInterface::onInitialize()
  {
    ros::NodeHandle nh;
    mode_pub_ = nh.advertise<autohyu_msgs::ADModeInput>("/hmi/ad_mode_input", 10);
  }

  void ModeSwitchInterface::updateModes()
  {
    // Manual mode overrides other modes
    if (manual_check_->isChecked()) {
      ready_check_->setChecked(false);
      lateral_check_->setChecked(false);
      longitudinal_check_->setChecked(false);
      lateral_check_->setEnabled(false);
      longitudinal_check_->setEnabled(false);
    } else {
      // If Ready is checked, enable Lateral and Longitudinal modes
      if (ready_check_->isChecked()) {
        lateral_check_->setEnabled(true);
        longitudinal_check_->setEnabled(true);
      } else {
        lateral_check_->setEnabled(false);
        lateral_check_->setChecked(false);
        longitudinal_check_->setEnabled(false);
        longitudinal_check_->setChecked(false);
      }
    }

    // Publish the message
    autohyu_msgs::ADModeInput msg;
    msg.ready_mode = ready_check_->isChecked();
    msg.lateral_mode = lateral_check_->isChecked();
    msg.longitudinal_mode = longitudinal_check_->isChecked();
    msg.manual_mode = manual_check_->isChecked();

    mode_pub_.publish(msg);
  }

  void ModeSwitchInterface::save(rviz::Config config) const
  {
    rviz::Panel::save(config);
  }

  void ModeSwitchInterface::load(const rviz::Config& config)
  {
    rviz::Panel::load(config);
  }

}  // namespace jsk_rviz_plugins


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::ModeSwitchInterface, rviz::Panel)
