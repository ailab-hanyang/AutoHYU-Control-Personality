#ifndef MODE_SWITCH_INTERFACE_H
#define MODE_SWITCH_INTERFACE_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#include <boost/thread.hpp>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#  include <QtWidgets>
#else
#  include <QtGui>
#endif
#endif

#include <autohyu_msgs/ADModeInput.h>

namespace jsk_rviz_plugins
{

  class ModeSwitchInterface: public rviz::Panel
  {
  Q_OBJECT
  public:
    ModeSwitchInterface(QWidget* parent = 0);

    virtual void onInitialize();
    virtual void load(const rviz::Config& config);
    virtual void save(rviz::Config config) const;

  protected Q_SLOTS:
    void updateModes();
  protected:
    QVBoxLayout* layout_;
    QCheckBox* ready_check_;
    QCheckBox* lateral_check_;
    QCheckBox* longitudinal_check_;
    QCheckBox* manual_check_;
    ros::Publisher mode_pub_;
  };

}  // namespace jsk_rviz_plugins


#endif  // MODE_SWITCH_INTERFACE_H
