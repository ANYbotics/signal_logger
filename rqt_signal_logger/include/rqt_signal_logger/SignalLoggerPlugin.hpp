/*
 * SignalLoggerPlugin.hpp
 *
 *  Created on: September 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// QT
#include <rqt_gui_cpp/plugin.h>
#include <ui_signal_logger_variables.h>
#include <ui_signal_logger_configure.h>
#include <QWidget>
#include <QTabWidget>
#include <QDoubleSpinBox>
#include <QScrollArea>
#include <QStatusBar>

#include <rqt_signal_logger/LogElement.hpp>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <signal_logger_msgs/LoadLoggerScript.h>
#include <signal_logger_msgs/SetLoggerElement.h>
#include <signal_logger_msgs/GetLoggerElement.h>
#include <signal_logger_msgs/GetLoggerElementNames.h>
#include <std_srvs/Trigger.h>

#include <list>
#include <memory>

class SignalLoggerPlugin : public rqt_gui_cpp::Plugin {
  Q_OBJECT
 public:
  enum class MessageType {
    ERROR,
    WARNING,
    SUCCESS,
    STATUS
  };

 public:
  SignalLoggerPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
  void statusMessage(std::string message, MessageType type, double displaySeconds = 0.0);

 private:
  Ui::SignalLoggerVariables varsUi_;
  Ui::SignalLoggerConfigure configureUi_;
  QTabWidget* tabWidget_;
  QWidget* varsWidget_;
  QWidget* configureWidget_;
  QGridLayout* paramsGrid_;
  QWidget* paramsWidget_;
  QWidget* paramsScrollHelperWidget_;
  QVBoxLayout* paramsScrollLayout_;

  // ROS services
  ros::ServiceClient getLoggerElementNamesClient_;
  ros::ServiceClient getLoggerElementClient_;
  ros::ServiceClient setLoggerElementClient_;
  ros::ServiceClient startLoggerClient_;
  ros::ServiceClient stopLoggerClient_;
  ros::ServiceClient saveLoggerDataClient_;
  ros::ServiceClient loadLoggerScriptClient_;

  std::vector<std::shared_ptr<LogElement>> logElements_;
  std::vector<std::string> logElementNames_;

  protected slots:
    void refreshAll();
    void changeAll();
    void drawParamList();
    void startLogger();
    void stopLogger();
    void saveLoggerData();
    void selectYamlFile();
    void loadYamlFile();
    void saveYamlFile();

  signals:
    void parametersChanged();

};


