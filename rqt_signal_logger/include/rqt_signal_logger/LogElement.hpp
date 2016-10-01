/*
 * DoubleParameter.hpp
 *
 *  Created on: August 2016
 *      Author: Christian Gehring
 */

#pragma once

#include <QWidget>
#include <QStringList>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QApplication>

#include <ros/ros.h>

#include <signal_logger_msgs/GetLoggerElement.h>
#include <signal_logger_msgs/SetLoggerElement.h>

//! This class draws and handles log elements.
class LogElement: public QObject {
  Q_OBJECT
 public:
  LogElement(const std::string& name,
             QWidget* widget,
             QGridLayout* grid,
             ros::ServiceClient* getLogElementClient,
             ros::ServiceClient* setLogElementClient,
             size_t maxParamNameWidth) {

    name_ = name;
    grid_ = grid;
    getLogElementClient_ = getLogElementClient;
    setLogElementClient_ = setLogElementClient;
    int iRow = grid->rowCount();

    labelParamNumber = new QLabel(widget);
    labelParamNumber->setObjectName(QString::fromUtf8("labelParamNumber"));
    labelParamNumber->setText(QString::number(iRow)+QString::fromUtf8(")"));

    std::string lineEditName = std::string{"lineEdit"} + name;
    labelParamName = new QLabel(widget);
    labelParamName->setObjectName(QString::fromStdString(lineEditName));
    labelParamName->setText(QString::fromStdString(name));
    labelParamName->setFixedSize(maxParamNameWidth-10, labelParamName->height());

    std::string spinBoxBufferSizeName = std::string{"spinBoxBufferSize"} + name;
    spinBoxBufferSize = new QDoubleSpinBox(widget);
    spinBoxBufferSize->setObjectName(QString::fromStdString(spinBoxBufferSizeName));
    spinBoxBufferSize->setMinimumSize(QSize(200, 0));
    spinBoxBufferSize->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
    spinBoxBufferSize->setMinimum(1);
    spinBoxBufferSize->setMaximum(1000);
    spinBoxBufferSize->setSingleStep(1);

    std::string spinBoxDividerName = std::string{"spinBoxDivider"} + name;
    spinBoxDivider = new QDoubleSpinBox(widget);
    spinBoxDivider->setObjectName(QString::fromStdString(spinBoxDividerName));
    spinBoxDivider->setMinimumSize(QSize(200, 0));
    spinBoxDivider->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
    spinBoxDivider->setMinimum(1);
    spinBoxDivider->setMaximum(100);
    spinBoxDivider->setSingleStep(1);

    labelBufferIsFull = new QLabel(widget);
    labelBufferIsFull->setObjectName(QString::fromUtf8("labelBufferIsFull"));
    labelBufferIsFull->setText(QString::fromUtf8("False"));

    std::string pushButtonName = std::string{"pushButtonChange"} + name;
    pushButtonChangeParam = new QPushButton(widget);
    pushButtonChangeParam->setObjectName(QString::fromStdString(pushButtonName));
    pushButtonChangeParam->setMaximumSize(QSize(60, 16777215));
    pushButtonChangeParam->setText(QApplication::translate("SignalLogger", "change", 0, QApplication::UnicodeUTF8));

    grid->addWidget(labelParamNumber,      iRow, 0, 1, 1);
    grid->addWidget(labelParamName,        iRow, 1, 1, 1);
    grid->addWidget(spinBoxBufferSize,     iRow, 2, 1, 1);
    grid->addWidget(spinBoxDivider,        iRow, 3, 1, 1);
    grid->addWidget(labelBufferIsFull,     iRow, 4, 1, 1);
    grid->addWidget(pushButtonChangeParam, iRow, 5, 1, 1);

    connect(pushButtonChangeParam, SIGNAL(pressed()), this, SLOT(pushButtonChangeParamPressed()));

    refreshElement();
  }

  virtual ~LogElement() {
    disconnect(pushButtonChangeParam, SIGNAL(pressed()), 0, 0);

    grid_->removeWidget(labelParamNumber);
    grid_->removeWidget(labelParamName);
    grid_->removeWidget(spinBoxBufferSize);
    grid_->removeWidget(spinBoxDivider);
    grid_->removeWidget(labelBufferIsFull);
    grid_->removeWidget(pushButtonChangeParam);

    delete labelParamNumber;
    delete labelParamName;
    delete spinBoxBufferSize;
    delete spinBoxDivider;
    delete labelBufferIsFull;
    delete pushButtonChangeParam;
  };

  public slots:

  void pushButtonChangeParamPressed() {
    ROS_INFO_STREAM("Change logger element " << name_);

    signal_logger_msgs::GetLoggerElementRequest reqGet;
    signal_logger_msgs::GetLoggerElementResponse resGet;

    reqGet.name = name_;
    getLogElementClient_->call(reqGet, resGet);

    signal_logger_msgs::SetLoggerElementRequest req;
    signal_logger_msgs::SetLoggerElementResponse res;

    req.log_element.name = name_;
    req.log_element.action = signal_logger_msgs::LogElement::SAVE_AND_PUBLISH_VAR;
    req.log_element.buffer_size = spinBoxBufferSize->value();
    req.log_element.divider = spinBoxDivider->value();
    req.log_element.is_buffer_looping = true;
    if(!setLogElementClient_->call(req, res)) {
      ROS_INFO_STREAM("Could not change logger element " << name_);
    }

  }

  void refreshElement() {
    signal_logger_msgs::GetLoggerElementRequest req;
    signal_logger_msgs::GetLoggerElementResponse res;

    req.name = name_;
    if (getLogElementClient_->exists()) {
      if (getLogElementClient_->call(req, res) && res.success) {
        spinBoxBufferSize->setValue( res.log_element.buffer_size );
        spinBoxDivider->setValue( res.log_element.divider );
        labelBufferIsFull->setText(QString::fromStdString(res.log_element.is_buffer_full?"True":"False"));
      }
      else {
        ROS_WARN_STREAM("Could not get parameter " << name_);
      }
    }
  }
 protected:
  std::string name_;
  ros::ServiceClient* getLogElementClient_;
  ros::ServiceClient* setLogElementClient_;
  QGridLayout* grid_;
 public:
  QLabel* labelParamNumber;
  QLabel* labelParamName;
  QDoubleSpinBox* spinBoxBufferSize;
  QDoubleSpinBox* spinBoxDivider;
  QLabel* labelBufferIsFull;

  QPushButton* pushButtonChangeParam;
};
