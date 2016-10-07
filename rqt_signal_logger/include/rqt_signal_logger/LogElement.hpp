/*
 * LogElement.hpp
 *
 *  Created on: October 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// rqt_signal_logger custom widgets
#include "rqt_signal_logger/BufferIndicator.hpp"

// msgs
#include "signal_logger_msgs/GetLoggerElement.h"
#include "signal_logger_msgs/SetLoggerElement.h"

// ros
#include <ros/ros.h>

// QT
#include <QApplication>
#include <QWidget>
#include <QGridLayout>
#include <QLabel>
#include <QCheckBox>
#include <QSpinBox>
#include <QComboBox>
#include <QPushButton>


//! This class draws and handles log elements.
class LogElement: public QObject {
  Q_OBJECT

 public:
  //! Enum mapping defining possible log actions
  enum class LogAction : unsigned int {
    SAVE_AND_PUBLISH = signal_logger_msgs::LogElement::SAVE_AND_PUBLISH_VAR,
    SAVE = signal_logger_msgs::LogElement::SAVE_VAR,
    PUBLISH = signal_logger_msgs::LogElement::PUBLISH_VAR
  };

 public:
  LogElement(const std::string& name,
             QWidget* widget,
             QGridLayout* grid,
             ros::ServiceClient* getLogElementClient,
             ros::ServiceClient* setLogElementClient,
             size_t maxParamNameWidth)
 {
    name_ = name;
    grid_ = grid;
    getLogElementClient_= getLogElementClient;
    setLogElementClient_ = setLogElementClient;
    // Get current row number
    int iRow = grid->rowCount();

    //! Add number label
    labelParamNumber = new QLabel(widget);
    labelParamNumber->setObjectName(QString::fromStdString(std::string{"labelParamNumber"} + name));
    labelParamNumber->setText(QString::number(iRow)+QString::fromUtf8(")"));

    //! Add is logging checkbox
    checkBoxIsLogging = new QCheckBox(widget);
    checkBoxIsLogging->setObjectName(QString::fromStdString(std::string{"checkBoxIsLogging"} + name));
    checkBoxIsLogging->setText("log?");
    checkBoxIsLogging->setLayoutDirection(Qt::LayoutDirection::RightToLeft);
    checkBoxIsLogging->setTristate(false);
    checkBoxIsLogging->setCheckState(Qt::CheckState::Unchecked);

    //! Add name label
    labelParamName = new QLabel(widget);
    labelParamName->setObjectName(QString::fromStdString(std::string{"labelParamName"} + name));
    labelParamName->setText(QString::fromStdString(name));
    labelParamName->setFixedSize(maxParamNameWidth-10, labelParamName->height());

    //! Add log type combobox
    comboBoxLogType = new QComboBox(widget);
    comboBoxLogType->setObjectName(QString::fromStdString(std::string{"comboBoxLogType"} + name));
    comboBoxLogType->insertItem(static_cast<int>(LogAction::SAVE_AND_PUBLISH), "Save and Publish");
    comboBoxLogType->insertItem(static_cast<int>(LogAction::SAVE),"Save");
    comboBoxLogType->insertItem(static_cast<int>(LogAction::PUBLISH),"Publish");

    //! Add divider label
    labelDivider = new QLabel(widget);
    labelDivider->setObjectName(QString::fromStdString(std::string{"labelDivider"} + name));
    labelDivider->setText(QString::fromStdString(std::string{"Divider:"}));
    labelDivider->setFixedWidth(labelDivider->fontMetrics().width(labelDivider->text())+10);
    labelDivider->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

    //! Add divider spinbox
    spinBoxDivider = new QSpinBox(widget);
    spinBoxDivider->setObjectName(QString::fromStdString(std::string{"spinBoxDivider"} + name));
    spinBoxDivider->setMinimumSize(QSize(50, 0));
    spinBoxDivider->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
    spinBoxDivider->setMinimum(1);
    spinBoxDivider->setMaximum(100);
    spinBoxDivider->setSingleStep(1);

    //! Add buffer label
    labelBuffer = new QLabel(widget);
    labelBuffer->setObjectName(QString::fromStdString(std::string{"labelBuffer"} + name));
    labelBuffer->setText(QString::fromStdString("Buffer size:"));
    labelBuffer->setFixedWidth(labelBuffer->fontMetrics().width(labelBuffer->text())+10);
    labelBuffer->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

    //! Add buffer size spinbox
    spinBoxBufferSize = new QSpinBox(widget);
    spinBoxBufferSize->setObjectName(QString::fromStdString(std::string{"spinBoxBufferSize"} + name));
    spinBoxBufferSize->setMinimumSize(QSize(50, 0));
    spinBoxBufferSize->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
    spinBoxBufferSize->setMinimum(1);
    spinBoxBufferSize->setMaximum(1000);
    spinBoxBufferSize->setSingleStep(1);

    //! Add buffer looping combobox
    comboBoxIsLooping = new QComboBox(widget);
    comboBoxIsLooping->setObjectName(QString::fromStdString(std::string{"comboBoxIsLooping"} + name));
    comboBoxIsLooping->addItem("Not Looping");
    comboBoxIsLooping->addItem("Looping");

    // Buffer Indicator
    bufferInd_ = new BufferIndicator(widget);
    bufferInd_->setObjectName(QString::fromStdString(std::string{"indicatorBuffer"} + name));
//    connect(bufferInd_, SIGNAL(refresh()), this, SLOT(refreshElement()));

    //! Change button
    pushButtonChangeParam = new QPushButton(widget);
    pushButtonChangeParam->setObjectName(QString::fromStdString(std::string{"pushButtonChange"} + name));
    pushButtonChangeParam->setText(QString::fromUtf8("change"));

    //! Change button
    pushButtonRefreshParam = new QPushButton(widget);
    pushButtonRefreshParam->setObjectName(QString::fromStdString(std::string{"pushButtonRefresh"} + name));
    pushButtonRefreshParam->setText(QString::fromUtf8("refresh"));

    grid->addWidget(labelParamNumber,      iRow, 0, 1, 1);
    grid->addWidget(checkBoxIsLogging,     iRow, 1, 1, 1);
    grid->addWidget(labelParamName,        iRow, 2, 1, 1);
    grid->addWidget(comboBoxLogType,       iRow, 3, 1, 1);
    grid->addWidget(labelDivider,          iRow, 4, 1, 1);
    grid->addWidget(spinBoxDivider,        iRow, 5, 1, 1);
    grid->addWidget(labelBuffer,           iRow, 6, 1, 1);
    grid->addWidget(spinBoxBufferSize,     iRow, 7, 1, 1);
    grid->addWidget(comboBoxIsLooping,     iRow, 8, 1, 1);
    grid->addWidget(bufferInd_,            iRow, 9, 1, 1);
    grid->addWidget(pushButtonChangeParam, iRow, 10, 1, 1);
    grid->addWidget(pushButtonRefreshParam,iRow, 11, 1, 1);

    connect(pushButtonChangeParam, SIGNAL(pressed()), this, SLOT(pushButtonChangeParamPressed()));
    connect(pushButtonRefreshParam, SIGNAL(pressed()), this, SLOT(refreshElement()));

    refreshElement();
 }

  virtual ~LogElement() {
    disconnect(pushButtonChangeParam, SIGNAL(pressed()), 0, 0);
    disconnect(pushButtonRefreshParam, SIGNAL(pressed()), 0, 0);

    grid_->removeWidget(labelParamNumber);
    grid_->removeWidget(checkBoxIsLogging);
    grid_->removeWidget(labelParamName);
    grid_->removeWidget(comboBoxLogType);
    grid_->removeWidget(labelDivider);
    grid_->removeWidget(spinBoxDivider);
    grid_->removeWidget(labelBuffer);
    grid_->removeWidget(spinBoxBufferSize);
    grid_->removeWidget(comboBoxIsLooping);
    grid_->removeWidget(bufferInd_);
    grid_->removeWidget(pushButtonChangeParam);
    grid_->removeWidget(pushButtonRefreshParam);

    delete labelParamNumber;
    delete checkBoxIsLogging;
    delete labelParamName;
    delete comboBoxLogType;
    delete labelDivider;
    delete spinBoxDivider;
    delete labelBuffer;
    delete spinBoxBufferSize;
    delete comboBoxIsLooping;
    delete bufferInd_;
    delete pushButtonChangeParam;
    delete pushButtonRefreshParam;

  };

  public slots:

  void pushButtonChangeParamPressed() {
    ROS_DEBUG_STREAM("Change logger element " << name_);

    signal_logger_msgs::GetLoggerElementRequest reqGet;
    signal_logger_msgs::GetLoggerElementResponse resGet;

    reqGet.name = name_;
    if(!getLogElementClient_->call(reqGet, resGet)) {
      ROS_INFO_STREAM("Could not change logger element " << name_);
      return;
    }

    signal_logger_msgs::SetLoggerElementRequest req;
    signal_logger_msgs::SetLoggerElementResponse res;

    req.log_element.name = name_;
    req.log_element.is_logged = static_cast<bool>(checkBoxIsLogging->checkState());
    req.log_element.divider = spinBoxDivider->value();
    req.log_element.action = comboBoxLogType->currentIndex();
    req.log_element.buffer_size = spinBoxBufferSize->value();
    req.log_element.is_buffer_looping = comboBoxIsLooping->currentIndex();

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
        checkBoxIsLogging->setCheckState( res.log_element.is_logged?Qt::CheckState::Checked : Qt::CheckState::Unchecked );
        spinBoxDivider->setValue( res.log_element.divider );
        comboBoxLogType->setCurrentIndex(res.log_element.action);
        spinBoxBufferSize->setValue( res.log_element.buffer_size );
        comboBoxIsLooping->setCurrentIndex(res.log_element.is_buffer_looping);
        bufferInd_->updateData(res.log_element.no_unread_items_in_buffer,res.log_element.no_items_in_buffer,res.log_element.buffer_size);
      }
      else {
        ROS_WARN_STREAM("Could not get parameter " << name_);
      }
    }
  }

 protected:
  std::string name_;
  QGridLayout* grid_;
  ros::ServiceClient* getLogElementClient_;
  ros::ServiceClient* setLogElementClient_;

 public:
  QLabel* labelParamNumber;
  QCheckBox* checkBoxIsLogging;
  QLabel* labelParamName;
  QLabel* labelDivider;
  QSpinBox* spinBoxDivider;
  QComboBox* comboBoxLogType;
  QLabel* labelBuffer;
  QSpinBox* spinBoxBufferSize;
  QComboBox* comboBoxIsLooping;
  BufferIndicator* bufferInd_;
  QPushButton* pushButtonRefreshParam;
  QPushButton* pushButtonChangeParam;

};
