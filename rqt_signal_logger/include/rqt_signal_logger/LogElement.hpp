/*
 * DoubleParameter.hpp
 *
 *  Created on: August 2016
 *      Author: Christian Gehring
 */

#pragma once

#include <QWidget>
#include <QStringList>
#include <QSpinBox>
#include <QGridLayout>
#include <QPushButton>
#include <QLineEdit>
#include <QCheckBox>
#include <QLabel>
#include <QApplication>
#include <QComboBox>
#include <rqt_signal_logger/LED.hpp>
#include <rqt_signal_logger/BufferIndicator.hpp>

#include <ros/ros.h>

#include <signal_logger_msgs/GetLoggerElement.h>
#include <signal_logger_msgs/SetLoggerElement.h>

//! This class draws and handles log elements.
class LogElement: public QObject {
  Q_OBJECT

 public:
  enum class LogType : unsigned int {
    SAVE_AND_PUBLISH = 0,
    SAVE = 1,
    PUBLISH = 2
  };

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

    //! Add number label
    std::string labelParamNumberName = std::string{"labelParamNumber"} + name;
    labelParamNumber = new QLabel(widget);
    labelParamNumber->setObjectName(QString::fromStdString(labelParamNumberName));
    labelParamNumber->setText(QString::number(iRow)+QString::fromUtf8(")"));

    //! Add is logging checkbox
    std::string checkBoxIsLoggingName = std::string{"checkBoxIsLogging"} + name;
    checkBoxIsLogging = new QCheckBox(widget);
    checkBoxIsLogging->setObjectName(QString::fromStdString(checkBoxIsLoggingName));
    checkBoxIsLogging->setText("log?");
    checkBoxIsLogging->setTristate(false);
    checkBoxIsLogging->setCheckState(Qt::CheckState::Unchecked);

    //! Add name label
    std::string labelParamNameName = std::string{"labelParamName"} + name;
    labelParamName = new QLabel(widget);
    labelParamName->setObjectName(QString::fromStdString(labelParamNameName));
    labelParamName->setText(QString::fromStdString(name));
    labelParamName->setFixedSize(maxParamNameWidth-10, labelParamName->height());

    //! Add divider label
    std::string labelDividerName = std::string{"labelDivider"} + name;
    labelDivider = new QLabel(widget);
    labelDivider->setObjectName(QString::fromStdString(labelDividerName));
    labelDivider->setText(QString::fromUtf8("Divider: "));

    //! Add divider spinbox
    std::string spinBoxDividerName = std::string{"spinBoxDivider"} + name;
    spinBoxDivider = new QSpinBox(widget);
    spinBoxDivider->setObjectName(QString::fromStdString(spinBoxDividerName));
    spinBoxDivider->setMinimumSize(QSize(50, 0));
    spinBoxDivider->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
    spinBoxDivider->setMinimum(1);
    spinBoxDivider->setMaximum(100);
    spinBoxDivider->setSingleStep(1);

    //! Add log type label
    std::string labelLogTypeName = std::string{"labelLogType"} + name;
    labelLogType = new QLabel(widget);
    labelLogType->setObjectName(QString::fromStdString(labelLogTypeName));
    labelLogType->setText(QString::fromUtf8("Log Type: "));

    //! Add log type combobox
    std::string comboBoxLogTypeName = std::string{"comboBoxLogType"} + name;
    comboBoxLogType = new QComboBox(widget);
    comboBoxLogType->setObjectName(QString::fromStdString(comboBoxLogTypeName));
    comboBoxLogType->addItem("Save and Publish");
    comboBoxLogType->addItem("Save");
    comboBoxLogType->addItem("Publish");

    //! Add buffer label
    std::string labelBufferName = std::string{"labelBuffer"} + name;
    labelBuffer = new QLabel(widget);
    labelBuffer->setObjectName(QString::fromStdString(labelBufferName));
    labelBuffer->setText(QString::fromUtf8("\t\t<b>Buffer:</b>\tsize:"));

    //! Add buffer size spinbox
    std::string spinBoxBufferSizeName = std::string{"spinBoxBufferSize"} + name;
    spinBoxBufferSize = new QSpinBox(widget);
    spinBoxBufferSize->setObjectName(QString::fromStdString(spinBoxBufferSizeName));
    spinBoxBufferSize->setMinimumSize(QSize(50, 0));
    spinBoxBufferSize->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
    spinBoxBufferSize->setMinimum(1);
    spinBoxBufferSize->setMaximum(1000);
    spinBoxBufferSize->setSingleStep(1);

    //! Add buffer looping combobox
    std::string comboBoxIsLoopingName = std::string{"comboBoxIsLooping"} + name;
    comboBoxIsLooping = new QComboBox(widget);
    comboBoxIsLooping->setObjectName(QString::fromStdString(comboBoxIsLoopingName));
    comboBoxIsLooping->addItem("Not Looping");
    comboBoxIsLooping->addItem("Looping");

    //! Led buffer full
    std::string ledBufferIsFullName = std::string{"ledBufferIsFull"} + name;
    ledBufferIsFull = new LED(widget);
    ledBufferIsFull->setObjectName(QString::fromStdString(ledBufferIsFullName));
    ledBufferIsFull->setColor(QColor("red"));

    std::string indicatorBufferName = std::string{"indicatorBuffer"} + name;
    bufferInd_ = new BufferIndicator(widget);
    bufferInd_->setObjectName(QString::fromStdString(indicatorBufferName));
    bufferInd_->updateData(25,50,100);

    //! Change button
    std::string pushButtonName = std::string{"pushButtonChange"} + name;
    pushButtonChangeParam = new QPushButton(widget);
    pushButtonChangeParam->setObjectName(QString::fromStdString(pushButtonName));
    pushButtonChangeParam->setMaximumSize(QSize(60, 16777215));
    pushButtonChangeParam->setText(QApplication::translate("SignalLogger", "change", 0, QApplication::UnicodeUTF8));

    grid->addWidget(labelParamNumber,      iRow, 0, 1, 1);
    grid->addWidget(checkBoxIsLogging,     iRow, 1, 1, 1);
    grid->addWidget(labelParamName,        iRow, 2, 1, 1);
    grid->addWidget(labelDivider,          iRow, 3, 1, 1);
    grid->addWidget(spinBoxDivider,        iRow, 4, 1, 1);
    grid->addWidget(labelLogType,          iRow, 5, 1, 1);
    grid->addWidget(comboBoxLogType,       iRow, 6, 1, 1);
    grid->addWidget(labelBuffer,           iRow, 7, 1, 1);
    grid->addWidget(spinBoxBufferSize,     iRow, 8, 1, 1);
    grid->addWidget(comboBoxIsLooping,     iRow, 9, 1, 1);
    grid->addWidget(ledBufferIsFull,       iRow, 10, 1, 1);
    grid->addWidget(bufferInd_,            iRow, 11, 1, 1);
    grid->addWidget(pushButtonChangeParam, iRow, 12, 1, 1);

    connect(pushButtonChangeParam, SIGNAL(pressed()), this, SLOT(pushButtonChangeParamPressed()));

    refreshElement();
  }

  virtual ~LogElement() {
    disconnect(pushButtonChangeParam, SIGNAL(pressed()), 0, 0);

    grid_->removeWidget(labelParamNumber);
    grid_->removeWidget(checkBoxIsLogging);
    grid_->removeWidget(labelParamName);
    grid_->removeWidget(labelDivider);
    grid_->removeWidget(spinBoxDivider);
    grid_->removeWidget(labelLogType);
    grid_->removeWidget(comboBoxLogType);
    grid_->removeWidget(labelBuffer);
    grid_->removeWidget(spinBoxBufferSize);
    grid_->removeWidget(comboBoxIsLooping);
    grid_->removeWidget(ledBufferIsFull);
    grid_->removeWidget(bufferInd_);
    grid_->removeWidget(pushButtonChangeParam);

    delete labelParamNumber;
    delete checkBoxIsLogging;
    delete labelParamName;
    delete labelDivider;
    delete spinBoxDivider;
    delete labelLogType;
    delete comboBoxLogType;
    delete labelBuffer;
    delete spinBoxBufferSize;
    delete comboBoxIsLooping;
    delete ledBufferIsFull;
    delete bufferInd_;
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
        ledBufferIsFull->setColor(QString::fromStdString(res.log_element.is_buffer_full?"red":"green"));
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
  QCheckBox* checkBoxIsLogging;
  QLabel* labelParamName;
  QLabel* labelDivider;
  QSpinBox* spinBoxDivider;
  QLabel* labelLogType;
  QComboBox* comboBoxLogType;
  QLabel* labelBuffer;
  QSpinBox* spinBoxBufferSize;
  QComboBox* comboBoxIsLooping;
  LED* ledBufferIsFull;
  BufferIndicator* bufferInd_;

  QPushButton* pushButtonChangeParam;
};
