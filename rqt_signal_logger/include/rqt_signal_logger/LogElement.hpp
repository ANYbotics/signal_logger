 /*!
 * @file     LogElement.hpp
 * @author   Gabriel Hottiger, Samuel Bachmann, Fernando Garcia
 * @date     October 10, 2016
 * @brief    Log element entry in the list.
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
#include <QLabel>
#include <QCheckBox>
#include <QSpinBox>
#include <QComboBox>
#include <QPushButton>

namespace rqt_signal_logger {

//! This class draws and handles log elements.
class LogElement: public QObject {
  Q_OBJECT

 public:
  //! Enum mapping defining possible log actions
  enum class LogAction : unsigned int {
    SAVE_AND_PUBLISH = signal_logger_msgs::LogElement::ACTION_SAVE_AND_PUBLISH,
    SAVE = signal_logger_msgs::LogElement::ACTION_SAVE,
    PUBLISH = signal_logger_msgs::LogElement::ACTION_PUBLISH
  };

  //! Enum mapping defining possible buffer types
  enum class BufferType : unsigned int {
    FIXED_SIZE = signal_logger_msgs::LogElement::BUFFERTYPE_FIXED_SIZE,
    LOOPING = signal_logger_msgs::LogElement::BUFFERTYPE_LOOPING,
    EXPONENTIALLY_GROWING = signal_logger_msgs::LogElement::BUFFERTYPE_EXPONENTIALLY_GROWING
  };

 public:
  LogElement(
              const std::string& name,
              QTreeWidgetItem* parent,
              QWidget* widget,
              QTreeWidget* treeWidget,
              ros::ServiceClient* getLogElementClient,
              ros::ServiceClient* setLogElementClient,
              size_t maxParamNameWidth)
 {
    name_ = name;
    parent_ = parent;
    treeWidget_ = treeWidget;
    getLogElementClient_= getLogElementClient;
    setLogElementClient_ = setLogElementClient;

    if (parent_){
      topLevelItem = new QTreeWidgetItem(parent_);
    }else {
      topLevelItem = new QTreeWidgetItem(treeWidget);
      treeWidget->addTopLevelItem(topLevelItem);
    }

    treeWidget->setColumnCount(11);

    //! Add name label
    labelParamName = new QLabel();
    labelParamName->setObjectName(QString::fromStdString(std::string{"labelParamName"} + name_));
    labelParamName->setText(QString::fromStdString(name_));
    labelParamName->setFixedSize(maxParamNameWidth-10, labelParamName->height());
    topLevelItem->setText(0, QString::fromStdString(name_));

    //! Add is logging checkbox
    checkBoxIsLogging = new QCheckBox();
    checkBoxIsLogging->setObjectName(QString::fromStdString(std::string{"checkBoxIsLogging"} + name_));
    checkBoxIsLogging->setTristate(false);
    checkBoxIsLogging->setCheckState(Qt::CheckState::Unchecked);
    checkBoxIsLogging->setMinimumHeight(30);
    treeWidget->setItemWidget(topLevelItem, 1, checkBoxIsLogging);

    //! Add log type combobox
    comboBoxLogType = new QComboBox();
    comboBoxLogType->setObjectName(QString::fromStdString(std::string{"comboBoxLogType"} + name_));
    comboBoxLogType->insertItem(static_cast<int>(LogAction::SAVE_AND_PUBLISH), "Save and Publish");
    comboBoxLogType->insertItem(static_cast<int>(LogAction::SAVE),"Save");
    comboBoxLogType->insertItem(static_cast<int>(LogAction::PUBLISH),"Publish");
    comboBoxLogType->setCurrentIndex(static_cast<int>(LogAction::SAVE_AND_PUBLISH));
    treeWidget->setItemWidget(topLevelItem, 2, comboBoxLogType);

    //! Add divider label
    labelDivider = new QLabel();
    labelDivider->setObjectName(QString::fromStdString(std::string{"labelDivider"} + name_));
    labelDivider->setText(QString::fromStdString(std::string{"Divider:"}));
    labelDivider->setFixedWidth(labelDivider->fontMetrics().width(labelDivider->text())+10);
    labelDivider->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
    treeWidget->setItemWidget(topLevelItem, 3, labelDivider);

    //! Add divider spinbox
    spinBoxDivider = new QSpinBox();
    spinBoxDivider->setObjectName(QString::fromStdString(std::string{"spinBoxDivider"} + name_));
    spinBoxDivider->setMinimumSize(QSize(50, 0));
    spinBoxDivider->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
    spinBoxDivider->setMinimum(1);
    spinBoxDivider->setMaximum(1000);
    spinBoxDivider->setSingleStep(1);
    treeWidget->setItemWidget(topLevelItem, 4, spinBoxDivider);

    //! Add buffer label
    labelBuffer = new QLabel();
    labelBuffer->setObjectName(QString::fromStdString(std::string{"labelBuffer"} + name_));
    labelBuffer->setText(QString::fromStdString("Buffer size:"));
    labelBuffer->setFixedWidth(labelBuffer->fontMetrics().width(labelBuffer->text())+10);
    labelBuffer->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
    treeWidget->setItemWidget(topLevelItem, 5, labelBuffer);

    //! Add buffer size spinbox
    spinBoxBufferSize = new QSpinBox();
    spinBoxBufferSize->setObjectName(QString::fromStdString(std::string{"spinBoxBufferSize"} + name_));
    spinBoxBufferSize->setMinimumSize(QSize(50, 0));
    spinBoxBufferSize->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
    spinBoxBufferSize->setMinimum(1);
    spinBoxBufferSize->setMaximum(100000);
    spinBoxBufferSize->setSingleStep(1);
    treeWidget->setItemWidget(topLevelItem, 6, spinBoxBufferSize);

    //! Add buffer looping combobox
    comboBoxBufferType = new QComboBox();
    comboBoxBufferType->setObjectName(QString::fromStdString(std::string{"comboBoxBufferType"} + name_));
    comboBoxBufferType->insertItem(static_cast<int>(BufferType::FIXED_SIZE), "Fixed Size");
    comboBoxBufferType->insertItem(static_cast<int>(BufferType::LOOPING), "Looping");
    comboBoxBufferType->insertItem(static_cast<int>(BufferType::EXPONENTIALLY_GROWING),"Growing");
    comboBoxBufferType->setCurrentIndex(static_cast<int>(BufferType::FIXED_SIZE));
    treeWidget->setItemWidget(topLevelItem, 7, comboBoxBufferType);

    //!  Buffer Indicator
    bufferInd_ = new BufferIndicator(treeWidget);
    bufferInd_->setObjectName(QString::fromStdString(std::string{"indicatorBuffer"} + name_));
//    connect(bufferInd_, SIGNAL(refresh()), this, SLOT(refreshElement()));
    treeWidget->setItemWidget(topLevelItem, 8, bufferInd_);

    //! Change button
    pushButtonChangeParam = new QPushButton();
    pushButtonChangeParam->setObjectName(QString::fromStdString(std::string{"pushButtonChangeParam"} + name_));
    pushButtonChangeParam->setText(QString::fromUtf8("change"));
    treeWidget->setItemWidget(topLevelItem, 9, pushButtonChangeParam);

    //! Refresh button
    pushButtonRefreshParam = new QPushButton();
    pushButtonRefreshParam->setObjectName(QString::fromStdString(std::string{"pushButtonRefreshParam"} + name_));
    pushButtonRefreshParam->setText(QString::fromUtf8("refresh"));
    treeWidget->setItemWidget(topLevelItem, 10, pushButtonRefreshParam);

    setTreeProperties(treeWidget);

    connect(pushButtonChangeParam, SIGNAL(pressed()), this, SLOT(updateChangeElement()));
    connect(pushButtonRefreshParam, SIGNAL(pressed()), this, SLOT(updateRefreshElement()));
    connect(checkBoxIsLogging, SIGNAL(stateChanged(int)), this, SLOT(updateIsLogging(int)));
    connect(comboBoxLogType, SIGNAL(currentTextChanged(QString)), this, SLOT(updateLogType(QString)));
    connect(spinBoxDivider, SIGNAL(valueChanged(int)), this, SLOT(updateDivider(int)));
    connect(spinBoxBufferSize, SIGNAL(valueChanged(int)), this, SLOT(updateBufferSize(int)));
    connect(comboBoxBufferType, SIGNAL(currentTextChanged(QString)), this, SLOT(updateBufferType(QString)));

    updateRefreshElement();
 }

  virtual ~LogElement() {
    disconnect(pushButtonChangeParam, SIGNAL(pressed()), 0, 0);
    disconnect(pushButtonRefreshParam, SIGNAL(pressed()), 0, 0);
    disconnect(checkBoxIsLogging, SIGNAL(stateChanged(int)), 0, 0);
    disconnect(comboBoxLogType, SIGNAL(currentTextChanged(QString)), 0, 0);
    disconnect(spinBoxDivider, SIGNAL(valueChanged(int)), 0, 0);
    disconnect(spinBoxBufferSize, SIGNAL(valueChanged(int)), 0, 0);
    disconnect(comboBoxBufferType, SIGNAL(currentTextChanged(QString)), 0, 0);

    delete pushButtonChangeParam;
    delete pushButtonRefreshParam;
    delete checkBoxIsLogging;
    delete labelParamName;
    delete comboBoxLogType;
    delete labelDivider;
    delete spinBoxDivider;
    delete labelBuffer;
    delete spinBoxBufferSize;
    delete comboBoxBufferType;
    delete bufferInd_;
  };

  void updateCheckBox(QTreeWidgetItem* item, const int status) {
    auto checkbox = qobject_cast<QCheckBox*>(treeWidget_->itemWidget(item, 1));
    checkbox->setChecked(status);
    for( int i = 0; i < item->childCount(); ++i )
      updateCheckBox(item->child(i), status);
  }

  void updateComboBox(QTreeWidgetItem* item, const QString text, const int column) {
    auto combobox = qobject_cast<QComboBox*>(treeWidget_->itemWidget(item, column));
    combobox->setCurrentText(text);
    for( int i = 0; i < item->childCount(); ++i )
      updateComboBox(item->child(i), text, column);
  }

  void updateSpinBox(QTreeWidgetItem* item, const int value, const int column) {
    auto spinbox = qobject_cast<QSpinBox*>(treeWidget_->itemWidget(item, column));
    spinbox->setValue(value);
    for( int i = 0; i < item->childCount(); ++i )
      updateSpinBox(item->child(i), value, column);
  }

  void expandEntries() {
    treeWidget_->expandAll();
  }

  public slots:

  void updateIsLogging(int checkBoxStatus) {
    updateCheckBox(topLevelItem, checkBoxStatus);
  }
  void updateLogType(QString comboBox) {
    updateComboBox(topLevelItem, comboBox, 2);
  }
  void updateDivider(int spinBox) {
    updateSpinBox(topLevelItem, spinBox, 4);
  }
  void updateBufferSize(int spinBoxBuffer) {
    updateSpinBox(topLevelItem, spinBoxBuffer, 6);
  }
  void updateBufferType(QString comboBox) {
    updateComboBox(topLevelItem, comboBox, 7);
  }
  bool updateChangeElement() {
    return changeElement(topLevelItem);
  }
  bool updateRefreshElement() {
    return refreshElement(topLevelItem);
  }

  bool changeElement(QTreeWidgetItem* item) {
    std::string name = item->text(0).toUtf8().constData();
    ROS_DEBUG_STREAM("Change logger element " << name);

    for( int i = 0; i < item->childCount(); ++i )
      changeElement(item->child(i));

    signal_logger_msgs::GetLoggerElementRequest reqGet;
    signal_logger_msgs::GetLoggerElementResponse resGet;

    reqGet.name = name;
    if(!getLogElementClient_->call(reqGet, resGet)) {
      ROS_DEBUG_STREAM("Could not change logger element " << name);
      return false;
    }

    signal_logger_msgs::SetLoggerElementRequest req;
    signal_logger_msgs::SetLoggerElementResponse res;

    req.log_element.name = name;
    req.log_element.is_logged = static_cast<bool>(checkBoxIsLogging->checkState());
    req.log_element.divider = spinBoxDivider->value();
    req.log_element.action = comboBoxLogType->currentIndex();
    req.log_element.buffer_size = spinBoxBufferSize->value();
    req.log_element.buffer_type = comboBoxBufferType->currentIndex();

    if(!setLogElementClient_->call(req, res)) {
      ROS_DEBUG_STREAM("Could not change logger element " << name);
      return false;
    }

    return true;
  }

  bool refreshElement(QTreeWidgetItem* item) {
    signal_logger_msgs::GetLoggerElementRequest req;
    signal_logger_msgs::GetLoggerElementResponse res;

    std::string name = item->text(0).toUtf8().constData();
    req.name = name;

    for( int i = 0; i < item->childCount(); ++i )
      refreshElement(item->child(i));

    if (getLogElementClient_->exists()) {
      if (getLogElementClient_->call(req, res) && res.success) {
        checkBoxIsLogging->setCheckState( res.log_element.is_logged?Qt::CheckState::Checked : Qt::CheckState::Unchecked );
        spinBoxDivider->setValue( res.log_element.divider );
        comboBoxLogType->setCurrentIndex(res.log_element.action);
        spinBoxBufferSize->setValue( res.log_element.buffer_size );
        comboBoxBufferType->setCurrentIndex(res.log_element.buffer_type);
        bufferInd_->updateData(res.log_element.no_unread_items_in_buffer,res.log_element.no_items_in_buffer,res.log_element.buffer_size);
      }
      else {
        ROS_DEBUG_STREAM("Could not get parameter " << name);
        return false;
      }
    }

    return true;
  }

  void setTreeProperties(QTreeWidget* treeWidget){
    treeWidget->setStyleSheet("QTreeView::item {"
                              "margin: 2px;"
                              "}");

    treeWidget->setFocusPolicy(Qt::NoFocus);

    treeWidget->header()->resizeSection(0, 400);
    treeWidget->header()->resizeSection(1, 20);
    treeWidget->header()->resizeSection(2, 150);
    treeWidget->header()->resizeSection(4, 70);
    treeWidget->header()->resizeSection(6, 80);
    treeWidget->header()->resizeSection(9, 80);
    treeWidget->header()->resizeSection(10, 80);

    treeWidget->header()->setStretchLastSection(false);

    QTreeWidgetItem* header = treeWidget->headerItem();
    for( int i = 0; i < treeWidget->columnCount(); ++i )
      header->setText(i, "");
  }

 protected:
  QTreeWidgetItem* topLevelItem;
  QTreeWidget* treeWidget_;
  std::string name_;
  QTreeWidgetItem* parent_;
  ros::ServiceClient* getLogElementClient_;
  ros::ServiceClient* setLogElementClient_;

 public:
  QCheckBox* checkBoxIsLogging;
  QLabel* labelParamName;
  QLabel* labelDivider;
  QSpinBox* spinBoxDivider;
  QComboBox* comboBoxLogType;
  QLabel* labelBuffer;
  QSpinBox* spinBoxBufferSize;
  QComboBox* comboBoxBufferType;
  BufferIndicator* bufferInd_;
  QPushButton* pushButtonRefreshParam;
  QPushButton* pushButtonChangeParam;

};

}
