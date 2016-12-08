/*
 * SignalLoggerPlugin.cpp
 *
 *  Created on: August 2016
 *      Author: Christian Gehring
 */

// rqt_signal_logger
#include "rqt_signal_logger/SignalLoggerPlugin.hpp"
#include "rqt_signal_logger/yaml_helper.hpp"

// yaml-cpp
#include <yaml-cpp/yaml.h>

// ros
#include <ros/package.h>
#include <pluginlib/class_list_macros.h>

// Qt
#include <QStringList>
#include <QGridLayout>
#include <QScrollArea>
#include <QScrollBar>
#include <QMessageBox>
#include <QFileDialog>
#include <QSignalMapper>

// STL
#include <fstream>

// System
#include <sys/stat.h>

namespace rqt_signal_logger {

static bool compareNoCase( const std::string& s1, const std::string& s2 ) {
  return strcasecmp( s1.c_str(), s2.c_str() ) <= 0;
}

struct size_less {
  template<class T> bool operator()(T const &a, T const &b) const
  { return a.size() < b.size(); }
};

static size_t getMaxParamNameWidth(std::vector<std::string> const &lines) {
  //use QFontMetrics this way;
  QFont font("", 0);
  QFontMetrics fm(font);

  auto it = std::max_element(lines.begin(), lines.end(), size_less());
  QString text = QString::fromStdString(*it);
  return fm.width(text);
}

SignalLoggerPlugin::SignalLoggerPlugin() :
                            rqt_gui_cpp::Plugin(),
                            tabWidget_(),
                            varsWidget_(0),
                            configureWidget_(0),
                            paramsGrid_(),
                            paramsWidget_(0),
                            paramsScrollHelperWidget_(0),
                            paramsScrollLayout_(0),
                            updateFrequency_(0.0)
{
  // Constructor is called first before initPlugin function, needless to say.
  // give QObjects reasonable names
  setObjectName("SignalLoggerPlugin");
}

void SignalLoggerPlugin::initPlugin(qt_gui_cpp::PluginContext& context) {
  // access standalone command line arguments
  QStringList argv = context.argv();

  // create the main widget
  tabWidget_ = new QTabWidget();

  // create the vars widget add it as first tab
  varsWidget_ = new QWidget();
  varsUi_.setupUi(varsWidget_);
  tabWidget_->addTab(varsWidget_, QString::fromUtf8("Variables"));

  // create the configure widget add it as second tab
  configureWidget_ = new QWidget();
  configureUi_.setupUi(configureWidget_);
  tabWidget_->addTab(configureWidget_, QString::fromUtf8("Configure"));

  // Set it up and add it to the user interface
  context.addWidget(tabWidget_);

  // Do some configuration
  varsUi_.taskComboBox->insertItem(static_cast<int>(TaskList::ENABLE_ALL), "Enable all elements");
  varsUi_.taskComboBox->insertItem(static_cast<int>(TaskList::DISABLE_ALL), "Disable all elements");
  varsUi_.taskComboBox->insertItem(static_cast<int>(TaskList::SET_DIVIDER), "Set divider to ");
  varsUi_.taskComboBox->insertItem(static_cast<int>(TaskList::SET_ACTION), "Set action to ");
  varsUi_.taskComboBox->insertItem(static_cast<int>(TaskList::SET_BUFFER_TYPE), "Set buffer type to ");
  varsUi_.taskComboBox->insertItem(static_cast<int>(TaskList::SET_BUFFER_SIZE), "Set buffer size to ");
  varsUi_.taskComboBox->insertItem(static_cast<int>(TaskList::SET_BUFFER_SIZE_FROM_TIME), "Setup buffer size from time");
  varsUi_.taskComboBox->setCurrentIndex(static_cast<int>(TaskList::ENABLE_ALL));
  taskChanged(static_cast<int>(TaskList::ENABLE_ALL));
  varsUi_.valueSpinBox->setMinimumWidth(varsUi_.valueSpinBox->fontMetrics().width(QString("(1) Save and Publish"))+30);

  /******************************
   * Connect ui forms to actions *
   ******************************/
  connect(varsUi_.pushButtonRefreshAll, SIGNAL(pressed()), this, SLOT(refreshAll()));
  connect(varsUi_.lineEditFilter, SIGNAL(returnPressed()), this ,SLOT(refreshAll()));
  connect(varsUi_.pushButtonChangeAll, SIGNAL(pressed()), this, SLOT(changeAll()));
  connect(this, SIGNAL(parametersChanged()), this, SLOT(drawParamList()));
  connect(varsUi_.taskComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(taskChanged(int)));
  connect(varsUi_.applyButton, SIGNAL(pressed()), this, SLOT(applyButtonPressed()));
  connect(varsUi_.valueSpinBox, SIGNAL(valueChanged(double)), this, SLOT(valueChanged(double)));

  connect(configureUi_.startLoggerButton, SIGNAL(pressed()), this, SLOT(startLogger()));
  connect(configureUi_.stopLoggerButton, SIGNAL(pressed()), this, SLOT(stopLogger()));
  connect(configureUi_.pathButton, SIGNAL(pressed()), this, SLOT(selectYamlFile()));
  connect(configureUi_.loadScriptButton, SIGNAL(pressed()), this, SLOT(loadYamlFile()));
  connect(configureUi_.saveScriptButton, SIGNAL(pressed()), this, SLOT(saveYamlFile()));
  connect(configureUi_.namespaceComboBox, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(setNamespace(const QString&)));
  connect(configureUi_.namespaceComboBox->lineEdit(), SIGNAL(returnPressed()), this, SLOT(addNamespace()));

  QSignalMapper* signalMapper = new QSignalMapper (this) ;
  connect(configureUi_.saveBinButton, SIGNAL(pressed()), signalMapper, SLOT(map()));
  connect(configureUi_.saveBagButton, SIGNAL(pressed()), signalMapper, SLOT(map()));
  connect(configureUi_.saveBothButton, SIGNAL(pressed()), signalMapper, SLOT(map()));
  signalMapper -> setMapping (configureUi_.saveBinButton, signal_logger_msgs::SaveLoggerDataRequest::LOGFILE_TYPE_BINARY) ;
  signalMapper -> setMapping (configureUi_.saveBagButton, signal_logger_msgs::SaveLoggerDataRequest::LOGFILE_TYPE_BAG) ;
  signalMapper -> setMapping (configureUi_.saveBothButton, signal_logger_msgs::SaveLoggerDataRequest::LOGFILE_TYPE_BINARY_AND_BAG) ;
  connect (signalMapper, SIGNAL(mapped(int)), this, SLOT(saveLoggerData(int))) ;

  getLoggerConfigurationServiceName_ = "/silo_ros/get_logger_configuration";
  getNodeHandle().getParam("get_logger_configuration_service", getLoggerConfigurationServiceName_);

  getParameterServiceName_  = "/silo_ros/get_logger_element";
  getNodeHandle().getParam("get_logger_element_service", getParameterServiceName_);

  setParameterServiceName_ = "/silo_ros/set_logger_element";
  getNodeHandle().getParam("set_logger_element_service", setParameterServiceName_);

  startLoggerServiceName_  = "/silo_ros/start_logger";
  getNodeHandle().getParam("start_logger_service", startLoggerServiceName_);

  stopLoggerServiceName_ = "/silo_ros/stop_logger";
  getNodeHandle().getParam("stop_logger_service", stopLoggerServiceName_);

  saveLoggerDataServiceName_  = "/silo_ros/save_logger_data";
  getNodeHandle().getParam("save_logger_data_service", saveLoggerDataServiceName_);

  loadLoggerScriptServiceName_  = "/silo_ros/load_logger_script";
  getNodeHandle().getParam("load_logger_script_service", loadLoggerScriptServiceName_);

  isLoggerRunningServiceName_ = "/silo_ros/is_logger_running";
  getNodeHandle().getParam("is_logger_running_service", isLoggerRunningServiceName_);

}

bool SignalLoggerPlugin::checkNamespace(const QString & text) {
  return ( ros::service::exists(text.toStdString()+getLoggerConfigurationServiceName_, false) &&
           ros::service::exists(text.toStdString()+getParameterServiceName_, false) &&
           ros::service::exists(text.toStdString()+setParameterServiceName_, false) &&
           ros::service::exists(text.toStdString()+startLoggerServiceName_, false) &&
           ros::service::exists(text.toStdString()+stopLoggerServiceName_, false) &&
           ros::service::exists(text.toStdString()+saveLoggerDataServiceName_, false) &&
           ros::service::exists(text.toStdString()+loadLoggerScriptServiceName_, false) &&
           ros::service::exists(text.toStdString()+isLoggerRunningServiceName_, false) );
}

void SignalLoggerPlugin::addNamespace() {
  QString text = configureUi_.namespaceComboBox->lineEdit()->text();

  if(text == QString("clear")) {
    configureUi_.namespaceComboBox->clear();
    configureUi_.namespaceComboBox->lineEdit()->clear();
    return;
  }

  if(!checkNamespace(text)) {
    configureUi_.namespaceComboBox->removeItem(configureUi_.namespaceComboBox->findText(text));
  }

}


void SignalLoggerPlugin::setNamespace(const QString & text) {
  shutdownROS();
  if(checkNamespace(text))
  {
    // ROS services
    getLoggerConfigurationClient_ = getNodeHandle().serviceClient<signal_logger_msgs::GetLoggerConfiguration>(text.toStdString()+getLoggerConfigurationServiceName_);
    getLoggerElementClient_ = getNodeHandle().serviceClient<signal_logger_msgs::GetLoggerElement>(text.toStdString()+getParameterServiceName_);
    setLoggerElementClient_ = getNodeHandle().serviceClient<signal_logger_msgs::SetLoggerElement>(text.toStdString()+setParameterServiceName_);
    startLoggerClient_ = getNodeHandle().serviceClient<std_srvs::Trigger>(text.toStdString()+startLoggerServiceName_);
    stopLoggerClient_ = getNodeHandle().serviceClient<std_srvs::Trigger>(text.toStdString()+stopLoggerServiceName_);
    saveLoggerDataClient_ = getNodeHandle().serviceClient<signal_logger_msgs::SaveLoggerData>(text.toStdString()+saveLoggerDataServiceName_);
    loadLoggerScriptClient_ = getNodeHandle().serviceClient<signal_logger_msgs::LoadLoggerScript>(text.toStdString()+loadLoggerScriptServiceName_);
    isLoggerRunningClient_ = getNodeHandle().serviceClient<std_srvs::Trigger>(text.toStdString()+isLoggerRunningServiceName_);
    refreshAll();
  }
}

void SignalLoggerPlugin::shutdownROS() {
  getLoggerConfigurationClient_.shutdown();
  getLoggerElementClient_.shutdown();
  setLoggerElementClient_.shutdown();
  startLoggerClient_.shutdown();
  stopLoggerClient_.shutdown();
  saveLoggerDataClient_.shutdown();
  loadLoggerScriptClient_.shutdown();
  isLoggerRunningClient_.shutdown();
}

void SignalLoggerPlugin::changeAll() {
  for (auto& elem : logElements_) {
    elem->changeElement();
  }
}

void SignalLoggerPlugin::refreshAll() {
  signal_logger_msgs::GetLoggerConfigurationRequest req;
  signal_logger_msgs::GetLoggerConfigurationResponse res;

  if (getLoggerConfigurationClient_.call(req,res)) {
    // Update parameter names
    logElementNames_ = res.log_element_names;
    updateFrequency_ = res.collect_frequency;
    configureUi_.pathEdit->setText(QString::fromStdString(res.script_filepath));
    varsUi_.ns->setText(QString::fromStdString(res.logger_namespace));
    varsUi_.freq->setText(QString::fromStdString(std::to_string(updateFrequency_)+std::string{" [Hz]"}));

    // Sort names alphabetically.
    std::sort(logElementNames_.begin(), logElementNames_.end(), compareNoCase );

    // Update GUI
    emit parametersChanged();
  }
  else {
    ROS_WARN("Ros service: 'Get Logger Configuration' returned false!");
  }
}

void SignalLoggerPlugin::startLogger() {
  std_srvs::Trigger::Request req;
  std_srvs::Trigger::Response res;
  if(startLoggerClient_.call(req, res) && res.success) {
    statusMessage("Successfully started logger!", MessageType::SUCCESS, 2.0);
  }
  else {
    statusMessage("Could not start logger!", MessageType::ERROR, 2.0);
  }
  checkLoggerState();
}

void SignalLoggerPlugin::stopLogger() {
  std_srvs::Trigger::Request req;
  std_srvs::Trigger::Response res;
  if(stopLoggerClient_.call(req, res) && res.success) {
    statusMessage("Successfully stopped logger!", MessageType::SUCCESS, 2.0);
  }
  else {
    statusMessage("Could not stop logger!", MessageType::ERROR, 2.0);
  }
  checkLoggerState();
}

void SignalLoggerPlugin::saveLoggerData(int type) {
  signal_logger_msgs::SaveLoggerData::Request req;
  signal_logger_msgs::SaveLoggerData::Response res;
  req.logfileType = type;
  if(saveLoggerDataClient_.call(req, res) && res.success) {
    std::string msg = std::string{"Successfully saved logger data to file."};
    statusMessage(msg, MessageType::SUCCESS, 2.0);
  }
  else {
    statusMessage("Could not save logger data!", MessageType::ERROR, 2.0);
  }
}

void SignalLoggerPlugin::selectYamlFile() {
  QString fileName = QFileDialog::getOpenFileName(configureWidget_,
                                                  tr("Open Logger Configuration File"), "/home", tr("YAML Files (*.yaml)"));
  configureUi_.pathEdit->setText(fileName);
}

void SignalLoggerPlugin::loadYamlFile() {
  signal_logger_msgs::LoadLoggerScriptRequest req;
  signal_logger_msgs::LoadLoggerScriptResponse res;
  req.filepath = configureUi_.pathEdit->text().toStdString();
  if(loadLoggerScriptClient_.call(req, res) && res.success) {
    statusMessage("Successfully loaded logger configuration file!", MessageType::SUCCESS, 2.0);
  }
  else {
    statusMessage("Could not load logger configuration file!", MessageType::ERROR, 2.0);
  }
  refreshAll();
}

void SignalLoggerPlugin::saveYamlFile() {
  // Check for file existance and ask user for overwrite
  std::string filename = configureUi_.pathEdit->text().toStdString();
  struct stat buffer;
  if(stat(filename.c_str(), &buffer) == 0)
  {
    QMessageBox msgBox;
    msgBox.setText("The configuration file already exists.");
    msgBox.setInformativeText("Do you want to overwrite it?");
    msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
    msgBox.setDefaultButton(QMessageBox::Cancel);
    if(msgBox.exec() == QMessageBox::Cancel)
    {
      statusMessage("Did not save logger configuration file!", MessageType::ERROR, 2.0);
      return;
    }
  }

  YAML::Node node;
  std::size_t j = 0;
  for ( std::size_t i = 0; i < logElements_.size(); ++i)
  {
    if(static_cast<bool>(logElements_[i]->checkBoxIsLogging->checkState())) {
      node["log_elements"][j]["name"] = logElements_[i]->labelParamName->text().toStdString();
      node["log_elements"][j]["divider"] = logElements_[i]->spinBoxDivider->value();
      node["log_elements"][j]["action"] = logElements_[i]->comboBoxLogType->currentIndex();
      node["log_elements"][j]["buffer"]["size"] = logElements_[i]->spinBoxBufferSize->value();
      node["log_elements"][j]["buffer"]["type"] = logElements_[i]->comboBoxBufferType->currentIndex();
      j++;
    }
  }

  // If there are logged elements save them to file
  if(j!=0) {
    std::ofstream outfile(filename);
    writeYamlOrderedMaps(outfile, node);
    outfile.close();
  }
  else {
    statusMessage("Did not save logger configuration file! No data enabled!", MessageType::ERROR, 2.0);
    return;
  }

  statusMessage("Successfully saved logger configuration file!", MessageType::SUCCESS, 2.0);
}

void SignalLoggerPlugin::taskChanged(int index) {
  // Restore Default
  varsUi_.valueSpinBox->setEnabled(true);
  varsUi_.valueSpinBox->setSuffix(QString::fromUtf8(""));
  varsUi_.valueSpinBox->setPrefix(QString::fromUtf8(""));
  varsUi_.valueSpinBox->setValue(0);
  varsUi_.valueSpinBox->setMinimum(0);
  varsUi_.valueSpinBox->setSingleStep(1);
  varsUi_.valueSpinBox->setDecimals(0);

  // Change depending on type
  switch(index) {
    case static_cast<int>(TaskList::ENABLE_ALL):
    case static_cast<int>(TaskList::DISABLE_ALL):
      varsUi_.valueSpinBox->setEnabled(false);
      break;
    case static_cast<int>(TaskList::SET_BUFFER_TYPE):
    case static_cast<int>(TaskList::SET_ACTION):
      varsUi_.valueSpinBox->setPrefix(QString::fromUtf8("("));
      valueChanged(0);
      break;
    case static_cast<int>(TaskList::SET_BUFFER_SIZE):
    case static_cast<int>(TaskList::SET_DIVIDER):
      varsUi_.valueSpinBox->setRange(0.0, 100000.0);
      break;
    case static_cast<int>(TaskList::SET_BUFFER_SIZE_FROM_TIME):
      varsUi_.valueSpinBox->setSuffix(QString::fromUtf8(" [sec]"));
      varsUi_.valueSpinBox->setDecimals(2);
      varsUi_.valueSpinBox->setRange(0.0, 600.0);
      break;
    default:
      break;
  }
}

void SignalLoggerPlugin::valueChanged(double value) {
  switch(varsUi_.taskComboBox->currentIndex()) {
     case static_cast<int>(TaskList::SET_ACTION):
       {
         varsUi_.valueSpinBox->setValue(((int)roundf(value))%4);
         switch(((int)roundf(value))%4) {
           case static_cast<int>(LogElement::LogAction::NONE):
              varsUi_.valueSpinBox->setSuffix(") None");
              break;
           case static_cast<int>(LogElement::LogAction::SAVE_AND_PUBLISH):
              varsUi_.valueSpinBox->setSuffix(") Save and Publish");
              break;
           case static_cast<int>(LogElement::LogAction::SAVE):
              varsUi_.valueSpinBox->setSuffix(") Save");
              break;
           case static_cast<int>(LogElement::LogAction::PUBLISH):
              varsUi_.valueSpinBox->setSuffix(") Publish");
              break;
           default:
              break;
         }
       }
       break;
     case static_cast<int>(TaskList::SET_BUFFER_TYPE):
       {
         varsUi_.valueSpinBox->setValue(((int)roundf(value))%3);
         switch(((int)roundf(value))%3) {
           case static_cast<int>(LogElement::BufferType::FIXED_SIZE):
              varsUi_.valueSpinBox->setSuffix(") Fixed Size");
              break;
           case static_cast<int>(LogElement::BufferType::LOOPING):
              varsUi_.valueSpinBox->setSuffix(") Looping");
              break;
           case static_cast<int>(LogElement::BufferType::EXPONENTIALLY_GROWING):
              varsUi_.valueSpinBox->setSuffix(") Growing");
              break;
           default:
              break;
         }
       }
       break;
     default:
       break;
   }
}

void SignalLoggerPlugin::applyButtonPressed() {

  switch(varsUi_.taskComboBox->currentIndex()) {
    case static_cast<int>(TaskList::ENABLE_ALL):
      for(auto & element : logElements_) {
        element->checkBoxIsLogging->setCheckState(Qt::CheckState::Checked);
      }
      break;
    case static_cast<int>(TaskList::DISABLE_ALL):
      for(auto & element : logElements_) {
        element->checkBoxIsLogging->setCheckState(Qt::CheckState::Unchecked);
      }
      break;
    case static_cast<int>(TaskList::SET_DIVIDER):
      for(auto & element : logElements_) {
        element->spinBoxDivider->setValue(varsUi_.valueSpinBox->value());
      }
      break;
    case static_cast<int>(TaskList::SET_ACTION):
      for(auto & element : logElements_) {
        element->comboBoxLogType->setCurrentIndex((int)varsUi_.valueSpinBox->value());
      }
      break;
    case static_cast<int>(TaskList::SET_BUFFER_TYPE):
      for(auto & element : logElements_) {
        element->comboBoxBufferType->setCurrentIndex((int)varsUi_.valueSpinBox->value());
      }
      break;
    case static_cast<int>(TaskList::SET_BUFFER_SIZE):
      for(auto & element : logElements_) {
        element->spinBoxBufferSize->setValue(varsUi_.valueSpinBox->value());
      }
      break;
    case static_cast<int>(TaskList::SET_BUFFER_SIZE_FROM_TIME):
      for(auto & element : logElements_) {
        element->spinBoxBufferSize->setValue(ceil((double) varsUi_.valueSpinBox->value() * updateFrequency_ / (double)element->spinBoxDivider->value()));
      }
      break;
    default:
      break;
  }

}

void SignalLoggerPlugin::checkLoggerState() {
  std_srvs::TriggerRequest req_islogging;
  std_srvs::TriggerResponse res_islogging;

  if (isLoggerRunningClient_.call(req_islogging, res_islogging)) {
    if(paramsWidget_)
      paramsWidget_->setEnabled(!res_islogging.success);
    varsUi_.applyButton->setEnabled(!res_islogging.success);
    varsUi_.pushButtonChangeAll->setEnabled(!res_islogging.success);
  }
  else {
    ROS_WARN("Ros service : 'Is Logger Running' return false!");
  }

  return;
}

void SignalLoggerPlugin::shutdownPlugin() {
  getLoggerConfigurationClient_.shutdown();
  getLoggerElementClient_.shutdown();
  setLoggerElementClient_.shutdown();
  startLoggerClient_.shutdown();
  stopLoggerClient_.shutdown();
  saveLoggerDataClient_.shutdown();
  loadLoggerScriptClient_.shutdown();
}

void SignalLoggerPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const {
  plugin_settings.setValue("lineEditFilter", varsUi_.lineEditFilter->displayText());
  plugin_settings.setValue("pathEdit", configureUi_.pathEdit->displayText());
  plugin_settings.setValue("nrNamespaces", configureUi_.namespaceComboBox->count());
  plugin_settings.setValue("currentNamespace", configureUi_.namespaceComboBox->currentIndex());

  for(int i = 0; i<configureUi_.namespaceComboBox->count(); ++i) {
    plugin_settings.setValue(QString::fromStdString("ns" + std::to_string(i)), configureUi_.namespaceComboBox->itemText(i));
  }
}

void SignalLoggerPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) {
  varsUi_.lineEditFilter->setText(plugin_settings.value("lineEditFilter").toString());
  configureUi_.pathEdit->setText(plugin_settings.value("pathEdit").toString());
  for(int i = 0; i<plugin_settings.value("nrNamespaces").toInt(); ++i) {
    configureUi_.namespaceComboBox->insertItem(i, plugin_settings.value(QString::fromStdString("ns" + std::to_string(i))).toString());
  }
  int currIdx = plugin_settings.value("currentNamespace").toInt();
  configureUi_.namespaceComboBox->setCurrentIndex(currIdx);
  setNamespace(configureUi_.namespaceComboBox->itemText(currIdx));
}

// Try to find the Needle in the Haystack - ignore case
// Taken from http://stackoverflow.com/questions/3152241/case-insensitive-stdstring-find
bool findStringIC(const std::string & strHaystack, const std::string & strNeedle)
{
  auto it = std::search(
      strHaystack.begin(), strHaystack.end(),
      strNeedle.begin(),   strNeedle.end(),
      [](char ch1, char ch2) { return std::toupper(ch1) == std::toupper(ch2); }
  );
  return (it != strHaystack.end() );
}

void SignalLoggerPlugin::drawParamList() {

  logElements_.clear();

  if (paramsWidget_) {
    // delete widget
    delete paramsWidget_->layout();
    delete paramsScrollHelperWidget_->layout();
    varsUi_.gridLayout->removeWidget(paramsWidget_);
    delete paramsWidget_;
  }

  paramsWidget_ = new QWidget();
  paramsWidget_->setObjectName(QString::fromUtf8("paramsWidget"));
  varsUi_.gridLayout->addWidget(paramsWidget_,2,0,10,1);

  paramsScrollHelperWidget_ = new QWidget(paramsWidget_);
  paramsGrid_= new QGridLayout(paramsScrollHelperWidget_);
  paramsGrid_->setObjectName(QString::fromUtf8("paramsGrid"));

  const size_t maxParamNameWidth = getMaxParamNameWidth(logElementNames_);

  // Create a line for each filtered parameter
  std::string filter = varsUi_.lineEditFilter->text().toStdString();
  for (auto& name : logElementNames_) {
    //    std::size_t found = name.find(filter);
    if (findStringIC(name, filter)) {
      logElements_.push_back(std::shared_ptr<LogElement>(new LogElement(name, varsWidget_, paramsGrid_, &getLoggerElementClient_, &setLoggerElementClient_, maxParamNameWidth)));
    }
  }
  // This needs to be done after everthing is setup.
  paramsScrollHelperWidget_->setLayout(paramsGrid_);

  // Put it into a scroll area
  QScrollArea* paramsScrollArea = new QScrollArea();
  paramsScrollArea->setWidget(paramsScrollHelperWidget_);

  // Make the scroll step the same width as the fixed widgets in the grid
  paramsScrollArea->horizontalScrollBar()->setSingleStep(paramsScrollHelperWidget_->width() / 24);

  paramsScrollLayout_ = new QVBoxLayout(paramsWidget_);
  paramsScrollLayout_->addWidget(paramsScrollArea);

  paramsWidget_->setLayout(paramsScrollLayout_);
  paramsScrollHelperWidget_->setLayout(paramsGrid_);

  this->checkLoggerState();

}

void SignalLoggerPlugin::statusMessage(std::string message, MessageType type, double displaySeconds) {
  switch(type) {
    case MessageType::ERROR:
      configureUi_.statusbar->setStyleSheet("color: red");
      break;
    case MessageType::WARNING:
      configureUi_.statusbar->setStyleSheet("color: yellow");
      break;
    case MessageType::SUCCESS:
      configureUi_.statusbar->setStyleSheet("color: green");
      break;
    case MessageType::STATUS:
      configureUi_.statusbar->setStyleSheet("color: black");
      break;
  }
  configureUi_.statusbar->showMessage(QString::fromStdString(message), displaySeconds*1000);
}

}

PLUGINLIB_DECLARE_CLASS(rqt_signal_logger, SignalLoggerPlugin, rqt_signal_logger::SignalLoggerPlugin, rqt_gui_cpp::Plugin)
