/*
 * SignalLoggerPlugin.cpp
 *
 *  Created on: August 2016
 *      Author: Christian Gehring
 */

#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QGridLayout>
#include <QScrollArea>
#include <QScrollBar>
#include <QMessageBox>
#include <QFileDialog>
#include <rqt_signal_logger/SignalLoggerPlugin.hpp>
#include <ros/package.h>

#include <yaml-cpp/yaml.h>
#include <rqt_signal_logger/yaml_helper.hpp>
#include <fstream>
#include <sys/stat.h>

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
                    paramsScrollLayout_(0)
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

  /******************************
   * Connect ui forms to actions *
   ******************************/
  connect(varsUi_.pushButtonRefreshAll, SIGNAL(pressed()), this, SLOT(refreshAll()));
  connect(varsUi_.lineEditFilter, SIGNAL(returnPressed()), this ,SLOT(refreshAll()));
  connect(varsUi_.pushButtonChangeAll, SIGNAL(pressed()), this, SLOT(changeAll()));
  connect(this, SIGNAL(parametersChanged()), this, SLOT(drawParamList()));

  connect(configureUi_.startLoggerButton, SIGNAL(pressed()), this, SLOT(startLogger()));
  connect(configureUi_.stopLoggerButton, SIGNAL(pressed()), this, SLOT(stopLogger()));
  connect(configureUi_.saveDataButton, SIGNAL(pressed()), this, SLOT(saveLoggerData()));
  connect(configureUi_.pathButton, SIGNAL(pressed()), this, SLOT(selectYamlFile()));
  connect(configureUi_.loadScriptButton, SIGNAL(pressed()), this, SLOT(loadYamlFile()));
  connect(configureUi_.saveScriptButton, SIGNAL(pressed()), this, SLOT(saveYamlFile()));

  /******************************/
  //#Fixme read from param server
  std::string getLogElementListServiceName{"/get_log_element_list"};
  std::string getParameterServiceName{"/get_log_element"};
  std::string setParameterListServiceName{"/set_log_element"};
  std::string startLoggerServiceName{"/start_logger"};
  std::string stopLoggerServiceName{"/stop_logger"};
  std::string saveLoggerDataServiceName{"/save_logger_data"};

  // ROS services
  getLogElementListClient_ = getNodeHandle().serviceClient<signal_logger_msgs::GetLoggerInfo>(getLogElementListServiceName);
  getLogElementClient_ = getNodeHandle().serviceClient<signal_logger_msgs::GetLoggerElement>(getParameterServiceName);
  setLogElementClient_ = getNodeHandle().serviceClient<signal_logger_msgs::SetLoggerElement>(setParameterListServiceName);
  startLoggerClient_ = getNodeHandle().serviceClient<std_srvs::Trigger>(startLoggerServiceName);
  stopLoggerClient_ = getNodeHandle().serviceClient<std_srvs::Trigger>(stopLoggerServiceName);
  saveLoggerDataClient_ = getNodeHandle().serviceClient<std_srvs::Trigger>(saveLoggerDataServiceName);
}

void SignalLoggerPlugin::changeAll() {
  for (auto& elem : logElements_) {
    elem->pushButtonChangeParamPressed();
  }
}

void SignalLoggerPlugin::refreshAll() {

  signal_logger_msgs::GetLoggerInfo::Request req;
  signal_logger_msgs::GetLoggerInfo::Response res;

  if (getLogElementListClient_.call(req,res)) {
    // Update parameter names
    logElementNames_ = res.active_log_elements;
    logElementNames_.insert(logElementNames_.end(),res.inactive_log_elements.begin(), res.inactive_log_elements.end());

    // Sort names alphabetically.
    std::sort(logElementNames_.begin(), logElementNames_.end(), compareNoCase );

    // Update GUI
    emit parametersChanged();
  }
  else {
    ROS_WARN("Could not get parameter list!");
  }
}

void SignalLoggerPlugin::startLogger() {
  std_srvs::Trigger::Request req;
  std_srvs::Trigger::Response res;
  startLoggerClient_.call(req, res);
  if(res.success) {
    statusMessage("Successfully started logger!", MessageType::SUCCESS, 2.0);
  }
  else {
    statusMessage("Could not start logger!", MessageType::ERROR, 2.0);
  }
}

void SignalLoggerPlugin::stopLogger() {
  std_srvs::Trigger::Request req;
  std_srvs::Trigger::Response res;
  stopLoggerClient_.call(req, res);
  if(res.success) {
    statusMessage("Successfully stopped logger!", MessageType::SUCCESS, 2.0);
  }
  else {
    statusMessage("Could not stop logger!", MessageType::ERROR, 2.0);
  }
}

void SignalLoggerPlugin::saveLoggerData() {
  std_srvs::Trigger::Request req;
  std_srvs::Trigger::Response res;
  saveLoggerDataClient_.call(req, res);
  if(res.success) {
    std::string msg = std::string{"Successfully saved logger data to file: "} + res.message;
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
  std::string filename = configureUi_.pathEdit->text().toStdString();
  struct stat buffer;
  if(stat(filename.c_str(), &buffer) == 0)
  {
    refreshAll();
    const size_t maxParamNameWidth = getMaxParamNameWidth(logElementNames_);
    logElements_.clear();
    try {
      YAML::Node config = YAML::LoadFile(filename);
      for( size_t i = 0; i < config["log_elements"].size(); ++i) {
        std::string name = config["log_elements"][i]["name"].as<std::string>();
        if(std::find(logElementNames_.begin(), logElementNames_.end(), name) != logElementNames_.end()) {
          logElements_.push_back(std::shared_ptr<LogElement>(new LogElement(name, varsWidget_, paramsGrid_, &getLogElementClient_, &setLogElementClient_, maxParamNameWidth)));
          logElements_.back()->checkBoxIsLogging->setCheckState(config["log_elements"][i]["logged"].as<bool>()?Qt::CheckState::Checked:Qt::CheckState::Unchecked);
          logElements_.back()->comboBoxIsLooping->setCurrentIndex(config["log_elements"][i]["buffer"]["looping"].as<bool>());
          logElements_.back()->spinBoxBufferSize->setValue(config["log_elements"][i]["buffer"]["size"].as<int>());
          logElements_.back()->spinBoxDivider->setValue(config["log_elements"][i]["divider"].as<int>());
          logElements_.back()->labelParamName->setText(QString::fromStdString(name));
        }
        else {
          ROS_WARN_STREAM("Could not load " << name << "from config file. Var not logged.");
        }
      }
      changeAll();
      refreshAll();
      statusMessage("Successfully loaded logger configuration file!", MessageType::SUCCESS, 2.0);
    }
    catch(YAML::Exception & e) {
      ROS_WARN_STREAM("Could not load config file, because exception occurred: "<<e.what());
      statusMessage("Logger configuration file can not be opened!", MessageType::ERROR, 2.0);
      return;
    }
  }
  else {
    statusMessage("Logger configuration file can not be opened!", MessageType::ERROR, 2.0);
  }

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
  for ( std::size_t i = 0; i < logElements_.size(); ++i)
  {
    node["log_elements"][i]["name"] = logElements_[i]->labelParamName->text().toStdString();
    node["log_elements"][i]["logged"] = static_cast<bool>(logElements_[i]->checkBoxIsLogging->checkState());
    node["log_elements"][i]["divider"] = logElements_[i]->spinBoxDivider->value();
    node["log_elements"][i]["action"] = logElements_[i]->comboBoxLogType->currentIndex();
    node["log_elements"][i]["buffer"]["size"] = logElements_[i]->spinBoxBufferSize->value();
    node["log_elements"][i]["buffer"]["looping"] = static_cast<bool>(logElements_[i]->comboBoxIsLooping->currentIndex());
  }

  std::ofstream outfile(filename);
  writeYamlOrderedMaps(outfile, node);
  outfile.close();

  statusMessage("Successfully saved logger configuration file!", MessageType::SUCCESS, 2.0);
}

void SignalLoggerPlugin::shutdownPlugin() {
  getLogElementListClient_.shutdown();
  getLogElementClient_.shutdown();
  setLogElementClient_.shutdown();
  startLoggerClient_.shutdown();
  stopLoggerClient_.shutdown();
  saveLoggerDataClient_.shutdown();
}

void SignalLoggerPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const {
  plugin_settings.setValue("lineEditFilter", varsUi_.lineEditFilter->displayText());
}

void SignalLoggerPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) {
  varsUi_.lineEditFilter->setText(plugin_settings.value("lineEditFilter").toString());
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
  varsUi_.gridLayout->addWidget(paramsWidget_, 10, 1, 1, 1);


  paramsScrollHelperWidget_ = new QWidget(paramsWidget_);
  paramsGrid_= new QGridLayout(paramsScrollHelperWidget_);
  paramsGrid_->setSpacing(6);
  paramsGrid_->setObjectName(QString::fromUtf8("paramsGrid"));

  const size_t maxParamNameWidth = getMaxParamNameWidth(logElementNames_);

  // Create a line for each filtered parameter
  std::string filter = varsUi_.lineEditFilter->text().toStdString();
  for (auto& name : logElementNames_) {
    std::size_t found = name.find(filter);
    if (found!=std::string::npos) {
      logElements_.push_back(std::shared_ptr<LogElement>(new LogElement(name, varsWidget_, paramsGrid_, &getLogElementClient_, &setLogElementClient_, maxParamNameWidth)));
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

PLUGINLIB_DECLARE_CLASS(rqt_signal_logger, SignalLoggerPlugin, SignalLoggerPlugin, rqt_gui_cpp::Plugin)


