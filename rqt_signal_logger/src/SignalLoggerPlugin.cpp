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
  std::string getLogElementListServiceName{"/sl_ros/get_logger_element_names"};
  std::string getParameterServiceName{"/sl_ros/get_logger_element"};
  std::string setParameterListServiceName{"/sl_ros/set_logger_element"};
  std::string startLoggerServiceName{"/sl_ros/start_logger"};
  std::string stopLoggerServiceName{"/sl_ros/stop_logger"};
  std::string saveLoggerDataServiceName{"/sl_ros/save_logger_data"};
  std::string loadLoggerScriptServiceName{"/sl_ros/load_logger_script"};

  // ROS services
  getLoggerElementNamesClient_ = getNodeHandle().serviceClient<signal_logger_msgs::GetLoggerElementNames>(getLogElementListServiceName);
  getLoggerElementClient_ = getNodeHandle().serviceClient<signal_logger_msgs::GetLoggerElement>(getParameterServiceName);
  setLoggerElementClient_ = getNodeHandle().serviceClient<signal_logger_msgs::SetLoggerElement>(setParameterListServiceName);
  startLoggerClient_ = getNodeHandle().serviceClient<std_srvs::Trigger>(startLoggerServiceName);
  stopLoggerClient_ = getNodeHandle().serviceClient<std_srvs::Trigger>(stopLoggerServiceName);
  saveLoggerDataClient_ = getNodeHandle().serviceClient<std_srvs::Trigger>(saveLoggerDataServiceName);
  loadLoggerScriptClient_ = getNodeHandle().serviceClient<signal_logger_msgs::LoadLoggerScript>(loadLoggerScriptServiceName);

}

void SignalLoggerPlugin::changeAll() {
  for (auto& elem : logElements_) {
    elem->pushButtonChangeParamPressed();
  }
}

void SignalLoggerPlugin::refreshAll() {

  signal_logger_msgs::GetLoggerElementNamesRequest req;
  signal_logger_msgs::GetLoggerElementNamesResponse res;

  if (getLoggerElementNamesClient_.call(req,res)) {
    // Update parameter names
    logElementNames_ = res.log_element_names;

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
  if(startLoggerClient_.call(req, res) && res.success) {
    statusMessage("Successfully started logger!", MessageType::SUCCESS, 2.0);
  }
  else {
    statusMessage("Could not start logger!", MessageType::ERROR, 2.0);
  }
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
}

void SignalLoggerPlugin::saveLoggerData() {
  std_srvs::Trigger::Request req;
  std_srvs::Trigger::Response res;
  if(saveLoggerDataClient_.call(req, res) && res.success) {
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
      node["log_elements"][j]["buffer"]["looping"] = static_cast<bool>(logElements_[i]->comboBoxIsLooping->currentIndex());
      j++;
    }
  }

  // If there are logged elements save them to file
  if(j!=0) {
    std::ofstream outfile(filename);
    writeYamlOrderedMaps(outfile, node);
    outfile.close();
  }

  statusMessage("Successfully saved logger configuration file!", MessageType::SUCCESS, 2.0);
}

void SignalLoggerPlugin::shutdownPlugin() {
  getLoggerElementNamesClient_.shutdown();
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

}

void SignalLoggerPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) {
  varsUi_.lineEditFilter->setText(plugin_settings.value("lineEditFilter").toString());
  configureUi_.pathEdit->setText(plugin_settings.value("pathEdit").toString());
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
  varsUi_.gridLayout->addWidget(paramsWidget_, 10, 1, 1, 1);


  paramsScrollHelperWidget_ = new QWidget(paramsWidget_);
  paramsGrid_= new QGridLayout(paramsScrollHelperWidget_);
  paramsGrid_->setSpacing(6);
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


