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
#include <rqt_signal_logger/SignalLoggerPlugin.hpp>
#include <ros/package.h>

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
    widget_(0),
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

    // create the main widget, set it up and add it to the user interface
    widget_ = new QWidget();
    ui_.setupUi(widget_);
    context.addWidget(widget_);

    /******************************
     * Connect ui forms to actions *
     ******************************/
    connect(ui_.pushButtonRefreshAll, SIGNAL(pressed()), this, SLOT(refreshAll()));
    connect(ui_.lineEditFilter, SIGNAL(returnPressed()), this ,SLOT(refreshAll()));
    connect(ui_.pushButtonChangeAll, SIGNAL(pressed()), this, SLOT(changeAll()));
    connect(this, SIGNAL(parametersChanged()), this, SLOT(drawParamList()));
    /******************************/
    //#Fixme read from param server
    std::string getLogElementListServiceName{"/get_log_element_list"};
    std::string getParameterServiceName{"/get_log_element"};
    std::string setParameterListServiceName{"/set_log_element"};

    // ROS services
    getLogElementList_ = getNodeHandle().serviceClient<signal_logger_msgs::GetLoggerInfo>(getLogElementListServiceName);
    getLogElementClient_ = getNodeHandle().serviceClient<signal_logger_msgs::GetLoggerElement>(getParameterServiceName);
    setLogElementClient_ = getNodeHandle().serviceClient<signal_logger_msgs::SetLoggerElement>(setParameterListServiceName);
 }



void SignalLoggerPlugin::changeAll() {
  for (auto& elem : logElements_) {
    elem->pushButtonChangeParamPressed();
  }
}

void SignalLoggerPlugin::refreshAll() {

  signal_logger_msgs::GetLoggerInfo::Request req;
  signal_logger_msgs::GetLoggerInfo::Response res;

  if (getLogElementList_.call(req,res)) {
    // Update parameter names
    logElementNames_ = res.active_log_elements;
    for(auto name: res.inactive_log_elements)
      logElementNames_.push_back(name);

    // Sort names alphabetically.
    std::sort(logElementNames_.begin(), logElementNames_.end(), compareNoCase );

    // Update GUI
    emit parametersChanged();
  }
  else {
    ROS_WARN("Could not get parameter list!");
  }
}

void SignalLoggerPlugin::shutdownPlugin() {
  getLogElementList_.shutdown();
  getLogElementClient_.shutdown();
  setLogElementClient_.shutdown();
}

void SignalLoggerPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const {
  plugin_settings.setValue("lineEditFilter", ui_.lineEditFilter->displayText());
}

void SignalLoggerPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) {
  ui_.lineEditFilter->setText(plugin_settings.value("lineEditFilter").toString());
}

void SignalLoggerPlugin::drawParamList() {

  logElements_.clear();

  if (paramsWidget_) {
    // delete widget
    delete paramsWidget_->layout();
    delete paramsScrollHelperWidget_->layout();
    ui_.gridLayout->removeWidget(paramsWidget_);
    delete paramsWidget_;
  }

  paramsWidget_ = new QWidget();
  paramsWidget_->setObjectName(QString::fromUtf8("paramsWidget"));
  ui_.gridLayout->addWidget(paramsWidget_, 10, 1, 1, 1);


  paramsScrollHelperWidget_ = new QWidget(paramsWidget_);
  paramsGrid_= new QGridLayout(paramsScrollHelperWidget_);
  paramsGrid_->setSpacing(6);
  paramsGrid_->setObjectName(QString::fromUtf8("paramsGrid"));

  const size_t maxParamNameWidth = getMaxParamNameWidth(logElementNames_);

  // Create a line for each filtered parameter
  std::string filter = ui_.lineEditFilter->text().toStdString();
  for (auto& name : logElementNames_) {
    std::size_t found = name.find(filter);
    if (found!=std::string::npos) {
      logElements_.push_back(std::shared_ptr<LogElement>(new LogElement(name, widget_, paramsGrid_, &getLogElementClient_, &setLogElementClient_, maxParamNameWidth)));
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

PLUGINLIB_DECLARE_CLASS(rqt_signal_logger, SignalLoggerPlugin, SignalLoggerPlugin, rqt_gui_cpp::Plugin)


