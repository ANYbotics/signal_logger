#include <math.h>
#include <rqt_signal_logger/BufferIndicator.hpp>

#include <QPainter>
#include <QGradient>
#include <QPaintDevice>
#include <QTimer>

#include <iostream>

BufferIndicator::
BufferIndicator(QWidget* parent) :
QWidget(parent),
diameter_(7),
nrUnreadElements_(0),
nrTotalElements_(0),
bufferSize_(0),
colorUnread_(QColor(85, 117, 168)),
colorTotal_(QColor(170, 196, 237,150)),
alignment_(Qt::AlignCenter)
{
  setDiameter(diameter_);
// Enable this to refresh on mouse over
  this->setMouseTracking(true);
}

BufferIndicator::
~BufferIndicator()
{
}


double BufferIndicator::
diameter() const
{
  return diameter_;
}

void BufferIndicator::
setDiameter(double diameter)
{
  diameter_ = diameter;

  pixX_ = round(double(height())/heightMM());
  pixY_ = round(double(width())/widthMM());

  diamX_ = diameter_*pixX_;
  diamY_ = diameter_*pixY_;

  update();
}

Qt::Alignment BufferIndicator::
alignment() const
{
  return alignment_;
}

void BufferIndicator::
setAlignment(Qt::Alignment alignment)
{
  alignment_ = alignment;

  update();
}
void BufferIndicator::
updateData(const std::size_t nrUnreadElements, std::size_t nrTotalElements, std::size_t bufferSize)
{
  // set sizes
  bufferSize_ = bufferSize;
  nrTotalElements_ = nrTotalElements;
  nrUnreadElements_ = nrUnreadElements;

  // set the tooltip for additional info
  std::string toolTip = std::string{"Buffer size: \t"} + std::to_string(bufferSize_)
  + std::string{"\nTotal items: \t"} + std::to_string(nrTotalElements_)
  + std::string{"\nUnread items: \t"} + std::to_string(nrUnreadElements_);
  this->setToolTip(QString::fromStdString(toolTip));

  // redraw
  update();
}

int BufferIndicator::
heightForWidth(int width) const
{
  return width;
}

QSize BufferIndicator::
sizeHint() const
{
  return QSize(diamX_, diamY_);
}

QSize BufferIndicator::
minimumSizeHint() const
{
  return QSize(diamX_, diamY_);
}

bool BufferIndicator::
event(QEvent *event){
  switch(event->type())
  {
    case QEvent::Enter:
      emit refresh();
      return true;
      break;
    default:
      break;
  }
  return QWidget::event(event);
}

void BufferIndicator::
paintEvent(QPaintEvent *event)
{
  Q_UNUSED(event);

  QPainter p(this);

  QRect geo = geometry();
  int width = geo.width();
  int height = geo.height();

  int x=0, y=0;
  if ( alignment_ & Qt::AlignLeft )
    x = 0;
  else if ( alignment_ & Qt::AlignRight )
    x = width-diamX_;
  else if ( alignment_ & Qt::AlignHCenter )
    x = (width-diamX_)/2;
  else if ( alignment_ & Qt::AlignJustify )
    x = 0;

  if ( alignment_ & Qt::AlignTop )
    y = 0;
  else if ( alignment_ & Qt::AlignBottom )
    y = height-diamY_;
  else if ( alignment_ & Qt::AlignVCenter )
    y = (height-diamY_)/2;

  QPointF topleft(x, y);
  QPointF bottomright(x+diamX_, y+diamY_);
  QPointF center(x+diamX_/2, y+diamY_/2);
  QRectF boundingRect(topleft, bottomright);
  QPointF whiteBoxTopLeft(x+diamX_/3, y+diamY_/3);
  QPointF whiteBoxBottomRight(x+2*diamX_/3, y+2*diamY_/3);
  QRectF whiteBoxBoundingRect(whiteBoxTopLeft, whiteBoxBottomRight);

  QPainterPath unreadArc;
  unreadArc.moveTo(center);
  unreadArc.arcTo(boundingRect, 90, - (double)nrUnreadElements_/(double)bufferSize_ * 360.0);

  QPainterPath totalArc;
  totalArc.moveTo(center);
  totalArc.arcTo(boundingRect, 90, - (double)nrTotalElements_/(double)bufferSize_ * 360.0);

  p.setRenderHint(QPainter::Antialiasing, true);
  p.setPen(Qt::NoPen);

  p.setBrush(QBrush(QColor("white")));
  p.drawEllipse(boundingRect);

  p.drawPath(totalArc);
  p.fillPath (totalArc, QBrush (colorTotal_));

  p.drawPath(unreadArc);
  p.fillPath (unreadArc, QBrush (colorUnread_));

  p.setBrush(QBrush(QWidget::palette().color(QWidget::backgroundRole())));
  p.drawEllipse(whiteBoxBoundingRect);




}
