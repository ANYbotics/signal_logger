#include <math.h>
#include <rqt_signal_logger/BufferIndicator.hpp>

#include <QPainter>
#include <QGradient>
#include <QPaintDevice>
#include <QTimer>

BufferIndicator::
BufferIndicator(QWidget* parent) :
QWidget(parent),
diameter_(5),
nrUnreadElements_(0),
nrTotalElements_(0),
bufferSize_(0),
colorUnread_(QColor("green")),
colorTotal_(QColor("blue")),
alignment_(Qt::AlignCenter)
{
  setDiameter(diameter_);
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
  std::string toolTip = std::string{"Buffer size: "} + std::to_string(bufferSize_)
                      + std::string{"\nTotal items: "} + std::to_string(nrTotalElements_)
                      + std::string{"\nUnread items: "} + std::to_string(nrUnreadElements_);
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

  QPainterPath unreadArc;
  unreadArc.moveTo(center);
  unreadArc.arcTo(boundingRect, 90, - (double)nrUnreadElements_/(double)bufferSize_ * 360.0);

  QPainterPath totalArc;
  totalArc.moveTo(center);
  totalArc.arcTo(boundingRect, 90, - (double)nrTotalElements_/(double)bufferSize_ * 360.0);

  p.setRenderHint(QPainter::Antialiasing, true);
  p.setPen(colorTotal_);
  p.drawPath(totalArc);
  p.setPen(colorUnread_);
  p.drawPath(unreadArc);
}
