#pragma once

#include <QtDesigner/QtDesigner>
#include <QWidget>

class QDESIGNER_WIDGET_EXPORT BufferIndicator : public QWidget
{
	Q_OBJECT

	Q_PROPERTY(double diameter READ diameter WRITE setDiameter) // mm
	Q_PROPERTY(Qt::Alignment alignment READ alignment WRITE setAlignment)

public:
	explicit BufferIndicator(QWidget* parent=0);
	~BufferIndicator();

	double diameter() const;
	void setDiameter(double diameter);

	Qt::Alignment alignment() const;
	void setAlignment(Qt::Alignment alignment);

public slots:
	void updateData(const std::size_t nrUnreadElements, std::size_t nrTotalElements, std::size_t bufferSize);

signals:
  void refresh();

public:
	int heightForWidth(int width) const;
	QSize sizeHint() const;
	QSize minimumSizeHint() const;

protected:
	void paintEvent(QPaintEvent* event);
	 bool event(QEvent *event);


private:
	double diameter_;
	std::size_t nrUnreadElements_;
	std::size_t nrTotalElements_;
	std::size_t bufferSize_;
	QColor colorUnread_;
  QColor colorTotal_;
	Qt::Alignment alignment_;

	//
	// Pixels per mm for x and y...
	//
	int pixX_, pixY_;

	//
	// Scaled values for x and y diameter.
	//
	int diamX_, diamY_;
};
