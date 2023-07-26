#ifndef ROBOT_MAP_WIDGET_H
#define ROBOT_MAP_WIDGET_H

#include <QWidget>
#include <QPointF>
#include <QList>
#include <QString>

class RobotMapWidget : public QWidget
{
    Q_OBJECT

public:
    RobotMapWidget(QWidget* parent = nullptr);
    ~RobotMapWidget();
    void addPoint(float x, float y);

public slots:
    void loadPointsFromCSV(const QString& filename);



protected:
    void paintEvent(QPaintEvent*) override;

private:
    QList<QPointF> points;
    double scaleFactorX;
    double scaleFactorY;
    double minX;
    double minY;
    double maxX;
    double maxY;
    void scalePoints();
};

#endif // ROBOT_MAP_WIDGET_H
