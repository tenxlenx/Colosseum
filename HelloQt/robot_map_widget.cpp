#include "robot_map_widget.h"

#include <QPainter>
#include <QFile>
#include <QTextStream>
#include <QStringList>
#include <algorithm>
#include "common/common_utils/Utils.hpp"
#include "common/Common.hpp"
#include "common/common_utils/FileSystem.hpp"


RobotMapWidget::RobotMapWidget(QWidget* parent) : QWidget(parent) {}

RobotMapWidget::~RobotMapWidget() = default;

void RobotMapWidget::addPoint(float x, float y)
{
    QPointF point(x, y); // Assuming X and Y axes are the ones needed
    points.append(point);
    scalePoints();
    this->update();
}

void RobotMapWidget::loadPointsFromCSV(const QString& filename)
{
    QFile file(filename);
    if (file.open(QIODevice::ReadOnly)) {
        QTextStream in(&file);
        while (!in.atEnd()) {
            QString line = in.readLine();
            QStringList fields = line.split(",");
            if (fields.size() >= 2) {
                points.append(QPointF(fields[1].toDouble(), fields[2].toDouble()));
            }
        }
        file.close();
        scalePoints();
        this->update();
    }
}

void RobotMapWidget::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    painter.setPen(QPen(Qt::blue, 5));
    for (const auto& point : points) {
        QPointF scaledPoint((point.x() - minX) * scaleFactorX, (point.y() - minY) * scaleFactorY);
        painter.drawPoint(scaledPoint);
    }
}

void RobotMapWidget::scalePoints()
{
    double maxX = std::max_element(points.begin(), points.end(), [](const QPointF& a, const QPointF& b) { return a.x() < b.x(); })->x();
    double maxY = std::max_element(points.begin(), points.end(), [](const QPointF& a, const QPointF& b) { return a.y() < b.y(); })->y();

    minX = std::min_element(points.begin(), points.end(), [](const QPointF& a, const QPointF& b) { return a.x() < b.x(); })->x();
    minY = std::min_element(points.begin(), points.end(), [](const QPointF& a, const QPointF& b) { return a.y() < b.y(); })->y();

    scaleFactorX = this->width() / (maxX - minX);
    scaleFactorY = this->height() / (maxY - minY);
}
