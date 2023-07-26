#ifndef GAME_VIEW_WIDGET_H
#define GAME_VIEW_WIDGET_H

#include <QWidget>
#include <QLabel>
#include <QTimer>
#include <QFileSystemWatcher>
#include "worker_thread.h"


class GameViewWidget : public QWidget
{
    Q_OBJECT
public:
    explicit GameViewWidget(QWidget* parent = nullptr, QLabel *gameImageViewLabel = nullptr);
    void startDisplaying(std::string map_name);
    ~GameViewWidget();

public slots:
    void displayImage(QString imagePath);

private:
    QLabel* imageLabel;
    WorkerThread* m_thread;
    QString screenshot_path;
};

#endif // GAME_VIEW_WIDGET_H