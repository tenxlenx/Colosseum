#include "worker_thread.h"
#include <QPixmap>
#include <QDir>


WorkerThread::WorkerThread(QString path, QObject* parent)
    : QThread(parent), m_path(std::move(path)) {}

void WorkerThread::run()
{
    while (!isInterruptionRequested()) {
        QDir dir(m_path);
        QStringList images = dir.entryList(QStringList() << "MovieFrame*.png", QDir::Files, QDir::Name);

        if (!images.isEmpty()) {
            QString imagePath = m_path + "/" + images.last();
            if (QFile::exists(imagePath)) {
                emit sendPath(imagePath);
            }
        }

        msleep(33); // around 30 FPS
    }
}