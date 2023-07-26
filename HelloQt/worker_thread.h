#ifndef WORKERTHREAD_H
#define WORKERTHREAD_H


#include <QThread>
#include <QString>

class WorkerThread : public QThread
{
    Q_OBJECT

public:
    explicit WorkerThread(QString path, QObject* parent = nullptr);
    void run() override;

signals:
    void sendPath(QString imagePath);

private:
    QString m_path;
};

#endif // WORKERTHREAD_H