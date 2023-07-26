
#include <game_view_widget.h>
#include <iostream>
#include <QDir>
#include <QHBoxLayout>
#include <filesystem>

void deleteImageFilesInFolder(QString path)
{
    QDir dir(path);

    dir.setNameFilters(QStringList() << "*.png" << "*.jpg");
    dir.setFilter(QDir::Files);

    foreach (QString dirFile, dir.entryList()) {
        dir.remove(dirFile);
    }
}

namespace fs = std::filesystem;
std::string find_screenshot_dir(const fs::path& root_path)
{
    if (!fs::exists(root_path) || !fs::is_directory(root_path)) {
        std::cerr << "Invalid path: " << root_path.string() << '\n';
        return "";
    }

    for (const auto& entry : fs::recursive_directory_iterator(root_path)) {
        if (entry.is_directory()) {
            auto path = entry.path();
            if (path.filename() == "Windows" && path.parent_path().filename() == "Screenshots") {
                return path.string();
            }
        }
    }
    std::cerr << "'Screenshots/Windows' directory not found in " << root_path.string() << '\n';
    return "";
}

GameViewWidget::GameViewWidget(QWidget* parent, QLabel* gameImageViewLabel)
    : QWidget(parent), imageLabel(gameImageViewLabel)
{

    


}

void GameViewWidget::startDisplaying(std::string mapName)
{
    // find screenshot folder
    screenshot_path = find_screenshot_dir(mapName).c_str();

    // delete previous images in the folder 
    deleteImageFilesInFolder(screenshot_path);

    m_thread = new WorkerThread(screenshot_path, this);
    connect(m_thread, &WorkerThread::sendPath, this, &GameViewWidget::displayImage);
    m_thread->start();
}

GameViewWidget::~GameViewWidget()
{
    m_thread->requestInterruption();
    m_thread->wait();
    delete m_thread;
    
}

void GameViewWidget::displayImage(QString imagePath)
{
    QPixmap pixmap(imagePath);
    imageLabel->setPixmap(pixmap.scaled(imageLabel->width(), imageLabel->height(), Qt::KeepAspectRatio));
}