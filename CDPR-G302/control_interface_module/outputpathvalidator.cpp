#include "outputpathvalidator.h"

#include <QCoreApplication>
#include <QDialog>
#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

namespace OutputPathValidator {

const QStringList kInstallationDataSubdirectories = {
    QStringLiteral("calibration_records"),
    QStringLiteral("config_profiles"),
    QStringLiteral("outputmsg"),
    QStringLiteral("test_evidence"),
    QStringLiteral("force_pid_records"),
    QStringLiteral("udp_received_trajectories"),
    QStringLiteral("assets")
};

// 内置 assets 不能作为旧版运行数据迁移：首次启动若把安装目录 assets 复制到
// data/assets，后续安装器更新后的内置资源会被旧副本覆盖。
const QStringList kLegacyMigratedDataSubdirectories = {
    QStringLiteral("calibration_records"),
    QStringLiteral("config_profiles"),
    QStringLiteral("outputmsg"),
    QStringLiteral("test_evidence"),
    QStringLiteral("force_pid_records"),
    QStringLiteral("udp_received_trajectories")
};

QString normalizedUserPathText(QString path)
{
    path = path.trimmed();
    if(path.size() >= 2 &&
            ((path.startsWith(QLatin1Char('"')) &&
              path.endsWith(QLatin1Char('"'))) ||
             (path.startsWith(QLatin1Char('\'')) &&
              path.endsWith(QLatin1Char('\''))))){
        path = path.mid(1, path.size() - 2).trimmed();
    }
    return path.isEmpty() ? QString() : QDir::cleanPath(path);
}

QString existingDirectoryForBrowse(const QString& pathText,
                                   const QString& fallbackDir)
{
    const QString cleanPath = normalizedUserPathText(pathText);
    if(!cleanPath.isEmpty()){
        const QFileInfo info(cleanPath);
        if(info.exists() && info.isDir()){
            return info.absoluteFilePath();
        }
        const QFileInfo parentInfo(info.absolutePath());
        if(parentInfo.exists() && parentInfo.isDir()){
            return parentInfo.absoluteFilePath();
        }
    }

    const QString cleanFallback = normalizedUserPathText(fallbackDir);
    const QFileInfo fallbackInfo(cleanFallback);
    if(fallbackInfo.exists() && fallbackInfo.isDir()){
        return fallbackInfo.absoluteFilePath();
    }
    return QCoreApplication::applicationDirPath();
}

OutputDirectoryValidation validateExistingWritableOutputDirectory(
        const QString& dirPath)
{
    OutputDirectoryValidation result;
    result.cleanPath = normalizedUserPathText(dirPath);
    if(result.cleanPath.isEmpty()){
        result.error = OutputDirectoryValidationError::Empty;
        return result;
    }

    const QFileInfo dirInfo(result.cleanPath);
    if(!dirInfo.exists()){
        result.error = OutputDirectoryValidationError::NotExists;
        return result;
    }
    if(!dirInfo.isDir()){
        result.error = OutputDirectoryValidationError::NotDirectory;
        return result;
    }
    if(!dirInfo.isWritable()){
        result.error = OutputDirectoryValidationError::NotWritable;
        return result;
    }
    return result;
}

OutputFileValidation validateWritableOutputFilePath(const QString& filePath)
{
    OutputFileValidation result;
    result.cleanPath = normalizedUserPathText(filePath);
    if(result.cleanPath.isEmpty()){
        result.error = OutputFileValidationError::Empty;
        return result;
    }

    const QFileInfo fileInfo(result.cleanPath);
    if(fileInfo.exists() && fileInfo.isDir()){
        result.parentPath = fileInfo.absoluteFilePath();
        result.error = OutputFileValidationError::TargetIsDirectory;
        return result;
    }

    result.parentPath = fileInfo.absolutePath();
    const QFileInfo parentInfo(result.parentPath);
    if(!parentInfo.exists()){
        result.error = OutputFileValidationError::ParentNotExists;
        return result;
    }
    if(!parentInfo.isDir()){
        result.error = OutputFileValidationError::ParentNotDirectory;
        return result;
    }
    if(!parentInfo.isWritable()){
        result.error = OutputFileValidationError::ParentNotWritable;
        return result;
    }
    return result;
}

bool sameDirectoryPath(const QString& left, const QString& right)
{
    return QDir::cleanPath(QFileInfo(left).absoluteFilePath()) ==
            QDir::cleanPath(QFileInfo(right).absoluteFilePath());
}

void copyMissingDirectoryTree(const QString& sourcePath,
                              const QString& targetPath,
                              int* copiedFileCount,
                              QStringList* errors)
{
    const QDir sourceDir(sourcePath);
    if(!sourceDir.exists() || sameDirectoryPath(sourcePath, targetPath)){
        return;
    }

    QDirIterator iterator(sourcePath,
                          QDir::Files | QDir::NoSymLinks,
                          QDirIterator::Subdirectories);
    while(iterator.hasNext()){
        const QString sourceFilePath = iterator.next();
        const QString relativePath = sourceDir.relativeFilePath(sourceFilePath);
        const QString targetFilePath = QDir(targetPath).filePath(relativePath);
        if(QFileInfo::exists(targetFilePath)){
            continue;
        }

        const QString targetParentPath = QFileInfo(targetFilePath).absolutePath();
        if(!QDir().mkpath(targetParentPath)){
            if(errors){
                errors->append(QStringLiteral("无法创建迁移目录 %1")
                               .arg(targetParentPath));
            }
            continue;
        }
        if(!QFile::copy(sourceFilePath, targetFilePath)){
            if(errors){
                errors->append(QStringLiteral("无法迁移历史文件 %1")
                               .arg(sourceFilePath));
            }
            continue;
        }
        if(copiedFileCount){
            ++(*copiedFileCount);
        }
    }
}

QString selectOutputDirectoryWithEditableText(QWidget* parent,
                                              const QString& title,
                                              const QString& labelText,
                                              const QString& startDir,
                                              const QString& acceptText)
{
    QDialog dialog(parent);
    dialog.setWindowTitle(title);
    dialog.setMinimumWidth(720);

    QVBoxLayout* layout = new QVBoxLayout(&dialog);
    QLabel* label = new QLabel(labelText, &dialog);
    layout->addWidget(label);

    QHBoxLayout* pathLayout = new QHBoxLayout();
    QLineEdit* pathEdit = new QLineEdit(QDir::toNativeSeparators(startDir), &dialog);
    pathEdit->setObjectName(QStringLiteral("outputDirectoryLineEdit"));
    pathLayout->addWidget(pathEdit, 1);
    QPushButton* browseButton = new QPushButton(QStringLiteral("浏览..."), &dialog);
    pathLayout->addWidget(browseButton);
    layout->addLayout(pathLayout);

    QHBoxLayout* actionLayout = new QHBoxLayout();
    actionLayout->addStretch();
    QPushButton* acceptButton = new QPushButton(acceptText, &dialog);
    QPushButton* cancelButton = new QPushButton(QStringLiteral("取消"), &dialog);
    acceptButton->setDefault(true);
    actionLayout->addWidget(acceptButton);
    actionLayout->addWidget(cancelButton);
    layout->addLayout(actionLayout);

    QObject::connect(browseButton, &QPushButton::clicked, &dialog,
                     [&dialog, title, pathEdit, startDir](){
        const QString browseStart =
                existingDirectoryForBrowse(pathEdit->text(), startDir);
        const QString selectedDir = QFileDialog::getExistingDirectory(
                    &dialog,
                    title,
                    browseStart,
                    QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
        if(!selectedDir.isEmpty()){
            pathEdit->setText(QDir::toNativeSeparators(QDir::cleanPath(selectedDir)));
        }
    });
    QObject::connect(acceptButton, &QPushButton::clicked, &dialog, &QDialog::accept);
    QObject::connect(cancelButton, &QPushButton::clicked, &dialog, &QDialog::reject);

    pathEdit->setFocus();
    pathEdit->selectAll();
    if(dialog.exec() != QDialog::Accepted){
        return QString();
    }
    return normalizedUserPathText(pathEdit->text());
}

QString selectOutputFilePathWithEditableText(QWidget* parent,
                                             const QString& title,
                                             const QString& labelText,
                                             const QString& initialFilePath,
                                             const QString& filter,
                                             const QString& acceptText)
{
    QDialog dialog(parent);
    dialog.setWindowTitle(title);
    dialog.setMinimumWidth(760);

    QVBoxLayout* layout = new QVBoxLayout(&dialog);
    QLabel* label = new QLabel(labelText, &dialog);
    layout->addWidget(label);

    QHBoxLayout* pathLayout = new QHBoxLayout();
    QLineEdit* pathEdit = new QLineEdit(QDir::toNativeSeparators(initialFilePath), &dialog);
    pathEdit->setObjectName(QStringLiteral("outputFileLineEdit"));
    pathLayout->addWidget(pathEdit, 1);
    QPushButton* browseButton = new QPushButton(QStringLiteral("浏览..."), &dialog);
    pathLayout->addWidget(browseButton);
    layout->addLayout(pathLayout);

    QHBoxLayout* actionLayout = new QHBoxLayout();
    actionLayout->addStretch();
    QPushButton* acceptButton = new QPushButton(acceptText, &dialog);
    QPushButton* cancelButton = new QPushButton(QStringLiteral("取消"), &dialog);
    acceptButton->setDefault(true);
    actionLayout->addWidget(acceptButton);
    actionLayout->addWidget(cancelButton);
    layout->addLayout(actionLayout);

    QObject::connect(browseButton, &QPushButton::clicked, &dialog,
                     [&dialog, title, pathEdit, initialFilePath, filter](){
        const QString currentPath = normalizedUserPathText(pathEdit->text());
        const QFileInfo currentInfo(currentPath.isEmpty() ? initialFilePath : currentPath);
        const QString browseDir =
                existingDirectoryForBrowse(currentInfo.absolutePath(),
                                           QFileInfo(initialFilePath).absolutePath());
        const QString suggestedFileName = currentInfo.fileName().isEmpty() ?
                    QFileInfo(initialFilePath).fileName() :
                    currentInfo.fileName();
        const QString selectedPath = QFileDialog::getSaveFileName(
                    &dialog,
                    title,
                    QDir(browseDir).filePath(suggestedFileName),
                    filter);
        if(!selectedPath.isEmpty()){
            pathEdit->setText(QDir::toNativeSeparators(QDir::cleanPath(selectedPath)));
        }
    });
    QObject::connect(acceptButton, &QPushButton::clicked, &dialog, &QDialog::accept);
    QObject::connect(cancelButton, &QPushButton::clicked, &dialog, &QDialog::reject);

    pathEdit->setFocus();
    pathEdit->selectAll();
    if(dialog.exec() != QDialog::Accepted){
        return QString();
    }
    return normalizedUserPathText(pathEdit->text());
}

} // namespace OutputPathValidator
