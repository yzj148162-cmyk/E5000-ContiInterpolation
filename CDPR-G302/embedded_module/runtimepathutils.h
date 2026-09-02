#ifndef RUNTIMEPATHUTILS_H
#define RUNTIMEPATHUTILS_H

#include <QCoreApplication>
#include <QDir>
#include <QString>

// 发布版的可写数据固定放在安装目录下的 data 子目录。安装器只给该目录授予
// 普通用户修改权限，程序文件、DLL 和内置资源目录仍保持只读。
namespace RuntimePathUtils {

inline QString installationDirectory()
{
    const QString applicationDirectory = QCoreApplication::applicationDirPath();
    return applicationDirectory.isEmpty() ? QDir::currentPath() : applicationDirectory;
}

inline QString dataRootDirectory()
{
    return QDir(installationDirectory()).filePath(QStringLiteral("data"));
}

inline QString dataPath(const QString& relativePath)
{
    return relativePath.trimmed().isEmpty() ?
                dataRootDirectory() :
                QDir(dataRootDirectory()).filePath(relativePath);
}

} // namespace RuntimePathUtils

#endif // RUNTIMEPATHUTILS_H
