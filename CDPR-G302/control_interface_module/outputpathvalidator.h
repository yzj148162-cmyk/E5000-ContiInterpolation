#ifndef OUTPUTPATHVALIDATOR_H
#define OUTPUTPATHVALIDATOR_H

#include <QString>
#include <QStringList>

class QWidget;

// 输出目录/文件路径的规范化、可写性校验、选择对话框及历史数据迁移工具。
namespace OutputPathValidator {

enum class OutputDirectoryValidationError {
    None,
    Empty,
    NotExists,
    NotDirectory,
    NotWritable
};

struct OutputDirectoryValidation {
    OutputDirectoryValidationError error = OutputDirectoryValidationError::None;
    QString cleanPath;

    bool ok() const
    {
        return error == OutputDirectoryValidationError::None;
    }
};

enum class OutputFileValidationError {
    None,
    Empty,
    ParentNotExists,
    ParentNotDirectory,
    ParentNotWritable,
    TargetIsDirectory
};

struct OutputFileValidation {
    OutputFileValidationError error = OutputFileValidationError::None;
    QString cleanPath;
    QString parentPath;

    bool ok() const
    {
        return error == OutputFileValidationError::None;
    }
};

extern const QStringList kInstallationDataSubdirectories;
extern const QStringList kLegacyMigratedDataSubdirectories;

QString normalizedUserPathText(QString path);
QString existingDirectoryForBrowse(const QString& pathText,
                                   const QString& fallbackDir);
OutputDirectoryValidation validateExistingWritableOutputDirectory(
        const QString& dirPath);
OutputFileValidation validateWritableOutputFilePath(const QString& filePath);
bool sameDirectoryPath(const QString& left, const QString& right);
void copyMissingDirectoryTree(const QString& sourcePath,
                              const QString& targetPath,
                              int* copiedFileCount,
                              QStringList* errors);
QString selectOutputDirectoryWithEditableText(QWidget* parent,
                                              const QString& title,
                                              const QString& labelText,
                                              const QString& startDir,
                                              const QString& acceptText);
QString selectOutputFilePathWithEditableText(QWidget* parent,
                                             const QString& title,
                                             const QString& labelText,
                                             const QString& initialFilePath,
                                             const QString& filter,
                                             const QString& acceptText);

} // namespace OutputPathValidator

#endif // OUTPUTPATHVALIDATOR_H
