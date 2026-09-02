#include "maintenancepackagevalidator.h"

#include "runtimepathutils.h"

#include <QByteArray>
#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDate>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLocale>
#include <QRegularExpression>
#include <QSaveFile>
#include <QTime>

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>

namespace MaintenancePackageValidator {

QStringList maintenanceUpdatePackageAllowedSuffixes()
{
    return {QStringLiteral("zip"), QStringLiteral("7z"), QStringLiteral("rar"),
            QStringLiteral("exe"), QStringLiteral("msi"), QStringLiteral("bin"),
            QStringLiteral("pkg"), QStringLiteral("json"), QStringLiteral("g302upd")};
}

QString maintenanceJsonString(const QJsonObject& object, std::initializer_list<QString> keys)
{
    for(const QString& key : keys){
        const QJsonValue value = object.value(key);
        if(value.isString()){
            const QString text = value.toString().trimmed();
            if(!text.isEmpty()){
                return text;
            }
        }
        else if(value.isDouble()){
            return QString::number(value.toDouble(), 'f', 0);
        }
    }
    return QString();
}

QString normalizeMaintenancePayloadTargetPath(QString path)
{
    path = path.trimmed();
    path.replace(QChar('\\'), QChar('/'));
    return QDir::cleanPath(path);
}

bool maintenancePayloadTargetPathIsSafe(const QString& path)
{
    const QString normalized = normalizeMaintenancePayloadTargetPath(path);
    return !normalized.isEmpty() &&
            normalized != QStringLiteral(".") &&
            normalized != QStringLiteral("..") &&
            !QDir::isAbsolutePath(normalized) &&
            !normalized.startsWith(QStringLiteral("../")) &&
            !normalized.contains(QStringLiteral("/../"));
}

bool appendMaintenancePayloadSummary(const QJsonValue& value,
                                     const QString& fieldName,
                                     int index,
                                     QStringList& summaries,
                                     QString* errorMessage)
{
    QString targetPath;
    QString sourceName;
    QString operation = QStringLiteral("add_or_replace");
    qint64 declaredSize = -1;

    if(value.isString()){
        targetPath = value.toString();
    }
    else if(value.isObject()){
        const QJsonObject object = value.toObject();
        targetPath = maintenanceJsonString(
                    object,
                    {QStringLiteral("target_path"), QStringLiteral("asset_path"),
                     QStringLiteral("install_path"), QStringLiteral("path"),
                     QStringLiteral("target")});
        sourceName = maintenanceJsonString(
                    object,
                    {QStringLiteral("source_name"), QStringLiteral("file_name"),
                     QStringLiteral("name"), QStringLiteral("source")});
        const QString declaredOperation = maintenanceJsonString(
                    object,
                    {QStringLiteral("operation"), QStringLiteral("action"),
                     QStringLiteral("mode")}).toLower();
        if(!declaredOperation.isEmpty()){
            operation = declaredOperation;
        }
        const QJsonValue sizeValue = object.value(QStringLiteral("size"));
        if(sizeValue.isDouble()){
            declaredSize = static_cast<qint64>(std::llround(sizeValue.toDouble()));
        }
    }
    else{
        if(errorMessage){
            *errorMessage = QStringLiteral("%1[%2] 不是有效的更新内容对象")
                    .arg(fieldName)
                    .arg(index);
        }
        return false;
    }

    targetPath = normalizeMaintenancePayloadTargetPath(targetPath);
    if(!maintenancePayloadTargetPathIsSafe(targetPath)){
        if(errorMessage){
            *errorMessage = QStringLiteral("%1[%2] 的目标路径无效或不安全：%3")
                    .arg(fieldName)
                    .arg(index)
                    .arg(targetPath);
        }
        return false;
    }

    const QStringList allowedOperations = {
        QStringLiteral("add"), QStringLiteral("replace"),
        QStringLiteral("add_or_replace"), QStringLiteral("remove")
    };
    if(!allowedOperations.contains(operation)){
        if(errorMessage){
            *errorMessage = QStringLiteral("%1[%2] 的操作类型无效：%3")
                    .arg(fieldName)
                    .arg(index)
                    .arg(operation);
        }
        return false;
    }

    QString summary = QStringLiteral("%1 %2").arg(operation, targetPath);
    if(!sourceName.trimmed().isEmpty()){
        summary += QStringLiteral(" <= %1").arg(sourceName.trimmed());
    }
    if(declaredSize >= 0){
        summary += QStringLiteral(" (%1 bytes)").arg(declaredSize);
    }
    summaries.append(summary);
    return true;
}

QStringList maintenanceUpdatePackagePayloadSummaries(const QJsonObject& manifest,
                                                     int* payloadCount,
                                                     QString* errorMessage)
{
    QStringList summaries;
    int count = 0;
    const QStringList payloadFields = {
        QStringLiteral("payloads"), QStringLiteral("asset_payloads"),
        QStringLiteral("asset_operations"), QStringLiteral("assets"),
        QStringLiteral("files")
    };
    for(const QString& fieldName : payloadFields){
        const QJsonValue value = manifest.value(fieldName);
        if(value.isUndefined() || value.isNull()){
            continue;
        }
        if(value.isArray()){
            const QJsonArray array = value.toArray();
            for(int index = 0; index < array.size(); ++index){
                if(!appendMaintenancePayloadSummary(array.at(index),
                                                    fieldName,
                                                    index,
                                                    summaries,
                                                    errorMessage)){
                    return QStringList();
                }
                ++count;
            }
        }
        else if(value.isObject() || value.isString()){
            if(!appendMaintenancePayloadSummary(value,
                                                fieldName,
                                                0,
                                                summaries,
                                                errorMessage)){
                return QStringList();
            }
            ++count;
        }
        else{
            if(errorMessage){
                *errorMessage = QStringLiteral("%1 字段不是有效的更新内容列表")
                        .arg(fieldName);
            }
            return QStringList();
        }
    }
    if(payloadCount){
        *payloadCount = count;
    }
    return summaries;
}

QJsonObject sanitizedMaintenanceUpdatePackageManifest(QJsonObject manifest)
{
    const auto sanitizePayloadObject = [](QJsonObject object){
        const QStringList contentKeys = {
            QStringLiteral("content_base64"),
            QStringLiteral("content"),
            QStringLiteral("data")
        };
        for(const QString& key : contentKeys){
            if(object.contains(key)){
                object.remove(key);
                object.insert(QStringLiteral("%1_omitted").arg(key), true);
            }
        }
        return object;
    };

    const QStringList payloadFields = {
        QStringLiteral("payloads"), QStringLiteral("asset_payloads"),
        QStringLiteral("asset_operations"), QStringLiteral("assets"),
        QStringLiteral("files")
    };
    for(const QString& fieldName : payloadFields){
        const QJsonValue value = manifest.value(fieldName);
        if(value.isArray()){
            QJsonArray sanitizedArray;
            const QJsonArray array = value.toArray();
            for(const QJsonValue& item : array){
                sanitizedArray.append(item.isObject() ?
                                          QJsonValue(sanitizePayloadObject(item.toObject())) :
                                          item);
            }
            manifest.insert(fieldName, sanitizedArray);
        }
        else if(value.isObject()){
            manifest.insert(fieldName, sanitizePayloadObject(value.toObject()));
        }
    }
    return sanitizePayloadObject(manifest);
}

QString maintenanceUpdatePackageWriteRootDir()
{
    // 本地更新包只允许覆盖 assets；将其写入安装目录 data/assets，避免普通用户
    // 在 Program Files 下修改程序文件或内置资源。
    return RuntimePathUtils::dataRootDirectory();
}

bool applyMaintenancePayloadObject(const QJsonObject& object,
                                   const QString& fieldName,
                                   int index,
                                   const QString& writeRoot,
                                   QJsonArray& appliedPayloads,
                                   QStringList& errors)
{
    QString targetPath = maintenanceJsonString(
                object,
                {QStringLiteral("target_path"), QStringLiteral("asset_path"),
                 QStringLiteral("install_path"), QStringLiteral("path"),
                 QStringLiteral("target")});
    targetPath = normalizeMaintenancePayloadTargetPath(targetPath);
    if(!maintenancePayloadTargetPathIsSafe(targetPath) ||
            !targetPath.startsWith(QStringLiteral("assets/"))){
        errors << QStringLiteral("%1[%2] 目标路径必须是安全的 assets/ 相对路径：%3")
                  .arg(fieldName)
                  .arg(index)
                  .arg(targetPath);
        return false;
    }

    QString operation = maintenanceJsonString(
                object,
                {QStringLiteral("operation"), QStringLiteral("action"),
                 QStringLiteral("mode")}).toLower();
    if(operation.isEmpty()){
        operation = QStringLiteral("add_or_replace");
    }
    if(operation != QStringLiteral("add") &&
            operation != QStringLiteral("replace") &&
            operation != QStringLiteral("add_or_replace")){
        errors << QStringLiteral("%1[%2] 当前仅支持 add、replace、add_or_replace 资产更新：%3")
                  .arg(fieldName)
                  .arg(index)
                  .arg(operation);
        return false;
    }

    const QString encodedContent = maintenanceJsonString(
                object,
                {QStringLiteral("content_base64"), QStringLiteral("base64")});
    if(encodedContent.isEmpty()){
        errors << QStringLiteral("%1[%2] 未携带 content_base64 内容")
                  .arg(fieldName)
                  .arg(index);
        return false;
    }

    const QByteArray payloadBytes =
            QByteArray::fromBase64(encodedContent.toLatin1());
    const QJsonValue sizeValue = object.value(QStringLiteral("size"));
    if(sizeValue.isDouble()){
        const qint64 declaredSize = static_cast<qint64>(std::llround(sizeValue.toDouble()));
        if(declaredSize != payloadBytes.size()){
            errors << QStringLiteral("%1[%2] 内容大小不一致：声明=%3，实际=%4")
                      .arg(fieldName)
                      .arg(index)
                      .arg(declaredSize)
                      .arg(payloadBytes.size());
            return false;
        }
    }

    const QString expectedSha256 = maintenanceJsonString(
                object,
                {QStringLiteral("sha256"), QStringLiteral("content_sha256")}).toLower();
    const QString actualSha256 = QString::fromLatin1(
                QCryptographicHash::hash(payloadBytes, QCryptographicHash::Sha256).toHex());
    if(!expectedSha256.isEmpty() && expectedSha256 != actualSha256){
        errors << QStringLiteral("%1[%2] SHA256 不一致：声明=%3，实际=%4")
                  .arg(fieldName)
                  .arg(index)
                  .arg(expectedSha256, actualSha256);
        return false;
    }

    const QString targetFilePath = QDir(writeRoot).filePath(targetPath);
    const QFileInfo targetInfo(targetFilePath);
    if(operation == QStringLiteral("add") && targetInfo.exists()){
        errors << QStringLiteral("%1[%2] 目标文件已存在，add 操作未覆盖：%3")
                  .arg(fieldName)
                  .arg(index)
                  .arg(targetFilePath);
        return false;
    }
    if(operation == QStringLiteral("replace") && !targetInfo.exists()){
        errors << QStringLiteral("%1[%2] 目标文件不存在，replace 操作无法执行：%3")
                  .arg(fieldName)
                  .arg(index)
                  .arg(targetFilePath);
        return false;
    }
    if(!QDir().mkpath(targetInfo.absolutePath())){
        errors << QStringLiteral("%1[%2] 无法创建目标目录：%3")
                  .arg(fieldName)
                  .arg(index)
                  .arg(targetInfo.absolutePath());
        return false;
    }

    QSaveFile outputFile(targetFilePath);
    if(!outputFile.open(QIODevice::WriteOnly)){
        errors << QStringLiteral("%1[%2] 无法写入目标文件：%3，原因=%4")
                  .arg(fieldName)
                  .arg(index)
                  .arg(targetFilePath, outputFile.errorString());
        return false;
    }
    if(outputFile.write(payloadBytes) != payloadBytes.size()){
        errors << QStringLiteral("%1[%2] 写入目标文件失败：%3，原因=%4")
                  .arg(fieldName)
                  .arg(index)
                  .arg(targetFilePath, outputFile.errorString());
        outputFile.cancelWriting();
        return false;
    }
    if(!outputFile.commit()){
        errors << QStringLiteral("%1[%2] 保存目标文件失败：%3，原因=%4")
                  .arg(fieldName)
                  .arg(index)
                  .arg(targetFilePath, outputFile.errorString());
        return false;
    }

    QJsonObject item;
    item.insert(QStringLiteral("target_path"), targetPath);
    item.insert(QStringLiteral("target_file"), targetFilePath);
    item.insert(QStringLiteral("operation"), operation);
    item.insert(QStringLiteral("size"), payloadBytes.size());
    item.insert(QStringLiteral("sha256"), actualSha256);
    item.insert(QStringLiteral("kept_previous_data_on_failure"), true);
    appliedPayloads.append(item);
    return true;
}

QJsonObject applyMaintenanceUpdatePackagePayloads(const MaintenanceUpdatePackageMetadata& metadata,
                                                  bool* ok)
{
    QJsonObject report;
    const QString writeRoot = maintenanceUpdatePackageWriteRootDir();
    report.insert(QStringLiteral("write_root"), writeRoot);
    report.insert(QStringLiteral("attempted_at"), QDateTime::currentDateTime().toString(Qt::ISODateWithMs));
    report.insert(QStringLiteral("kept_previous_data_on_failure"), true);

    QJsonArray appliedPayloads;
    QStringList errors;
    int attemptedCount = 0;
    const QStringList payloadFields = {
        QStringLiteral("payloads"), QStringLiteral("asset_payloads"),
        QStringLiteral("asset_operations"), QStringLiteral("assets"),
        QStringLiteral("files")
    };
    for(const QString& fieldName : payloadFields){
        const QJsonValue value = metadata.manifest.value(fieldName);
        if(value.isUndefined() || value.isNull()){
            continue;
        }
        if(value.isArray()){
            const QJsonArray array = value.toArray();
            for(int index = 0; index < array.size(); ++index){
                ++attemptedCount;
                if(!array.at(index).isObject()){
                    errors << QStringLiteral("%1[%2] 不是可应用的资产内容对象")
                              .arg(fieldName)
                              .arg(index);
                    continue;
                }
                applyMaintenancePayloadObject(array.at(index).toObject(),
                                              fieldName,
                                              index,
                                              writeRoot,
                                              appliedPayloads,
                                              errors);
            }
        }
        else if(value.isObject()){
            ++attemptedCount;
            applyMaintenancePayloadObject(value.toObject(),
                                          fieldName,
                                          0,
                                          writeRoot,
                                          appliedPayloads,
                                          errors);
        }
        else{
            ++attemptedCount;
            errors << QStringLiteral("%1 字段不是可应用的资产内容对象").arg(fieldName);
        }
    }

    report.insert(QStringLiteral("attempted_count"), attemptedCount);
    report.insert(QStringLiteral("applied_count"), appliedPayloads.size());
    report.insert(QStringLiteral("applied_payloads"), appliedPayloads);
    report.insert(QStringLiteral("errors"), QJsonArray::fromStringList(errors));
    const bool success = attemptedCount > 0 && errors.isEmpty() &&
            appliedPayloads.size() == attemptedCount;
    report.insert(QStringLiteral("success"), success);
    if(ok){
        *ok = success;
    }
    return report;
}

bool parseMaintenanceVersionComponents(const QString& rawVersion,
                                       QVector<int>& components,
                                       QString* normalized,
                                       QString* errorMessage)
{
    const QRegularExpression expression(
                QStringLiteral("^\\s*[vV]?(\\d+(?:\\.\\d+){1,3})(?:[-+_].*)?\\s*$"));
    const QRegularExpressionMatch match = expression.match(rawVersion.trimmed());
    if(!match.hasMatch()){
        if(errorMessage){
            *errorMessage = QStringLiteral("版本号格式无效，应为 1.2、1.2.3、1.2.3.4 或 v1.2.3");
        }
        return false;
    }

    const QString versionText = match.captured(1);
    const QStringList parts = versionText.split(QStringLiteral("."));
    components.clear();
    components.reserve(parts.size());
    for(const QString& part : parts){
        bool ok = false;
        const qlonglong value = part.toLongLong(&ok);
        if(!ok || value < 0 || value > std::numeric_limits<int>::max()){
            if(errorMessage){
                *errorMessage = QStringLiteral("版本号包含无效数字段：%1").arg(part);
            }
            return false;
        }
        components.push_back(static_cast<int>(value));
    }
    if(normalized){
        *normalized = versionText;
    }
    return true;
}

int compareMaintenanceVersions(const QVector<int>& left, const QVector<int>& right)
{
    const int count = std::max(left.size(), right.size());
    for(int index = 0; index < count; ++index){
        const int leftValue = index < left.size() ? left.at(index) : 0;
        const int rightValue = index < right.size() ? right.at(index) : 0;
        if(leftValue < rightValue){
            return -1;
        }
        if(leftValue > rightValue){
            return 1;
        }
    }
    return 0;
}

QString maintenanceBuildTimestampVersion()
{
    const QString buildDateText = QString::fromLatin1(__DATE__).simplified();
    const QString buildTimeText = QString::fromLatin1(__TIME__);
    const QDate buildDate = QLocale::c().toDate(buildDateText, QStringLiteral("MMM d yyyy"));
    const QTime buildTime = QTime::fromString(buildTimeText, QStringLiteral("HH:mm:ss"));
    if(buildDate.isValid() && buildTime.isValid()){
        return QStringLiteral("%1.%2")
                .arg(buildDate.toString(QStringLiteral("yyyyMMdd")),
                     buildTime.toString(QStringLiteral("HHmmss")));
    }
    return QStringLiteral("1.0.0");
}

QString currentMaintenanceSoftwareVersion(QString* source)
{
    const QString applicationVersion = QCoreApplication::applicationVersion().trimmed();
    QVector<int> parsed;
    QString normalized;
    if(parseMaintenanceVersionComponents(applicationVersion, parsed, &normalized, nullptr)){
        if(source){
            *source = QStringLiteral("QCoreApplication::applicationVersion");
        }
        return normalized;
    }

    if(source){
        *source = QStringLiteral("build_timestamp");
    }
    return maintenanceBuildTimestampVersion();
}

QString extractMaintenanceVersionFromFileName(const QString& fileName)
{
    const QFileInfo info(fileName);
    const QString baseName = info.completeBaseName();
    const QRegularExpression expression(
                QStringLiteral("(?:^|[\\s_\\-])v?(\\d+(?:\\.\\d+){1,3})(?=$|[\\s_\\-])"),
                QRegularExpression::CaseInsensitiveOption);
    const QRegularExpressionMatch match = expression.match(baseName);
    return match.hasMatch() ? match.captured(1) : QString();
}

bool textIndicatesG302Target(const QString& text)
{
    return text.contains(QStringLiteral("G302"), Qt::CaseInsensitive);
}

bool readMaintenanceUpdatePackageManifest(const QString& filePath,
                                          QJsonObject& manifest,
                                          QString& manifestSource,
                                          QString& errorMessage)
{
    const QFileInfo info(filePath);
    const QString suffix = info.suffix().toLower();
    const QStringList jsonLikeSuffixes = {QStringLiteral("json"), QStringLiteral("pkg"),
                                          QStringLiteral("g302upd"), QStringLiteral("bin")};
    if(!jsonLikeSuffixes.contains(suffix)){
        errorMessage = QStringLiteral("该扩展名不按 JSON 清单包读取");
        return false;
    }
    if(info.size() > 4 * 1024 * 1024){
        errorMessage = QStringLiteral("JSON 清单包超过 4MB，未按清单解析");
        return false;
    }

    QFile file(filePath);
    if(!file.open(QIODevice::ReadOnly)){
        errorMessage = QStringLiteral("无法读取更新包清单：%1").arg(file.errorString());
        return false;
    }
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parseError);
    if(parseError.error != QJsonParseError::NoError || !document.isObject()){
        errorMessage = QStringLiteral("更新包不是有效 JSON 清单：%1").arg(parseError.errorString());
        return false;
    }

    manifest = document.object();
    manifestSource = QStringLiteral("package_json_manifest");
    return true;
}

bool extractMaintenanceUpdatePackageMetadata(const QString& filePath,
                                             MaintenanceUpdatePackageMetadata& metadata,
                                             QString& errorMessage)
{
    const QFileInfo info(filePath);
    QJsonObject manifest;
    QString manifestSource;
    QString manifestError;
    if(readMaintenanceUpdatePackageManifest(filePath, manifest, manifestSource, manifestError)){
        metadata.manifest = manifest;
        metadata.manifestSource = manifestSource;
        metadata.packageName = maintenanceJsonString(
                    manifest,
                    {QStringLiteral("package_name"), QStringLiteral("name"), QStringLiteral("software_name")});
        metadata.packageId = maintenanceJsonString(
                    manifest,
                    {QStringLiteral("package_id"), QStringLiteral("id")});
        metadata.packageVersion = maintenanceJsonString(
                    manifest,
                    {QStringLiteral("package_version"), QStringLiteral("version"), QStringLiteral("software_version")});
        metadata.targetSoftware = maintenanceJsonString(
                    manifest,
                    {QStringLiteral("target_software"), QStringLiteral("target"), QStringLiteral("software")});
        metadata.buildTime = maintenanceJsonString(
                    manifest,
                    {QStringLiteral("build_time"), QStringLiteral("build_date"), QStringLiteral("created_at")});
        metadata.releaseNotes = maintenanceJsonString(
                    manifest,
                    {QStringLiteral("release_notes"), QStringLiteral("note"), QStringLiteral("description")});
        metadata.versionSource = QStringLiteral("manifest");

        QString payloadError;
        metadata.payloadSummaries = maintenanceUpdatePackagePayloadSummaries(
                    manifest,
                    &metadata.payloadCount,
                    &payloadError);
        if(!payloadError.isEmpty()){
            errorMessage = QStringLiteral("更新包内容无效：%1").arg(payloadError);
            return false;
        }
    }
    else{
        metadata.manifestError = manifestError;
    }

    if(metadata.packageName.trimmed().isEmpty()){
        metadata.packageName = info.completeBaseName();
    }
    if(metadata.packageId.trimmed().isEmpty()){
        metadata.packageId = info.completeBaseName();
    }
    if(metadata.packageVersion.trimmed().isEmpty()){
        metadata.packageVersion = extractMaintenanceVersionFromFileName(info.fileName());
        if(!metadata.packageVersion.isEmpty()){
            metadata.versionSource = QStringLiteral("file_name");
        }
    }
    if(metadata.targetSoftware.trimmed().isEmpty()){
        const QString targetProbe = QStringLiteral("%1 %2")
                .arg(metadata.packageName, info.fileName());
        if(textIndicatesG302Target(targetProbe)){
            metadata.targetSoftware = QStringLiteral("G302");
        }
    }

    if(metadata.packageVersion.trimmed().isEmpty()){
        errorMessage = QStringLiteral("未识别更新包版本；请在 JSON 清单中提供 package_version，或在文件名中包含 v1.2.3");
        return false;
    }

    QVector<int> parsedVersion;
    QString normalizedVersion;
    QString versionError;
    if(!parseMaintenanceVersionComponents(metadata.packageVersion,
                                          parsedVersion,
                                          &normalizedVersion,
                                          &versionError)){
        errorMessage = QStringLiteral("更新包版本号无效：%1").arg(versionError);
        return false;
    }
    metadata.packageVersion = normalizedVersion;

    if(metadata.targetSoftware.trimmed().isEmpty()){
        errorMessage = QStringLiteral("更新包未声明目标软件；请提供 target_software=G302 或在文件名中包含 G302");
        return false;
    }
    if(!textIndicatesG302Target(metadata.targetSoftware)){
        errorMessage = QStringLiteral("更新包目标软件不匹配：%1").arg(metadata.targetSoftware);
        return false;
    }

    return true;
}

QJsonObject maintenanceUpdatePackageMetadataToJson(const MaintenanceUpdatePackageMetadata& metadata)
{
    QJsonObject object;
    object.insert(QStringLiteral("package_name"), metadata.packageName);
    object.insert(QStringLiteral("package_id"), metadata.packageId);
    object.insert(QStringLiteral("package_version"), metadata.packageVersion);
    object.insert(QStringLiteral("target_software"), metadata.targetSoftware);
    object.insert(QStringLiteral("build_time"), metadata.buildTime);
    object.insert(QStringLiteral("release_notes"), metadata.releaseNotes);
    object.insert(QStringLiteral("payload_count"), metadata.payloadCount);
    object.insert(QStringLiteral("payload_summaries"),
                  QJsonArray::fromStringList(metadata.payloadSummaries));
    object.insert(QStringLiteral("version_source"), metadata.versionSource);
    object.insert(QStringLiteral("manifest_source"), metadata.manifestSource);
    object.insert(QStringLiteral("manifest_error"), metadata.manifestError);
    if(!metadata.manifest.isEmpty()){
        object.insert(QStringLiteral("manifest"),
                      sanitizedMaintenanceUpdatePackageManifest(metadata.manifest));
    }
    return object;
}


} // namespace MaintenancePackageValidator
