#include "structuredfaultlogwriter.h"

#include <QByteArray>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QSaveFile>
#include <QThread>

#include <utility>

#pragma execution_character_set("utf-8")

namespace {
constexpr qint64 kStructuredFaultLogMaxBytes = 16 * 1024 * 1024;
constexpr int kStructuredFaultLogArchiveCount = 4;
}

StructuredFaultLogWriter::StructuredFaultLogWriter(QString outputFilePath,
                                                   QObject* parent)
    : QObject(parent),
      filePath(std::move(outputFilePath))
{
}

QString StructuredFaultLogWriter::rotatedFilePath(int index) const
{
    const QFileInfo info(filePath);
    return QDir(info.absolutePath()).filePath(
                QStringLiteral("%1.%2.%3")
                .arg(info.completeBaseName())
                .arg(index)
                .arg(info.suffix()));
}

QString StructuredFaultLogWriter::uniqueLegacyBackupPath() const
{
    const QFileInfo info(filePath);
    const QString stamp = QDateTime::currentDateTime().toString(
                QStringLiteral("yyyyMMdd_HHmmss_zzz"));
    return QDir(info.absolutePath()).filePath(
                QStringLiteral("%1.legacy_%2.json")
                .arg(info.completeBaseName(), stamp));
}

void StructuredFaultLogWriter::reportError(const QString& message)
{
    if(message.isEmpty() || message == lastReportedError){
        return;
    }
    lastReportedError = message;
    emit writeError(message);
}

bool StructuredFaultLogWriter::convertLegacyObjectFile()
{
    QFile legacyFile(filePath);
    if(!legacyFile.open(QIODevice::ReadOnly | QIODevice::Text)){
        reportError(QStringLiteral("无法读取旧结构化故障日志 %1：%2")
                    .arg(QDir::toNativeSeparators(filePath),
                         legacyFile.errorString()));
        return false;
    }
    const QByteArray legacyBytes = legacyFile.readAll();
    legacyFile.close();

    QJsonParseError parseError;
    const QJsonDocument legacyDocument =
            QJsonDocument::fromJson(legacyBytes, &parseError);
    if(parseError.error != QJsonParseError::NoError ||
            !legacyDocument.isObject()){
        reportError(QStringLiteral("旧结构化故障日志无法转换为JSONL，已保留原文件：%1")
                    .arg(parseError.errorString()));
        return false;
    }

    const QJsonArray records = legacyDocument.object()
            .value(QStringLiteral("records")).toArray();
    const QString backupPath = uniqueLegacyBackupPath();
    if(!QFile::copy(filePath, backupPath)){
        reportError(QStringLiteral("旧结构化故障日志备份失败，未执行转换：%1")
                    .arg(QDir::toNativeSeparators(backupPath)));
        return false;
    }

    QSaveFile convertedFile(filePath);
    if(!convertedFile.open(QIODevice::WriteOnly | QIODevice::Text)){
        reportError(QStringLiteral("无法创建JSONL结构化故障日志 %1：%2")
                    .arg(QDir::toNativeSeparators(filePath),
                         convertedFile.errorString()));
        return false;
    }
    for(const QJsonValue& value : records){
        if(!value.isObject()){
            continue;
        }
        QByteArray line = QJsonDocument(value.toObject()).toJson(
                    QJsonDocument::Compact);
        line.append('\n');
        if(convertedFile.write(line) != line.size()){
            convertedFile.cancelWriting();
            reportError(QStringLiteral("旧结构化故障日志转换写入失败：%1")
                        .arg(convertedFile.errorString()));
            return false;
        }
    }
    if(!convertedFile.commit()){
        reportError(QStringLiteral("旧结构化故障日志转换提交失败：%1")
                    .arg(convertedFile.errorString()));
        return false;
    }
    return true;
}

bool StructuredFaultLogWriter::prepareJsonLinesFile()
{
    if(prepared){
        return true;
    }
    const QFileInfo info(filePath);
    if(!QDir().mkpath(info.absolutePath())){
        reportError(QStringLiteral("无法创建结构化故障日志目录：%1")
                    .arg(QDir::toNativeSeparators(info.absolutePath())));
        return false;
    }

    QFile existing(filePath);
    if(existing.exists()){
        if(!existing.open(QIODevice::ReadOnly | QIODevice::Text)){
            reportError(QStringLiteral("无法检查结构化故障日志格式 %1：%2")
                        .arg(QDir::toNativeSeparators(filePath),
                             existing.errorString()));
            return false;
        }
        QByteArray firstNonEmptyLine;
        while(!existing.atEnd() && firstNonEmptyLine.isEmpty()){
            firstNonEmptyLine = existing.readLine().trimmed();
        }
        existing.close();
        // 旧实现使用Indented整体JSON，首个非空行严格为“{”；真正JSONL的
        // 每个对象均在同一行。只在日志线程首次写入时执行一次后台转换。
        if(firstNonEmptyLine == QByteArray("{") &&
                !convertLegacyObjectFile()){
            return false;
        }
    }
    prepared = true;
    lastReportedError.clear();
    return true;
}

bool StructuredFaultLogWriter::rotateIfNeeded(qint64 incomingBytes)
{
    const QFileInfo currentInfo(filePath);
    if(!currentInfo.exists() ||
            currentInfo.size() + incomingBytes <=
                kStructuredFaultLogMaxBytes){
        return true;
    }

    const QString oldestPath = rotatedFilePath(
                kStructuredFaultLogArchiveCount);
    if(QFileInfo::exists(oldestPath) && !QFile::remove(oldestPath)){
        reportError(QStringLiteral("无法删除最旧结构化故障日志归档：%1")
                    .arg(QDir::toNativeSeparators(oldestPath)));
        return false;
    }
    for(int index = kStructuredFaultLogArchiveCount - 1;
        index >= 1;
        --index){
        const QString source = rotatedFilePath(index);
        if(!QFileInfo::exists(source)){
            continue;
        }
        const QString target = rotatedFilePath(index + 1);
        if(!QFile::rename(source, target)){
            reportError(QStringLiteral("结构化故障日志归档轮转失败：%1 -> %2")
                        .arg(QDir::toNativeSeparators(source),
                             QDir::toNativeSeparators(target)));
            return false;
        }
    }
    if(!QFile::rename(filePath, rotatedFilePath(1))){
        reportError(QStringLiteral("当前结构化故障日志轮转失败：%1")
                    .arg(QDir::toNativeSeparators(filePath)));
        return false;
    }
    return true;
}

void StructuredFaultLogWriter::appendRecord(QJsonObject record)
{
    if(stopping || record.isEmpty() || !prepareJsonLinesFile()){
        return;
    }

    QByteArray line = QJsonDocument(record).toJson(QJsonDocument::Compact);
    line.append('\n');
    if(!rotateIfNeeded(line.size())){
        return;
    }

    QFile output(filePath);
    if(!output.open(QIODevice::WriteOnly |
                    QIODevice::Append |
                    QIODevice::Text)){
        reportError(QStringLiteral("无法追加结构化故障日志 %1：%2")
                    .arg(QDir::toNativeSeparators(filePath),
                         output.errorString()));
        return;
    }
    const qint64 written = output.write(line);
    const bool flushed = output.flush();
    output.close();
    if(written != line.size() || !flushed){
        reportError(QStringLiteral("结构化故障日志追加不完整：期望%1 B，实际%2 B")
                    .arg(line.size())
                    .arg(written));
        return;
    }
    lastReportedError.clear();
}

void StructuredFaultLogWriter::finish()
{
    if(stopping){
        return;
    }
    stopping = true;
    emit finished();
    QThread::currentThread()->quit();
}
