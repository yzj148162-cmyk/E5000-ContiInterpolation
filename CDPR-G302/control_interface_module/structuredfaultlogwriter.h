#ifndef STRUCTUREDFAULTLOGWRITER_H
#define STRUCTUREDFAULTLOGWRITER_H

#include <QObject>
#include <QJsonObject>
#include <QString>

#pragma execution_character_set("utf-8")

// 结构化故障日志的唯一写入端。对象固定运行在独立日志线程中，Qt 事件队列
// 提供串行顺序；GUI 和安全停机链路只排队，不等待文件系统。
class StructuredFaultLogWriter : public QObject
{
    Q_OBJECT

public:
    explicit StructuredFaultLogWriter(QString filePath,
                                      QObject* parent = nullptr);

public slots:
    void appendRecord(QJsonObject record);
    void finish();

signals:
    void writeError(const QString& message);
    void finished();

private:
    bool prepareJsonLinesFile();
    bool convertLegacyObjectFile();
    bool rotateIfNeeded(qint64 incomingBytes);
    QString rotatedFilePath(int index) const;
    QString uniqueLegacyBackupPath() const;
    void reportError(const QString& message);

    QString filePath;
    bool prepared = false;
    bool stopping = false;
    QString lastReportedError;
};

#endif // STRUCTUREDFAULTLOGWRITER_H
