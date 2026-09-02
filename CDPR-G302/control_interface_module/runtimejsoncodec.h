#ifndef RUNTIMEJSONCODEC_H
#define RUNTIMEJSONCODEC_H

#include <QJsonArray>
#include <QJsonValue>
#include <QtGlobal>

#include <vector>

// 运行快照所用的基础容器 JSON 转换；不决定快照来源和保存时机。
namespace RuntimeJsonCodec {

QJsonArray toJsonArray(const std::vector<double>& values);
QJsonArray toJsonArray(const std::vector<std::vector<double>>& values);
QJsonArray toJsonArray(const std::vector<int>& values);
QJsonArray toJsonArray(const std::vector<qint64>& values);
qint64 jsonIntFromQuint64(quint64 value);

std::vector<double> fromJsonArray(const QJsonArray& array);
std::vector<std::vector<double>> fromJsonMatrix(const QJsonArray& array);
std::vector<int> fromJsonIntArray(const QJsonArray& array);
std::vector<qint64> fromJsonInt64Array(const QJsonArray& array);
qint64 jsonValueToInt64(const QJsonValue& value, qint64 fallback = -1);

} // namespace RuntimeJsonCodec

#endif // RUNTIMEJSONCODEC_H
