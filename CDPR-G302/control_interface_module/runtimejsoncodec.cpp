#include "runtimejsoncodec.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace RuntimeJsonCodec {

QJsonArray toJsonArray(const std::vector<double>& values)
{
    QJsonArray array;
    for(double value : values){
        array.append(value);
    }
    return array;
}

QJsonArray toJsonArray(const std::vector<std::vector<double>>& values)
{
    QJsonArray array;
    for(const std::vector<double>& row : values){
        array.append(toJsonArray(row));
    }
    return array;
}

QJsonArray toJsonArray(const std::vector<int>& values)
{
    QJsonArray array;
    for(int value : values){
        array.append(value);
    }
    return array;
}

QJsonArray toJsonArray(const std::vector<qint64>& values)
{
    QJsonArray array;
    for(qint64 value : values){
        array.append(static_cast<double>(value));
    }
    return array;
}

qint64 jsonIntFromQuint64(quint64 value)
{
    return static_cast<qint64>(std::min<quint64>(
                                  value,
                                  static_cast<quint64>(std::numeric_limits<qint64>::max())));
}

std::vector<double> fromJsonArray(const QJsonArray& array)
{
    std::vector<double> values;
    values.reserve(array.size());
    for(const QJsonValue& value : array){
        values.push_back(value.toDouble());
    }
    return values;
}

std::vector<std::vector<double>> fromJsonMatrix(const QJsonArray& array)
{
    std::vector<std::vector<double>> values;
    values.reserve(array.size());
    for(const QJsonValue& value : array){
        if(!value.isArray()){
            continue;
        }
        values.push_back(fromJsonArray(value.toArray()));
    }
    return values;
}

std::vector<int> fromJsonIntArray(const QJsonArray& array)
{
    std::vector<int> values;
    values.reserve(array.size());
    for(const QJsonValue& value : array){
        values.push_back(value.toInt());
    }
    return values;
}

std::vector<qint64> fromJsonInt64Array(const QJsonArray& array)
{
    std::vector<qint64> values;
    values.reserve(array.size());
    for(const QJsonValue& value : array){
        if(value.isDouble()){
            values.push_back(static_cast<qint64>(std::llround(value.toDouble())));
        }
        else if(value.isString()){
            bool ok = false;
            const qint64 parsed = value.toString().toLongLong(&ok);
            values.push_back(ok ? parsed : 0);
        }
        else{
            values.push_back(0);
        }
    }
    return values;
}

qint64 jsonValueToInt64(const QJsonValue& value, qint64 fallback)
{
    if(value.isDouble()){
        return static_cast<qint64>(value.toDouble(static_cast<double>(fallback)));
    }
    if(value.isString()){
        bool ok = false;
        const qint64 parsed = value.toString().toLongLong(&ok);
        if(ok){
            return parsed;
        }
    }
    return fallback;
}

} // namespace RuntimeJsonCodec
