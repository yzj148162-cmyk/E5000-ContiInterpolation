#ifndef CSVEXPORTUTILS_H
#define CSVEXPORTUTILS_H

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QStringList>
#include <QTextStream>

namespace CsvExport {

inline QString escapedField(QString value)
{
    const bool needsQuotes = value.contains(QLatin1Char(',')) ||
            value.contains(QLatin1Char('"')) ||
            value.contains(QLatin1Char('\r')) ||
            value.contains(QLatin1Char('\n'));
    if(value.contains(QLatin1Char('"'))){
        value.replace(QStringLiteral("\""), QStringLiteral("\"\""));
    }
    return needsQuotes ? QStringLiteral("\"%1\"").arg(value) : value;
}

inline void writeRow(QTextStream& stream, const QStringList& fields)
{
    for(int fieldIndex = 0; fieldIndex < fields.size(); ++fieldIndex){
        if(fieldIndex > 0){
            stream << ',';
        }
        stream << escapedField(fields[fieldIndex]);
    }
    stream << "\r\n";
}

inline QString jsonPointerToken(QString token)
{
    token.replace(QStringLiteral("~"), QStringLiteral("~0"));
    token.replace(QStringLiteral("/"), QStringLiteral("~1"));
    return token;
}

inline QString jsonValueTypeName(const QJsonValue& value)
{
    switch(value.type()){
    case QJsonValue::Null:
        return QStringLiteral("null");
    case QJsonValue::Bool:
        return QStringLiteral("boolean");
    case QJsonValue::Double:
        return QStringLiteral("number");
    case QJsonValue::String:
        return QStringLiteral("string");
    case QJsonValue::Array:
        return QStringLiteral("array");
    case QJsonValue::Object:
        return QStringLiteral("object");
    case QJsonValue::Undefined:
    default:
        return QStringLiteral("undefined");
    }
}

inline QString jsonScalarText(const QJsonValue& value)
{
    if(value.isString()){
        return value.toString();
    }
    if(value.isBool()){
        return value.toBool() ? QStringLiteral("true") : QStringLiteral("false");
    }
    if(value.isDouble()){
        QJsonArray wrapper;
        wrapper.append(value);
        const QByteArray json = QJsonDocument(wrapper).toJson(QJsonDocument::Compact);
        return json.size() >= 2 ? QString::fromUtf8(json.mid(1, json.size() - 2)) : QString();
    }
    return QString();
}

inline void writeJsonNodeAsLongTable(QTextStream& stream,
                                     const QJsonValue& value,
                                     const QString& pointer,
                                     const QString& parentPointer,
                                     const QString& key,
                                     int arrayIndex,
                                     qint64& nodeOrder)
{
    writeRow(stream,
             {QString::number(nodeOrder++),
              pointer.isEmpty() ? QStringLiteral("$") : pointer,
              parentPointer,
              key,
              arrayIndex >= 0 ? QString::number(arrayIndex) : QString(),
              jsonValueTypeName(value),
              jsonScalarText(value)});

    if(value.isObject()){
        const QJsonObject object = value.toObject();
        for(auto it = object.constBegin(); it != object.constEnd(); ++it){
            const QString childPointer = pointer + QLatin1Char('/') + jsonPointerToken(it.key());
            writeJsonNodeAsLongTable(stream,
                                     it.value(),
                                     childPointer,
                                     pointer.isEmpty() ? QStringLiteral("$") : pointer,
                                     it.key(),
                                     -1,
                                     nodeOrder);
        }
    }
    else if(value.isArray()){
        const QJsonArray array = value.toArray();
        for(int index = 0; index < array.size(); ++index){
            const QString childPointer = pointer + QLatin1Char('/') + QString::number(index);
            writeJsonNodeAsLongTable(stream,
                                     array.at(index),
                                     childPointer,
                                     pointer.isEmpty() ? QStringLiteral("$") : pointer,
                                     QString(),
                                     index,
                                     nodeOrder);
        }
    }
}

inline bool writeJsonLongTable(QTextStream& stream, const QJsonValue& root)
{
    writeRow(stream,
             {QStringLiteral("node_order"),
              QStringLiteral("json_pointer"),
              QStringLiteral("parent_pointer"),
              QStringLiteral("key"),
              QStringLiteral("array_index"),
              QStringLiteral("value_type"),
              QStringLiteral("value")});
    qint64 nodeOrder = 0;
    writeJsonNodeAsLongTable(stream, root, QString(), QString(), QString(), -1, nodeOrder);
    return stream.status() == QTextStream::Ok;
}

struct SessionState
{
    QString section;
    QStringList currentHeader;
    qint64 sourceRowIndex = 0;
};

inline bool isSessionHeaderRow(const QStringList& cells)
{
    if(cells.isEmpty()){
        return false;
    }
    const QString first = cells.front().trimmed();
    return first.startsWith(QStringLiteral("时间戳")) ||
            first.startsWith(QStringLiteral("轨迹点(0起)")) ||
            first == QStringLiteral("序号") ||
            first == QStringLiteral("point_index");
}

inline QString sessionRecordType(const QStringList& cells,
                                 bool sectionRow,
                                 bool blankRow,
                                 bool headerRow)
{
    if(sectionRow){
        return QStringLiteral("section");
    }
    if(blankRow){
        return QStringLiteral("blank");
    }
    if(headerRow){
        return QStringLiteral("header");
    }
    if(!cells.isEmpty() && cells.front() == QStringLiteral("说明")){
        return QStringLiteral("description");
    }
    if(!cells.isEmpty() && cells.front() == QStringLiteral("无有效数据")){
        return QStringLiteral("no_data");
    }
    if(cells.size() <= 2){
        return QStringLiteral("metadata");
    }
    return QStringLiteral("data");
}

inline bool writeSessionSourceLine(QTextStream& stream,
                                   const QString& sourceLine,
                                   SessionState& state)
{
    const bool blankRow = sourceLine.isEmpty();
    QStringList cells = sourceLine.split(QLatin1Char('\t'), Qt::KeepEmptyParts);
    if(blankRow){
        cells = {QString()};
    }
    const QString trimmedLine = sourceLine.trimmed();
    const bool sectionRow = !sourceLine.contains(QLatin1Char('\t')) &&
            trimmedLine.startsWith(QLatin1Char('[')) &&
            trimmedLine.endsWith(QLatin1Char(']')) &&
            trimmedLine.size() >= 2;
    if(sectionRow){
        state.section = trimmedLine.mid(1, trimmedLine.size() - 2);
        state.currentHeader.clear();
    }
    if(!sectionRow && !blankRow && cells.size() > 2){
        state.currentHeader = cells;
    }
    writeRow(stream, cells);
    ++state.sourceRowIndex;
    return stream.status() == QTextStream::Ok;
}

inline bool writeSessionText(QTextStream& stream,
                             const QString& sourceText,
                             SessionState& state)
{
    QString normalized = sourceText;
    normalized.replace(QStringLiteral("\r\n"), QStringLiteral("\n"));
    const QStringList lines = normalized.split(QLatin1Char('\n'), Qt::KeepEmptyParts);
    for(const QString& line : lines){
        if(!writeSessionSourceLine(stream, line, state)){
            return false;
        }
    }
    return true;
}

} // namespace CsvExport

#endif // CSVEXPORTUTILS_H
