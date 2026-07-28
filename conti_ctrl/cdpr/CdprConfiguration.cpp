#include "CdprConfiguration.h"

#include <QCryptographicHash>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSet>

#include <cmath>

namespace {
double arrayNumber(const QJsonArray &values, int index)
{
    return index >= 0 && index < values.size()
        ? values.at(index).toDouble() : 0.0;
}

CdprVector3 readVector3(const QJsonValue &value)
{
    const QJsonArray values = value.toArray();
    return {arrayNumber(values, 0), arrayNumber(values, 1),
            arrayNumber(values, 2)};
}

QJsonArray writeVector3(const CdprVector3 &value)
{
    return {value.x, value.y, value.z};
}

template <size_t Size>
QJsonArray writeArray(const std::array<double, Size> &values)
{
    QJsonArray result;
    for (double value : values) {
        result.append(value);
    }
    return result;
}

template <size_t Size>
std::array<double, Size> readArray(const QJsonValue &value)
{
    std::array<double, Size> result {};
    const QJsonArray values = value.toArray();
    for (size_t index = 0; index < Size; ++index) {
        result[index] = arrayNumber(values, static_cast<int>(index));
    }
    return result;
}

CdprRigidBodyConfig readRigidBody(const QJsonObject &object)
{
    CdprRigidBodyConfig result;
    result.massKg = object.value(QStringLiteral("mass_kg")).toDouble();
    result.centerOfMassM = readVector3(object.value(QStringLiteral("center_of_mass_m")));
    result.inertiaKgM2 =
        readArray<9>(object.value(QStringLiteral("inertia_kg_m2")));
    return result;
}

QJsonObject writeRigidBody(const CdprRigidBodyConfig &value)
{
    return {
        {QStringLiteral("mass_kg"), value.massKg},
        {QStringLiteral("center_of_mass_m"), writeVector3(value.centerOfMassM)},
        {QStringLiteral("inertia_kg_m2"), writeArray(value.inertiaKgM2)}
    };
}

CdprForceSensorConfig readForceSensor(const QJsonObject &object)
{
    CdprForceSensorConfig result;
    result.rotationSensorToPlatform = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    result.originInPlatformM =
        readVector3(object.value(QStringLiteral("origin_in_platform_m")));
    if (object.contains(QStringLiteral("rotation_sensor_to_platform"))) {
        result.rotationSensorToPlatform =
            readArray<9>(object.value(QStringLiteral("rotation_sensor_to_platform")));
    }
    result.sensorSign =
        object.value(QStringLiteral("sensor_sign")).toInt(1);
    return result;
}

QJsonObject writeForceSensor(const CdprForceSensorConfig &value)
{
    return {
        {QStringLiteral("origin_in_platform_m"),
         writeVector3(value.originInPlatformM)},
        {QStringLiteral("rotation_sensor_to_platform"),
         writeArray(value.rotationSensorToPlatform)},
        {QStringLiteral("sensor_sign"), value.sensorSign}
    };
}

bool finiteVector(const CdprVector3 &value)
{
    return std::isfinite(value.x) && std::isfinite(value.y)
        && std::isfinite(value.z);
}

template <size_t Size>
bool finiteArray(const std::array<double, Size> &values)
{
    for (double value : values) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    return true;
}

bool isRotationMatrix(const std::array<double, 9> &matrix)
{
    constexpr double tolerance = 1.0e-6;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            double dot = 0.0;
            for (int index = 0; index < 3; ++index) {
                dot += matrix[static_cast<size_t>(row * 3 + index)]
                    * matrix[static_cast<size_t>(column * 3 + index)];
            }
            const double expected = row == column ? 1.0 : 0.0;
            if (std::abs(dot - expected) > tolerance) {
                return false;
            }
        }
    }
    const double determinant =
        matrix[0] * (matrix[4] * matrix[8] - matrix[5] * matrix[7])
        - matrix[1] * (matrix[3] * matrix[8] - matrix[5] * matrix[6])
        + matrix[2] * (matrix[3] * matrix[7] - matrix[4] * matrix[6]);
    return std::abs(determinant - 1.0) <= tolerance;
}

QString vectorText(const CdprVector3 &value)
{
    return QStringLiteral("(%1, %2, %3)")
        .arg(value.x, 0, 'f', 4)
        .arg(value.y, 0, 'f', 4)
        .arg(value.z, 0, 'f', 4);
}

QJsonObject toJson(const CdprConfiguration &configuration)
{
    QJsonArray cables;
    for (const CdprCableAxisConfig &cable : configuration.cables) {
        cables.append(QJsonObject {
            {QStringLiteral("cable"), cable.cable},
            {QStringLiteral("axis"), cable.axis},
            {QStringLiteral("direction"), cable.direction},
            {QStringLiteral("frame_anchor_m"), writeVector3(cable.frameAnchorM)},
            {QStringLiteral("platform_anchor_m"), writeVector3(cable.platformAnchorM)},
            {QStringLiteral("initial_length_m"), cable.initialCableLengthM},
            {QStringLiteral("drum_radius_m"), cable.drumRadiusM},
            {QStringLiteral("minimum_length_m"), cable.minimumCableLengthM},
            {QStringLiteral("maximum_length_m"), cable.maximumCableLengthM}
        });
    }
    return {
        {QStringLiteral("schema_version"), 1},
        {QStringLiteral("name"), configuration.name},
        {QStringLiteral("parameters_confirmed"), configuration.parametersConfirmed},
        {QStringLiteral("coordinate_convention"), configuration.coordinateConvention},
        {QStringLiteral("initial_platform_pose"),
         writeArray(configuration.initialPlatformPose)},
        {QStringLiteral("physical_platform"), writeRigidBody(configuration.physicalPlatform)},
        {QStringLiteral("virtual_body"), writeRigidBody(configuration.virtualBody)},
        {QStringLiteral("force_sensor"), writeForceSensor(configuration.forceSensor)},
        {QStringLiteral("cables"), cables},
        {QStringLiteral("control"), QJsonObject {
             {QStringLiteral("period_us"), configuration.controlPeriodUs},
             {QStringLiteral("maximum_position_error_deg"),
              configuration.maximumPositionErrorDegree}
         }}
    };
}
}

bool CdprConfigurationFile::load(const QString &path,
                                 CdprConfiguration &configuration,
                                 QStringList &errors)
{
    errors.clear();
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        errors.append(QStringLiteral("无法读取配置文件：%1").arg(file.errorString()));
        return false;
    }
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (document.isNull() || !document.isObject()) {
        errors.append(QStringLiteral("JSON解析失败：%1（偏移%2）")
                          .arg(parseError.errorString())
                          .arg(parseError.offset));
        return false;
    }

    const QJsonObject root = document.object();
    if (root.value(QStringLiteral("schema_version")).toInt() != 1) {
        errors.append(QStringLiteral("不支持的schema_version，仅支持版本1。"));
        return false;
    }

    CdprConfiguration result;
    result.name = root.value(QStringLiteral("name")).toString();
    result.parametersConfirmed =
        root.value(QStringLiteral("parameters_confirmed")).toBool();
    result.coordinateConvention =
        root.value(QStringLiteral("coordinate_convention")).toString();
    result.initialPlatformPose =
        readArray<6>(root.value(QStringLiteral("initial_platform_pose")));
    result.physicalPlatform =
        readRigidBody(root.value(QStringLiteral("physical_platform")).toObject());
    result.virtualBody =
        readRigidBody(root.value(QStringLiteral("virtual_body")).toObject());
    result.forceSensor =
        readForceSensor(root.value(QStringLiteral("force_sensor")).toObject());
    const QJsonArray cables = root.value(QStringLiteral("cables")).toArray();
    if (cables.size() != 8) {
        errors.append(QStringLiteral("cables必须且只能包含8项。"));
        return false;
    }
    for (int index = 0; index < cables.size(); ++index) {
        const QJsonObject item = cables.at(index).toObject();
        CdprCableAxisConfig cable;
        cable.cable = item.value(QStringLiteral("cable")).toInt(index);
        cable.axis = item.value(QStringLiteral("axis")).toInt(-1);
        cable.direction = item.value(QStringLiteral("direction")).toInt();
        cable.frameAnchorM = readVector3(item.value(QStringLiteral("frame_anchor_m")));
        cable.platformAnchorM =
            readVector3(item.value(QStringLiteral("platform_anchor_m")));
        cable.initialCableLengthM =
            item.value(QStringLiteral("initial_length_m")).toDouble();
        cable.drumRadiusM = item.value(QStringLiteral("drum_radius_m")).toDouble();
        cable.minimumCableLengthM =
            item.value(QStringLiteral("minimum_length_m")).toDouble();
        cable.maximumCableLengthM =
            item.value(QStringLiteral("maximum_length_m")).toDouble();
        result.cables[static_cast<size_t>(index)] = cable;
    }
    const QJsonObject control = root.value(QStringLiteral("control")).toObject();
    result.controlPeriodUs = control.value(QStringLiteral("period_us")).toInt();
    result.maximumPositionErrorDegree =
        control.value(QStringLiteral("maximum_position_error_deg")).toDouble();

    configuration = result;
    errors = validate(configuration);
    return true;
}

bool CdprConfigurationFile::writeTemplate(const QString &path, QString &error)
{
    CdprConfiguration configuration;
    configuration.name = QStringLiteral("CDPR参数模板");
    configuration.coordinateConvention =
        QStringLiteral("世界坐标右手系；平台连接点使用平台局部坐标");
    configuration.physicalPlatform.massKg = 1.0;
    configuration.virtualBody.massKg = 1.0;
    configuration.physicalPlatform.inertiaKgM2 = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    configuration.virtualBody.inertiaKgM2 = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    configuration.forceSensor.rotationSensorToPlatform =
        {1, 0, 0, 0, 1, 0, 0, 0, 1};
    configuration.forceSensor.sensorSign = 1;
    const std::array<CdprVector3, 8> signs {{
        {-1, -1, -1}, {-1, -1, 1}, {-1, 1, -1}, {-1, 1, 1},
        {1, -1, -1}, {1, -1, 1}, {1, 1, -1}, {1, 1, 1}
    }};
    for (int index = 0; index < 8; ++index) {
        CdprCableAxisConfig &cable =
            configuration.cables[static_cast<size_t>(index)];
        cable.cable = index;
        cable.axis = index;
        cable.direction = 1;
        cable.frameAnchorM = signs[static_cast<size_t>(index)];
        cable.platformAnchorM = {
            signs[static_cast<size_t>(index)].x * 0.1,
            signs[static_cast<size_t>(index)].y * 0.1,
            signs[static_cast<size_t>(index)].z * 0.1
        };
        cable.initialCableLengthM = 1.0;
        cable.drumRadiusM = 0.02;
        cable.minimumCableLengthM = 0.1;
        cable.maximumCableLengthM = 3.0;
    }

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        error = QStringLiteral("无法写入模板：%1").arg(file.errorString());
        return false;
    }
    if (file.write(QJsonDocument(toJson(configuration)).toJson(
                       QJsonDocument::Indented)) < 0) {
        error = QStringLiteral("写入模板失败：%1").arg(file.errorString());
        return false;
    }
    return true;
}

QStringList CdprConfigurationFile::validate(
    const CdprConfiguration &configuration)
{
    QStringList errors;
    if (configuration.name.trimmed().isEmpty()) {
        errors.append(QStringLiteral("配置名称不能为空。"));
    }
    if (!configuration.parametersConfirmed) {
        errors.append(QStringLiteral(
            "参数尚未确认：核对模板后将parameters_confirmed设为true。"));
    }
    if (configuration.coordinateConvention.trimmed().isEmpty()) {
        errors.append(QStringLiteral("必须明确坐标系和姿态约定。"));
    }
    if (configuration.physicalPlatform.massKg <= 0.0
        || configuration.virtualBody.massKg <= 0.0) {
        errors.append(QStringLiteral("物理平台质量和虚拟刚体质量必须大于0。"));
    }
    if (!finiteArray(configuration.initialPlatformPose)) {
        errors.append(QStringLiteral("初始末端位姿必须全部为有限数。"));
    }
    if (!finiteVector(configuration.physicalPlatform.centerOfMassM)
        || !finiteArray(configuration.physicalPlatform.inertiaKgM2)
        || !finiteVector(configuration.virtualBody.centerOfMassM)
        || !finiteArray(configuration.virtualBody.inertiaKgM2)) {
        errors.append(QStringLiteral("刚体质心和惯量参数必须全部为有限数。"));
    }
    if (!finiteVector(configuration.forceSensor.originInPlatformM)
        || !finiteArray(configuration.forceSensor.rotationSensorToPlatform)) {
        errors.append(QStringLiteral("力传感器安装参数必须全部为有限数。"));
    } else if (!isRotationMatrix(configuration.forceSensor.rotationSensorToPlatform)) {
        errors.append(QStringLiteral(
            "力传感器到平台的旋转矩阵必须正交且行列式为+1。"));
    }
    if (configuration.forceSensor.sensorSign != -1
        && configuration.forceSensor.sensorSign != 1) {
        errors.append(QStringLiteral("力传感器符号只能为+1或-1。"));
    }
    if (configuration.controlPeriodUs <= 0
        || configuration.controlPeriodUs % 250 != 0) {
        errors.append(QStringLiteral("控制周期必须为正数且是250 us的整数倍。"));
    }
    if (configuration.maximumPositionErrorDegree <= 0.0) {
        errors.append(QStringLiteral("最大位置误差必须大于0。"));
    }

    QSet<int> axes;
    for (int index = 0; index < 8; ++index) {
        const CdprCableAxisConfig &cable =
            configuration.cables[static_cast<size_t>(index)];
        const QString prefix = QStringLiteral("绳%1：").arg(index);
        if (cable.cable != index) {
            errors.append(prefix + QStringLiteral("cable编号必须与数组顺序一致。"));
        }
        if (cable.axis < 0 || cable.axis > 7 || axes.contains(cable.axis)) {
            errors.append(prefix + QStringLiteral("轴号必须在0~7内且不可重复。"));
        }
        axes.insert(cable.axis);
        if (cable.direction != -1 && cable.direction != 1) {
            errors.append(prefix + QStringLiteral("方向只能为+1或-1。"));
        }
        if (!finiteVector(cable.frameAnchorM)
            || !finiteVector(cable.platformAnchorM)) {
            errors.append(prefix + QStringLiteral("连接点坐标必须为有限数。"));
        }
        if (cable.drumRadiusM <= 0.0) {
            errors.append(prefix + QStringLiteral("卷筒半径必须大于0。"));
        }
        if (cable.minimumCableLengthM < 0.0
            || cable.maximumCableLengthM <= cable.minimumCableLengthM
            || cable.initialCableLengthM < cable.minimumCableLengthM
            || cable.initialCableLengthM > cable.maximumCableLengthM) {
            errors.append(prefix + QStringLiteral("绳长初值或上下限不合法。"));
        }
    }
    return errors;
}

QString CdprConfigurationFile::identifier(
    const CdprConfiguration &configuration)
{
    const QByteArray canonical =
        QJsonDocument(toJson(configuration)).toJson(QJsonDocument::Compact);
    return QString::fromLatin1(
        QCryptographicHash::hash(canonical, QCryptographicHash::Sha256).toHex().left(12));
}

QString CdprConfigurationFile::summary(
    const CdprConfiguration &configuration)
{
    return QStringLiteral(
        "名称：%1\n坐标约定：%2\n物理平台 / 虚拟刚体质量：%3 / %4 kg\n"
        "控制周期：%5 us\n最大位置误差：%6°\n"
        "绳0出绳点示例：%7；平台点：%8")
        .arg(configuration.name, configuration.coordinateConvention)
        .arg(configuration.physicalPlatform.massKg, 0, 'f', 4)
        .arg(configuration.virtualBody.massKg, 0, 'f', 4)
        .arg(configuration.controlPeriodUs)
        .arg(configuration.maximumPositionErrorDegree, 0, 'f', 4)
        .arg(vectorText(configuration.cables[0].frameAnchorM),
             vectorText(configuration.cables[0].platformAnchorM));
}
