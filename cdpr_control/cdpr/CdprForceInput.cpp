#include "CdprForceInput.h"

#include <cctype>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>

#include <QStringList>

namespace {
bool finite6(const CdprVector6 &values)
{
    for (double value : values) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    return true;
}
CdprVector3 rotate(const std::array<double, 9> &rotation,
                   const CdprVector3 &value)
{
    return {
        rotation[0] * value.x + rotation[1] * value.y
            + rotation[2] * value.z,
        rotation[3] * value.x + rotation[4] * value.y
            + rotation[5] * value.z,
        rotation[6] * value.x + rotation[7] * value.y
            + rotation[8] * value.z
    };
}

CdprVector3 cross(const CdprVector3 &left, const CdprVector3 &right)
{
    return {
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x
    };
}

constexpr double kPi = 3.14159265358979323846;
constexpr double kMaximumForceN = 10000.0;
constexpr double kMaximumMomentNm = 10000.0;

struct ExpressionNode
{
    enum class Kind { Constant, Time, Add, Subtract, Multiply, Divide,
                      Power, Negate, Function };
    Kind kind = Kind::Constant;
    double value = 0.0;
    QString function;
    std::shared_ptr<ExpressionNode> left;
    std::shared_ptr<ExpressionNode> right;

    double evaluate(double timeS) const
    {
        switch (kind) {
        case Kind::Constant: return value;
        case Kind::Time: return timeS;
        case Kind::Add: return left->evaluate(timeS) + right->evaluate(timeS);
        case Kind::Subtract: return left->evaluate(timeS) - right->evaluate(timeS);
        case Kind::Multiply: return left->evaluate(timeS) * right->evaluate(timeS);
        case Kind::Divide: return left->evaluate(timeS) / right->evaluate(timeS);
        case Kind::Power: return std::pow(left->evaluate(timeS), right->evaluate(timeS));
        case Kind::Negate: return -left->evaluate(timeS);
        case Kind::Function: {
            const double argument = left->evaluate(timeS);
            if (function == QStringLiteral("sin")) return std::sin(argument);
            if (function == QStringLiteral("cos")) return std::cos(argument);
            if (function == QStringLiteral("tan")) return std::tan(argument);
            if (function == QStringLiteral("exp")) return std::exp(argument);
            if (function == QStringLiteral("sqrt")) return std::sqrt(argument);
            if (function == QStringLiteral("abs")) return std::abs(argument);
            if (function == QStringLiteral("log")) return std::log(argument);
            return std::numeric_limits<double>::quiet_NaN();
        }
        }
        return std::numeric_limits<double>::quiet_NaN();
    }
};

class ExpressionParser
{
public:
    explicit ExpressionParser(QString expression)
        : text_(std::move(expression)) {}

    std::shared_ptr<ExpressionNode> parse(QString *errorText)
    {
        position_ = 0;
        error_.clear();
        const auto result = parseExpression();
        skipSpace();
        if (result && position_ != text_.size() && error_.isEmpty()) {
            error_ = QStringLiteral("位置%1附近存在无法识别的内容。")
                         .arg(position_ + 1);
        }
        if (!result || !error_.isEmpty()) {
            if (errorText) *errorText = error_.isEmpty()
                ? QStringLiteral("表达式为空或不完整。") : error_;
            return {};
        }
        if (errorText) errorText->clear();
        return result;
    }

private:
    std::shared_ptr<ExpressionNode> parseExpression()
    {
        auto left = parseTerm();
        while (left) {
            skipSpace();
            if (!consume('+') && !consume('-')) break;
            const QChar operation = text_.at(position_ - 1);
            auto right = parseTerm();
            if (!right) return {};
            auto node = std::make_shared<ExpressionNode>();
            node->kind = operation == '+' ? ExpressionNode::Kind::Add
                                           : ExpressionNode::Kind::Subtract;
            node->left = left;
            node->right = right;
            left = node;
        }
        return left;
    }

    std::shared_ptr<ExpressionNode> parseTerm()
    {
        auto left = parsePower();
        while (left) {
            skipSpace();
            if (!consume('*') && !consume('/')) break;
            const QChar operation = text_.at(position_ - 1);
            auto right = parsePower();
            if (!right) return {};
            auto node = std::make_shared<ExpressionNode>();
            node->kind = operation == '*' ? ExpressionNode::Kind::Multiply
                                           : ExpressionNode::Kind::Divide;
            node->left = left;
            node->right = right;
            left = node;
        }
        return left;
    }

    std::shared_ptr<ExpressionNode> parsePower()
    {
        auto left = parseUnary();
        if (!left) return {};
        skipSpace();
        if (!consume('^')) return left;
        auto right = parsePower();
        if (!right) return {};
        auto node = std::make_shared<ExpressionNode>();
        node->kind = ExpressionNode::Kind::Power;
        node->left = left;
        node->right = right;
        return node;
    }

    std::shared_ptr<ExpressionNode> parseUnary()
    {
        skipSpace();
        if (consume('+')) return parseUnary();
        if (consume('-')) {
            auto value = parseUnary();
            if (!value) return {};
            auto node = std::make_shared<ExpressionNode>();
            node->kind = ExpressionNode::Kind::Negate;
            node->left = value;
            return node;
        }
        return parsePrimary();
    }

    std::shared_ptr<ExpressionNode> parsePrimary()
    {
        skipSpace();
        if (consume('(')) {
            auto value = parseExpression();
            skipSpace();
            if (!consume(')')) {
                error_ = QStringLiteral("缺少右括号，位置%1。").arg(position_ + 1);
                return {};
            }
            return value;
        }
        if (position_ < text_.size()
            && (text_.at(position_).isDigit() || text_.at(position_) == '.')) {
            return parseNumber();
        }
        if (position_ < text_.size()
            && (text_.at(position_).isLetter() || text_.at(position_) == '_')) {
            const QString identifier = parseIdentifier().toLower();
            if (identifier == QStringLiteral("t")) {
                auto node = std::make_shared<ExpressionNode>();
                node->kind = ExpressionNode::Kind::Time;
                return node;
            }
            if (identifier == QStringLiteral("pi")) {
                auto node = std::make_shared<ExpressionNode>();
                node->value = kPi;
                return node;
            }
            const QStringList functions {
                QStringLiteral("sin"), QStringLiteral("cos"),
                QStringLiteral("tan"), QStringLiteral("exp"),
                QStringLiteral("sqrt"), QStringLiteral("abs"),
                QStringLiteral("log")
            };
            if (!functions.contains(identifier)) {
                error_ = QStringLiteral("未知标识符“%1”。").arg(identifier);
                return {};
            }
            skipSpace();
            if (!consume('(')) {
                error_ = QStringLiteral("函数%1后缺少左括号。").arg(identifier);
                return {};
            }
            auto argument = parseExpression();
            skipSpace();
            if (!argument || !consume(')')) {
                error_ = QStringLiteral("函数%1缺少右括号。").arg(identifier);
                return {};
            }
            auto node = std::make_shared<ExpressionNode>();
            node->kind = ExpressionNode::Kind::Function;
            node->function = identifier;
            node->left = argument;
            return node;
        }
        error_ = QStringLiteral("位置%1需要数字、t、pi或函数。")
                     .arg(position_ + 1);
        return {};
    }

    std::shared_ptr<ExpressionNode> parseNumber()
    {
        const int start = position_;
        bool exponentSeen = false;
        while (position_ < text_.size()) {
            const QChar character = text_.at(position_);
            if (character.isDigit() || character == '.') {
                ++position_;
            } else if ((character == 'e' || character == 'E') && !exponentSeen) {
                exponentSeen = true;
                ++position_;
                if (position_ < text_.size()
                    && (text_.at(position_) == '+' || text_.at(position_) == '-')) {
                    ++position_;
                }
            } else {
                break;
            }
        }
        bool ok = false;
        const double value = text_.mid(start, position_ - start).toDouble(&ok);
        if (!ok || !std::isfinite(value)) {
            error_ = QStringLiteral("位置%1的数字无效。").arg(start + 1);
            return {};
        }
        auto node = std::make_shared<ExpressionNode>();
        node->value = value;
        return node;
    }

    QString parseIdentifier()
    {
        const int start = position_;
        while (position_ < text_.size()
               && (text_.at(position_).isLetterOrNumber()
                   || text_.at(position_) == '_')) {
            ++position_;
        }
        return text_.mid(start, position_ - start);
    }

    void skipSpace()
    {
        while (position_ < text_.size() && text_.at(position_).isSpace()) {
            ++position_;
        }
    }

    bool consume(QChar expected)
    {
        if (position_ < text_.size() && text_.at(position_) == expected) {
            ++position_;
            return true;
        }
        return false;
    }

    QString text_;
    int position_ = 0;
    QString error_;
};

bool componentWithinLimit(int index, double value)
{
    const double limit = index < 3 ? kMaximumForceN : kMaximumMomentNm;
    return std::isfinite(value) && std::abs(value) <= limit;
}
}

CdprWrenchTransformer::CdprWrenchTransformer(
    const CdprForceSensorConfig &configuration)
    : configuration_(configuration)
{
}

CdprWrenchTransformResult CdprWrenchTransformer::toPlatformCenterOfMass(
    const CdprWrenchSample &sensorSample) const
{
    CdprWrenchTransformResult result;
    result.sample.stamp = sensorSample.stamp;
    result.sample.coordinate =
        CdprWrenchCoordinate::PlatformBodyAtCenterOfMass;
    if (!sensorSample.valid || !sensorSample.stamp.valid) {
        result.errorText = QStringLiteral("F/T输入帧无效。");
        return result;
    }
    if (sensorSample.coordinate != CdprWrenchCoordinate::Sensor) {
        result.errorText = QStringLiteral("F/T输入不是传感器坐标系。");
        return result;
    }
    if (!finite6(sensorSample.wrench)) {
        result.errorText = QStringLiteral("F/T输入包含非有限数。");
        return result;
    }

    CdprVector6 calibrated {};
    for (int index = 0; index < 6; ++index) {
        calibrated[static_cast<size_t>(index)] =
            static_cast<double>(configuration_.wrenchReactionSign)
            * configuration_.channelScale[static_cast<size_t>(index)]
            * (sensorSample.wrench[static_cast<size_t>(index)]
               - configuration_.channelBias[static_cast<size_t>(index)]);
    }

    const CdprVector3 forceSensor {
        calibrated[0], calibrated[1], calibrated[2]
    };
    const CdprVector3 momentSensor {
        calibrated[3], calibrated[4], calibrated[5]
    };
    const CdprVector3 forcePlatform =
        rotate(configuration_.rotationSensorToPlatform, forceSensor);
    const CdprVector3 momentAtSensorPlatform =
        rotate(configuration_.rotationSensorToPlatform, momentSensor);
    const CdprVector3 offsetMoment =
        cross(configuration_.originInPlatformM, forcePlatform);
    const CdprVector3 momentAtCenterOfMass {
        momentAtSensorPlatform.x + offsetMoment.x,
        momentAtSensorPlatform.y + offsetMoment.y,
        momentAtSensorPlatform.z + offsetMoment.z
    };

    result.sample.wrench = {
        forcePlatform.x, forcePlatform.y, forcePlatform.z,
        momentAtCenterOfMass.x, momentAtCenterOfMass.y,
        momentAtCenterOfMass.z
    };
    result.sample.valid = finite6(result.sample.wrench);
    if (!result.sample.valid) {
        result.errorText = QStringLiteral("F/T坐标变换结果包含非有限数。");
    }
    return result;
}

bool SimulatedWrenchSource::configure(
    const CdprSimulatedWrenchProfile &profile, QString *errorText)
{
    if (!finite6(profile.value) || !finite6(profile.bias)) {
        if (errorText) *errorText = QStringLiteral("六维力参数包含非有限数。");
        return false;
    }
    if (profile.mode == CdprSimulatedWrenchMode::Pulse
        && (!std::isfinite(profile.pulseStartS) || profile.pulseStartS < 0.0
            || !std::isfinite(profile.pulseDurationS)
            || profile.pulseDurationS <= 0.0)) {
        if (errorText) *errorText = QStringLiteral("脉冲开始时间必须非负，持续时间必须大于0。");
        return false;
    }
    if (profile.mode == CdprSimulatedWrenchMode::Sine
        && (!std::isfinite(profile.sineFrequencyHz)
            || profile.sineFrequencyHz <= 0.0
            || profile.sineFrequencyHz >= 100.0
            || !std::isfinite(profile.sinePhaseRad))) {
        if (errorText) *errorText = QStringLiteral(
            "正弦频率必须大于0且低于5 ms采样的奈奎斯特频率100 Hz。");
        return false;
    }

    if (profile.mode != CdprSimulatedWrenchMode::Formula) {
        for (int index = 0; index < kCdprDofCount; ++index) {
            const size_t component = static_cast<size_t>(index);
            double maximumMagnitude = std::abs(profile.value[component]);
            if (profile.mode == CdprSimulatedWrenchMode::Pulse
                || profile.mode == CdprSimulatedWrenchMode::Sine) {
                maximumMagnitude += std::abs(profile.bias[component]);
            }
            const double limit = index < 3 ? kMaximumForceN : kMaximumMomentNm;
            if (maximumMagnitude > limit) {
                if (errorText) {
                    *errorText = QStringLiteral("第%1个分量的理论峰值超出±%2限制。")
                                     .arg(index + 1).arg(limit, 0, 'f', 0);
                }
                return false;
            }
        }
    }

    std::array<std::function<double(double)>, kCdprDofCount> evaluators {};
    if (profile.mode == CdprSimulatedWrenchMode::Formula) {
        for (int index = 0; index < kCdprDofCount; ++index) {
            QString parseError;
            ExpressionParser parser(profile.expressions[static_cast<size_t>(index)]);
            const auto root = parser.parse(&parseError);
            if (!root) {
                if (errorText) {
                    *errorText = QStringLiteral("第%1个分量表达式无效：%2")
                                     .arg(index + 1).arg(parseError);
                }
                return false;
            }
            evaluators[static_cast<size_t>(index)] =
                [root](double timeS) { return root->evaluate(timeS); };
        }
    }

    const CdprSimulatedWrenchProfile previousProfile = profile_;
    const auto previousEvaluators = formulaEvaluators_;
    const bool previousConfigured = configured_;
    profile_ = profile;
    formulaEvaluators_ = std::move(evaluators);
    configured_ = true;
    CdprVector6 check {};
    QString evaluationError;
    for (int sampleIndex = 0; sampleIndex <= 200; ++sampleIndex) {
        const double timeS = static_cast<double>(sampleIndex) * 0.025;
        if (!evaluate(timeS, check, &evaluationError)) {
            profile_ = previousProfile;
            formulaEvaluators_ = previousEvaluators;
            configured_ = previousConfigured;
            if (errorText) {
                *errorText = QStringLiteral("0～5 s预检失败：%1")
                                 .arg(evaluationError);
            }
            return false;
        }
    }
    if (errorText) errorText->clear();
    return true;
}

CdprSimulatedWrenchProfile SimulatedWrenchSource::profile() const
{
    return profile_;
}

void SimulatedWrenchSource::setSensorWrench(const CdprVector6 &wrench)
{
    CdprSimulatedWrenchProfile profile;
    profile.mode = CdprSimulatedWrenchMode::Constant;
    profile.value = wrench;
    configure(profile, nullptr);
}

CdprVector6 SimulatedWrenchSource::sensorWrench() const
{
    CdprVector6 result {};
    evaluate(0.0, result, nullptr);
    return result;
}

CdprWrenchSample SimulatedWrenchSource::sample(
    const CdprFrameStamp &stamp, double elapsedS) const
{
    CdprWrenchSample result;
    result.stamp = stamp;
    QString error;
    const bool evaluated = evaluate(elapsedS, result.wrench, &error);
    result.coordinate = CdprWrenchCoordinate::Sensor;
    result.valid = stamp.valid && evaluated;
    return result;
}

bool SimulatedWrenchSource::evaluate(
    double elapsedS, CdprVector6 &wrench, QString *errorText) const
{
    wrench = {};
    if (!configured_ || !std::isfinite(elapsedS) || elapsedS < 0.0) {
        if (errorText) *errorText = QStringLiteral("模拟力配置或时间无效。");
        return false;
    }
    switch (profile_.mode) {
    case CdprSimulatedWrenchMode::Constant:
        wrench = profile_.value;
        break;
    case CdprSimulatedWrenchMode::Pulse:
        wrench = profile_.bias;
        if (elapsedS >= profile_.pulseStartS
            && elapsedS < profile_.pulseStartS + profile_.pulseDurationS) {
            for (int index = 0; index < kCdprDofCount; ++index) {
                wrench[static_cast<size_t>(index)] +=
                    profile_.value[static_cast<size_t>(index)];
            }
        }
        break;
    case CdprSimulatedWrenchMode::Sine: {
        const double wave = std::sin(2.0 * kPi * profile_.sineFrequencyHz
                                     * elapsedS + profile_.sinePhaseRad);
        for (int index = 0; index < kCdprDofCount; ++index) {
            const size_t component = static_cast<size_t>(index);
            wrench[component] = profile_.bias[component]
                + profile_.value[component] * wave;
        }
        break;
    }
    case CdprSimulatedWrenchMode::Formula:
        for (int index = 0; index < kCdprDofCount; ++index) {
            const auto &evaluator = formulaEvaluators_[static_cast<size_t>(index)];
            if (!evaluator) {
                if (errorText) *errorText = QStringLiteral("表达式尚未编译。");
                return false;
            }
            wrench[static_cast<size_t>(index)] = evaluator(elapsedS);
        }
        break;
    }
    for (int index = 0; index < kCdprDofCount; ++index) {
        if (!componentWithinLimit(index, wrench[static_cast<size_t>(index)])) {
            if (errorText) {
                *errorText = QStringLiteral("第%1个分量在t=%2 s超出有限幅值范围。")
                                 .arg(index + 1).arg(elapsedS, 0, 'f', 6);
            }
            return false;
        }
    }
    if (errorText) errorText->clear();
    return true;
}

QString SimulatedWrenchSource::summary() const
{
    switch (profile_.mode) {
    case CdprSimulatedWrenchMode::Constant: return QStringLiteral("定值");
    case CdprSimulatedWrenchMode::Pulse:
        return QStringLiteral("有限脉冲（%1～%2 s）")
            .arg(profile_.pulseStartS, 0, 'f', 3)
            .arg(profile_.pulseStartS + profile_.pulseDurationS, 0, 'f', 3);
    case CdprSimulatedWrenchMode::Sine:
        return QStringLiteral("正弦（%1 Hz，相位%2 rad）")
            .arg(profile_.sineFrequencyHz, 0, 'f', 3)
            .arg(profile_.sinePhaseRad, 0, 'f', 3);
    case CdprSimulatedWrenchMode::Formula: return QStringLiteral("自定义公式");
    }
    return QStringLiteral("未知");
}

void TraceFtSensorSource::acceptSameFrameSample(
    const CdprWrenchSample &sample)
{
    latestSample_ = sample;
    configured_ = sample.coordinate == CdprWrenchCoordinate::Sensor;
}

void TraceFtSensorSource::clear()
{
    latestSample_ = {};
    configured_ = false;
}

bool TraceFtSensorSource::configured() const
{
    return configured_;
}

QString TraceFtSensorSource::statusText() const
{
    return configured_
        ? QStringLiteral("Trace F/T已接收同帧样本")
        : QStringLiteral("Trace F/T对象类型待配置，当前不生成有效力帧");
}

CdprWrenchSample TraceFtSensorSource::latestSample() const
{
    return configured_ ? latestSample_ : CdprWrenchSample {};
}
