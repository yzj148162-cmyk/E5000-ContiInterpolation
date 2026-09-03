#include "wrenchsource.h"

#include <cmath>
#include <limits>
#include <memory>
#include <utility>

#include <QStringList>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMaximumForceN = 10000.0;
constexpr double kMaximumMomentNm = 10000.0;

bool finite6(const ForceInteractionVector6& values)
{
    for(double value : values){
        if(!std::isfinite(value)){
            return false;
        }
    }
    return true;
}

struct ExpressionNode
{
    enum class Kind {
        Constant, Time, Add, Subtract, Multiply, Divide,
        Power, Negate, Function
    };
    Kind kind = Kind::Constant;
    double value = 0.0;
    QString function;
    std::shared_ptr<ExpressionNode> left;
    std::shared_ptr<ExpressionNode> right;

    double evaluate(double timeS) const
    {
        switch(kind){
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
            if(function == QStringLiteral("sin")) return std::sin(argument);
            if(function == QStringLiteral("cos")) return std::cos(argument);
            if(function == QStringLiteral("tan")) return std::tan(argument);
            if(function == QStringLiteral("exp")) return std::exp(argument);
            if(function == QStringLiteral("sqrt")) return std::sqrt(argument);
            if(function == QStringLiteral("abs")) return std::abs(argument);
            if(function == QStringLiteral("log")) return std::log(argument);
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
        : text_(std::move(expression))
    {
    }

    std::shared_ptr<ExpressionNode> parse(QString* errorMessage)
    {
        position_ = 0;
        error_.clear();
        const auto result = parseExpression();
        skipSpace();
        if(result && position_ != text_.size() && error_.isEmpty()){
            error_ = QStringLiteral("位置%1附近存在无法识别的内容").arg(position_ + 1);
        }
        if(!result || !error_.isEmpty()){
            if(errorMessage){
                *errorMessage = error_.isEmpty() ?
                            QStringLiteral("表达式为空或不完整") : error_;
            }
            return {};
        }
        if(errorMessage){
            errorMessage->clear();
        }
        return result;
    }

private:
    std::shared_ptr<ExpressionNode> parseExpression()
    {
        auto left = parseTerm();
        while(left){
            skipSpace();
            if(!consume('+') && !consume('-')){
                break;
            }
            const QChar operation = text_.at(position_ - 1);
            auto right = parseTerm();
            if(!right){
                return {};
            }
            auto node = std::make_shared<ExpressionNode>();
            node->kind = operation == '+' ? ExpressionNode::Kind::Add :
                                             ExpressionNode::Kind::Subtract;
            node->left = left;
            node->right = right;
            left = node;
        }
        return left;
    }

    std::shared_ptr<ExpressionNode> parseTerm()
    {
        auto left = parsePower();
        while(left){
            skipSpace();
            if(!consume('*') && !consume('/')){
                break;
            }
            const QChar operation = text_.at(position_ - 1);
            auto right = parsePower();
            if(!right){
                return {};
            }
            auto node = std::make_shared<ExpressionNode>();
            node->kind = operation == '*' ? ExpressionNode::Kind::Multiply :
                                             ExpressionNode::Kind::Divide;
            node->left = left;
            node->right = right;
            left = node;
        }
        return left;
    }

    std::shared_ptr<ExpressionNode> parsePower()
    {
        auto left = parseUnary();
        if(!left){
            return {};
        }
        skipSpace();
        if(!consume('^')){
            return left;
        }
        auto right = parsePower();
        if(!right){
            return {};
        }
        auto node = std::make_shared<ExpressionNode>();
        node->kind = ExpressionNode::Kind::Power;
        node->left = left;
        node->right = right;
        return node;
    }

    std::shared_ptr<ExpressionNode> parseUnary()
    {
        skipSpace();
        if(consume('+')){
            return parseUnary();
        }
        if(consume('-')){
            auto value = parseUnary();
            if(!value){
                return {};
            }
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
        if(consume('(')){
            auto value = parseExpression();
            skipSpace();
            if(!consume(')')){
                error_ = QStringLiteral("位置%1缺少右括号").arg(position_ + 1);
                return {};
            }
            return value;
        }
        if(position_ < text_.size() &&
                (text_.at(position_).isDigit() || text_.at(position_) == '.')){
            return parseNumber();
        }
        if(position_ < text_.size() &&
                (text_.at(position_).isLetter() || text_.at(position_) == '_')){
            const QString identifier = parseIdentifier().toLower();
            if(identifier == QStringLiteral("t")){
                auto node = std::make_shared<ExpressionNode>();
                node->kind = ExpressionNode::Kind::Time;
                return node;
            }
            if(identifier == QStringLiteral("pi")){
                auto node = std::make_shared<ExpressionNode>();
                node->value = kPi;
                return node;
            }
            const QStringList functions{
                QStringLiteral("sin"), QStringLiteral("cos"),
                QStringLiteral("tan"), QStringLiteral("exp"),
                QStringLiteral("sqrt"), QStringLiteral("abs"),
                QStringLiteral("log")
            };
            if(!functions.contains(identifier)){
                error_ = QStringLiteral("未知标识符“%1”").arg(identifier);
                return {};
            }
            skipSpace();
            if(!consume('(')){
                error_ = QStringLiteral("函数%1后缺少左括号").arg(identifier);
                return {};
            }
            auto argument = parseExpression();
            skipSpace();
            if(!argument || !consume(')')){
                error_ = QStringLiteral("函数%1缺少右括号").arg(identifier);
                return {};
            }
            auto node = std::make_shared<ExpressionNode>();
            node->kind = ExpressionNode::Kind::Function;
            node->function = identifier;
            node->left = argument;
            return node;
        }
        error_ = QStringLiteral("位置%1需要数字、t、pi或函数").arg(position_ + 1);
        return {};
    }

    std::shared_ptr<ExpressionNode> parseNumber()
    {
        const int start = position_;
        bool exponentSeen = false;
        while(position_ < text_.size()){
            const QChar character = text_.at(position_);
            if(character.isDigit() || character == '.'){
                ++position_;
            }
            else if((character == 'e' || character == 'E') && !exponentSeen){
                exponentSeen = true;
                ++position_;
                if(position_ < text_.size() &&
                        (text_.at(position_) == '+' || text_.at(position_) == '-')){
                    ++position_;
                }
            }
            else{
                break;
            }
        }
        bool ok = false;
        const double value = text_.mid(start, position_ - start).toDouble(&ok);
        if(!ok || !std::isfinite(value)){
            error_ = QStringLiteral("位置%1的数字无效").arg(start + 1);
            return {};
        }
        auto node = std::make_shared<ExpressionNode>();
        node->value = value;
        return node;
    }

    QString parseIdentifier()
    {
        const int start = position_;
        while(position_ < text_.size() &&
              (text_.at(position_).isLetterOrNumber() || text_.at(position_) == '_')){
            ++position_;
        }
        return text_.mid(start, position_ - start);
    }

    void skipSpace()
    {
        while(position_ < text_.size() && text_.at(position_).isSpace()){
            ++position_;
        }
    }

    bool consume(QChar expected)
    {
        if(position_ < text_.size() && text_.at(position_) == expected){
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

} // namespace

bool SimulatedWrenchSource::configure(const SimulatedWrenchProfile& profile,
                                      double samplePeriodS,
                                      QString* errorMessage)
{
    configured_ = false;
    if(!finite6(profile.amplitude) || !finite6(profile.bias)){
        if(errorMessage){
            *errorMessage = QStringLiteral("六维力参数包含非有限数");
        }
        return false;
    }
    if(!std::isfinite(samplePeriodS) || samplePeriodS <= 0.0){
        if(errorMessage){
            *errorMessage = QStringLiteral("模拟力采样周期必须大于0");
        }
        return false;
    }
    if(profile.mode == SimulatedWrenchMode::Pulse &&
            (!std::isfinite(profile.pulseStartS) || profile.pulseStartS < 0.0 ||
             !std::isfinite(profile.pulseDurationS) || profile.pulseDurationS <= 0.0)){
        if(errorMessage){
            *errorMessage = QStringLiteral("脉冲开始时间必须非负，持续时间必须大于0");
        }
        return false;
    }
    if(profile.mode == SimulatedWrenchMode::Sine){
        const double nyquistHz = 0.5 / samplePeriodS;
        if(!std::isfinite(profile.sineFrequencyHz) ||
                profile.sineFrequencyHz <= 0.0 ||
                profile.sineFrequencyHz >= nyquistHz ||
                !std::isfinite(profile.sinePhaseRad)){
            if(errorMessage){
                *errorMessage = QStringLiteral("正弦频率必须大于0且低于奈奎斯特上限%1 Hz")
                        .arg(nyquistHz, 0, 'g', 8);
            }
            return false;
        }
    }

    std::array<std::function<double(double)>, kForceInteractionDofCount> evaluators{};
    if(profile.mode == SimulatedWrenchMode::Formula){
        for(int index = 0; index < kForceInteractionDofCount; ++index){
            QString parseError;
            ExpressionParser parser(profile.expressions[static_cast<size_t>(index)]);
            const auto root = parser.parse(&parseError);
            if(!root){
                if(errorMessage){
                    *errorMessage = QStringLiteral("第%1个公式无效：%2")
                            .arg(index + 1).arg(parseError);
                }
                return false;
            }
            evaluators[static_cast<size_t>(index)] =
                    [root](double timeS){ return root->evaluate(timeS); };
        }
    }
    else{
        for(int index = 0; index < kForceInteractionDofCount; ++index){
            double maximumMagnitude = std::abs(profile.amplitude[static_cast<size_t>(index)]);
            if(profile.mode == SimulatedWrenchMode::Pulse ||
                    profile.mode == SimulatedWrenchMode::Sine){
                maximumMagnitude += std::abs(profile.bias[static_cast<size_t>(index)]);
            }
            if(!componentWithinLimit(index, maximumMagnitude)){
                if(errorMessage){
                    *errorMessage = QStringLiteral("第%1个分量的理论峰值超出保护范围")
                            .arg(index + 1);
                }
                return false;
            }
        }
    }

    profile_ = profile;
    evaluators_ = std::move(evaluators);
    samplePeriodS_ = samplePeriodS;
    configured_ = true;
    if(errorMessage){
        errorMessage->clear();
    }
    return true;
}

SimulatedWrenchProfile SimulatedWrenchSource::profile() const
{
    return profile_;
}

bool SimulatedWrenchSource::evaluate(double elapsedS,
                                     ForceInteractionVector6& wrench,
                                     QString* errorMessage) const
{
    wrench = {};
    if(!configured_ || !std::isfinite(elapsedS) || elapsedS < 0.0){
        if(errorMessage){
            *errorMessage = QStringLiteral("模拟力源未配置或时间无效");
        }
        return false;
    }
    for(int index = 0; index < kForceInteractionDofCount; ++index){
        const size_t offset = static_cast<size_t>(index);
        switch(profile_.mode){
        case SimulatedWrenchMode::Constant:
            wrench[offset] = profile_.amplitude[offset];
            break;
        case SimulatedWrenchMode::Pulse:
            wrench[offset] = profile_.bias[offset];
            if(elapsedS >= profile_.pulseStartS &&
                    elapsedS < profile_.pulseStartS + profile_.pulseDurationS){
                wrench[offset] += profile_.amplitude[offset];
            }
            break;
        case SimulatedWrenchMode::Sine:
            wrench[offset] = profile_.bias[offset] + profile_.amplitude[offset] *
                    std::sin(2.0 * kPi * profile_.sineFrequencyHz * elapsedS +
                             profile_.sinePhaseRad);
            break;
        case SimulatedWrenchMode::Formula:
            wrench[offset] = evaluators_[offset] ?
                        evaluators_[offset](elapsedS) :
                        std::numeric_limits<double>::quiet_NaN();
            break;
        }
        if(!componentWithinLimit(index, wrench[offset])){
            if(errorMessage){
                *errorMessage = QStringLiteral("t=%1 s时第%2个模拟分量无效或超限")
                        .arg(elapsedS, 0, 'g', 10).arg(index + 1);
            }
            return false;
        }
    }
    if(errorMessage){
        errorMessage->clear();
    }
    return true;
}

ForceInteractionWrenchSample SimulatedWrenchSource::sample(
        const ForceInteractionFrameStamp& stamp,
        double elapsedS) const
{
    ForceInteractionWrenchSample result;
    result.stamp = stamp;
    // 模拟量与未来真实F/T Trace保持相同语义：六个分量均为传感器
    // 测量参考点、传感器坐标系S下的原始力旋量。进入动力学前必须统一
    // 经过WrenchTransformer，不能绕过安装旋转和r x F力矩平移。
    result.coordinate = ForceInteractionWrenchCoordinate::Sensor;
    QString errorMessage;
    result.valid = stamp.valid && evaluate(elapsedS, result.wrench, &errorMessage);
    return result;
}

QString SimulatedWrenchSource::summary() const
{
    if(!configured_){
        return QStringLiteral("模拟力源未配置");
    }
    static const QStringList names{
        QStringLiteral("常值"), QStringLiteral("脉冲"),
        QStringLiteral("正弦"), QStringLiteral("公式")
    };
    return QStringLiteral("%1（传感器坐标系S原始输入），采样周期=%2 ms")
            .arg(names.value(static_cast<int>(profile_.mode), QStringLiteral("未知")))
            .arg(samplePeriodS_ * 1000.0, 0, 'f', 3);
}
