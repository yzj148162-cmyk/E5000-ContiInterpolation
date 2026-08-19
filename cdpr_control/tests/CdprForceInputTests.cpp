#include "cdpr/CdprForceInput.h"

#include <QCoreApplication>
#include <QDebug>

#include <cmath>

namespace {
bool near(double left, double right, double tolerance = 1.0e-9)
{
    return std::abs(left - right) <= tolerance;
}

bool require(bool condition, const QString &message)
{
    if (!condition) qCritical().noquote() << message;
    return condition;
}
}

int main(int argc, char *argv[])
{
    QCoreApplication application(argc, argv);
    Q_UNUSED(application)

    SimulatedWrenchSource source;
    QString error;

    CdprSimulatedWrenchProfile constant;
    constant.value = {1, 2, 3, 4, 5, 6};
    if (!require(source.configure(constant, &error), error)) return 1;
    CdprVector6 wrench {};
    if (!require(source.evaluate(12.0, wrench, &error), error)
        || !require(near(wrench[0], 1.0) && near(wrench[5], 6.0),
                    QStringLiteral("定值模式结果错误。"))) return 1;

    CdprSimulatedWrenchProfile pulse;
    pulse.mode = CdprSimulatedWrenchMode::Pulse;
    pulse.value[0] = 5.0;
    pulse.bias[0] = 1.0;
    pulse.pulseStartS = 0.2;
    pulse.pulseDurationS = 0.3;
    if (!require(source.configure(pulse, &error), error)) return 1;
    source.evaluate(0.1, wrench, &error);
    if (!require(near(wrench[0], 1.0), QStringLiteral("脉冲前偏置错误。"))) return 1;
    source.evaluate(0.3, wrench, &error);
    if (!require(near(wrench[0], 6.0), QStringLiteral("脉冲区间结果错误。"))) return 1;
    source.evaluate(0.5, wrench, &error);
    if (!require(near(wrench[0], 1.0), QStringLiteral("脉冲结束边界错误。"))) return 1;

    CdprSimulatedWrenchProfile sine;
    sine.mode = CdprSimulatedWrenchMode::Sine;
    sine.value[1] = 2.0;
    sine.bias[1] = 3.0;
    sine.sineFrequencyHz = 0.5;
    sine.sinePhaseRad = 0.0;
    if (!require(source.configure(sine, &error), error)) return 1;
    source.evaluate(0.5, wrench, &error);
    if (!require(near(wrench[1], 5.0, 1.0e-8),
                 QStringLiteral("正弦模式结果错误。"))) return 1;

    CdprSimulatedWrenchProfile formula;
    formula.mode = CdprSimulatedWrenchMode::Formula;
    formula.expressions[0] = QStringLiteral("5*sin(2*pi*0.5*t)");
    formula.expressions[3] = QStringLiteral("abs(-2)+sqrt(4)");
    if (!require(source.configure(formula, &error), error)) return 1;
    source.evaluate(0.5, wrench, &error);
    if (!require(near(wrench[0], 5.0, 1.0e-8)
                 && near(wrench[3], 4.0, 1.0e-8),
                 QStringLiteral("自定义公式结果错误。"))) return 1;

    formula.expressions[0] = QStringLiteral("sin(");
    if (!require(!source.configure(formula, &error)
                 && !error.isEmpty(),
                 QStringLiteral("非法表达式未被拒绝。"))) return 1;
    source.evaluate(0.5, wrench, &error);
    if (!require(near(wrench[0], 5.0, 1.0e-8),
                 QStringLiteral("非法表达式破坏了上一份有效配置。"))) return 1;

    qInfo() << "CdprForceInputTests passed";
    return 0;
}
