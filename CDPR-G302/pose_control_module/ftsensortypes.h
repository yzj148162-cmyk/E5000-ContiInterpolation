#ifndef FTSENSORTYPES_H
#define FTSENSORTYPES_H

#include <array>

#include <QtGlobal>

constexpr int kFtSensorWrenchChannelCount = 6;

struct FtSensorTraceSample
{
    std::array<qint32, kFtSensorWrenchChannelCount> raw{};
    std::array<double, kFtSensorWrenchChannelCount> value{};
    std::array<bool, kFtSensorWrenchChannelCount> channelValid{};
    quint32 statusCode = 0;
    quint32 sampleCounter = 0;
    qint32 temperatureRaw = 0;
    double temperatureC = 0.0;
    qint64 wallClockUs = 0;
    qint64 monotonicUs = 0;
    quint32 traceFrameSequence = 0;
    bool statusValid = false;
    bool sampleCounterValid = false;
    bool temperatureValid = false;
    bool traceFrameSequenceValid = false;

    bool wrenchComplete() const
    {
        for(bool valid : channelValid){
            if(!valid){
                return false;
            }
        }
        return true;
    }
};

#endif // FTSENSORTYPES_H
