#ifndef FORCEINTERACTIONTENSIONSHADOWREPLAY_H
#define FORCEINTERACTIONTENSIONSHADOWREPLAY_H

#include <QString>

struct ForceInteractionTensionShadowReplayResult
{
    bool completed = false;
    QString summary;
};

class ForceInteractionTensionShadowReplay
{
public:
    // Read-only M1 entry: reads an exported replay JSON/CSV pair and writes
    // only the normal *_kinematic_analysis.csv result beside the source log.
    static ForceInteractionTensionShadowReplayResult run(
            const QString& replayConfigPath);
};

#endif // FORCEINTERACTIONTENSIONSHADOWREPLAY_H
