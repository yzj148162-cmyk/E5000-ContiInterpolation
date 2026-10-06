#ifndef FORCEINTERACTIONTENSIONSHADOWVALIDATOR_H
#define FORCEINTERACTIONTENSIONSHADOWVALIDATOR_H

#include <QString>

// 2026-10-05: M0 pure-algorithm acceptance for the force-interaction tension
// backend.  It deliberately owns no UI, Trace or hardware dependencies.
class ForceInteractionTensionShadowValidator
{
public:
    static bool runSelfChecks(QString* errorMessage = nullptr);
};

#endif // FORCEINTERACTIONTENSIONSHADOWVALIDATOR_H
