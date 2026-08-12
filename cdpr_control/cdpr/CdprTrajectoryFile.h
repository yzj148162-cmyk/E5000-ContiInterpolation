#ifndef CDPRTRAJECTORYFILE_H
#define CDPRTRAJECTORYFILE_H

#include "CdprControlTypes.h"

#include <QString>

class CdprTrajectoryFile
{
public:
    static bool exportExpectedTrajectory(const CdprOfflinePvtPlan &plan,
                                         const QString &path,
                                         QString &sha256,
                                         QString &errorMessage);
    static bool prepareCache(CdprOfflinePvtPlan &plan,
                             const QString &cacheRoot,
                             QString &errorMessage);
    static bool verify(const QString &path, const QString &expectedSha256,
                       QString &errorMessage);
    static QString sha256(const QString &path, QString &errorMessage);
};

#endif // CDPRTRAJECTORYFILE_H
