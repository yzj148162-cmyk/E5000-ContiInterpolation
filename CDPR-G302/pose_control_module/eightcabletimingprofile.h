#ifndef EIGHTCABLETIMINGPROFILE_H
#define EIGHTCABLETIMINGPROFILE_H
#include <QtGlobal>
#include <array>

// One versioned catalogue shared by UI, admission, runtime and recording.
// Budgets are engineering limits, not a hard-real-time capability claim.
struct EightCableTimingProfile
{
    int version = 3; // bounded late admission with an unchanged absolute completion deadline
    qint64 innerUs = 0, outerUs = 0;
    qint64 innerStartLatenessUs = 3000, outerStartLatenessUs = 3000;
    qint64 outerComputeUs = 2000, outerPublishUs = 12000;
    qint64 batchExecutionUs = 2000;
    qint64 commandGapUs = 0, innerStateGapUs = 0;
    qint64 outerObservationGapUs = 0, targetLifetimeUs = 0;
    constexpr bool valid() const { return innerUs > 0 && outerUs > 0; }
    static constexpr std::array<int,4> innerOptionsUs{{5000,6000,8000,10000}};
    static constexpr bool supportsInner(qint64 value) {
        for(const int option:innerOptionsUs){if(value==option){return true;}}
        return false;
    }
    static constexpr EightCableTimingProfile lookup(qint64 inner, qint64 outer) {
        EightCableTimingProfile p;
        if(!supportsInner(inner) || outer < 5*inner || outer > 10*inner || outer%inner!=0){return p;}
        p.innerUs=inner; p.outerUs=outer;
        p.commandGapUs=2*inner+4000; p.innerStateGapUs=p.commandGapUs;
        p.outerObservationGapUs=outer+38000;
        p.targetLifetimeUs=outer+inner+13000;
        return p;
    }
    // Completion must satisfy ALL deadlines, not only the slot boundary.
    constexpr qint64 innerCommandDeadlineUs(qint64 due, qint64 lastCommit,
                                            qint64 targetValidUntil) const {
        if(!valid() || due<=0){return 0;}
        qint64 deadline=due+innerUs;
        if(lastCommit>0 && lastCommit+commandGapUs<deadline){deadline=lastCommit+commandGapUs;}
        if(targetValidUntil>0 && targetValidUntil<deadline){deadline=targetValidUntil;}
        return deadline;
    }
    constexpr bool hasBatchTime(qint64 now, qint64 deadline) const {
        return valid() && deadline>now && deadline-now>=batchExecutionUs;
    }
    static constexpr bool supportsOuter(qint64 outer) {
        for(const int inner:innerOptionsUs){if(lookup(inner,outer).valid()){return true;}}
        return false;
    }
};

// Absolute slots. Inspect/consume never reset origin after a late observation.
// Scheduling timestamps are host timestamps, independent of sensor timestamps.
struct EightCablePeriodicClock
{
    struct Tick {
        bool due=false, executable=false;
        quint64 index=0, missed=0;
        qint64 dueUs=0, startUs=0, latenessUs=0;
    };
    qint64 originUs=0, periodUs=0;
    quint64 nextIndex=0, totalMissed=0;
    void reset(qint64 origin=0,qint64 period=0){originUs=origin;periodUs=period;nextIndex=0;totalMissed=0;}
    qint64 nextDueUs() const {return originUs>0 && periodUs>0 ? originUs+qint64(nextIndex)*periodUs : 0;}
    Tick take(qint64 now,qint64 maximumLatenessUs){
        Tick t;
        if(originUs<=0 || periodUs<=0 || now<nextDueUs()){return t;}
        t.due=true;t.startUs=now;
        t.index=quint64((now-originUs)/periodUs);
        t.missed=t.index-nextIndex;
        t.dueUs=originUs+qint64(t.index)*periodUs;t.latenessUs=now-t.dueUs;
        t.executable=t.latenessUs<=maximumLatenessUs;
        if(!t.executable){++t.missed;}
        totalMissed+=t.missed;nextIndex=t.index+1;
        return t;
    }
};
struct EightCableLoopTimingDiagnostic
{
    bool enabled=false, outerPass=false, innerDue=false, outerDue=false;
    bool outerPublished=false, outerPending=false, innerLate=false;
    quint64 innerTick=0, outerTick=0, innerMissed=0, outerMissed=0;
    quint64 publishedVersion=0, selectedVersion=0, appliedVersion=0;
    qint64 innerDueUs=0, innerStartedUs=0, innerFinishedUs=0;
    qint64 outerDueUs=0, outerStartedUs=0, outerCalculationStartedUs=0, outerFinishedUs=0;
    qint64 targetSourceUs=0, targetPublishedUs=0, targetValidUntilUs=0;
    qint64 commandCompletedUs=0;
    qint64 innerWakeLatenessUs=0, innerDeadlineUs=0;
    qint64 innerBudgetCheckedUs=0, innerBudgetRemainingUs=0;
    // 0 not admitted, 1 admitted to calculate, 2 start too late,
    // 3 insufficient budget before calculation, 4 before dispatch,
    // 5 successful commit, 6 late acknowledgement, 7 hardware-queue no-write deferral.
    int innerAdmission=0;
    int outerOutcome=0; // 0 none, 1 pending, 2 published, 3 late, 4 stale epoch, 5 busy, 6 rejected
};
#endif

