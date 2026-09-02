#ifndef GUITIMINGDIAGNOSTICS_H
#define GUITIMINGDIAGNOSTICS_H

#include <QString>
#include <QStringList>
#include <QtGlobal>

#include <array>
#include <cstddef>

#pragma execution_character_set("utf-8")

// GUI_PERF_DIAG：仅用于末端遥控期间的GUI性能归因。
// 本类不继承QObject，不创建定时器，不写文件，也不参与任何控制或安全判定；
// 删除本文件及MainWindow中带GUI_PERF_DIAG标记的接入点即可整体移除。
class GuiTimingDiagnostics
{
public:
    enum class Profile {
        Inactive = 0,
        EndpointRemote
    };

    enum class Section {
        EventLoopGap = 0,
        ControlSnapshotDispatch,
        RemoteLeasePublish,
        ControlConfigSync,
        ControlSnapshotApply,
        RemoteStatusRefresh,
        OverspeedLatchCheck,
        ControlBoxCommunicationCheck,
        ForwardKinematicsUi,
        CableHomeUi,
        RunModeUi,
        SafetyConfigSync,
        UdpStatusUpdate,
        RuntimeDiagnostics,
        UiEventLogTotal,
        UiEventLogMkdir,
        UiEventLogOpen,
        UiEventLogWrite,
        UiEventLogClose,
        StructuredFaultLogTotal,
        StructuredFaultLogMkdir,
        StructuredFaultLogRead,
        StructuredFaultLogParse,
        StructuredFaultLogSerialize,
        StructuredFaultLogOpen,
        StructuredFaultLogWrite,
        StructuredFaultLogFlushClose,
        Count
    };

    enum class LeaseTransition {
        StaleLatched = 0,
        Recovered
    };

    class ScopedSection
    {
    public:
        ScopedSection(GuiTimingDiagnostics* diagnostics,
                      Section section,
                      quint64 workItemCount = 0,
                      quint64 dataSizeBytes = 0);
        ~ScopedSection();

        void setWorkItemCount(quint64 count);
        void setDataSizeBytes(quint64 bytes);

        ScopedSection(const ScopedSection&) = delete;
        ScopedSection& operator=(const ScopedSection&) = delete;

    private:
        GuiTimingDiagnostics* diagnostics = nullptr;
        Section section = Section::EventLoopGap;
        qint64 startedUs = 0;
        quint64 workItemCount = 0;
        quint64 dataSizeBytes = 0;
    };

    void startProfile(Profile profile, quint64 sessionToken);
    QStringList finishProfile(quint64 expectedSessionToken = 0);
    void discardProfile();

    Profile profile() const;
    quint64 activeSessionToken() const;

    void observeControlSnapshotTick();
    void beginControlSnapshotDispatch();
    void endControlSnapshotDispatch();
    void observeLeaseTransition(quint64 sessionToken,
                                LeaseTransition transition,
                                qint64 uiAgeUs);

private:
    struct SectionStats {
        quint64 count = 0;
        qint64 latestUs = 0;
        qint64 totalUs = 0;
        qint64 maximumUs = 0;
        quint64 over10MsCount = 0;
        quint64 over50MsCount = 0;
        quint64 over100MsCount = 0;
        quint64 over250MsCount = 0;
        quint64 latestWorkItemCount = 0;
        quint64 totalWorkItemCount = 0;
        quint64 latestDataSizeBytes = 0;
        quint64 totalDataSizeBytes = 0;
        quint64 maximumDataSizeBytes = 0;
    };

    struct SlowEvent {
        Section section = Section::Count;
        qint64 completedUs = 0;
        qint64 durationUs = 0;
    };

    static constexpr std::size_t kSectionCount =
            static_cast<std::size_t>(Section::Count);
    static constexpr std::size_t kSlowEventCapacity = 128;
    static constexpr std::size_t kDispatchStackCapacity = 8;

    static qint64 monotonicNowUs();
    static const char* sectionName(Section section);
    static int correlationSpecificity(Section section);
    bool isCapturing() const;
    void resetSamples();
    void observeSection(Section section,
                        qint64 durationUs,
                        quint64 workItemCount,
                        quint64 dataSizeBytes);
    QString formatSectionStats(Section section) const;
    QString formatFileSectionStats(Section section) const;

    Profile currentProfile = Profile::Inactive;
    quint64 sessionToken = 0;
    qint64 profileStartedUs = 0;
    qint64 lastControlSnapshotTickUs = 0;
    std::array<SectionStats, kSectionCount> sectionStats{};
    std::array<SlowEvent, kSlowEventCapacity> slowEvents{};
    std::size_t slowEventWriteIndex = 0;
    std::size_t slowEventCount = 0;
    std::array<qint64, kDispatchStackCapacity> dispatchStartStack{};
    std::size_t dispatchDepth = 0;
    quint64 dispatchStackOverflowCount = 0;
    quint64 leaseStaleCount = 0;
    quint64 leaseRecoveredCount = 0;
    qint64 latestLeaseAgeUs = -1;
    qint64 maximumLeaseAgeUs = -1;
    qint64 latestLeaseStaleObservedUs = 0;
    Section leaseCorrelatedSection = Section::Count;
    qint64 leaseCorrelatedDurationUs = 0;
};

#endif // GUITIMINGDIAGNOSTICS_H
