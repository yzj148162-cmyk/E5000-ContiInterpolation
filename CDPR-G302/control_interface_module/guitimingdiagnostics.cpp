#include "guitimingdiagnostics.h"

#include <algorithm>
#include <chrono>

namespace {
constexpr qint64 kSlowSectionThresholdUs = 10 * 1000;
constexpr qint64 kLeaseCorrelationWindowUs = 1000 * 1000;
}

GuiTimingDiagnostics::ScopedSection::ScopedSection(
        GuiTimingDiagnostics* timingDiagnostics,
        Section timingSection,
        quint64 timingWorkItemCount,
        quint64 timingDataSizeBytes)
    : diagnostics(timingDiagnostics),
      section(timingSection),
      workItemCount(timingWorkItemCount),
      dataSizeBytes(timingDataSizeBytes)
{
    if(diagnostics && diagnostics->isCapturing()){
        startedUs = GuiTimingDiagnostics::monotonicNowUs();
    }
    else{
        diagnostics = nullptr;
    }
}

GuiTimingDiagnostics::ScopedSection::~ScopedSection()
{
    if(!diagnostics || startedUs <= 0){
        return;
    }
    const qint64 completedUs = GuiTimingDiagnostics::monotonicNowUs();
    diagnostics->observeSection(section,
                                std::max<qint64>(0, completedUs - startedUs),
                                workItemCount,
                                dataSizeBytes);
}

void GuiTimingDiagnostics::ScopedSection::setWorkItemCount(quint64 count)
{
    workItemCount = count;
}

void GuiTimingDiagnostics::ScopedSection::setDataSizeBytes(quint64 bytes)
{
    dataSizeBytes = bytes;
}

qint64 GuiTimingDiagnostics::monotonicNowUs()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<qint64>(
                std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

const char* GuiTimingDiagnostics::sectionName(Section section)
{
    switch(section){
    case Section::EventLoopGap:
        return "事件循环间隔";
    case Section::ControlSnapshotDispatch:
        return "50ms回调总段";
    case Section::RemoteLeasePublish:
        return "UI租约发布";
    case Section::ControlConfigSync:
        return "控制配置同步";
    case Section::ControlSnapshotApply:
        return "控制快照应用";
    case Section::RemoteStatusRefresh:
        return "遥控状态刷新";
    case Section::OverspeedLatchCheck:
        return "超速锁存检查";
    case Section::ControlBoxCommunicationCheck:
        return "控制盒通信检查";
    case Section::ForwardKinematicsUi:
        return "正解界面刷新";
    case Section::CableHomeUi:
        return "绳索零位界面";
    case Section::RunModeUi:
        return "运行模式界面";
    case Section::SafetyConfigSync:
        return "安全配置同步";
    case Section::UdpStatusUpdate:
        return "UDP状态更新";
    case Section::RuntimeDiagnostics:
        return "运行诊断更新";
    case Section::UiEventLogTotal:
        return "UI事件日志总段";
    case Section::UiEventLogMkdir:
        return "UI日志建目录";
    case Section::UiEventLogOpen:
        return "UI日志打开";
    case Section::UiEventLogWrite:
        return "UI日志写入";
    case Section::UiEventLogClose:
        return "UI日志关闭";
    case Section::StructuredFaultLogTotal:
        return "结构化故障日志总段";
    case Section::StructuredFaultLogMkdir:
        return "故障日志建目录";
    case Section::StructuredFaultLogRead:
        return "故障日志读取";
    case Section::StructuredFaultLogParse:
        return "故障日志解析";
    case Section::StructuredFaultLogSerialize:
        return "故障日志序列化";
    case Section::StructuredFaultLogOpen:
        return "故障日志打开";
    case Section::StructuredFaultLogWrite:
        return "故障日志写入";
    case Section::StructuredFaultLogFlushClose:
        return "故障日志刷新关闭";
    case Section::Count:
        return "无";
    }
    return "未知";
}

int GuiTimingDiagnostics::correlationSpecificity(Section section)
{
    switch(section){
    case Section::Count:
        return -1;
    case Section::EventLoopGap:
        return 0;
    case Section::ControlSnapshotDispatch:
    case Section::UiEventLogTotal:
    case Section::StructuredFaultLogTotal:
        return 1;
    default:
        return 2;
    }
}

bool GuiTimingDiagnostics::isCapturing() const
{
    return currentProfile == Profile::EndpointRemote;
}

void GuiTimingDiagnostics::resetSamples()
{
    sectionStats.fill(SectionStats{});
    slowEvents.fill(SlowEvent{});
    slowEventWriteIndex = 0;
    slowEventCount = 0;
    dispatchStartStack.fill(0);
    dispatchDepth = 0;
    dispatchStackOverflowCount = 0;
    lastControlSnapshotTickUs = 0;
    leaseStaleCount = 0;
    leaseRecoveredCount = 0;
    latestLeaseAgeUs = -1;
    maximumLeaseAgeUs = -1;
    latestLeaseStaleObservedUs = 0;
    leaseCorrelatedSection = Section::Count;
    leaseCorrelatedDurationUs = 0;
}

void GuiTimingDiagnostics::startProfile(Profile newProfile,
                                        quint64 newSessionToken)
{
    resetSamples();
    currentProfile = newProfile;
    sessionToken = newProfile == Profile::Inactive ? 0 : newSessionToken;
    profileStartedUs = isCapturing() ? monotonicNowUs() : 0;
}

void GuiTimingDiagnostics::discardProfile()
{
    currentProfile = Profile::Inactive;
    sessionToken = 0;
    profileStartedUs = 0;
    resetSamples();
}

GuiTimingDiagnostics::Profile GuiTimingDiagnostics::profile() const
{
    return currentProfile;
}

quint64 GuiTimingDiagnostics::activeSessionToken() const
{
    return sessionToken;
}

void GuiTimingDiagnostics::observeSection(Section section,
                                          qint64 durationUs,
                                          quint64 workItemCount,
                                          quint64 dataSizeBytes)
{
    if(!isCapturing() || section == Section::Count){
        return;
    }
    const std::size_t index = static_cast<std::size_t>(section);
    if(index >= sectionStats.size()){
        return;
    }

    SectionStats& stats = sectionStats[index];
    ++stats.count;
    stats.latestUs = durationUs;
    stats.totalUs += durationUs;
    stats.maximumUs = std::max(stats.maximumUs, durationUs);
    stats.latestWorkItemCount = workItemCount;
    stats.totalWorkItemCount += workItemCount;
    stats.latestDataSizeBytes = dataSizeBytes;
    stats.totalDataSizeBytes += dataSizeBytes;
    stats.maximumDataSizeBytes = std::max(stats.maximumDataSizeBytes,
                                         dataSizeBytes);
    if(durationUs > 10 * 1000){
        ++stats.over10MsCount;
    }
    if(durationUs > 50 * 1000){
        ++stats.over50MsCount;
    }
    if(durationUs > 100 * 1000){
        ++stats.over100MsCount;
    }
    if(durationUs > 250 * 1000){
        ++stats.over250MsCount;
    }

    const qint64 slowThresholdUs = section == Section::EventLoopGap ?
                60 * 1000 : kSlowSectionThresholdUs;
    if(durationUs >= slowThresholdUs){
        SlowEvent& event = slowEvents[slowEventWriteIndex];
        event.section = section;
        event.completedUs = monotonicNowUs();
        event.durationUs = durationUs;
        slowEventWriteIndex = (slowEventWriteIndex + 1) % slowEvents.size();
        slowEventCount = std::min(slowEventCount + 1, slowEvents.size());
    }
}

void GuiTimingDiagnostics::observeControlSnapshotTick()
{
    if(!isCapturing()){
        return;
    }
    const qint64 nowUs = monotonicNowUs();
    if(lastControlSnapshotTickUs > 0){
        const qint64 tickGapUs =
                std::max<qint64>(0, nowUs - lastControlSnapshotTickUs);
        observeSection(Section::EventLoopGap,
                       tickGapUs,
                       1,
                       0);
        if(latestLeaseStaleObservedUs > 0 &&
                nowUs - latestLeaseStaleObservedUs <=
                    kLeaseCorrelationWindowUs &&
                correlationSpecificity(leaseCorrelatedSection) <
                    correlationSpecificity(Section::EventLoopGap)){
            leaseCorrelatedSection = Section::EventLoopGap;
            leaseCorrelatedDurationUs = tickGapUs;
        }
    }
    lastControlSnapshotTickUs = nowUs;
}

void GuiTimingDiagnostics::beginControlSnapshotDispatch()
{
    if(!isCapturing()){
        return;
    }
    if(dispatchDepth >= dispatchStartStack.size()){
        ++dispatchStackOverflowCount;
        return;
    }
    dispatchStartStack[dispatchDepth++] = monotonicNowUs();
}

void GuiTimingDiagnostics::endControlSnapshotDispatch()
{
    if(!isCapturing() || dispatchDepth == 0){
        return;
    }
    const qint64 startedUs = dispatchStartStack[--dispatchDepth];
    dispatchStartStack[dispatchDepth] = 0;
    observeSection(Section::ControlSnapshotDispatch,
                   std::max<qint64>(0, monotonicNowUs() - startedUs),
                   1,
                   0);
}

void GuiTimingDiagnostics::observeLeaseTransition(
        quint64 transitionSessionToken,
        LeaseTransition transition,
        qint64 uiAgeUs)
{
    if(!isCapturing() || transitionSessionToken == 0 ||
            transitionSessionToken != sessionToken){
        return;
    }
    latestLeaseAgeUs = uiAgeUs;
    maximumLeaseAgeUs = std::max(maximumLeaseAgeUs, uiAgeUs);
    if(transition == LeaseTransition::Recovered){
        ++leaseRecoveredCount;
        return;
    }

    ++leaseStaleCount;
    const qint64 nowUs = monotonicNowUs();
    latestLeaseStaleObservedUs = nowUs;
    Section correlatedSection = Section::Count;
    qint64 correlatedDurationUs = 0;
    for(std::size_t offset = 0; offset < slowEventCount; ++offset){
        const std::size_t index =
                (slowEventWriteIndex + slowEvents.size() - 1 - offset) %
                slowEvents.size();
        const SlowEvent& event = slowEvents[index];
        if(event.section == Section::Count || event.completedUs <= 0){
            continue;
        }
        if(nowUs - event.completedUs > kLeaseCorrelationWindowUs){
            break;
        }
        const int eventSpecificity = correlationSpecificity(event.section);
        const int currentSpecificity = correlationSpecificity(correlatedSection);
        if(eventSpecificity > currentSpecificity ||
                (eventSpecificity == currentSpecificity &&
                 event.durationUs > correlatedDurationUs)){
            correlatedSection = event.section;
            correlatedDurationUs = event.durationUs;
        }
    }
    const int correlatedSpecificity = correlationSpecificity(correlatedSection);
    const int previousSpecificity = correlationSpecificity(leaseCorrelatedSection);
    if(correlatedSpecificity > previousSpecificity ||
            (correlatedSpecificity == previousSpecificity &&
             correlatedDurationUs >= leaseCorrelatedDurationUs)){
        leaseCorrelatedSection = correlatedSection;
        leaseCorrelatedDurationUs = correlatedDurationUs;
    }
}

QString GuiTimingDiagnostics::formatSectionStats(Section section) const
{
    const SectionStats& stats =
            sectionStats[static_cast<std::size_t>(section)];
    if(stats.count == 0){
        return QStringLiteral("%1=无样本")
                .arg(QString::fromUtf8(sectionName(section)));
    }
    return QStringLiteral(
                "%1=%2次 最近/均值/最大=%3/%4/%5 us，>10/50/100/250ms=%6/%7/%8/%9")
            .arg(QString::fromUtf8(sectionName(section)))
            .arg(stats.count)
            .arg(stats.latestUs)
            .arg(stats.totalUs / static_cast<qint64>(stats.count))
            .arg(stats.maximumUs)
            .arg(stats.over10MsCount)
            .arg(stats.over50MsCount)
            .arg(stats.over100MsCount)
            .arg(stats.over250MsCount);
}

QString GuiTimingDiagnostics::formatFileSectionStats(Section section) const
{
    const SectionStats& stats =
            sectionStats[static_cast<std::size_t>(section)];
    QString result = formatSectionStats(section);
    if(stats.count > 0 && stats.totalWorkItemCount > 0){
        result += QStringLiteral("，工作项最近/累计=%1/%2")
                .arg(stats.latestWorkItemCount)
                .arg(stats.totalWorkItemCount);
    }
    if(stats.count > 0 && stats.totalDataSizeBytes > 0){
        result += QStringLiteral("，数据最近/累计/最大=%1/%2/%3 B")
                .arg(stats.latestDataSizeBytes)
                .arg(stats.totalDataSizeBytes)
                .arg(stats.maximumDataSizeBytes);
    }
    return result;
}

QStringList GuiTimingDiagnostics::finishProfile(quint64 expectedSessionToken)
{
    if(!isCapturing()){
        return {};
    }
    if(expectedSessionToken != 0 && expectedSessionToken != sessionToken){
        return {};
    }

    const quint64 finishedSessionToken = sessionToken;
    const qint64 durationUs = profileStartedUs > 0 ?
                std::max<qint64>(0, monotonicNowUs() - profileStartedUs) : 0;

    // 先退出profile，再构造QString和输出摘要；格式化开销不会污染采样。
    currentProfile = Profile::Inactive;
    sessionToken = 0;
    profileStartedUs = 0;
    lastControlSnapshotTickUs = 0;
    dispatchDepth = 0;

    QStringList lines;
    lines << QStringLiteral(
                 "GUI性能归因[最终]：profile=EndpointRemote，会话=%1，时长=%2 ms，UI租约超时/恢复=%3/%4，最近/最大租约年龄=%5/%6 us，分段栈溢出=%7")
             .arg(finishedSessionToken)
             .arg(durationUs / 1000)
             .arg(leaseStaleCount)
             .arg(leaseRecoveredCount)
             .arg(latestLeaseAgeUs)
             .arg(maximumLeaseAgeUs)
             .arg(dispatchStackOverflowCount);
    lines << QStringLiteral("GUI性能归因[周期]：%1；%2；%3；%4；%5；%6；%7")
             .arg(formatSectionStats(Section::EventLoopGap),
                  formatSectionStats(Section::ControlSnapshotDispatch),
                  formatSectionStats(Section::RemoteLeasePublish),
                  formatSectionStats(Section::ControlConfigSync),
                  formatSectionStats(Section::ControlSnapshotApply),
                  formatSectionStats(Section::RemoteStatusRefresh),
                  formatSectionStats(Section::ForwardKinematicsUi));
    lines << QStringLiteral("GUI性能归因[周期续]：%1；%2；%3；%4；%5；%6；%7")
             .arg(formatSectionStats(Section::OverspeedLatchCheck),
                  formatSectionStats(Section::ControlBoxCommunicationCheck),
                  formatSectionStats(Section::CableHomeUi),
                  formatSectionStats(Section::RunModeUi),
                  formatSectionStats(Section::SafetyConfigSync),
                  formatSectionStats(Section::UdpStatusUpdate),
                  formatSectionStats(Section::RuntimeDiagnostics));
    lines << QStringLiteral("GUI性能归因[UI事件日志]：%1；%2；%3；%4；%5")
             .arg(formatFileSectionStats(Section::UiEventLogTotal),
                  formatSectionStats(Section::UiEventLogMkdir),
                  formatSectionStats(Section::UiEventLogOpen),
                  formatFileSectionStats(Section::UiEventLogWrite),
                  formatSectionStats(Section::UiEventLogClose));
    lines << QStringLiteral("GUI性能归因[结构化故障日志]：%1；%2；%3；%4；%5；%6；%7；%8")
             .arg(formatFileSectionStats(Section::StructuredFaultLogTotal),
                  formatSectionStats(Section::StructuredFaultLogMkdir),
                  formatFileSectionStats(Section::StructuredFaultLogRead),
                  formatSectionStats(Section::StructuredFaultLogParse),
                  formatSectionStats(Section::StructuredFaultLogOpen),
                  formatFileSectionStats(Section::StructuredFaultLogSerialize),
                  formatFileSectionStats(Section::StructuredFaultLogWrite),
                  formatSectionStats(Section::StructuredFaultLogFlushClose));
    lines << QStringLiteral("GUI性能归因[租约关联]：最近1 s慢段最大候选=%1/%2 us；仅表示时间相关，不自动判定因果")
             .arg(QString::fromUtf8(sectionName(leaseCorrelatedSection)))
             .arg(leaseCorrelatedDurationUs);
    return lines;
}
