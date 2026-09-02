#include "softwarefaultguard.h"

#include "hardwareinterface.h"

#include <QDir>
#include <QFileInfo>
#include <qt_windows.h>

#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <exception>
#include <string>

namespace SoftwareFaultGuard {
namespace {

std::atomic<HardwareInterface*> gSoftwareFaultGuardHardware{nullptr};
std::atomic_bool gSoftwareFaultGuardInstalled{false};
std::atomic_bool gSoftwareFaultGuardHandling{false};
std::wstring gSoftwareFaultGuardLogPath;

const char* softwareGuardEventDisplayText(const char* eventType)
{
    if(eventType && std::strcmp(eventType, "process_unhandled_exception") == 0){
        return "\\u8fdb\\u7a0b\\u672a\\u5904\\u7406\\u5f02\\u5e38";
    }
    if(eventType && std::strcmp(eventType, "process_terminate") == 0){
        return "\\u8fdb\\u7a0b\\u5f02\\u5e38\\u7ec8\\u6b62";
    }
    if(eventType && std::strcmp(eventType, "process_signal") == 0){
        return "\\u8fdb\\u7a0b\\u4fe1\\u53f7\\u5f02\\u5e38";
    }
    return "\\u8f6f\\u4ef6\\u6545\\u969c\\u4e8b\\u4ef6";
}

std::string buildSoftwareFaultGuardRecordLine(const char* eventType,
                                              const char* source,
                                              const char* summary,
                                              const char* detail,
                                              bool stopAttempted,
                                              bool stopSucceeded,
                                              unsigned long extraCode = 0)
{
    SYSTEMTIME now;
    GetLocalTime(&now);

    FILETIME utcFileTime;
    GetSystemTimeAsFileTime(&utcFileTime);
    ULARGE_INTEGER fileTimeValue;
    fileTimeValue.LowPart = utcFileTime.dwLowDateTime;
    fileTimeValue.HighPart = utcFileTime.dwHighDateTime;
    constexpr unsigned long long kUnixEpochFileTime100Ns = 116444736000000000ULL;
    const unsigned long long occurredAtMs =
            fileTimeValue.QuadPart > kUnixEpochFileTime100Ns ?
                (fileTimeValue.QuadPart - kUnixEpochFileTime100Ns) / 10000ULL :
                0ULL;

    char timestamp[64] = {};
    std::snprintf(timestamp,
                  sizeof(timestamp),
                  "%04d-%02d-%02dT%02d:%02d:%02d.%03d",
                  now.wYear,
                  now.wMonth,
                  now.wDay,
                  now.wHour,
                  now.wMinute,
                  now.wSecond,
                  now.wMilliseconds);

    const char* eventTypeText = softwareGuardEventDisplayText(eventType);
    const char* faultTypeText = "\\u8f6f\\u4ef6\\u5361\\u6b7b/\\u5d29\\u6e83";
    const char* stopLevelText = "\\u5b89\\u5168\\u6025\\u505c";

    char buffer[4096] = {};
    if(extraCode != 0){
        std::snprintf(buffer,
                      sizeof(buffer),
                      "{\"schema_version\":2,\"occurred_at\":\"%s\",\"occurred_at_ms\":%llu,"
                      "\"logged_at\":\"%s\",\"logged_at_ms\":%llu,"
                      "\"event_type\":\"%s\",\"event_type_text\":\"%s\","
                      "\"fault_code\":\"software_hang\",\"fault_code_value\":10,"
                      "\"fault_type\":\"software_hang\",\"fault_type_text\":\"%s\","
                      "\"stop_level\":\"emergency_stop\",\"stop_level_text\":\"%s\","
                      "\"stop_level_value\":3,\"source\":\"%s\","
                      "\"summary\":\"%s\",\"detail\":\"%s\","
                      "\"display_message\":\"%s; fault_type=%s; occurred_at=%s; "
                      "stop_level=%s; summary=%s; detail=%s\","
                      "\"reset_required\":true,"
                      "\"stop_action_attempted\":%s,\"stop_action_succeeded\":%s,"
                      "\"exception_code\":\"0x%08lX\"}\n",
                      timestamp,
                      occurredAtMs,
                      timestamp,
                      occurredAtMs,
                      eventType,
                      eventTypeText,
                      faultTypeText,
                      stopLevelText,
                      source,
                      summary,
                      detail,
                      eventTypeText,
                      faultTypeText,
                      timestamp,
                      stopLevelText,
                      summary,
                      detail,
                      stopAttempted ? "true" : "false",
                      stopSucceeded ? "true" : "false",
                      extraCode);
    }
    else{
        std::snprintf(buffer,
                      sizeof(buffer),
                      "{\"schema_version\":2,\"occurred_at\":\"%s\",\"occurred_at_ms\":%llu,"
                      "\"logged_at\":\"%s\",\"logged_at_ms\":%llu,"
                      "\"event_type\":\"%s\",\"event_type_text\":\"%s\","
                      "\"fault_code\":\"software_hang\",\"fault_code_value\":10,"
                      "\"fault_type\":\"software_hang\",\"fault_type_text\":\"%s\","
                      "\"stop_level\":\"emergency_stop\",\"stop_level_text\":\"%s\","
                      "\"stop_level_value\":3,\"source\":\"%s\","
                      "\"summary\":\"%s\",\"detail\":\"%s\","
                      "\"display_message\":\"%s; fault_type=%s; occurred_at=%s; "
                      "stop_level=%s; summary=%s; detail=%s\","
                      "\"reset_required\":true,"
                      "\"stop_action_attempted\":%s,\"stop_action_succeeded\":%s}\n",
                      timestamp,
                      occurredAtMs,
                      timestamp,
                      occurredAtMs,
                      eventType,
                      eventTypeText,
                      faultTypeText,
                      stopLevelText,
                      source,
                      summary,
                      detail,
                      eventTypeText,
                      faultTypeText,
                      timestamp,
                      stopLevelText,
                      summary,
                      detail,
                      stopAttempted ? "true" : "false",
                      stopSucceeded ? "true" : "false");
    }
    return std::string(buffer);
}

void appendSoftwareFaultGuardRecord(const char* eventType,
                                    const char* source,
                                    const char* summary,
                                    const char* detail,
                                    bool stopAttempted,
                                    bool stopSucceeded,
                                    unsigned long extraCode = 0)
{
    if(gSoftwareFaultGuardLogPath.empty()){
        return;
    }

    HANDLE fileHandle = CreateFileW(gSoftwareFaultGuardLogPath.c_str(),
                                    FILE_APPEND_DATA,
                                    FILE_SHARE_READ | FILE_SHARE_WRITE,
                                    nullptr,
                                    OPEN_ALWAYS,
                                    FILE_ATTRIBUTE_NORMAL,
                                    nullptr);
    if(fileHandle == INVALID_HANDLE_VALUE){
        return;
    }

    const std::string line = buildSoftwareFaultGuardRecordLine(eventType,
                                                               source,
                                                               summary,
                                                               detail,
                                                               stopAttempted,
                                                               stopSucceeded,
                                                               extraCode);
    DWORD written = 0;
    WriteFile(fileHandle,
              line.data(),
              static_cast<DWORD>(line.size()),
              &written,
              nullptr);
    CloseHandle(fileHandle);
}

bool attemptSoftwareFaultGuardEmergencyStop()
{
    HardwareInterface* hardware = gSoftwareFaultGuardHardware.load();
    if(!hardware){
        return false;
    }
    return hardware->emergencyStopAll();
}

LONG WINAPI softwareFaultGuardUnhandledExceptionFilter(EXCEPTION_POINTERS* exceptionInfo)
{
    if(!gSoftwareFaultGuardHandling.exchange(true)){
        const unsigned long exceptionCode =
                exceptionInfo && exceptionInfo->ExceptionRecord ?
                    exceptionInfo->ExceptionRecord->ExceptionCode : 0UL;
        const bool stopOk = attemptSoftwareFaultGuardEmergencyStop();
        appendSoftwareFaultGuardRecord("process_unhandled_exception",
                                       "SetUnhandledExceptionFilter",
                                       "software_crash_guard_triggered",
                                       "Unhandled exception captured; best-effort emergency stop issued before process termination",
                                       true,
                                       stopOk,
                                       exceptionCode);
    }
    return EXCEPTION_EXECUTE_HANDLER;
}

void softwareFaultGuardTerminateHandler()
{
    if(!gSoftwareFaultGuardHandling.exchange(true)){
        const bool stopOk = attemptSoftwareFaultGuardEmergencyStop();
        appendSoftwareFaultGuardRecord("process_terminate",
                                       "std::terminate",
                                       "software_crash_guard_triggered",
                                       "Unhandled terminate reached; best-effort emergency stop issued before process termination",
                                       true,
                                       stopOk);
    }
    TerminateProcess(GetCurrentProcess(), 3);
}

void softwareFaultGuardSignalHandler(int signalNumber)
{
    if(!gSoftwareFaultGuardHandling.exchange(true)){
        char detail[128] = {};
        std::snprintf(detail,
                      sizeof(detail),
                      "Fatal C signal %d captured; best-effort emergency stop issued before process termination",
                      signalNumber);
        const bool stopOk = attemptSoftwareFaultGuardEmergencyStop();
        appendSoftwareFaultGuardRecord("process_signal",
                                       "std::signal",
                                       "software_crash_guard_triggered",
                                       detail,
                                       true,
                                       stopOk,
                                       static_cast<unsigned long>(signalNumber));
    }
    TerminateProcess(GetCurrentProcess(), static_cast<UINT>(128 + signalNumber));
}

} // namespace

void install(HardwareInterface* hardware, const QString& logPath)
{
    QDir().mkpath(QFileInfo(logPath).absolutePath());
    gSoftwareFaultGuardLogPath = QDir::toNativeSeparators(logPath).toStdWString();
    gSoftwareFaultGuardHardware.store(hardware);
    gSoftwareFaultGuardHandling.store(false);

    if(gSoftwareFaultGuardInstalled.exchange(true)){
        return;
    }

    SetUnhandledExceptionFilter(softwareFaultGuardUnhandledExceptionFilter);
    std::set_terminate(softwareFaultGuardTerminateHandler);
    std::signal(SIGABRT, softwareFaultGuardSignalHandler);
    std::signal(SIGFPE, softwareFaultGuardSignalHandler);
    std::signal(SIGILL, softwareFaultGuardSignalHandler);
    std::signal(SIGSEGV, softwareFaultGuardSignalHandler);
}

void uninstall()
{
    gSoftwareFaultGuardHardware.store(nullptr);
}

} // namespace SoftwareFaultGuard
