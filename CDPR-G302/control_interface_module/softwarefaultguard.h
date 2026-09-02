#ifndef SOFTWAREFAULTGUARD_H
#define SOFTWAREFAULTGUARD_H

#include <QString>

class HardwareInterface;

// 进程级致命异常保护：记录故障并尽力向硬件发出急停。
namespace SoftwareFaultGuard {

void install(HardwareInterface* hardware, const QString& logPath);
void uninstall();

} // namespace SoftwareFaultGuard

#endif // SOFTWAREFAULTGUARD_H
