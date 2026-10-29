// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The driver's 32-bit build for Steam's own process: only the watchdog (ovrd_watchdog.hpp), which Steam loads
 *         from bin/linux32/ while SteamVR is not running. SteamVR itself loads the 64-bit driver from bin/linux64/.
 *
 * Built by scripts/riftlinux/steamrt-build.sh with -m32, the C++ library linked in statically.
 */

#include "ovrd_watchdog.hpp"

#include <cstring>

namespace {

class WatchdogProvider : public vr::IVRWatchdogProvider
{
public:
	vr::EVRInitError
	Init(vr::IVRDriverContext *pDriverContext) override
	{
		VR_INIT_WATCHDOG_DRIVER_CONTEXT(pDriverContext);
		m_watchdog.start();
		return vr::VRInitError_None;
	}

	void
	Cleanup() override
	{
		m_watchdog.stop();
		VR_CLEANUP_WATCHDOG_DRIVER_CONTEXT();
	}

private:
	rl::watchdog::Watchdog m_watchdog;
};

WatchdogProvider g_provider;

} // namespace

extern "C" __attribute__((visibility("default"))) void *
HmdDriverFactory(const char *pInterfaceName, int *pReturnCode)
{
	if (pInterfaceName != nullptr && strcmp(pInterfaceName, vr::IVRWatchdogProvider_Version) == 0) {
		return &g_provider;
	}
	if (pReturnCode != nullptr) {
		*pReturnCode = vr::VRInitError_Init_InterfaceNotFound;
	}
	return nullptr;
}
