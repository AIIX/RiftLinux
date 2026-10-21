// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  riftlinux-input-check: what SteamVR delivers to an application from the Touch controllers, as a game sees
 *         it: every button, touch, trigger, grip and stick, and the hand skeleton's finger curls. For QA:
 *         the tester presses each in turn and the log shows what arrived.
 *
 * Usage: riftlinux-input-check <SteamVR's vrclient.so> [seconds, default 120] [actions.json]
 * A SteamVR overlay application with its own actions (resources/input-check, next to the panels) in a global action
 * set: it sees the input whatever application has focus; while it runs that application gets none of it. Prints one
 * line per change.
 */

#include "openvr.h"
#include "IVRClientCore_003.h"

#include <dlfcn.h>
#include <limits.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

namespace {

double
now_s()
{
	return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct Action
{
	std::string name;
	enum Type
	{
		Boolean,
		Vector1,
		Vector2,
		Skeleton
	} type;
	vr::VRActionHandle_t handle = vr::k_ulInvalidActionHandle;
	bool active = false;
	bool state = false;
	float x = 0.0f, y = 0.0f;
	float curl[vr::VRFinger_Count] = {};
	int level = -1;
};

} // namespace

int
main(int argc, char **argv)
{
	if (argc < 2) {
		fprintf(stderr, "Usage: riftlinux-input-check <vrclient.so> [seconds] [actions.json]\n");
		return 2;
	}
	const double run_s = argc > 2 ? atof(argv[2]) : 120.0;
	std::string manifest;
	if (argc > 3) {
		manifest = argv[3];
	} else {
		char exe[PATH_MAX];
		const ssize_t n = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
		if (n > 0) {
			exe[n] = '\0';
			const std::string dir(exe, std::string(exe).rfind('/'));
			manifest = dir + "/../../resources/input-check/actions.json";
		}
	}
	char real[PATH_MAX];
	if (manifest.empty() || realpath(manifest.c_str(), real) == nullptr) {
		fprintf(stderr, "riftlinux-input-check: no action manifest %s\n", manifest.c_str());
		return 1;
	}
	manifest = real;

	void *lib = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
	if (lib == nullptr) {
		fprintf(stderr, "riftlinux-input-check: %s\n", dlerror());
		return 1;
	}
	using Factory = void *(*)(const char *, int *);
	auto factory = reinterpret_cast<Factory>(dlsym(lib, "VRClientCoreFactory"));
	int rc = 0;
	auto *core = factory != nullptr
	                 ? static_cast<vr::IVRClientCore_003 *>(factory(vr::IVRClientCore_003_Version, &rc))
	                 : nullptr;
	if (core == nullptr) {
		fprintf(stderr, "riftlinux-input-check: no IVRClientCore_003 in %s\n", argv[1]);
		return 1;
	}
	vr::EVRInitError err = core->Init(vr::VRApplication_Overlay, nullptr);
	if (err != vr::VRInitError_None) {
		fprintf(stderr, "riftlinux-input-check: SteamVR said %s\n", core->GetIDForVRInitError(err));
		return 1;
	}
	auto *input = static_cast<vr::IVRInput *>(core->GetGenericInterface(vr::IVRInput_Version, &err));
	auto *system = static_cast<vr::IVRSystem *>(core->GetGenericInterface(vr::IVRSystem_Version, &err));
	if (input == nullptr || system == nullptr) {
		fprintf(stderr, "riftlinux-input-check: no IVRInput\n");
		core->Cleanup();
		return 1;
	}
	vr::EVRInputError ierr = input->SetActionManifestPath(manifest.c_str());
	printf("manifest %s: %d\n", manifest.c_str(), static_cast<int>(ierr));

	std::vector<Action> actions;
	for (const char *side : {"left", "right"}) {
		const std::string s = side;
		for (const char *b : s == "left" ? std::vector<const char *>{"x", "y"} : std::vector<const char *>{"a", "b"}) {
			actions.push_back({s + "_" + b + "_click", Action::Boolean});
			actions.push_back({s + "_" + b + "_touch", Action::Boolean});
		}
		if (s == "left") {
			actions.push_back({"left_menu_click", Action::Boolean});
		}
		actions.push_back({s + "_trigger_value", Action::Vector1});
		actions.push_back({s + "_trigger_click", Action::Boolean});
		actions.push_back({s + "_trigger_touch", Action::Boolean});
		actions.push_back({s + "_grip_value", Action::Vector1});
		actions.push_back({s + "_grip_click", Action::Boolean});
		actions.push_back({s + "_stick", Action::Vector2});
		actions.push_back({s + "_stick_click", Action::Boolean});
		actions.push_back({s + "_stick_touch", Action::Boolean});
		actions.push_back({s + "_thumbrest_touch", Action::Boolean});
		actions.push_back({s + "_skeleton", Action::Skeleton});
	}
	for (Action &a : actions) {
		ierr = input->GetActionHandle(("/actions/check/in/" + a.name).c_str(), &a.handle);
		if (ierr != vr::VRInputError_None) {
			printf("no action %s: %d\n", a.name.c_str(), static_cast<int>(ierr));
		}
	}
	vr::VRActionSetHandle_t set_handle = vr::k_ulInvalidActionSetHandle;
	input->GetActionSetHandle("/actions/check", &set_handle);
	vr::VRActiveActionSet_t set = {};
	set.ulActionSet = set_handle;
	set.ulRestrictedToDevice = vr::k_ulInvalidInputValueHandle;
	set.ulSecondaryActionSet = vr::k_ulInvalidActionSetHandle;
	set.nPriority = vr::k_nActionSetOverlayGlobalPriorityMin;

	const double start = now_s();
	printf("checking for %.0f s: press, touch and move each control in turn\n", run_s);
	fflush(stdout);
	double next_summary = start + 5.0;
	while (now_s() - start < run_s) {
		vr::VREvent_t event;
		while (system->PollNextEvent(&event, sizeof(event))) {
			if (event.eventType == vr::VREvent_Quit) {
				core->Cleanup();
				return 0;
			}
		}
		ierr = input->UpdateActionState(&set, sizeof(set), 1);
		const double t = now_s() - start;
		for (Action &a : actions) {
			bool active = false;
			switch (a.type) {
			case Action::Boolean: {
				vr::InputDigitalActionData_t d = {};
				if (input->GetDigitalActionData(a.handle, &d, sizeof(d), vr::k_ulInvalidInputValueHandle) ==
				    vr::VRInputError_None) {
					active = d.bActive;
					if (d.bActive && d.bState != a.state) {
						printf("%7.2f  %-22s %s\n", t, a.name.c_str(), d.bState ? "ON" : "off");
						a.state = d.bState;
					}
				}
				break;
			}
			case Action::Vector1:
			case Action::Vector2: {
				vr::InputAnalogActionData_t d = {};
				if (input->GetAnalogActionData(a.handle, &d, sizeof(d), vr::k_ulInvalidInputValueHandle) ==
				    vr::VRInputError_None) {
					active = d.bActive;
					const bool moved = std::fabs(d.x - a.x) >= 0.25f || std::fabs(d.y - a.y) >= 0.25f ||
					                   ((d.x == 0.0f) != (a.x == 0.0f)) || ((d.y == 0.0f) != (a.y == 0.0f));
					if (d.bActive && moved) {
						if (a.type == Action::Vector1) {
							printf("%7.2f  %-22s %.2f\n", t, a.name.c_str(), d.x);
						} else {
							printf("%7.2f  %-22s %+.2f %+.2f\n", t, a.name.c_str(), d.x, d.y);
						}
						a.x = d.x;
						a.y = d.y;
					}
				}
				break;
			}
			case Action::Skeleton: {
				vr::InputSkeletalActionData_t d = {};
				if (input->GetSkeletalActionData(a.handle, &d, sizeof(d)) == vr::VRInputError_None) {
					active = d.bActive;
				}
				vr::EVRSkeletalTrackingLevel level = vr::VRSkeletalTracking_Estimated;
				if (active && input->GetSkeletalTrackingLevel(a.handle, &level) == vr::VRInputError_None &&
				    static_cast<int>(level) != a.level) {
					printf("%7.2f  %-22s tracking level %d (0 estimated, 1 partial, 2 full)\n", t,
					       a.name.c_str(), static_cast<int>(level));
					a.level = static_cast<int>(level);
				}
				vr::VRSkeletalSummaryData_t sum = {};
				if (active &&
				    input->GetSkeletalSummaryData(a.handle, vr::VRSummaryType_FromDevice, &sum) ==
				        vr::VRInputError_None) {
					bool moved = false;
					for (int f = 0; f < vr::VRFinger_Count; f++) {
						moved = moved || std::fabs(sum.flFingerCurl[f] - a.curl[f]) >= 0.2f;
					}
					if (moved) {
						printf("%7.2f  %-22s curl thumb %.2f index %.2f middle %.2f ring %.2f pinky %.2f\n", t,
						       a.name.c_str(), sum.flFingerCurl[0], sum.flFingerCurl[1], sum.flFingerCurl[2],
						       sum.flFingerCurl[3], sum.flFingerCurl[4]);
						for (int f = 0; f < vr::VRFinger_Count; f++) {
							a.curl[f] = sum.flFingerCurl[f];
						}
					}
				}
				break;
			}
			}
			if (active != a.active) {
				printf("%7.2f  %-22s %s\n", t, a.name.c_str(), active ? "bound" : "NOT BOUND / inactive");
				a.active = active;
			}
		}
		if (now_s() >= next_summary) {
			int bound = 0;
			for (const Action &a : actions) {
				bound += a.active ? 1 : 0;
			}
			printf("%7.2f  -- %d of %zu actions bound (update %d)\n", t, bound, actions.size(),
			       static_cast<int>(ierr));
			next_summary += 15.0;
		}
		fflush(stdout);
		std::this_thread::sleep_for(std::chrono::milliseconds(11));
	}
	core->Cleanup();
	return 0;
}
