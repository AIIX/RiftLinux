// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Headless replay of constellation tracker recordings.
 *
 * Feeds a recording made with CONSTELLATION_TRACKER_DATA_RECORDER_OUTPUT back through the
 * tracker's own code, deterministically and without hardware, and records what the tracker
 * does into a new recording that the analysis scripts read.
 *
 * Mode `acquire`: every camera sample goes through the slow (correspondence search) path from
 * scratch: blob labels wiped, no prior, no camera pose, rig calibration off. It answers "what
 * does acquisition find in this frame", which is what a pose scoring change has to be judged on.
 *
 * Mode `track`: the tracker as it runs live, with the recorded camera placements and the IMU fed
 * in time order: fast paths with a prior, then the slow search where they fail. The prior is the
 * device's last solves (`--prior raw`, what the Touch driver did) or the sensor fusion's output
 * (`--prior fusion`, what the HMD driver does). Blob labels are wiped on every sample, there is no
 * blobwatch to carry them, so the RANSAC blob recovery path never runs.
 *
 * @note Uses tracker internals, like the GUI's playback scene. Not an API example.
 * @ingroup tracking
 */

#include "xrt/xrt_config_build.h"
#include "xrt/xrt_defines.h"

#include "util/u_logging.h"
#include "util/u_time.h"

#include "math/m_relation_history.h"
#include "math/m_gyro_rest_bias.h"
#include "math/m_api.h"
#include "math/m_vec3.h"

#include "tracking/t_camera_models.h"

#include "constellation/t_constellation_tracker_internal.hpp"
#include "constellation/t_constellation_tracker_dataset.hpp"
#include "constellation/t_rift_blobwatch.h"

#include "util/u_frame.h"
#include "util/u_notice.h"
#include "util/u_json.h"

#include <atomic>
#include <chrono>
#include <thread>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>
#include <algorithm>

#include <sys/prctl.h>


namespace ct = xrt::tracking::constellation;

namespace {

extern "C" void
stub_push_constellation_tracker_sample(t_constellation_tracker_device *device, t_constellation_tracker_sample *sample)
{
	(void)device;
	(void)sample;
}

t_constellation_tracker_device stub_device = {
    .push_constellation_tracker_sample = stub_push_constellation_tracker_sample,
};

extern "C" void
stub_mark_blob_device(struct t_blobwatch *tbw,
                      const struct t_blob_observation *tbo,
                      t_constellation_device_id_t device_id)
{
	(void)tbw;
	(void)tbo;
	(void)device_id;
}

extern "C" void
stub_blobwatch_destroy(struct t_blobwatch *tbw)
{
	(void)tbw;
}

t_blobwatch stub_blobwatch = {
    .mark_blob_device = stub_mark_blob_device,
    .destroy = stub_blobwatch_destroy,
};

//! A device in `track` mode: keeps its solves, and hands the tracker its prior.
struct ReplayDevice
{
	t_constellation_tracker_device device;
	t_constellation_tracker_tracking_source source;
	m_relation_history *history{nullptr};
	ct::ConstellationTracker *tracker{nullptr};
	t_constellation_device_id_t id{};
	bool fusion_prior{false};
	//! The fusion is the prior only within this long of a solve, 0 for always: the Touch driver's rule.
	timepoint_ns fusion_fresh_ns{0};
	timepoint_ns last_solve_ns{0};

	/*!
	 * After that, the Touch driver hands over its IMU's orientation and the position held where it was last
	 * seen. Emulated here from the last solve, turned by the gyro since.
	 */
	bool have_hold{false};
	xrt_pose hold_pose = XRT_POSE_IDENTITY;
	timepoint_ns hold_imu_ns{0};

	void
	pushImu(const xrt_imu_sample &sample)
	{
		if (!this->have_hold || sample.timestamp_ns <= this->hold_imu_ns) {
			return;
		}

		const float dt = static_cast<float>(time_ns_to_s(sample.timestamp_ns - this->hold_imu_ns));
		this->hold_imu_ns = sample.timestamp_ns;
		if (dt > 0.1f) {
			return;
		}

		const xrt_vec3 gyro = {static_cast<float>(sample.gyro_rad_secs.x),
		                       static_cast<float>(sample.gyro_rad_secs.y),
		                       static_cast<float>(sample.gyro_rad_secs.z)};
		math_quat_integrate_velocity(&this->hold_pose.orientation, &gyro, dt, &this->hold_pose.orientation);
	}

	~ReplayDevice()
	{
		m_relation_history_destroy(&this->history);
	}
};

extern "C" void
replay_push_sample(t_constellation_tracker_device *device, t_constellation_tracker_sample *sample)
{
	ReplayDevice *rd = container_of(device, ReplayDevice, device);
	if (!sample->has_world_pose) {
		return;
	}

	if (sample->timestamp_ns >= rd->last_solve_ns) {
		rd->have_hold = true;
		rd->hold_pose = sample->world_pose;
		rd->hold_imu_ns = sample->timestamp_ns;
	}
	rd->last_solve_ns = std::max(rd->last_solve_ns, sample->timestamp_ns);

	xrt_space_relation relation = XRT_SPACE_RELATION_ZERO;
	relation.pose = sample->world_pose;
	relation.relation_flags =
	    (xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	                               XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                               XRT_SPACE_RELATION_POSITION_TRACKED_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT);
	m_relation_history_push(rd->history, &relation, sample->timestamp_ns);
}

extern "C" void
replay_get_tracked_pose(t_constellation_tracker_tracking_source *source,
                        timepoint_ns when_ns,
                        xrt_space_relation *out_relation)
{
	ReplayDevice *rd = container_of(source, ReplayDevice, source);
	*out_relation = XRT_SPACE_RELATION_ZERO;

	if (rd->fusion_prior) {
		if (rd->fusion_fresh_ns > 0 &&
		    (rd->last_solve_ns == 0 || when_ns - rd->last_solve_ns > rd->fusion_fresh_ns)) {
			if (rd->have_hold) {
				out_relation->pose = rd->hold_pose;
				out_relation->relation_flags =
				    (xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
				                               XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
				                               XRT_SPACE_RELATION_POSITION_VALID_BIT);
			}
			return;
		}
		rd->tracker->getTrackedPose(rd->id, when_ns, *out_relation, false);
	} else {
		m_relation_history_get(rd->history, when_ns, out_relation);
	}
}

struct Options
{
	std::string mode;
	std::string in_file;
	std::string out_file;
	std::string calibration_from;
	double from_s{0.0};
	double to_s{-1.0};
	uint32_t stride{1};
	int mosaic{-1};
	int camera{-1};

	/*
	 * Recordings don't carry the devices' match parameters. Unless given, a device with up to 24 LEDs gets the CV1
	 * Touch's (7 / 4, rift_touch_calibration.cpp), others the CV1 HMD's (7 / 5, rift_driver.c).
	 */
	bool match_parameters_given{false};
	uint32_t min_leds_without_prior{7};
	uint32_t min_leds_with_prior{5};

	//! `track` mode: use the sensor fusion's output as the prior, rather than the last solves.
	bool fusion_prior{false};
	//! `track` mode: IMU samples up to this long after a camera sample are in when it is processed.
	double imu_lead_ms{10.0};
	//! `track` mode: query every device's output pose this often, as an app would, 0 for never.
	double app_hz{0.0};
	//! `track` mode: query every device's output pose this long after each camera sample, once the fusion has
	//! taken it in, as an app between two cameras' samples of one exposure would; negative for never.
	double app_after_each_ms{-1.0};

	//! `track` mode: the samples carry the tracker's own camera poses, as live, instead of the recorded ones, so
	//! that what the tracker does to them (the moved-camera watch placing one again) takes effect.
	bool live_camera_poses{false};
	/*!
	 * `track` mode: from this many seconds on, the tracker takes one camera to be moved by this much in its own
	 * frame (mm, degrees about its y, x, z): what a camera physically moved after it was placed looks like. Implies
	 * live camera poses. Negative time for none.
	 */
	double move_at_s{-1.0};
	int move_mosaic{0};
	int move_camera{0};
	double move_mm[3]{0.0, 0.0, 0.0};
	double move_deg[3]{0.0, 0.0, 0.0};
	//! `track` mode: every sensor measured again from this many seconds on, as the Rift tab's button asks. Implies
	//! live camera poses. Negative for none.
	double recalibrate_at_s{-1.0};
	/*!
	 * `track` mode: the whole rig starts turned about up and moved by this (degrees, mm), the same room in another
	 * tracking space, as a measurement whose solve picked another origin left it. Implies live camera poses.
	 */
	double rig_offset[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	//! `frames` mode: a directory of recorded camera frames and IMU samples (frames.zst, frames.csv, imu.csv).
	std::string frames_dir;
	//! `frames` mode: the blobwatch's thresholds, the Rift driver's by default.
	int pixel_threshold{RIFT_BLOBWATCH_PIXEL_THRESHOLD_CV1};
	int blob_threshold{RIFT_BLOBWATCH_BLOB_REQUIRED_THRESHOLD_CV1};
	//! `track` mode: how long after its last solve a Touch's fusion pose is used (RIFT_TOUCH_POSITION_HOLD_NS).
	double touch_fresh_ms{150.0};
	//! `frames` mode: take the Touch gyros' bias out whenever they lie still, as the Rift driver does.
	bool rest_bias{true};
	//! `frames` mode: blobs added to every frame of their camera (camera,dx,dy,ux,uy,bbx,bby,bbw,bbh,sx,sy), e.g.
	//! a live session's daylight spots on top of another session's frames.
	std::string inject_blobs;
	//! The headset's adjustable part (rear headband LEDs, as the Rift driver reports them), see load_loose_part.
	std::string loose_part;
	//! `frames` mode: cameras whose frames are left out, as if not connected (camera indices of mosaic 0).
	std::vector<int> skip_cameras;
	//! `frames` mode: the blobwatch's largest blob, wider or taller is dropped (injected blobs too).
	int max_blob_width{RIFT_BLOBWATCH_DEFAULT_MAX_BLOB_WIDTH};
	//! `frames` mode: hand frames over at the pace they were captured, as live, not as fast as they go.
	bool realtime{false};
	//! `frames` mode: run the tracker's camera threads as live does, fast and slow searches racing the frames.
	bool threaded{false};
	//! `frames` mode: place the cameras where the template's last samples have them (a run that placed them
	//! again, when the sensors stood elsewhere than in the template's own session), not its first.
	bool placement_last{false};
	//! `frames` mode: every so many seconds the controllers are forgotten (prior, last pose, labels), so they have
	//! to be found again from scratch: how a lost controller comes back, measured. 0 never.
	double forget_every_s{0.0};

	//! Overrides every LED's visibility angle when positive, in degrees.
	double led_visibility_deg{-1.0};
};

void
usage()
{
	fprintf(
	    stderr,
	    "Usage: monado-constellation-replay acquire|track <in.ctdata> <out.ctdata> [options]\n"
	    "\n"
	    "  acquire   Run the slow search on every camera sample from scratch (no labels, prior,\n"
	    "            camera pose or rig calibration) and record the solves, plus the input's IMU\n"
	    "            samples, to <out.ctdata>.\n"
	    "  track     Run the tracker as live: recorded camera placements, IMU in time order, fast\n"
	    "            paths with a prior, slow search when they fail. Records solves, IMU, and with\n"
	    "            --prior fusion the fusion's output poses.\n"
	    "\n"
	    "Options:\n"
	    "  --from-s S     Skip samples earlier than S seconds after the first one.\n"
	    "  --to-s S       Stop at S seconds after the first sample.\n"
	    "  --stride N     Replay every Nth sample only.\n"
	    "  --camera M/C   Replay only camera C of mosaic M.\n"
	    "  --live-camera-poses 0|1  track: samples carry the tracker's own camera poses, as live (0: recorded).\n"
	    "  --move-camera M/C,dx,dy,dz,yaw,pitch,roll@T  track: from T s on the tracker takes camera C of mosaic M\n"
	    "                 to be moved by dx,dy,dz mm and yaw,pitch,roll degrees in its own frame, as if it had\n"
	    "                 been moved after it was placed. Implies --live-camera-poses 1.\n"
	    "  --recalibrate-at T     track: every sensor measured again from T s on, as the Rift tab's button asks;\n"
	    "                         where the cameras end up shows whether the room was kept. Implies\n"
	    "                         --live-camera-poses 1.\n"
	    "  --rig-offset yaw,dx,dy,dz[,pitch,roll]  track: the whole rig starts turned yaw degrees about up (and\n"
	    "                         tilted pitch about x, roll about z: a room placed tilted) and moved by\n"
	    "                         dx,dy,dz mm: the same room in another tracking space. Implies\n"
	    "                         --live-camera-poses 1.\n"
	    "  --calibration-from F   Use the camera calibrations recorded in F instead (same cameras, in the same\n"
	    "                         order), re-undistorting every blob from its distorted center.\n"
	    "  --min-leds-no-prior N  LEDs needed for a solve without a prior (default 7, the CV1 HMD's).\n"
	    "  --min-leds-prior N     LEDs needed for a solve that agrees with a prior (default 5).\n"
	    "  --led-visibility-deg A Treat LEDs as visible up to A degrees off their normal (default: as "
	    "recorded).\n"
	    "  --prior raw|fusion     track: the prior is the last solves (default) or the sensor fusion.\n"
	    "  --imu-lead-ms T        track: IMU samples up to T ms after a camera sample are in when it is\n"
	    "                         processed (default 10).\n"
	    "  --touch-fresh-ms T     track: a Touch's fusion pose is used up to T ms after its last solve\n"
	    "                         (default 150, the Rift driver's).\n"
	    "  --frames-dir D         frames: run the Rift blobwatch and the tracker on raw sensor frames and IMU\n"
	    "                         samples from D (frames.zst, frames.csv, imu.csv), <in.ctdata> giving the cameras,\n"
	    "                         their placement and the devices. Implies --prior fusion.\n"
	    "  --placement-last 0|1  frames: the cameras placed as in <in.ctdata>'s last samples (a run that\n"
	    "                         placed them again), not its first (0).\n"
	    "  --skip-camera C        frames: leave camera C's frames out, as if it were not connected (repeatable).\n"
	    "  --loose-part FILE      frames, track: the headset's adjustable part (the CV1's rear headband LEDs)\n"
	    "                         from FILE, JSON {\"serial\": S, \"leds\": [{\"id\", \"position\": [x, y, z],\n"
	    "                         \"normal\": [x, y, z]}]} in the Rift driver's device axes; the tracker measures\n"
	    "                         where it sits and then tracks it (CONSTELLATION_TRACKER_LOOSE_PART).\n"
	    "  --pixel-threshold N, --blob-threshold N   frames: the blobwatch's thresholds (0x24, 0x40).\n"
	    "  --rest-bias 0|1   frames: take the Touch gyros' bias out whenever they lie still (1).\n"
	    "  --inject-blobs FILE   frames: add these blobs to every frame of their camera.\n"
	    "  --max-blob-width N    frames: the blobwatch's largest blob (50).\n"
	    "  --realtime 0|1        frames: hand frames over at the pace they were captured (0).\n"
	    "  --threaded 0|1        frames: run the tracker's camera threads as live (0, one thread).\n"
	    "  --forget-every S      frames: forget the controllers every S seconds, to measure finding them again.\n"
	    "  --app-hz F             track: query every device's output pose F times a second of the\n"
	    "                         recording, as an app would, recorded as output poses (default never).\n"
	    "                         The fusion prior's queries are recorded too, at camera sample times.\n"
	    "  --app-after-each-ms T  track: query every device's output pose T ms after each camera sample,\n"
	    "                         once the fusion has it, as an app between two cameras' samples would.\n"
	    "\n"
	    "The tracker's environment options apply, CONSTELLATION_TRACKER_GRAVITY_TOLERANCE_DEG among them.\n");
}

bool
parse_options(int argc, char **argv, Options &o)
{
	if (argc < 4) {
		return false;
	}
	o.mode = argv[1];
	o.in_file = argv[2];
	o.out_file = argv[3];

	for (int i = 4; i < argc; i++) {
		std::string arg = argv[i];
		if (i + 1 >= argc) {
			return false;
		}
		const char *value = argv[++i];
		if (arg == "--from-s") {
			o.from_s = atof(value);
		} else if (arg == "--to-s") {
			o.to_s = atof(value);
		} else if (arg == "--stride") {
			o.stride = static_cast<uint32_t>(std::max(1, atoi(value)));
		} else if (arg == "--min-leds-no-prior") {
			o.min_leds_without_prior = static_cast<uint32_t>(atoi(value));
			o.match_parameters_given = true;
		} else if (arg == "--led-visibility-deg") {
			o.led_visibility_deg = atof(value);
		} else if (arg == "--min-leds-prior") {
			o.min_leds_with_prior = static_cast<uint32_t>(atoi(value));
			o.match_parameters_given = true;
		} else if (arg == "--prior") {
			if (strcmp(value, "fusion") == 0) {
				o.fusion_prior = true;
			} else if (strcmp(value, "raw") != 0) {
				return false;
			}
		} else if (arg == "--inject-blobs") {
			o.inject_blobs = value;
		} else if (arg == "--max-blob-width") {
			o.max_blob_width = atoi(value);
		} else if (arg == "--realtime") {
			o.realtime = atoi(value) != 0;
		} else if (arg == "--threaded") {
			o.threaded = atoi(value) != 0;
		} else if (arg == "--skip-camera") {
			o.skip_cameras.push_back(atoi(value));
		} else if (arg == "--loose-part") {
			o.loose_part = value;
		} else if (arg == "--placement-last") {
			o.placement_last = atoi(value) != 0;
		} else if (arg == "--forget-every") {
			o.forget_every_s = atof(value);
		} else if (arg == "--rest-bias") {
			o.rest_bias = atoi(value) != 0;
		} else if (arg == "--imu-lead-ms") {
			o.imu_lead_ms = atof(value);
		} else if (arg == "--frames-dir") {
			o.frames_dir = value;
		} else if (arg == "--pixel-threshold") {
			o.pixel_threshold = atoi(value);
		} else if (arg == "--blob-threshold") {
			o.blob_threshold = atoi(value);
		} else if (arg == "--app-hz") {
			o.app_hz = atof(value);
		} else if (arg == "--app-after-each-ms") {
			o.app_after_each_ms = atof(value);
		} else if (arg == "--touch-fresh-ms") {
			o.touch_fresh_ms = atof(value);
		} else if (arg == "--calibration-from") {
			o.calibration_from = value;
		} else if (arg == "--camera") {
			if (sscanf(value, "%d/%d", &o.mosaic, &o.camera) != 2) {
				return false;
			}
		} else if (arg == "--live-camera-poses") {
			o.live_camera_poses = atoi(value) != 0;
		} else if (arg == "--move-camera") {
			if (sscanf(value, "%d/%d,%lf,%lf,%lf,%lf,%lf,%lf@%lf", &o.move_mosaic, &o.move_camera, &o.move_mm[0],
			           &o.move_mm[1], &o.move_mm[2], &o.move_deg[0], &o.move_deg[1], &o.move_deg[2],
			           &o.move_at_s) != 9) {
				return false;
			}
			o.live_camera_poses = true;
		} else if (arg == "--recalibrate-at") {
			o.recalibrate_at_s = atof(value);
			o.live_camera_poses = true;
		} else if (arg == "--rig-offset") {
			const int n = sscanf(value, "%lf,%lf,%lf,%lf,%lf,%lf", &o.rig_offset[0], &o.rig_offset[1],
			                     &o.rig_offset[2], &o.rig_offset[3], &o.rig_offset[4], &o.rig_offset[5]);
			if (n != 4 && n != 6) {
				return false;
			}
			o.live_camera_poses = true;
		} else {
			return false;
		}
	}

	return o.mode == "acquire" || o.mode == "track" || (o.mode == "frames" && !o.frames_dir.empty());
}

/*!
 * The headset's adjustable part from --loose-part: LEDs in the Rift driver's device axes, given the same radius and
 * visibility as the headset's own (@p like). Empty when not asked for or unreadable.
 */
struct LoosePart
{
	std::vector<t_constellation_tracker_led> leds;
	std::string serial;
};

static LoosePart
load_loose_part(const Options &o, const t_constellation_tracker_led &like)
{
	LoosePart out;
	if (o.loose_part.empty()) {
		return out;
	}
	FILE *f = fopen(o.loose_part.c_str(), "r");
	if (f == nullptr) {
		fprintf(stderr, "Can't read %s\n", o.loose_part.c_str());
		return out;
	}
	std::string text;
	char buf[4096];
	size_t n;
	while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
		text.append(buf, n);
	}
	fclose(f);
	cJSON *root = cJSON_Parse(text.c_str());
	const cJSON *serial = u_json_get(root, "serial");
	if (cJSON_IsString(serial)) {
		out.serial = serial->valuestring;
	}
	const cJSON *led = nullptr;
	cJSON_ArrayForEach(led, u_json_get(root, "leds"))
	{
		int id = -1;
		u_json_get_int(u_json_get(led, "id"), &id);
		const cJSON *p = u_json_get(led, "position");
		const cJSON *nrm = u_json_get(led, "normal");
		if (id < 0 || cJSON_GetArraySize(p) != 3 || cJSON_GetArraySize(nrm) != 3) {
			continue;
		}
		t_constellation_tracker_led l = like;
		l.id = static_cast<t_constellation_led_id_it>(id);
		l.position = {(float)cJSON_GetArrayItem(p, 0)->valuedouble, (float)cJSON_GetArrayItem(p, 1)->valuedouble,
		              (float)cJSON_GetArrayItem(p, 2)->valuedouble};
		l.normal = {(float)cJSON_GetArrayItem(nrm, 0)->valuedouble, (float)cJSON_GetArrayItem(nrm, 1)->valuedouble,
		            (float)cJSON_GetArrayItem(nrm, 2)->valuedouble};
		out.leds.push_back(l);
	}
	cJSON_Delete(root);
	fprintf(stderr, "Adjustable part: %zu LEDs from %s\n", out.leds.size(), o.loose_part.c_str());
	return out;
}

//! Hands the part to the headset (the device with the most LEDs).
static void
give_loose_part(t_constellation_tracker_device_params &params, LoosePart &part, bool is_headset)
{
	if (!is_headset || part.leds.empty()) {
		return;
	}
	params.loose_part.leds = part.leds.data();
	params.loose_part.led_count = part.leds.size();
	snprintf(params.serial, sizeof(params.serial), "%s", part.serial.c_str());
}

int
do_replay(const Options &o)
{
	const bool track = o.mode == "track";

	ct::DatasetReader dataset(o.in_file);

	fprintf(stderr, "Read %s: %zu mosaics, %zu devices, %zu camera samples, %zu IMU samples, %zu solves\n",
	        o.in_file.c_str(), dataset.mosaics.size(), dataset.devices.size(), dataset.samples.size(),
	        dataset.imu_samples.size(), dataset.solves.size());
	if (o.match_parameters_given) {
		fprintf(stderr, "Match parameters: %u LEDs without a prior, %u with one\n", o.min_leds_without_prior,
		        o.min_leds_with_prior);
	}

	if (dataset.samples.empty() || dataset.devices.empty()) {
		fprintf(stderr, "Nothing to replay.\n");
		return 1;
	}

	// Camera models the blobs get re-undistorted with, empty when the recorded ones are used.
	std::vector<std::vector<t_camera_model_params>> undistort_models;
	if (!o.calibration_from.empty()) {
		ct::DatasetReader other(o.calibration_from);
		if (other.mosaics.size() != dataset.mosaics.size()) {
			fprintf(stderr, "%s has %zu mosaics, the recording %zu.\n", o.calibration_from.c_str(),
			        other.mosaics.size(), dataset.mosaics.size());
			return 1;
		}

		undistort_models.resize(other.mosaics.size());
		for (size_t i = 0; i < other.mosaics.size(); i++) {
			auto &calibrations = other.mosaics[i].camera_calibrations;
			if (calibrations.size() != dataset.mosaics[i].camera_calibrations.size()) {
				fprintf(stderr, "Mosaic %zu has a different camera count in %s.\n", i,
				        o.calibration_from.c_str());
				return 1;
			}

			for (auto &calibration : calibrations) {
				t_camera_model_params model_params;
				t_camera_model_params_from_t_camera_calibration(&calibration, &model_params);
				undistort_models[i].push_back(model_params);
			}
			dataset.mosaics[i].camera_calibrations = calibrations;
		}

		fprintf(stderr, "Camera calibrations from %s, distortion model %d\n", o.calibration_from.c_str(),
		        static_cast<int>(dataset.mosaics[0].camera_calibrations[0].distortion_model));
	}

	t_constellation_tracker_params params = {
	    .flags = T_CONSTELLATION_TRACKER_FLAGS_DETERMINISTIC,
	    .mosaics = {},
	    .num_mosaics = dataset.mosaics.size(),
	};

	for (size_t i = 0; i < dataset.mosaics.size(); i++) {
		params.mosaics[i] = {
		    .tracking_origin = nullptr,
		    .cameras = {},
		    .num_cameras = dataset.mosaics[i].camera_calibrations.size(),
		};

		for (size_t j = 0; j < dataset.mosaics[i].camera_calibrations.size(); j++) {
			params.mosaics[i].cameras[j] = {
			    .calibration = dataset.mosaics[i].camera_calibrations[j],
			    .pose_in_origin = XRT_POSE_IDENTITY,
			    .has_concrete_pose = false,
			    // No serial, so a stored rig is never restored into a replay.
			    .serial = {},
			    .blob_sink = nullptr,
			};
			// Measuring again keeps each camera's placement by serial: made-up ones, matching no stored rig.
			if (o.recalibrate_at_s >= 0.0) {
				snprintf(params.mosaics[i].cameras[j].serial, sizeof(params.mosaics[i].cameras[j].serial),
				         "replay-%zu-%zu", i, j);
			}
		}
	}

	auto tracker = std::make_unique<ct::ConstellationTracker>(&params);

	// Acquisition only: keep the rig calibration from ever starting, it would otherwise begin collecting and
	// disable every device but the first.
	tracker->calibration_held = true;

	// Replace any recorder the environment asked for, before the devices are added so their info is recorded.
	tracker->data_recorder = std::make_unique<ct::DataRecorder>(tracker.get(), o.out_file);

	std::map<t_constellation_device_id_t, t_constellation_device_id_t> device_ids;
	std::vector<std::unique_ptr<ReplayDevice>> replay_devices;
	for (auto &device : dataset.devices) {
		if (o.led_visibility_deg > 0.0) {
			for (auto &led : device.leds) {
				led.visibility_angle = static_cast<float>(DEG_TO_RAD(o.led_visibility_deg));
			}
		}

		auto rd = std::make_unique<ReplayDevice>();
		rd->device.push_constellation_tracker_sample = replay_push_sample;
		rd->source.get_tracked_pose = replay_get_tracked_pose;
		rd->tracker = tracker.get();
		rd->fusion_prior = o.fusion_prior;
		m_relation_history_create(&rd->history);

		t_constellation_tracker_device_params device_params = {
		    .led_model = device.led_model,
		    .tracking_source = track ? &rd->source : nullptr,
		    .imu_sink = nullptr,
		    .loose_part = {},
		    .serial = {},
		};
		if (device.led_model.led_count > 0) {
			static LoosePart part = load_loose_part(o, device.led_model.leds[0]);
			give_loose_part(device_params, part, device.led_model.led_count > 24);
		}

		const bool touch = device.led_model.led_count <= 24;
		if (touch) {
			// RIFT_TOUCH_POSITION_HOLD_NS
			rd->fusion_fresh_ns = static_cast<timepoint_ns>(o.touch_fresh_ms * U_TIME_1MS_IN_NS);
		}
		device_params.led_model.match_parameters = {
		    .min_leds_for_correspondence_search_without_prior =
		        o.match_parameters_given ? o.min_leds_without_prior : 7u,
		    .min_leds_for_correspondence_search_with_prior =
		        o.match_parameters_given ? o.min_leds_with_prior : (touch ? 4u : 5u),
		};
		fprintf(stderr, "Device %d: %zu LEDs, %u LEDs without a prior, %u with one\n", device.id,
		        device.led_model.led_count,
		        device_params.led_model.match_parameters.min_leds_for_correspondence_search_without_prior,
		        device_params.led_model.match_parameters.min_leds_for_correspondence_search_with_prior);

		rd->id = tracker->addDevice(&device_params, track ? &rd->device : &stub_device);
		device_ids[device.id] = rd->id;
		replay_devices.push_back(std::move(rd));
	}

	std::map<t_constellation_device_id_t, ct::Device *> devices_by_id;
	for (auto &device : tracker->devices) {
		device->enabled.store(true);
		devices_by_id[device->id] = device.get();
	}

	/*
	 * Live camera poses: each camera starts where the recording had it, and the samples get its pose from the
	 * tracker as live, so a camera the tracker places again takes effect. The recorded poses stay the truth a
	 * simulated move is measured against.
	 */
	std::map<std::pair<size_t, size_t>, xrt_pose> recorded_cam_pose;
	for (const auto &recorded : dataset.samples) {
		if (recorded.Txr_world_cam.has_value()) {
			recorded_cam_pose.emplace(std::make_pair(recorded.mosaic_index, recorded.camera_index),
			                          recorded.Txr_world_cam.value());
		}
	}
	// Where each camera starts: as recorded, or the whole rig in another tracking space (--rig-offset).
	std::map<std::pair<size_t, size_t>, xrt_pose> start_cam_pose;
	if (track && o.live_camera_poses) {
		xrt_quat turn;
		const xrt_vec3 up = {0, 1, 0};
		math_quat_from_angle_vector(static_cast<float>(DEG_TO_RAD(o.rig_offset[0])), &up, &turn);
		const xrt_vec3 x_axis = {1, 0, 0};
		const xrt_vec3 z_axis = {0, 0, 1};
		xrt_quat pitch, roll;
		math_quat_from_angle_vector(static_cast<float>(DEG_TO_RAD(o.rig_offset[4])), &x_axis, &pitch);
		math_quat_from_angle_vector(static_cast<float>(DEG_TO_RAD(o.rig_offset[5])), &z_axis, &roll);
		math_quat_rotate(&pitch, &turn, &turn);
		math_quat_rotate(&roll, &turn, &turn);
		const xrt_pose offset = {turn, {static_cast<float>(o.rig_offset[1] / 1000.0),
		                                static_cast<float>(o.rig_offset[2] / 1000.0),
		                                static_cast<float>(o.rig_offset[3] / 1000.0)}};
		for (const auto &[key, pose] : recorded_cam_pose) {
			if (key.first >= tracker->mosaics.size() || key.second >= tracker->mosaics[key.first]->cameras.size()) {
				continue;
			}
			ct::Camera *camera = tracker->mosaics[key.first]->cameras[key.second].get();
			xrt_pose start;
			math_pose_transform(&offset, &pose, &start);
			start_cam_pose[key] = start;
			std::unique_lock lock(camera->processing_lock);
			camera->locked_data.Txr_origin_cam = start;
			camera->locked_data.has_concrete_pose = true;
		}
		fprintf(stderr, "Live camera poses: %zu camera(s) placed where the recording had them\n",
		        recorded_cam_pose.size());
	}
	bool moved = false;
	bool recalibrated = false;
	bool recalibrate_refused_logged = false;

	// In time order, as the tracker would see them. Pushing them records them too, in `track` mode.
	std::vector<const ct::DatasetImuSample *> imu_in_order;
	for (const auto &imu : dataset.imu_samples) {
		auto it = device_ids.find(imu.device_id);
		if (it == device_ids.end()) {
			continue;
		}
		if (track) {
			// Recordings from before the Rift driver left out idle Touch reports have them as near zero.
			const xrt_vec3_f64 &a = imu.sample.accel_m_s2;
			if (a.x * a.x + a.y * a.y + a.z * a.z < 1.0) {
				continue;
			}
			imu_in_order.push_back(&imu);
		} else {
			// Carry the IMU over so the output stands on its own for the analysis scripts.
			tracker->data_recorder->recordImuSample(it->second, imu.sample);
		}
	}
	std::stable_sort(imu_in_order.begin(), imu_in_order.end(),
	                 [](const ct::DatasetImuSample *a, const ct::DatasetImuSample *b) {
		                 return a->sample.timestamp_ns < b->sample.timestamp_ns;
	                 });
	size_t imu_next = 0;
	const timepoint_ns imu_lead_ns = static_cast<timepoint_ns>(o.imu_lead_ms * U_TIME_1MS_IN_NS);

	const timepoint_ns app_period_ns = o.app_hz > 0.0 ? static_cast<timepoint_ns>(U_TIME_1S_IN_NS / o.app_hz) : 0;
	timepoint_ns app_next_ns = 0;

	const timepoint_ns t0 = dataset.samples.front().timestamp_ns;
	const timepoint_ns from_ns = t0 + static_cast<timepoint_ns>(o.from_s * U_TIME_1S_IN_NS);
	const timepoint_ns to_ns = o.to_s < 0.0 ? INT64_MAX : t0 + static_cast<timepoint_ns>(o.to_s * U_TIME_1S_IN_NS);

	auto start = std::chrono::steady_clock::now();
	std::vector<double> fast_ms;
	std::vector<double> slow_ms;
	uint64_t considered = 0;
	uint64_t replayed = 0;

	for (const auto &recorded : dataset.samples) {
		if (recorded.timestamp_ns < from_ns || recorded.timestamp_ns > to_ns) {
			continue;
		}
		if (o.mosaic >= 0 && (static_cast<int>(recorded.mosaic_index) != o.mosaic ||
		                      static_cast<int>(recorded.camera_index) != o.camera)) {
			continue;
		}
		if (recorded.mosaic_index >= tracker->mosaics.size() ||
		    recorded.camera_index >= tracker->mosaics[recorded.mosaic_index]->cameras.size()) {
			continue;
		}
		if (considered++ % o.stride != 0) {
			continue;
		}

		while (imu_next < imu_in_order.size() &&
		       imu_in_order[imu_next]->sample.timestamp_ns <= recorded.timestamp_ns + imu_lead_ns) {
			const ct::DatasetImuSample *imu = imu_in_order[imu_next++];
			devices_by_id[device_ids[imu->device_id]]->pushImuSample(imu->sample);
			for (auto &rd : replay_devices) {
				if (rd->id == device_ids[imu->device_id]) {
					rd->pushImu(imu->sample);
				}
			}
		}

		// The app's queries up to this sample, with what the tracker had by then.
		if (track && app_period_ns > 0) {
			if (app_next_ns == 0) {
				app_next_ns = recorded.timestamp_ns;
			}
			for (; app_next_ns <= recorded.timestamp_ns; app_next_ns += app_period_ns) {
				for (auto &rd : replay_devices) {
					xrt_space_relation relation;
					if (rd->fusion_fresh_ns > 0 &&
					    (rd->last_solve_ns == 0 ||
					     app_next_ns - rd->last_solve_ns > rd->fusion_fresh_ns)) {
						continue;
					}
					tracker->getTrackedPose(rd->id, app_next_ns, relation);
				}
			}
		}

		ct::CameraSample sample = recorded;
		sample.source = &stub_blobwatch;
		if (!track) {
			sample.Txr_world_cam = std::nullopt;
		}
		ct::Camera *camera = tracker->mosaics[sample.mosaic_index]->cameras[sample.camera_index].get();
		if (track && o.live_camera_poses) {
			if (!moved && o.move_at_s >= 0.0 && sample.timestamp_ns >= t0 + static_cast<timepoint_ns>(o.move_at_s * U_TIME_1S_IN_NS) &&
			    static_cast<size_t>(o.move_mosaic) < tracker->mosaics.size() &&
			    static_cast<size_t>(o.move_camera) < tracker->mosaics[o.move_mosaic]->cameras.size()) {
				moved = true;
				ct::Camera *mc = tracker->mosaics[o.move_mosaic]->cameras[o.move_camera].get();
				xrt_quat yaw, pitch, roll, yaw_pitch, q;
				const xrt_vec3 y_axis = {0, 1, 0}, x_axis = {1, 0, 0}, z_axis = {0, 0, 1};
				math_quat_from_angle_vector(static_cast<float>(DEG_TO_RAD(o.move_deg[0])), &y_axis, &yaw);
				math_quat_from_angle_vector(static_cast<float>(DEG_TO_RAD(o.move_deg[1])), &x_axis, &pitch);
				math_quat_from_angle_vector(static_cast<float>(DEG_TO_RAD(o.move_deg[2])), &z_axis, &roll);
				math_quat_rotate(&yaw, &pitch, &yaw_pitch);
				math_quat_rotate(&yaw_pitch, &roll, &q);
				const xrt_pose delta = {q, {static_cast<float>(o.move_mm[0] / 1000.0),
				                            static_cast<float>(o.move_mm[1] / 1000.0),
				                            static_cast<float>(o.move_mm[2] / 1000.0)}};
				std::unique_lock lock(mc->processing_lock);
				xrt_pose before = mc->locked_data.Txr_origin_cam;
				math_pose_transform(&before, &delta, &mc->locked_data.Txr_origin_cam);
				fprintf(stderr, "Camera %d/%d taken as moved at %.1f s by (%.0f, %.0f, %.0f) mm, %.1f/%.1f/%.1f degrees\n",
				        o.move_mosaic, o.move_camera, (sample.timestamp_ns - t0) / 1e9, o.move_mm[0], o.move_mm[1],
				        o.move_mm[2], o.move_deg[0], o.move_deg[1], o.move_deg[2]);
			}
			// Asked again each sample while the tracker is busy, as a user would press again later.
			if (!recalibrated && o.recalibrate_at_s >= 0.0 &&
			    sample.timestamp_ns >= t0 + static_cast<timepoint_ns>(o.recalibrate_at_s * U_TIME_1S_IN_NS)) {
				bool quarantined = false;
				{
					std::unique_lock watch_lock(tracker->rig_watch.lock);
					quarantined = tracker->rig_watch.quarantined.has_value();
				}
				// The replay keeps the room's measuring from starting (above); asked for, it may now.
				if (!recalibrate_refused_logged) {
					tracker->calibration_held = false;
				}
				const bool solving = tracker->running_calibration.load();
				const bool unplaced = tracker->needSensorCalibration();
				recalibrated = tracker->recalibrate();
				if (recalibrated || !recalibrate_refused_logged) {
					recalibrate_refused_logged = true;
					fprintf(stderr, "Measuring every sensor again at %.1f s: %s (solve running %d, a camera "
					        "unplaced %d, one being placed again %d)\n", (sample.timestamp_ns - t0) / 1e9,
					        recalibrated ? "started" : "refused, asked again", solving, unplaced, quarantined);
				}
			}
			sample.Txr_world_cam = camera->getWorldPose(sample.timestamp_ns);
		}
		// Before the labels go: live, the blobwatch carries a tracked device's labels into the next frame.
		camera->classifyStatic(sample);
		// No device state means every enabled device gets a fresh slow search.
		sample.device_count = 0;
		for (uint32_t i = 0; i < sample.blob_count; i++) {
			t_blob &b = sample.blobs[i];
			b.matched_device_id = XRT_CONSTELLATION_INVALID_DEVICE_ID;
			b.matched_device_led_id = XRT_CONSTELLATION_INVALID_LED_ID;

			if (!undistort_models.empty()) {
				// The same steps as the blobwatch, with the replacement model.
				const t_camera_model_params &m =
				    undistort_models[sample.mosaic_index][sample.camera_index];
				float nx, ny;
				t_camera_models_undistort(&m, b.center_distorted.x, b.center_distorted.y, &nx, &ny);
				b.center_undistorted.x = nx * m.fx + m.cx;
				b.center_undistorted.y = ny * m.fy + m.cy;
				b.center_homogenized = {nx, ny, 1.0f};
			}
		}

		if (!track) {
			camera->processSampleSlow(sample);
		} else {
			const auto fast_start = std::chrono::steady_clock::now();
			const bool need_slow = camera->processSampleFast(sample);
			const auto slow_start = std::chrono::steady_clock::now();
			fast_ms.push_back(std::chrono::duration<double, std::milli>(slow_start - fast_start).count());
			if (need_slow) {
				camera->processSampleSlow(sample);
				slow_ms.push_back(std::chrono::duration<double, std::milli>(
				                      std::chrono::steady_clock::now() - slow_start)
				                      .count());
			}

			// The fusion runs on its own thread, let it catch up as it would in real time.
			if (o.fusion_prior || o.app_hz > 0.0 || o.app_after_each_ms >= 0.0) {
				for (auto &device : tracker->devices) {
					device->sensor_fusion->waitIdle();
				}
			}
			if (o.app_after_each_ms >= 0.0) {
				const timepoint_ns when_ns =
				    sample.timestamp_ns +
				    static_cast<timepoint_ns>(o.app_after_each_ms * U_TIME_1MS_IN_NS);
				for (auto &rd : replay_devices) {
					xrt_space_relation relation;
					tracker->getTrackedPose(rd->id, when_ns, relation);
				}
			}
		}
		replayed++;

		if (replayed % 2000 == 0) {
			double elapsed =
			    std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
			fprintf(stderr, "  %" PRIu64 " samples replayed, %.1f s, %.1f samples/s\n", replayed, elapsed,
			        replayed / elapsed);
		}
	}

	double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
	fprintf(stderr, "Replayed %" PRIu64 " of %" PRIu64 " samples in %.1f s, output %s\n", replayed, considered,
	        elapsed, o.out_file.c_str());

	for (auto *v : {&fast_ms, &slow_ms}) {
		if (v->empty()) {
			continue;
		}
		std::sort(v->begin(), v->end());
		auto p = [&](double q) { return (*v)[std::min(v->size() - 1, static_cast<size_t>(q * v->size()))]; };
		fprintf(stderr,
		        "%s processing: %zu frames, ms p50 %.2f p90 %.2f p99 %.2f max %.2f, over 19.2 ms: %zu\n",
		        v == &fast_ms ? "Fast" : "Slow", v->size(), p(0.5), p(0.9), p(0.99), v->back(),
		        static_cast<size_t>(std::count_if(v->begin(), v->end(), [](double x) { return x > 19.2; })));
	}

	for (auto &device : tracker->devices) {
		fprintf(stderr,
		        "Device %d: %" PRIu64 " pose(s) rejected for disagreeing with the IMU on down, %" PRIu64
		        " for leaving a fresh prior\n",
		        device->id, device->gravity_rejections_total.load(), device->prior_rejections_total.load());
	}

	if (recalibrated) {
		struct u_notice_recalibration change = {};
		if (u_notice_get_recalibration(&change)) {
			fprintf(stderr, "Measured again: room %s (%d cameras kept their place)\n",
			        change.kept ? "kept" : "NOT kept", change.anchors);
		} else {
			fprintf(stderr, "Measured again: NOT finished by the end of the replay\n");
		}
	}

	// Where the cameras ended up against where the recording had them.
	if (track && o.live_camera_poses) {
		for (const auto &[key, truth] : recorded_cam_pose) {
			if (key.first >= tracker->mosaics.size() || key.second >= tracker->mosaics[key.first]->cameras.size()) {
				continue;
			}
			ct::Camera *camera = tracker->mosaics[key.first]->cameras[key.second].get();
			xrt_pose now;
			{
				std::unique_lock lock(camera->processing_lock);
				now = camera->locked_data.Txr_origin_cam;
			}
			xrt_quat d;
			math_quat_unrotate(&truth.orientation, &now.orientation, &d);
			fprintf(stderr, "Camera %zu/%zu at the end: %.1f mm and %.2f degrees from where the recording had it\n",
			        key.first, key.second, m_vec3_len(now.position - truth.position) * 1000.0f,
			        RAD_TO_DEG(2.0f * acosf(CLAMP(fabsf(d.w), 0.0f, 1.0f))));
			if (o.recalibrate_at_s >= 0.0 && start_cam_pose.count(key) != 0) {
				const xrt_pose &start = start_cam_pose[key];
				math_quat_unrotate(&start.orientation, &now.orientation, &d);
				fprintf(stderr,
				        "Camera %zu/%zu at the end: %.1f mm and %.2f degrees from where the replay started it "
				        "(the room as it was)\n",
				        key.first, key.second, m_vec3_len(now.position - start.position) * 1000.0f,
				        RAD_TO_DEG(2.0f * acosf(CLAMP(fabsf(d.w), 0.0f, 1.0f))));
			}
		}
	}

	return 0;
}

} // namespace

/*!
 * `frames` mode: raw sensor frames, recorded elsewhere (the Oculus runtime's, from a USB capture), through the Rift
 * blobwatch into the tracker, with the IMU samples in time order. The cameras, their placement and the devices come
 * from a recording of the same setup.
 */
//! Adds fixed blobs to every observation of one camera on their way from the blobwatch to the tracker.
struct InjectSink
{
	t_blob_sink base{};
	t_blob_sink *next{nullptr};
	std::vector<t_blob> blobs;
	std::vector<t_blob> scratch;
	//! Until then the labels of devices other than the headset are taken off, see --forget-every.
	std::atomic<int64_t> strip_until_ns{0};
};

extern "C" void
inject_push_blobs(t_blob_sink *tbs, t_blob_observation *tbo)
{
	InjectSink *s = container_of(tbs, InjectSink, base);
	// Appended, so the blobwatch's own blobs keep their indices: it maps labels back by index.
	s->scratch.assign(tbo->blobs, tbo->blobs + tbo->num_blobs);
	if (tbo->timestamp_ns < s->strip_until_ns.load()) {
		for (t_blob &b : s->scratch) {
			if (b.matched_device_id != XRT_CONSTELLATION_INVALID_DEVICE_ID && b.matched_device_id != 0) {
				b.matched_device_id = XRT_CONSTELLATION_INVALID_DEVICE_ID;
				b.matched_device_led_id = XRT_CONSTELLATION_INVALID_LED_ID;
			}
		}
	}
	for (const t_blob &b : s->blobs) {
		if (s->scratch.size() >= XRT_CONSTELLATION_MAX_BLOBS_PER_FRAME) {
			break;
		}
		s->scratch.push_back(b);
	}
	t_blob_observation copy = *tbo;
	copy.blobs = s->scratch.data();
	copy.num_blobs = static_cast<uint32_t>(s->scratch.size());
	t_blob_sink_push_blobs(s->next, &copy);
}

extern "C" void
inject_destroy(t_blob_sink *tbs)
{
	(void)tbs;
}

int
do_frames(const Options &o)
{
	ct::DatasetReader dataset(o.in_file);
	std::map<std::pair<size_t, size_t>, xrt_pose> cam_pose;
	for (const auto &sample : dataset.samples) {
		if (sample.Txr_world_cam.has_value()) {
			const auto key = std::make_pair(sample.mosaic_index, sample.camera_index);
			if (o.placement_last) {
				cam_pose[key] = sample.Txr_world_cam.value();
			} else {
				cam_pose.emplace(key, sample.Txr_world_cam.value());
			}
		}
	}

	t_constellation_tracker_params params = {
	    .flags = o.threaded ? T_CONSTELLATION_TRACKER_FLAGS_NONE : T_CONSTELLATION_TRACKER_FLAGS_DETERMINISTIC,
	    .mosaics = {},
	    .num_mosaics = dataset.mosaics.size(),
	};
	for (size_t i = 0; i < dataset.mosaics.size(); i++) {
		params.mosaics[i] = {.tracking_origin = nullptr,
		                     .cameras = {},
		                     .num_cameras = dataset.mosaics[i].camera_calibrations.size()};
		for (size_t j = 0; j < params.mosaics[i].num_cameras; j++) {
			auto it = cam_pose.find({i, j});
			params.mosaics[i].cameras[j] = {
			    .calibration = dataset.mosaics[i].camera_calibrations[j],
			    .pose_in_origin = it != cam_pose.end() ? it->second : xrt_pose XRT_POSE_IDENTITY,
			    .has_concrete_pose = it != cam_pose.end(),
			    .serial = {},
			    .blob_sink = nullptr,
			};
			fprintf(stderr, "Camera %zu/%zu: %s\n", i, j,
			        it == cam_pose.end() ? "NOT placed"
			        : o.placement_last ? "placed as at the end of the template"
			                           : "placed from the template");
		}
	}

	auto tracker = std::make_unique<ct::ConstellationTracker>(&params);
	tracker->calibration_held = true;
	tracker->data_recorder = std::make_unique<ct::DataRecorder>(tracker.get(), o.out_file);

	std::map<t_constellation_device_id_t, t_constellation_device_id_t> device_ids;
	std::vector<std::unique_ptr<ReplayDevice>> replay_devices;
	for (auto &device : dataset.devices) {
		auto rd = std::make_unique<ReplayDevice>();
		rd->device.push_constellation_tracker_sample = replay_push_sample;
		rd->source.get_tracked_pose = replay_get_tracked_pose;
		rd->tracker = tracker.get();
		rd->fusion_prior = true;
		m_relation_history_create(&rd->history);
		const bool touch = device.led_model.led_count <= 24;
		if (touch) {
			rd->fusion_fresh_ns = static_cast<timepoint_ns>(o.touch_fresh_ms * U_TIME_1MS_IN_NS);
		}
		t_constellation_tracker_device_params device_params = {
		    .led_model = device.led_model,
		    .tracking_source = &rd->source,
		    .imu_sink = nullptr,
		    .loose_part = {},
		    .serial = {},
		};
		if (device.led_model.led_count > 0) {
			static LoosePart part = load_loose_part(o, device.led_model.leds[0]);
			give_loose_part(device_params, part, device.led_model.led_count > 24);
		}
		device_params.led_model.match_parameters = {
		    .min_leds_for_correspondence_search_without_prior = 7u,
		    .min_leds_for_correspondence_search_with_prior = touch ? 4u : 5u,
		};
		rd->id = tracker->addDevice(&device_params, &rd->device);
		device_ids[device.id] = rd->id;
		replay_devices.push_back(std::move(rd));
	}
	std::map<t_constellation_device_id_t, ct::Device *> devices_by_id;
	for (auto &device : tracker->devices) {
		device->enabled.store(true);
		devices_by_id[device->id] = device.get();
	}

	// Blobs to add on the way to the tracker, per camera, those larger than the blobwatch keeps left out.
	std::vector<InjectSink> inject(params.mosaics[0].num_cameras);
	if (!o.inject_blobs.empty()) {
		FILE *f = fopen(o.inject_blobs.c_str(), "r");
		if (f == nullptr) {
			fprintf(stderr, "Can't open %s\n", o.inject_blobs.c_str());
			return 1;
		}
		char line[512];
		if (fgets(line, sizeof(line), f) == nullptr) {
			line[0] = '\0';
		}
		int cam, bbx, bby, bbw, bbh;
		float dx, dy, ux, uy, sx, sy;
		size_t total = 0, kept = 0;
		while (fscanf(f, "%d,%f,%f,%f,%f,%d,%d,%d,%d,%f,%f", &cam, &dx, &dy, &ux, &uy, &bbx, &bby, &bbw, &bbh,
		              &sx, &sy) == 11) {
			total++;
			if (cam < 0 || static_cast<size_t>(cam) >= inject.size() || bbw > o.max_blob_width ||
			    bbh > o.max_blob_width) {
				continue;
			}
			const t_camera_calibration &calib = dataset.mosaics[0].camera_calibrations[cam];
			t_blob b = {};
			b.blob_id = 0x7f000000u + static_cast<uint32_t>(total);
			b.matched_device_id = XRT_CONSTELLATION_INVALID_DEVICE_ID;
			b.matched_device_led_id = XRT_CONSTELLATION_INVALID_LED_ID;
			b.center_distorted = {dx, dy};
			b.center_undistorted = {ux, uy};
			b.center_homogenized = {
			    static_cast<float>((ux - calib.intrinsics[0][2]) / calib.intrinsics[0][0]),
			    static_cast<float>((uy - calib.intrinsics[1][2]) / calib.intrinsics[1][1]), 1.0f};
			b.bounding_box = {{bbx, bby}, {bbw, bbh}};
			b.size = {sx, sy};
			b.brightness = 0.5f;
			inject[cam].blobs.push_back(b);
			kept++;
		}
		fclose(f);
		fprintf(stderr, "Injecting %zu of %zu blobs from %s (max blob width %d)\n", kept, total,
		        o.inject_blobs.c_str(), o.max_blob_width);
	}

	// One blobwatch per camera of the first mosaic, feeding the tracker's camera.
	struct xrt_frame_context xfctx = {};
	std::vector<struct xrt_frame_sink *> frame_sinks;
	for (size_t j = 0; j < params.mosaics[0].num_cameras; j++) {
		struct t_rift_blobwatch_params bp = {
		    .pixel_threshold = static_cast<uint8_t>(o.pixel_threshold),
		    .blob_required_threshold = static_cast<uint8_t>(o.blob_threshold),
		    .max_match_dist = RIFT_BLOBWATCH_DEFAULT_MAX_MATCH_DIST,
		    .max_blob_width = static_cast<uint16_t>(o.max_blob_width),
		    .camera_calibration = dataset.mosaics[0].camera_calibrations[j],
		};
		struct xrt_frame_sink *sink = nullptr;
		struct t_blobwatch *bw = nullptr;
		t_blob_sink *blob_sink = params.mosaics[0].cameras[j].blob_sink;
		if (!inject[j].blobs.empty() || o.forget_every_s > 0.0) {
			inject[j].base.push_blobs = inject_push_blobs;
			inject[j].base.destroy = inject_destroy;
			inject[j].next = blob_sink;
			blob_sink = &inject[j].base;
		}
		if (t_rift_blobwatch_create(&bp, &xfctx, blob_sink, &sink, &bw) != 0) {
			fprintf(stderr, "Failed to create the blobwatch for camera %zu\n", j);
			return 1;
		}
		frame_sinks.push_back(sink);
	}
	fprintf(stderr, "Blobwatch thresholds: pixel %d, blob %d\n", o.pixel_threshold, o.blob_threshold);

	// Frame index and IMU samples.
	std::vector<std::pair<int, timepoint_ns>> index;
	{
		FILE *f = fopen((o.frames_dir + "/frames.csv").c_str(), "r");
		char line[256];
		if (fgets(line, sizeof(line), f) == nullptr) {
			line[0] = '\0'; // Empty file, the loop below reads nothing.
		}
		long k, cam;
		long long exp_ns, first_ns;
		while (fscanf(f, "%ld,%ld,%lld,%lld", &k, &cam, &exp_ns, &first_ns) == 4) {
			index.emplace_back(static_cast<int>(cam), static_cast<timepoint_ns>(exp_ns));
		}
		fclose(f);
	}
	std::vector<std::pair<int, xrt_imu_sample>> imu;
	{
		FILE *f = fopen((o.frames_dir + "/imu.csv").c_str(), "r");
		char line[256];
		if (fgets(line, sizeof(line), f) == nullptr) {
			line[0] = '\0'; // Empty file, the loop below reads nothing.
		}
		int dev;
		long long ts;
		double v[6];
		while (fscanf(f, "%d,%lld,%lf,%lf,%lf,%lf,%lf,%lf", &dev, &ts, &v[0], &v[1], &v[2], &v[3], &v[4],
		              &v[5]) == 8) {
			xrt_imu_sample s = {};
			s.timestamp_ns = ts;
			s.accel_m_s2 = {v[0], v[1], v[2]};
			s.gyro_rad_secs = {v[3], v[4], v[5]};
			imu.emplace_back(dev, s);
		}
		fclose(f);
	}
	fprintf(stderr, "%zu frames, %zu IMU samples\n", index.size(), imu.size());

	std::map<t_constellation_device_id_t, m_gyro_rest_bias> rest_bias;
	if (o.rest_bias) {
		for (auto &rd : replay_devices) {
			if (rd->fusion_fresh_ns > 0) {
				m_gyro_rest_bias_init(&rest_bias[rd->id]);
			}
		}
	}

	FILE *zf = popen(("zstd -dc '" + o.frames_dir + "/frames.zst'").c_str(), "r");
	if (zf == nullptr) {
		fprintf(stderr, "Can't run zstd\n");
		return 1;
	}

	const timepoint_ns imu_lead_ns = static_cast<timepoint_ns>(o.imu_lead_ms * U_TIME_1MS_IN_NS);
	const timepoint_ns app_period_ns = o.app_hz > 0.0 ? static_cast<timepoint_ns>(U_TIME_1S_IN_NS / o.app_hz) : 0;
	timepoint_ns app_next_ns = 0;
	size_t imu_next = 0;
	std::vector<uint8_t> pixels;
	auto start = std::chrono::steady_clock::now();
	size_t done = 0;
	for (size_t k = 0; k < index.size(); k++) {
		uint8_t header[17];
		if (fread(header, 1, sizeof(header), zf) != sizeof(header)) {
			break;
		}
		uint32_t w, h;
		memcpy(&w, header + 9, 4);
		memcpy(&h, header + 13, 4);
		pixels.resize(static_cast<size_t>(w) * h);
		if (fread(pixels.data(), 1, pixels.size(), zf) != pixels.size()) {
			break;
		}

		const int cam = index[k].first;
		const timepoint_ns ts = index[k].second;
		if (o.realtime) {
			static timepoint_ns first_ts = 0;
			static auto first_wall = std::chrono::steady_clock::now();
			if (first_ts == 0) {
				first_ts = ts;
				first_wall = std::chrono::steady_clock::now();
			}
			std::this_thread::sleep_until(first_wall + std::chrono::nanoseconds(ts - first_ts));
		}
		if (cam < 0 || static_cast<size_t>(cam) >= frame_sinks.size() ||
		    std::find(o.skip_cameras.begin(), o.skip_cameras.end(), cam) != o.skip_cameras.end()) {
			continue;
		}
		if (o.from_s > 0.0 &&
		    ts < index.front().second + static_cast<timepoint_ns>(o.from_s * U_TIME_1S_IN_NS)) {
			continue;
		}
		if (o.to_s >= 0.0 && ts > index.front().second + static_cast<timepoint_ns>(o.to_s * U_TIME_1S_IN_NS)) {
			break;
		}

		while (imu_next < imu.size() && imu[imu_next].second.timestamp_ns <= ts + imu_lead_ns) {
			const auto &[dev, sample] = imu[imu_next++];
			auto it = device_ids.find(static_cast<t_constellation_device_id_t>(dev));
			if (it == device_ids.end()) {
				continue;
			}
			xrt_imu_sample corrected = sample;
			auto rb = rest_bias.find(it->second);
			if (rb != rest_bias.end()) {
				const xrt_vec3 gyro = {static_cast<float>(sample.gyro_rad_secs.x),
				                       static_cast<float>(sample.gyro_rad_secs.y),
				                       static_cast<float>(sample.gyro_rad_secs.z)};
				const xrt_vec3 accel = {static_cast<float>(sample.accel_m_s2.x),
				                        static_cast<float>(sample.accel_m_s2.y),
				                        static_cast<float>(sample.accel_m_s2.z)};
				if (m_gyro_rest_bias_push(&rb->second, sample.timestamp_ns, &gyro, &accel) &&
				    rb->second.still_windows == 1) {
					fprintf(stderr,
					        "Device %d lies still at %.1f s, gyro bias %.4f %.4f %.4f rad/s\n", dev,
					        time_ns_to_s(sample.timestamp_ns - index.front().second),
					        rb->second.bias.x, rb->second.bias.y, rb->second.bias.z);
				}
				xrt_vec3 out;
				m_gyro_rest_bias_apply(&rb->second, &gyro, &out);
				corrected.gyro_rad_secs = {out.x, out.y, out.z};
			}
			devices_by_id[it->second]->pushImuSample(corrected);
			for (auto &rd : replay_devices) {
				if (rd->id == it->second) {
					rd->pushImu(corrected);
				}
			}
		}

		if (o.forget_every_s > 0.0) {
			static timepoint_ns next_forget_ns = 0;
			const timepoint_ns every_ns = static_cast<timepoint_ns>(o.forget_every_s * U_TIME_1S_IN_NS);
			if (next_forget_ns == 0) {
				next_forget_ns = ts + every_ns;
			}
			if (ts >= next_forget_ns) {
				next_forget_ns += every_ns;
				for (auto &rd : replay_devices) {
					if (rd->fusion_fresh_ns <= 0) {
						continue; // The headset stays.
					}
					rd->last_solve_ns = 0;
					rd->have_hold = false;
				}
				for (auto &device : tracker->devices) {
					if (device->id == 0) {
						continue;
					}
					{
						std::unique_lock<xrt::auxiliary::os::Mutex> lock(device->data_lock);
						device->locked_data.last_known_pose.reset();
					}
					device->sensor_fusion->requestReset();
				}
				for (auto &sink : inject) {
					sink.strip_until_ns.store(ts + 300 * U_TIME_1MS_IN_NS);
				}
				fprintf(stderr, "FORGET %" PRIi64 "\n", ts);
			}
		}

		if (app_period_ns > 0) {
			if (app_next_ns == 0) {
				app_next_ns = ts;
			}
			for (; app_next_ns <= ts; app_next_ns += app_period_ns) {
				for (auto &rd : replay_devices) {
					if (rd->fusion_fresh_ns > 0 &&
					    (rd->last_solve_ns == 0 ||
					     app_next_ns - rd->last_solve_ns > rd->fusion_fresh_ns)) {
						continue;
					}
					xrt_space_relation relation;
					tracker->getTrackedPose(rd->id, app_next_ns, relation);
				}
			}
		}

		struct xrt_frame *frame = nullptr;
		u_frame_create_one_off(XRT_FORMAT_L8, w, h, &frame);
		for (uint32_t y = 0; y < h; y++) {
			memcpy(frame->data + y * frame->stride, pixels.data() + static_cast<size_t>(y) * w, w);
		}
		frame->timestamp = ts;
		frame->source_timestamp = ts;
		frame->source_sequence = k;
		xrt_sink_push_frame(frame_sinks[cam], frame);
		xrt_frame_reference(&frame, nullptr);

		for (auto &device : tracker->devices) {
			device->sensor_fusion->waitIdle();
		}
		if (++done % 2000 == 0) {
			fprintf(stderr, "  %zu frames, %.0f s\n", done,
			        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
		}
	}
	pclose(zf);

	fprintf(stderr, "Replayed %zu frames in %.1f s, output %s\n", done,
	        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count(), o.out_file.c_str());
	for (auto &device : tracker->devices) {
		fprintf(stderr,
		        "Device %d: %" PRIu64 " pose(s) rejected for disagreeing with the IMU on down, %" PRIu64
		        " for leaving a binding prior, %" PRIu64
		        " slow-search solve(s) for fitting loosely; fusion started over %" PRIu64 " time(s)\n",
		        device->id, device->gravity_rejections_total.load(), device->prior_rejections_total.load(),
		        device->rms_rejections_total.load(), device->fusion_resets_total.load());
	}

	// The blobwatches first, then the tracker and its threads, before the dataset its LED models point into goes.
	xrt_frame_context_destroy_nodes(&xfctx);
	tracker.reset();
	return 0;
}

int
main(int argc, char **argv)
{
	Options o;
	if (!parse_options(argc, argv, o)) {
		usage();
		return 2;
	}

	/*
	 * No core dumps unless asked for (CONSTELLATION_REPLAY_CORE_DUMPS=1): replays run on a desktop, where a crash
	 * would reach the desktop's crash reporter and show up as a crashed program.
	 */
	const char *dumps = getenv("CONSTELLATION_REPLAY_CORE_DUMPS");
	if (dumps == nullptr || atoi(dumps) == 0) {
		prctl(PR_SET_DUMPABLE, 0, 0, 0, 0);
	}

	try {
		return o.mode == "frames" ? do_frames(o) : do_replay(o);
	} catch (const std::exception &e) {
		fprintf(stderr, "Error: %s\n", e.what());
		return 1;
	}
}
