// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The Rift tab's drawing, layout and hit test, shared by SteamVR's dashboard tab (riftlinux-notices) and the
 *         panel of RiftLinux's own runtime (monado-service), so both are one design.
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

// X11 defines None as a macro; Hit::None is a name here.
#pragma push_macro("None")
#undef None

namespace rl::tab {

/*
 *
 * Drawing: the tab is put together here from pieces make_notice_images.py rendered (raw RGBA, no PNG decoder here)
 * and handed to SteamVR as a PNG file.
 *
 */

struct Image
{
	int w = 0;
	int h = 0;
	std::vector<uint8_t> px; //!< RGBA, not premultiplied.
};

//! "RLRG", width, height (32-bit little end), then the pixels.
inline bool
load_raw(const std::string &path, Image &out)
{
	FILE *f = fopen(path.c_str(), "rb");
	if (f == nullptr) {
		return false;
	}
	char magic[4];
	uint32_t wh[2];
	bool ok = fread(magic, 1, 4, f) == 4 && memcmp(magic, "RLRG", 4) == 0 && fread(wh, 4, 2, f) == 2 &&
	          wh[0] > 0 && wh[1] > 0 && wh[0] <= 4096 && wh[1] <= 4096;
	if (ok) {
		out.w = static_cast<int>(wh[0]);
		out.h = static_cast<int>(wh[1]);
		out.px.resize(static_cast<size_t>(out.w) * out.h * 4);
		ok = fread(out.px.data(), 1, out.px.size(), f) == out.px.size();
	}
	fclose(f);
	return ok;
}

struct Rgba
{
	uint8_t r, g, b, a;
};

inline void
blend(Image &dst, int x, int y, Rgba c, float cover)
{
	if (x < 0 || y < 0 || x >= dst.w || y >= dst.h) {
		return;
	}
	const float a = c.a / 255.0f * std::clamp(cover, 0.0f, 1.0f);
	if (a <= 0.0f) {
		return;
	}
	uint8_t *d = &dst.px[(static_cast<size_t>(y) * dst.w + x) * 4];
	const float da = d[3] / 255.0f;
	const float oa = a + da * (1.0f - a);
	const uint8_t src[3] = {c.r, c.g, c.b};
	for (int i = 0; i < 3; i++) {
		d[i] = static_cast<uint8_t>(std::lround((src[i] * a + d[i] * da * (1.0f - a)) / oa));
	}
	d[3] = static_cast<uint8_t>(std::lround(oa * 255.0f));
}

//! A rectangle with round corners, edges smoothed.
inline void
fill_round_rect(Image &dst, float x0, float y0, float x1, float y1, float radius, Rgba c)
{
	const float cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
	const float hx = (x1 - x0) / 2 - radius, hy = (y1 - y0) / 2 - radius;
	for (int y = static_cast<int>(y0) - 1; y <= static_cast<int>(y1) + 1; y++) {
		for (int x = static_cast<int>(x0) - 1; x <= static_cast<int>(x1) + 1; x++) {
			const float qx = std::fabs(x + 0.5f - cx) - hx, qy = std::fabs(y + 0.5f - cy) - hy;
			const float out = std::hypot(std::max(qx, 0.0f), std::max(qy, 0.0f));
			const float d = out + std::min(std::max(qx, qy), 0.0f) - radius;
			blend(dst, x, y, c, 0.5f - d);
		}
	}
}

inline void
fill_circle(Image &dst, float cx, float cy, float r, Rgba c)
{
	fill_round_rect(dst, cx - r, cy - r, cx + r, cy + r, r, c);
}

//! Copies @p src over @p dst at (x, y), part of it when @p sw is given (an atlas cell).
inline void
draw(Image &dst, const Image &src, int x, int y, int sx = 0, int sw = -1)
{
	sw = sw < 0 ? src.w : sw;
	for (int j = 0; j < src.h; j++) {
		for (int i = 0; i < sw; i++) {
			const uint8_t *s = &src.px[(static_cast<size_t>(j) * src.w + sx + i) * 4];
			blend(dst, x + i, y + j, Rgba{s[0], s[1], s[2], s[3]}, 1.0f);
		}
	}
}

//! @p src over @p dst at (x, y), its coverage scaled by @p alpha (greyed-out text).
inline void
draw_faded(Image &dst, const Image &src, int x, int y, float alpha)
{
	for (int j = 0; j < src.h; j++) {
		for (int i = 0; i < src.w; i++) {
			const uint8_t *s = &src.px[(static_cast<size_t>(j) * src.w + i) * 4];
			blend(dst, x + i, y + j, Rgba{s[0], s[1], s[2], s[3]}, alpha);
		}
	}
}

/*
 * The Rift tab: cards in two columns, each row a control with its name and a line on what it does. The
 * rows, their pieces (make_notice_images.py TAB_CARDS, the same ids) and where everything sits are defined here once:
 * drawing and the laser's clicks both follow this layout, so a new control is a new row, not new coordinates.
 */
constexpr int kTabW = 1600;
constexpr int kTabMargin = 40;
//! A card's width and its inner padding: make_notice_images.py TAB_CARD_W, TAB_PAD (its text is wrapped to fit).
constexpr int kTabCardW = 744;
constexpr int kTabPad = 32;
constexpr int kTabCardGap = 28;
//! Where the cards start, below the title.
constexpr int kTabTop = 176;
constexpr int kTabSwitchW = 104;
constexpr int kTabSwitchH = 56;
constexpr int kTabButtonW = 240;
constexpr int kTabButtonH = 64;
//! "Reset" and "Mute", on a slider's name line, left of its value.
constexpr int kTabSmallW = 124;
constexpr int kTabSmallH = 48;
constexpr int kTabValueW = 150;
constexpr int kTabKnobR = 22;
//! The height slider's range: the floor moved this far, centimetres.
constexpr int kHeightMinCm = -50;
constexpr int kHeightMaxCm = 50;
//! The render scale slider's range, % of the CV1's 1:1 pixels (display.render_scale).
constexpr int kScaleMin = 50;
constexpr int kScaleMax = 200;
//! The page buttons on the title's line.
constexpr int kTabPageH = 52;
//! Glyphs in the digit strip: "0123456789%+-".
constexpr int kGlyphs = 14;

const Rgba kAccent{50, 105, 190, 255};
const Rgba kInactive{90, 92, 100, 255};

//! The picture's settings: usable in SteamVR direct mode only.
struct PictureRow
{
	bool direct = false;
	bool on = false;
	//! Contrast and highlights, %.
	int strength = 50;
	//! Colour (vibrance), %.
	int colour = 50;
	//! Motion smoothing.
	bool smoothing = false;
	//! HDR Bright: HDR content placed brighter on the panel.
	bool bright = false;
	//! HDR content on screen now (the driver's "cmd-hdr-content"): HDR Bright acts on nothing else.
	bool hdr_content = false;

	bool
	operator==(const PictureRow &o) const
	{
		return direct == o.direct && on == o.on && strength == o.strength && colour == o.colour &&
		       smoothing == o.smoothing && bright == o.bright && hdr_content == o.hdr_content;
	}
};

//! The headset's state and the display settings, from the driver ("cmd-status", "cmd-display").
struct TabInfo
{
	//! Worn 1, not 0, not known -1.
	int worn = -1;
	//! Controllers' batteries, %: -1 no reading yet, -2 no such controller.
	int left = -2, right = -2;
	bool sensors_known = false;
	int sensors_found = 0, sensors_placed = 0;
	bool sensors_measuring = false;
	//! What shows the picture: 0 SteamVR's compositor, 1 direct mode on X11, 2 direct mode on Wayland, 3 RiftLinux's
	//! own runtime (native OpenXR applications).
	int display = 0;
	//! The tab in RiftLinux's own runtime, not in SteamVR: no direct mode switch, the runtime's restart line.
	bool native = false;
	//! Direct mode and the render scale as saved (the next start) and as running now; the fast path (live).
	bool direct_saved = true, direct_running = false;
	int scale_saved = 100, scale_running = 100;
	bool fast_path = true;
	/*!
	 * The Performance card (its switch on): the game's frames a second now and over the last 10 s, frames a second
	 * motion smoothing makes (-1 when it is off), the game's GPU time a frame in tenths of a ms, frames dropped in
	 * the last 10 s; -1 not known yet.
	 */
	bool perf_on = false;
	int perf_fps = -1, perf_avg = -1, perf_made = -1, perf_gpu_x10 = -1, perf_dropped = -1;
	//! The runtime's own panel: an application is running (its Close button acts), and one is being closed.
	bool app_running = false, app_closing = false;

	bool
	operator==(const TabInfo &o) const
	{
		return worn == o.worn && left == o.left && right == o.right && sensors_known == o.sensors_known &&
		       sensors_found == o.sensors_found && sensors_placed == o.sensors_placed &&
		       sensors_measuring == o.sensors_measuring && display == o.display && direct_saved == o.direct_saved &&
		       direct_running == o.direct_running && scale_saved == o.scale_saved &&
		       scale_running == o.scale_running && fast_path == o.fast_path && native == o.native &&
		       perf_on == o.perf_on && perf_fps == o.perf_fps && perf_avg == o.perf_avg &&
		       perf_made == o.perf_made && perf_gpu_x10 == o.perf_gpu_x10 && perf_dropped == o.perf_dropped &&
		       app_running == o.app_running && app_closing == o.app_closing;
	}
};

//! Everything the tab shows.
struct TabState
{
	//! The page: 0 settings, 1 display and status.
	int page = 0;
	//! The headphones' volume, %; < 0 not known yet.
	int volume = -1;
	bool muted = false;
	//! The microphone's (the default input's) level, %; < 0 not known (or no microphone is the default).
	int mic = -1;
	bool mic_muted = false;
	int height_cm = 0;
	//! "Set floor" lit for a moment after it is pressed.
	bool floor_lit = false;
	PictureRow picture;
	TabInfo info;

	bool
	operator==(const TabState &o) const
	{
		return page == o.page && volume == o.volume && muted == o.muted && mic == o.mic &&
		       mic_muted == o.mic_muted && height_cm == o.height_cm &&
		       floor_lit == o.floor_lit && picture == o.picture && info == o.info;
	}
};

enum class Kind
{
	Slider,
	Switch,
	Button,
	Note,
	//! A status line: its name, and a value put together here.
	Info,
};

enum class Ctl
{
	Volume,
	//! The headset's microphone: the default input while VR runs.
	Mic,
	Floor,
	Height,
	Recentre,
	AutoHdr,
	Strength,
	Colour,
	Bright,
	Smoothing,
	StHeadset,
	StLeft,
	StRight,
	StSensors,
	StDisplay,
	DirectMode,
	RenderScale,
	FastPath,
	//! Every sensor measured again, as on a first start (the floor is kept).
	Recalibrate,
	//! The Performance card: its switch, and its numbers.
	PerfOn,
	PfGame,
	PfAvg,
	PfSmooth,
	PfGpu,
	PfDropped,
	//! The runtime's own panel: asks the running application to quit (SteamVR's dashboard has its own Quit).
	CloseApp,
};

struct RowSpec
{
	int card;
	Ctl ctl;
	Kind kind;
	const char *id;
};

constexpr const char *kCards[] = {"sound", "view", "picture", "motion", "button", "status", "display", "perf", "app"};
constexpr int kCardCount = 9;
//! Each card's page (0 settings, 1 display and status) and column (0 left, 1 right); within a column, cards go top to
//! bottom in this order.
constexpr int kCardPage[kCardCount] = {0, 0, 0, 0, 0, 1, 1, 1, 0};
constexpr int kCardColumn[kCardCount] = {0, 0, 1, 1, 0, 0, 1, 1, 1};
//! Cards whose settings are Monado's compositor's: greyed out without direct mode.
constexpr bool kCardNeedsDirect[kCardCount] = {false, false, true, true, false, false, false, false, false};
constexpr RowSpec kRows[] = {
    {0, Ctl::Volume, Kind::Slider, "volume"},     {0, Ctl::Mic, Kind::Slider, "mic"},
    {1, Ctl::Floor, Kind::Button, "floor"},
    {1, Ctl::Height, Kind::Slider, "height"},     {4, Ctl::Recentre, Kind::Note, "button"},
    {2, Ctl::AutoHdr, Kind::Switch, "autohdr"},   {2, Ctl::Strength, Kind::Slider, "strength"},
    {2, Ctl::Colour, Kind::Slider, "colour"},     {2, Ctl::Bright, Kind::Switch, "bright"},
    {3, Ctl::Smoothing, Kind::Switch, "smoothing"},
    {5, Ctl::StHeadset, Kind::Info, "st-headset"},  {5, Ctl::StLeft, Kind::Info, "st-left"},
    {5, Ctl::StRight, Kind::Info, "st-right"},      {5, Ctl::StSensors, Kind::Info, "st-sensors"},
    {5, Ctl::Recalibrate, Kind::Button, "recalibrate"},
    {5, Ctl::StDisplay, Kind::Info, "st-display"},  {6, Ctl::DirectMode, Kind::Switch, "directmode"},
    {6, Ctl::RenderScale, Kind::Slider, "renderscale"}, {6, Ctl::FastPath, Kind::Switch, "fastpath"},
    {7, Ctl::PerfOn, Kind::Switch, "perf"},         {7, Ctl::PfGame, Kind::Info, "pf-game"},
    {7, Ctl::PfAvg, Kind::Info, "pf-avg"},          {7, Ctl::PfSmooth, Kind::Info, "pf-smooth"},
    {7, Ctl::PfGpu, Kind::Info, "pf-gpu"},          {7, Ctl::PfDropped, Kind::Info, "pf-dropped"},
    {8, Ctl::CloseApp, Kind::Button, "closeapp"},
};
constexpr int kRowCount = sizeof(kRows) / sizeof(kRows[0]);

inline int
row_of(Ctl ctl)
{
	for (int r = 0; r < kRowCount; r++) {
		if (kRows[r].ctl == ctl) {
			return r;
		}
	}
	return 0;
}

//! A strip of "0123456789%+-" in cells of one width, and where each is inked, so they sit as close as text does.
struct Digits
{
	Image img;
	int ink_x0[kGlyphs] = {};
	int ink_x1[kGlyphs] = {};

	static int
	glyph(char c)
	{
		return c == '%' ? 10 : c == '+' ? 11 : c == '-' ? 12 : c == '.' ? 13 : c - '0';
	}

	void
	find_ink()
	{
		const int cell = img.w / kGlyphs;
		for (int i = 0; i < kGlyphs; i++) {
			ink_x0[i] = cell;
			ink_x1[i] = -1;
			for (int x = 0; x < cell; x++) {
				for (int y = 0; y < img.h; y++) {
					if (img.px[(static_cast<size_t>(y) * img.w + i * cell + x) * 4 + 3] > 0) {
						ink_x0[i] = std::min(ink_x0[i], x);
						ink_x1[i] = std::max(ink_x1[i], x);
					}
				}
			}
			if (ink_x1[i] < ink_x0[i]) {
				ink_x0[i] = 0;
				ink_x1[i] = cell - 1;
			}
		}
	}

	//! @p text ("85%", "+5") as an image of its own.
	Image
	render(const std::string &text) const
	{
		constexpr int kGap = 3;
		const int cell = img.w / kGlyphs;
		int width = 0;
		for (char c : text) {
			width += ink_x1[glyph(c)] - ink_x0[glyph(c)] + 1 + kGap;
		}
		Image out;
		out.w = std::max(1, width - kGap);
		out.h = img.h;
		out.px.assign(static_cast<size_t>(out.w) * out.h * 4, 0);
		int x = 0;
		for (char c : text) {
			const int i = glyph(c), w = ink_x1[i] - ink_x0[i] + 1;
			draw(out, img, x, 0, i * cell + ink_x0[i], w);
			x += w + kGap;
		}
		return out;
	}
};

struct Pieces
{
	Image title, subtitle, needs_direct, mute, muted, setfloor, reset, measure, close, cm;
	Digits digits, digits_small;
	//! Under HDR Bright, the card's full width: what it needs, and whether there is HDR content on screen now.
	Image bright_note, hdr_on, hdr_off;
	//! Under direct mode and the render scale when the saved value waits for SteamVR's next start.
	Image restart;
	//! The page buttons: "Settings", "Display & status".
	Image page[2];
	//! The status card's words.
	Image worn, not_worn, no_data, of, placed, measuring, direct_wayland, direct_x11, steamvr, sep;
	//! The Performance card's words (optional pieces: an older set lacks them).
	Image fps, ms, off, frames;
	//! The runtime's own panel (optional pieces: an older set lacks them): its name, its restart line.
	Image runtime, restart_runtime;
	Image card[kCardCount];
	//! Each row's name (none for notes) and its explanation (none for status lines).
	Image label[kRowCount], help[kRowCount];

	bool
	load(const std::string &dir)
	{
		const auto piece = [&](const std::string &name, Image &img) {
			if (load_raw(dir + "/tab-" + name + ".rgba", img)) {
				return true;
			}
			fprintf(stderr, "riftlinux-notices: no tab piece %s\n", name.c_str());
			return false;
		};
		bool ok = piece("title", title) && piece("subtitle", subtitle) && piece("needs-direct", needs_direct) &&
		          piece("btn-mute", mute) && piece("btn-muted", muted) && piece("btn-setfloor", setfloor) &&
		          piece("btn-reset", reset) && piece("btn-measure", measure) && piece("btn-close", close) &&
		          piece("cm", cm) &&
		          piece("digits", digits.img) &&
		          piece("digits-small", digits_small.img) && piece("bright-note", bright_note) &&
		          piece("bright-status-on", hdr_on) && piece("bright-status-off", hdr_off) &&
		          piece("restart", restart) && piece("page-settings", page[0]) && piece("page-display", page[1]) &&
		          piece("word-worn", worn) && piece("word-not-worn", not_worn) && piece("word-no-data", no_data) &&
		          piece("word-of", of) && piece("word-placed", placed) && piece("word-measuring", measuring) &&
		          piece("word-direct-wayland", direct_wayland) && piece("word-direct-x11", direct_x11) &&
		          piece("word-steamvr", steamvr) && piece("word-sep", sep);
		for (int c = 0; ok && c < kCardCount; c++) {
			ok = piece(std::string("card-") + kCards[c], card[c]);
		}
		load_raw(dir + "/tab-word-runtime.rgba", runtime);
		load_raw(dir + "/tab-word-fps.rgba", fps);
		load_raw(dir + "/tab-word-ms.rgba", ms);
		load_raw(dir + "/tab-word-off.rgba", off);
		load_raw(dir + "/tab-word-frames.rgba", frames);
		load_raw(dir + "/tab-restart-runtime.rgba", restart_runtime);
		for (int r = 0; ok && r < kRowCount; r++) {
			ok = (kRows[r].kind == Kind::Note || piece(std::string(kRows[r].id) + "-label", label[r])) &&
			     (kRows[r].kind == Kind::Info || piece(std::string(kRows[r].id) + "-help", help[r]));
		}
		if (!ok) {
			return false;
		}
		digits.find_ink();
		digits_small.find_ink();
		return true;
	}
};

struct Rect
{
	int x0 = 0, y0 = 0, x1 = 0, y1 = 0;

	bool
	has(float x, float y, int slack = 0) const
	{
		return x >= x0 - slack && x <= x1 + slack && y >= y0 - slack && y <= y1 + slack;
	}
};

//! Where one row's parts sit, pixels from the tab's top left.
struct RowPlace
{
	Rect row;
	int label_y = 0, help_y = 0;
	//! A switch or a button at the row's right.
	Rect control;
	//! A slider's "Reset" or "Mute", on its name line; its track and knob; its value, right-aligned, centred on y.
	Rect small;
	int track_x0 = 0, track_x1 = 0, track_y = 0;
	int value_right = 0, value_y = 0;
	//! Lines below a row across the card's width (HDR Bright's note and status).
	int extra_y = 0;
};

struct Layout
{
	int h = 0;
	//! The page buttons on the title's line.
	Rect page[2];
	Rect card[kCardCount];
	int heading_y[kCardCount] = {};
	RowPlace rows[kRowCount];
	//! Rows left out: the direct mode switch in the runtime's own panel.
	bool hidden[kRowCount] = {};
	//! Cards left out (every row hidden).
	bool card_hidden[kCardCount] = {};
};

//! The tab's layout from its pieces' sizes (the explanations wrap to one, two or three lines); @p native: the
//! runtime's own panel, without the direct mode switch.
inline Layout
tab_layout(const Pieces &p, bool native = false)
{
	Layout L;
	for (int r = 0; r < kRowCount; r++) {
		// The runtime counts the game's frames itself; GPU time and late frames are SteamVR's timings only. Closing
		// the application is the runtime's: SteamVR's dashboard has its own Quit.
		L.hidden[r] = native ? (kRows[r].ctl == Ctl::DirectMode || kRows[r].ctl == Ctl::PfGpu ||
		                        kRows[r].ctl == Ctl::PfDropped)
		                     : kRows[r].ctl == Ctl::CloseApp;
	}
	for (int c = 0; c < kCardCount; c++) {
		bool any = false;
		for (int r = 0; r < kRowCount; r++) {
			any = any || (kRows[r].card == c && !L.hidden[r]);
		}
		L.card_hidden[c] = !any;
	}
	// The page buttons, right-aligned on the title's line.
	int px1 = kTabW - kTabMargin;
	const int py = 34 + p.title.h / 2;
	for (int i = 1; i >= 0; i--) {
		const int w = p.page[i].w + 48;
		L.page[i] = {px1 - w, py - kTabPageH / 2, px1, py + kTabPageH / 2};
		px1 -= w + 12;
	}
	int column_y[2][2] = {{kTabTop, kTabTop}, {kTabTop, kTabTop}};
	const int column_gap = kTabW - 2 * kTabMargin - 2 * kTabCardW;
	for (int c = 0; c < kCardCount; c++) {
		if (L.card_hidden[c]) {
			continue;
		}
		const int column = kCardColumn[c];
		const int x0 = kTabMargin + column * (kTabCardW + column_gap);
		const int ix0 = x0 + kTabPad, ix1 = x0 + kTabCardW - kTabPad;
		int y = column_y[kCardPage[c]][column];
		L.card[c] = {x0, y, x0 + kTabCardW, y};
		L.heading_y[c] = y + 26;
		y = L.heading_y[c] + p.card[c].h + 8;
		for (int r = 0; r < kRowCount; r++) {
			if (kRows[r].card != c || L.hidden[r]) {
				continue;
			}
			RowPlace &R = L.rows[r];
			R.row = {x0, y, x0 + kTabCardW, y};
			y += 16;
			const Image &label = p.label[r], &help = p.help[r];
			switch (kRows[r].kind) {
			case Kind::Note:
				R.help_y = y;
				y += help.h;
				break;
			case Kind::Info:
				R.label_y = y;
				R.value_right = ix1;
				R.value_y = y + label.h / 2;
				y += label.h;
				break;
			case Kind::Switch:
			case Kind::Button: {
				R.label_y = y;
				R.help_y = y + label.h + 2;
				const int text_h = label.h + 2 + help.h;
				const bool sw = kRows[r].kind == Kind::Switch;
				const int cw = sw ? kTabSwitchW : kTabButtonW, ch = sw ? kTabSwitchH : kTabButtonH;
				const int block = std::max(text_h, ch);
				const int cy = y + block / 2;
				R.control = {ix1 - cw, cy - ch / 2, ix1, cy + ch / 2};
				y += block;
				// HDR Bright: below it, the card's width, what it needs and whether there is HDR content now.
				if (kRows[r].ctl == Ctl::Bright) {
					R.extra_y = y + 8;
					y = R.extra_y + p.bright_note.h + 6 + std::max(p.hdr_on.h, p.hdr_off.h);
				}
				// Direct mode: room for the line saying a change waits for SteamVR's next start.
				if (kRows[r].ctl == Ctl::DirectMode) {
					R.extra_y = y + 6;
					y = R.extra_y + p.restart.h;
				}
				break;
			}
			case Kind::Slider: {
				R.label_y = y;
				R.value_right = ix1;
				R.value_y = y + label.h / 2;
				R.small = {ix1 - kTabValueW - 16 - kTabSmallW, R.value_y - kTabSmallH / 2, ix1 - kTabValueW - 16,
				           R.value_y + kTabSmallH / 2};
				R.help_y = y + label.h + 2;
				y = R.help_y + help.h + 14 + kTabKnobR;
				R.track_x0 = ix0 + kTabKnobR;
				R.track_x1 = ix1 - kTabKnobR;
				R.track_y = y;
				y += kTabKnobR;
				if (kRows[r].ctl == Ctl::RenderScale) {
					R.extra_y = y + 10;
					y = R.extra_y + p.restart.h;
				}
				break;
			}
			}
			y += 16;
			R.row.y1 = y;
		}
		L.card[c].y1 = y + 10;
		column_y[kCardPage[c]][column] = L.card[c].y1 + kTabCardGap;
	}
	// One size for both pages (the overlay's image keeps its size): the taller one's.
	L.h = std::max(std::max(column_y[0][0], column_y[0][1]), std::max(column_y[1][0], column_y[1][1])) - kTabCardGap +
	      kTabMargin;
	return L;
}

//! @p text ("85%", "+5") right-aligned at @p right_x, centred on @p y, faded by @p alpha; returns its left edge.
inline int
draw_number(Image &img, const Digits &digits, const std::string &text, int right_x, int y, float alpha = 1.0f)
{
	const Image number = digits.render(text);
	draw_faded(img, number, right_x - number.w, y - number.h / 2, alpha);
	return right_x - number.w;
}

//! Pieces side by side, centred on one line, @p gap apart.
inline Image
side_by_side(const std::vector<Image> &parts, int gap)
{
	Image out;
	for (const Image &part : parts) {
		out.w += part.w + (out.w > 0 ? gap : 0);
		out.h = std::max(out.h, part.h);
	}
	out.px.assign(static_cast<size_t>(std::max(out.w, 1)) * std::max(out.h, 1) * 4, 0);
	int x = 0;
	for (const Image &part : parts) {
		draw(out, part, x, (out.h - part.h) / 2);
		x += part.w + gap;
	}
	return out;
}

//! A status line's value: what the driver said, in words and numbers.
inline Image
status_value(const Pieces &p, const TabState &s, Ctl ctl)
{
	const TabInfo &i = s.info;
	switch (ctl) {
	case Ctl::StHeadset: return i.worn < 0 ? p.no_data : i.worn ? p.worn : p.not_worn;
	case Ctl::StLeft:
	case Ctl::StRight: {
		const int v = ctl == Ctl::StLeft ? i.left : i.right;
		return v >= 0 ? p.digits_small.render(std::to_string(std::min(v, 100)) + "%") : p.no_data;
	}
	case Ctl::StSensors: {
		if (!i.sensors_known) {
			return p.no_data;
		}
		std::vector<Image> parts = {p.digits_small.render(std::to_string(i.sensors_placed)), p.of,
		                            p.digits_small.render(std::to_string(i.sensors_found)), p.placed};
		if (i.sensors_measuring) {
			parts.push_back(p.sep);
			parts.push_back(p.measuring);
		}
		return side_by_side(parts, 10);
	}
	case Ctl::StDisplay:
		return i.display == 3 ? p.runtime : i.display == 2 ? p.direct_wayland : i.display == 1 ? p.direct_x11 : p.steamvr;
	case Ctl::PfGame:
	case Ctl::PfAvg:
	case Ctl::PfSmooth:
	case Ctl::PfGpu:
	case Ctl::PfDropped: {
		if (!i.perf_on) {
			return p.off;
		}
		const int v = ctl == Ctl::PfGame     ? i.perf_fps
		              : ctl == Ctl::PfAvg    ? i.perf_avg
		              : ctl == Ctl::PfSmooth ? i.perf_made
		              : ctl == Ctl::PfGpu    ? i.perf_gpu_x10
		                                     : i.perf_dropped;
		if (ctl == Ctl::PfSmooth && v < 0 && i.perf_fps >= 0) {
			return p.off;
		}
		if (v < 0) {
			return p.no_data;
		}
		if (ctl == Ctl::PfGpu) {
			return side_by_side({p.digits_small.render(std::to_string(v / 10) + "." + std::to_string(v % 10)), p.ms},
			                    8);
		}
		if (ctl == Ctl::PfDropped) {
			return side_by_side({p.digits_small.render(std::to_string(v)), p.frames}, 8);
		}
		return side_by_side({p.digits_small.render((ctl == Ctl::PfSmooth ? "+" : "") + std::to_string(v)), p.fps},
		                    8);
	}
	default: return Image{};
	}
}

//! A slider: track, filled from @p fill_from up to @p frac, knob.
inline void
draw_slider(Image &img, const RowPlace &R, int oy, float frac, float fill_from, Rgba fill, bool enabled)
{
	const int y = R.track_y - oy;
	const float x0 = R.track_x0, x1 = R.track_x1;
	const float at = x0 + (x1 - x0) * std::clamp(frac, 0.0f, 1.0f);
	const float from = x0 + (x1 - x0) * fill_from;
	fill_round_rect(img, x0 - 6, y - 6, x1 + 6, y + 6, 6, Rgba{40, 43, 52, 255});
	fill_round_rect(img, std::min(at, from) - 6, y - 6, std::max(at, from) + 6, y + 6, 6, fill);
	fill_circle(img, at, y, kTabKnobR, enabled ? Rgba{225, 228, 235, 255} : Rgba{110, 114, 124, 255});
}

//! A switch: on (accent, knob right) or off (dark, knob left).
inline void
draw_switch(Image &img, const Rect &r, int oy, bool on, bool enabled)
{
	const float h = static_cast<float>(r.y1 - r.y0);
	const Rgba track = on ? (enabled ? kAccent : Rgba{45, 62, 92, 255}) : Rgba{44, 48, 58, 255};
	fill_round_rect(img, r.x0, r.y0 - oy, r.x1, r.y1 - oy, h / 2, track);
	const float knob_r = h / 2 - 6;
	const float cx = on ? r.x1 - h / 2 : r.x0 + h / 2;
	fill_circle(img, cx, (r.y0 + r.y1) / 2.0f - oy, knob_r,
	            enabled ? Rgba{235, 238, 243, 255} : Rgba{110, 114, 124, 255});
}

//! A button with @p label centred on it; its label faded when @p alpha < 1 (greyed out).
inline void
draw_button(Image &img, const Rect &r, int oy, const Image &label, Rgba fill, float alpha = 1.0f)
{
	fill_round_rect(img, r.x0, r.y0 - oy, r.x1, r.y1 - oy, 16, fill);
	draw_faded(img, label, (r.x0 + r.x1 - label.w) / 2, (r.y0 + r.y1 - label.h) / 2 - oy, alpha);
}

//! One card at its place, moved up by @p oy (the volume shown on its own).
inline void
draw_card(Image &img, const Pieces &p, const Layout &L, const TabState &s, int c, int oy)
{
	const PictureRow &pic = s.picture;
	const bool enabled = !kCardNeedsDirect[c] || pic.direct;
	const float text_alpha = enabled ? 1.0f : 0.3f;
	const Rect &card = L.card[c];
	fill_round_rect(img, card.x0, card.y0 - oy, card.x1, card.y1 - oy, 24, Rgba{21, 23, 29, 255});
	draw(img, p.card[c], card.x0 + kTabPad, L.heading_y[c] - oy);
	bool first = true;
	for (int r = 0; r < kRowCount; r++) {
		if (kRows[r].card != c || L.hidden[r]) {
			continue;
		}
		const RowPlace &R = L.rows[r];
		if (!first) {
			fill_round_rect(img, card.x0 + kTabPad, R.row.y0 - oy - 1, card.x1 - kTabPad, R.row.y0 - oy + 1, 1,
			                Rgba{36, 39, 47, 255});
		}
		first = false;
		const int ix0 = card.x0 + kTabPad;
		// The fast path is Monado's compositor's: its row greyed without direct mode, its card not.
		const bool row_enabled = enabled && (kRows[r].ctl != Ctl::FastPath || pic.direct);
		const float row_alpha = row_enabled ? 1.0f : 0.3f;
		if (kRows[r].kind != Kind::Note) {
			draw_faded(img, p.label[r], ix0, R.label_y - oy, row_alpha);
		}
		if (kRows[r].kind != Kind::Info) {
			draw_faded(img, p.help[r], ix0, R.help_y - oy, row_alpha);
		}
		switch (kRows[r].ctl) {
		case Ctl::Volume: {
			const bool known = s.volume >= 0;
			draw_button(img, R.small, oy, s.muted ? p.muted : p.mute,
			            s.muted ? Rgba{170, 45, 38, 255} : Rgba{38, 42, 52, 255});
			draw_slider(img, R, oy, std::clamp(s.volume, 0, 100) / 100.0f, 0.0f,
			            s.muted || !known ? kInactive : kAccent, true);
			if (known) {
				draw_number(img, p.digits, std::to_string(std::clamp(s.volume, 0, 100)) + "%", R.value_right,
				            R.value_y - oy);
			}
			break;
		}
		case Ctl::Mic: {
			const bool known = s.mic >= 0;
			draw_button(img, R.small, oy, s.mic_muted ? p.muted : p.mute,
			            s.mic_muted ? Rgba{170, 45, 38, 255} : Rgba{38, 42, 52, 255}, known ? 1.0f : 0.3f);
			draw_slider(img, R, oy, std::clamp(s.mic, 0, 100) / 100.0f, 0.0f,
			            s.mic_muted || !known ? kInactive : kAccent, known);
			if (known) {
				draw_number(img, p.digits, std::to_string(std::clamp(s.mic, 0, 100)) + "%", R.value_right,
				            R.value_y - oy);
			}
			break;
		}
		case Ctl::Floor:
			draw_button(img, R.control, oy, p.setfloor, s.floor_lit ? kAccent : Rgba{38, 42, 52, 255});
			break;
		case Ctl::Height: {
			draw_button(img, R.small, oy, p.reset, Rgba{38, 42, 52, 255});
			const float zero = (0.0f - kHeightMinCm) / (kHeightMaxCm - kHeightMinCm);
			draw_slider(img, R, oy, (float)(s.height_cm - kHeightMinCm) / (kHeightMaxCm - kHeightMinCm), zero, kAccent,
			            true);
			draw(img, p.cm, R.value_right - p.cm.w, R.value_y - oy - p.cm.h / 2 + 3);
			draw_number(img, p.digits, (s.height_cm > 0 ? "+" : "") + std::to_string(s.height_cm),
			            R.value_right - p.cm.w - 8, R.value_y - oy);
			break;
		}
		case Ctl::Recentre: break;
		case Ctl::AutoHdr: draw_switch(img, R.control, oy, pic.on, enabled); break;
		case Ctl::Strength:
		case Ctl::Colour: {
			const int v = std::clamp(kRows[r].ctl == Ctl::Strength ? pic.strength : pic.colour, 0, 100);
			draw_button(img, R.small, oy, p.reset, enabled ? Rgba{38, 42, 52, 255} : Rgba{28, 30, 37, 255},
			            text_alpha);
			draw_slider(img, R, oy, v / 100.0f, 0.0f, enabled && pic.on ? kAccent : kInactive, enabled);
			draw_number(img, p.digits, std::to_string(v) + "%", R.value_right, R.value_y - oy, text_alpha);
			break;
		}
		case Ctl::Bright: {
			draw_switch(img, R.control, oy, pic.bright, enabled);
			draw_faded(img, p.bright_note, ix0, R.extra_y - oy, text_alpha);
			const Image &status = pic.hdr_content && enabled ? p.hdr_on : p.hdr_off;
			const int sy = R.extra_y + p.bright_note.h + 6 - oy;
			Image dot;
			dot.w = dot.h = 18;
			dot.px.assign(static_cast<size_t>(dot.w) * dot.h * 4, 0);
			fill_circle(dot, 9, 9, 7,
			            pic.hdr_content && enabled ? Rgba{80, 200, 110, 255} : Rgba{110, 114, 124, 255});
			draw_faded(img, dot, ix0, sy + (status.h - dot.h) / 2, text_alpha);
			draw_faded(img, status, ix0 + 26, sy, text_alpha);
			break;
		}
		case Ctl::Smoothing: draw_switch(img, R.control, oy, pic.smoothing, enabled); break;
		case Ctl::StHeadset:
		case Ctl::StLeft:
		case Ctl::StRight:
		case Ctl::StSensors:
		case Ctl::StDisplay:
		case Ctl::PfGame:
		case Ctl::PfAvg:
		case Ctl::PfSmooth:
		case Ctl::PfGpu:
		case Ctl::PfDropped: {
			const Image value = status_value(p, s, kRows[r].ctl);
			draw(img, value, R.value_right - value.w, R.value_y - oy - value.h / 2);
			break;
		}
		case Ctl::DirectMode:
			draw_switch(img, R.control, oy, s.info.direct_saved, true);
			if (s.info.direct_saved != s.info.direct_running) {
				draw(img, p.restart, ix0, R.extra_y - oy);
			}
			break;
		case Ctl::RenderScale: {
			const int v = std::clamp(s.info.scale_saved, kScaleMin, kScaleMax);
			draw_button(img, R.small, oy, p.reset, Rgba{38, 42, 52, 255});
			draw_slider(img, R, oy, (float)(v - kScaleMin) / (kScaleMax - kScaleMin), 0.0f, kAccent, true);
			draw_number(img, p.digits, std::to_string(v) + "%", R.value_right, R.value_y - oy);
			if (s.info.scale_saved != s.info.scale_running) {
				draw(img, s.info.native && p.restart_runtime.w > 0 ? p.restart_runtime : p.restart, ix0,
				     R.extra_y - oy);
			}
			break;
		}
		case Ctl::FastPath: draw_switch(img, R.control, oy, s.info.fast_path && pic.direct, row_enabled); break;
		case Ctl::PerfOn: draw_switch(img, R.control, oy, s.info.perf_on, true); break;
		case Ctl::CloseApp: {
			// Greyed with no application running, or while one is being closed.
			const bool can = s.info.app_running && !s.info.app_closing;
			draw_button(img, R.control, oy, p.close, can ? Rgba{120, 40, 36, 255} : Rgba{28, 30, 37, 255},
			            can ? 1.0f : 0.3f);
			break;
		}
		case Ctl::Recalibrate:
			// Greyed while measuring is under way (automatic or asked for): it takes no press then.
			draw_button(img, R.control, oy, p.measure,
			            s.info.sensors_measuring ? Rgba{28, 30, 37, 255} : Rgba{38, 42, 52, 255},
			            s.info.sensors_measuring ? 0.3f : 1.0f);
			break;
		}
	}
	// Without direct mode: why the card is grey, a tag on its heading's line.
	if (!enabled) {
		const int x1 = card.x1 - kTabPad, x0 = x1 - p.needs_direct.w - 28;
		const int cy = L.heading_y[c] - oy + p.card[c].h / 2;
		const int h = p.needs_direct.h + 12;
		fill_round_rect(img, x0, cy - h / 2, x1, cy + h / 2, h / 2.0f, Rgba{58, 44, 18, 255});
		draw(img, p.needs_direct, x0 + 14, cy - p.needs_direct.h / 2);
	}
}

//! The whole tab.
inline Image
render_tab(const Pieces &p, const Layout &L, const TabState &s)
{
	Image img;
	img.w = kTabW;
	img.h = L.h;
	img.px.assign(static_cast<size_t>(img.w) * img.h * 4, 0);
	fill_round_rect(img, 0, 0, img.w, img.h, 36, Rgba{10, 11, 15, 255});
	draw(img, p.title, kTabMargin + 4, 34);
	draw(img, p.subtitle, kTabMargin + 6, 34 + p.title.h + 2);
	for (int i = 0; i < 2; i++) {
		draw_button(img, L.page[i], 0, p.page[i], s.page == i ? kAccent : Rgba{38, 42, 52, 255});
	}
	for (int c = 0; c < kCardCount; c++) {
		if (kCardPage[c] == s.page && !L.card_hidden[c]) {
			draw_card(img, p, L, s, c, 0);
		}
	}
	return img;
}

//! The sound card alone, shown in front of the view when the volume changes from elsewhere.
inline Image
render_volume_panel(const Pieces &p, const Layout &L, const TabState &s)
{
	const Rect &card = L.card[0];
	Image img;
	img.w = kTabW;
	img.h = card.y1 - card.y0;
	img.px.assign(static_cast<size_t>(img.w) * img.h * 4, 0);
	draw_card(img, p, L, s, 0, card.y0);
	// Only the card's width: its column's left edge to its right.
	Image out;
	out.w = card.x1 - card.x0;
	out.h = img.h;
	out.px.resize(static_cast<size_t>(out.w) * out.h * 4);
	for (int y = 0; y < out.h; y++) {
		memcpy(&out.px[static_cast<size_t>(y) * out.w * 4], &img.px[(static_cast<size_t>(y) * img.w + card.x0) * 4],
		       static_cast<size_t>(out.w) * 4);
	}
	return out;
}

//! What the laser's point is on: a row's control, its small button ("Reset", "Mute"), or its slider's track.
struct Hit
{
	int row = -1;
	enum
	{
		None,
		Control,
		Small,
		Track,
		//! A page button; @ref row is the page.
		Page,
	} part = None;
};

//! A slider's value as a fraction of its track, as @p s shows it; -1 for rows that are no slider.
inline float
slider_frac(const TabState &s, Ctl ctl)
{
	switch (ctl) {
	case Ctl::Volume: return std::clamp(s.volume, 0, 100) / 100.0f;
	case Ctl::Mic: return std::clamp(s.mic, 0, 100) / 100.0f;
	case Ctl::Height: return (float)(s.height_cm - kHeightMinCm) / (kHeightMaxCm - kHeightMinCm);
	case Ctl::Strength: return std::clamp(s.picture.strength, 0, 100) / 100.0f;
	case Ctl::Colour: return std::clamp(s.picture.colour, 0, 100) / 100.0f;
	case Ctl::RenderScale:
		return (float)(std::clamp(s.info.scale_saved, kScaleMin, kScaleMax) - kScaleMin) / (kScaleMax - kScaleMin);
	default: return -1.0f;
	}
}

//! Where a slider's knob is drawn at @p frac.
inline float
knob_x(const Layout &L, int row, float frac)
{
	const RowPlace &R = L.rows[row];
	return R.track_x0 + (R.track_x1 - R.track_x0) * std::clamp(frac, 0.0f, 1.0f);
}

//! How far from a slider's knob the laser still takes it, pixels: well beyond the knob, short of the track's ends.
constexpr float kKnobGrab = 2.2f * kTabKnobR;

/*!
 * What the laser's point is on. With @p shown (what the tab shows), only a switch itself or a slider's knob counts:
 * a whole row toggled its switch and a press near a track jumped its slider, and the laser swept over them by
 * accident (Direct mode switched off as the controllers came back from measuring).
 */
inline Hit
tab_hit(const Layout &L, int page, float x, float y, const TabState *shown = nullptr)
{
	for (int i = 0; i < 2; i++) {
		if (L.page[i].has(x, y, 8)) {
			return {i, Hit::Page};
		}
	}
	for (int r = 0; r < kRowCount; r++) {
		const RowPlace &R = L.rows[r];
		if (kCardPage[kRows[r].card] != page || L.hidden[r]) {
			continue;
		}
		switch (kRows[r].kind) {
		case Kind::Switch:
			// Without what is shown: the whole row toggles its switch.
			if (shown != nullptr ? R.control.has(x, y, 14) : R.row.has(x, y)) {
				return {r, Hit::Control};
			}
			break;
		case Kind::Button:
			if (R.control.has(x, y, 10)) {
				return {r, Hit::Control};
			}
			break;
		case Kind::Slider:
			if (R.small.has(x, y, 8)) {
				return {r, Hit::Small};
			}
			if (shown != nullptr) {
				const float frac = slider_frac(*shown, kRows[r].ctl);
				if (frac >= 0.0f && std::fabs(x - knob_x(L, r, frac)) <= kKnobGrab &&
				    std::fabs(y - R.track_y) <= 36) {
					return {r, Hit::Track};
				}
			} else if (x >= R.track_x0 - 30 && x <= R.track_x1 + 30 && std::fabs(y - R.track_y) <= 36) {
				return {r, Hit::Track};
			}
			break;
		case Kind::Note:
		case Kind::Info: break;
		}
	}
	return {};
}

//! Where along a slider's track the point is, 0 to 1.
inline float
track_frac(const Layout &L, int row, float x)
{
	const RowPlace &R = L.rows[row];
	return std::clamp((x - R.track_x0) / static_cast<float>(R.track_x1 - R.track_x0), 0.0f, 1.0f);
}

/*!
 * One press of the trigger on the tab, the same in SteamVR's tab and the runtime's panel. A switch, a button or a
 * page acts when the trigger is let go on the one it was pressed on. A slider's knob, grabbed, follows the laser from
 * where it was grabbed (no jump to the point), once the laser has moved a few pixels.
 */
struct Press
{
	Hit hit;
	float x0 = 0.0f, y0 = 0.0f;
	//! The knob's centre minus the laser's point when grabbed.
	float grab_dx = 0.0f;
	bool moved = false;

	static constexpr float kDeadPx = 8.0f;

	//! The trigger pressed at (@p x, @p y) on what @p shown shows.
	Hit
	begin(const Layout &L, int page, float x, float y, const TabState &shown)
	{
		hit = tab_hit(L, page, x, y, &shown);
		x0 = x;
		y0 = y;
		moved = false;
		grab_dx = 0.0f;
		if (hit.part == Hit::Track) {
			grab_dx = knob_x(L, hit.row, slider_frac(shown, kRows[hit.row].ctl)) - x;
		}
		return hit;
	}

	//! A grabbed slider's new fraction while held at @p x, once moved past the dead zone.
	std::optional<float>
	drag(const Layout &L, float x)
	{
		if (hit.part != Hit::Track) {
			return std::nullopt;
		}
		moved = moved || std::fabs(x - x0) >= kDeadPx;
		if (!moved) {
			return std::nullopt;
		}
		return track_frac(L, hit.row, x + grab_dx);
	}

	//! Let go at (@p x, @p y): the switch, button or page to act on when it was pressed and let go on the same one,
	//! else none.
	Hit
	release(const Layout &L, int page, float x, float y, const TabState &shown)
	{
		const Hit pressed = hit;
		hit = {};
		if (pressed.part == Hit::None || pressed.part == Hit::Track) {
			return {};
		}
		const Hit now = tab_hit(L, page, x, y, &shown);
		return now.part == pressed.part && now.row == pressed.row ? pressed : Hit{};
	}
};


} // namespace rl::tab

#pragma pop_macro("None")
