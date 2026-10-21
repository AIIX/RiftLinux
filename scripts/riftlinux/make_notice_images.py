#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
# SPDX-License-Identifier: BSL-1.0
"""Renders the notices the SteamVR driver shows in the headset (riftlinux-notices) as PNG panels, one per notice and
sensor number: src/xrt/targets/steamvr_drv/resources/notices/<key>[-<n>].png. Rendered here, on a machine with fonts,
so the headset shows the same text everywhere.

Usage: make_notice_images.py [out dir]
"""

import os
import sys

from PIL import Image, ImageDraw, ImageFont

FONT_DIRS = ["/usr/share/fonts/truetype/noto", "/usr/share/fonts/truetype/dejavu"]
AMBER, GREEN, BLUE, RED = (240, 170, 40), (80, 200, 110), (80, 150, 240), (230, 80, 70)
SENSORS = range(1, 5)  # the CV1 runs up to four sensors

# key: (accent, title, body); {n} is the sensor's number.
NOTICES = {
    "sensor-moved": (AMBER, "Sensor {n} moved", "Measuring its new place; tracking goes on meanwhile."),
    "sensor-placed": (GREEN, "Sensor {n} placed again", "All sensors are tracking."),
    "sensor-saved": (GREEN, "Sensor {n}: new place saved", "Fine-tuned and kept for next time."),
    # Shown again while the headset is worn and no sensor is placed yet (the controllers are hidden until then).
    # Placing a sensor quickly needs the head turned through views (about 20 degrees or more): said plainly.
    "calibration-start": (BLUE, "Measuring where the sensors are",
                          "Face the sensors and slowly turn your head left and right,\n"
                          "then up and down. Keep going until this notice goes away.\n"
                          "The controllers appear when it is done."),
    "calibration-done": (GREEN, "Sensors measured", "Their places are saved for next time."),
    "calibration-failed": (RED, "Measuring the sensors didn't work", "It tries again: face the sensors, turn your head left and right."),
    "display-vrr": (AMBER, "The picture may tear", "NVIDIA and a VRR monitor: run riftlinux-setup nvidia-vrr"),
    # The system around the driver (posted at start, shown when the headset is put on).
    "rt-refused": (AMBER, "Real-time priority refused", "Start Steam from your desktop, not a remote login."),
    "rt-missing": (AMBER, "RealtimeKit is not installed", "Install rtkit: under load the picture may stutter."),
    # 30 s after the headset was first put on, a controller that never reported.
    "controllers-none": (BLUE, "No controllers found yet", "Press a button on each to wake it, or check its battery."),
    "controller-left-none": (BLUE, "Left controller not found", "Press a button to wake it, or check its battery."),
    "controller-right-none": (BLUE, "Right controller not found", "Press a button to wake it, or check its battery."),
}

# The welcome panel, shown once after installing: what to know without reading anything first.
WELCOME_TITLE = "Welcome to RiftLinux"
WELCOME_SUB = "Your Rift CV1 is ready. No setup needed: the room and the sensors are taken care of."
WELCOME_ROWS = [
    ("Rift tab", "Press the right Oculus button: sound, height, floor, picture, display and status."),
    ("Recentre", "Hold the right Oculus button for a second."),
    ("Floor", "Lay a controller on the floor, then Set floor in the Rift tab (View)."),
    ("Controllers", "Press any button to wake them."),
]
WELCOME_FOOT = "Shown once. Every setting is also explained in the Rift tab."


def font(name, size):
    for d in FONT_DIRS:
        for f in (name, name.replace("NotoSans", "DejaVuSans")):
            path = os.path.join(d, f)
            if os.path.exists(path):
                return ImageFont.truetype(path, size)
    raise SystemExit(f"no font {name}")


TITLE_SIZE, BODY_SIZE, LEFT, RIGHT = 64, 40, 72, 56


def text_width(text, f):
    left, _, right, _ = f.getbbox(text)
    return right - left


BODY_LINE = 54  # A body line's height; a body may have several ("\n").


def render(accent, title, body, path, w):
    lines = body.split("\n")
    h, r = 220 + BODY_LINE * (len(lines) - 1), 36
    img = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    d.rounded_rectangle((0, 0, w - 1, h - 1), radius=r, fill=(18, 20, 26, 230), outline=(255, 255, 255, 40), width=2)
    d.rounded_rectangle((24, 30, 40, h - 30), radius=8, fill=accent + (255,))
    d.text((LEFT, 38), title, font=font("NotoSans-Bold.ttf", TITLE_SIZE), fill=(255, 255, 255, 255))
    for i, line in enumerate(lines):
        d.text((LEFT, 128 + BODY_LINE * i), line, font=font("NotoSans-Regular.ttf", BODY_SIZE), fill=(215, 220, 230, 255))
    img.save(path, optimize=True)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "..", "..", "src", "xrt", "targets", "steamvr_drv",
                                                             "resources", "notices")
    os.makedirs(out, exist_ok=True)
    panels = []
    for key, (accent, title, body) in NOTICES.items():
        for n in (SENSORS if "{n}" in title else [None]):
            name = f"{key}-{n}.png" if n else f"{key}.png"
            panels.append((accent, title.format(n=n), body.format(n=n), os.path.join(out, name)))
    # One width for all, the widest text's: the headset shows every panel at the same size.
    tf, bf = font("NotoSans-Bold.ttf", TITLE_SIZE), font("NotoSans-Regular.ttf", BODY_SIZE)
    w = max(max([text_width(p[1], tf)] + [text_width(line, bf) for line in p[2].split("\n")]) for p in panels) + LEFT + RIGHT
    for accent, title, body, path in panels:
        render(accent, title, body, path, w)
    print(f"{len(panels)} notices, {w} px wide, in {os.path.normpath(out)}")
    render_welcome(os.path.join(out, "welcome.png"))
    render_dark_test(os.path.join(out, "test-dark.png"))
    render_hdr_chart(os.path.join(out, "hdr-chart.rlhd"))
    render_tab(out)


def write_raw(img, path):
    """The helper has no PNG decoder: its pieces are raw RGBA, after "RLRG", width and height (32-bit, little end)."""
    img = img.convert("RGBA")
    with open(path, "wb") as f:
        f.write(b"RLRG" + img.width.to_bytes(4, "little") + img.height.to_bytes(4, "little") + img.tobytes())


def text_sprite(text, f, colour):
    left, top, right, bottom = f.getbbox(text)
    img = Image.new("RGBA", (right - left + 4, bottom - top + 4), colour + (0,))
    ImageDraw.Draw(img).text((2 - left, 2 - top), text, font=f, fill=colour + (255,))
    return img


# The "Rift" tab in SteamVR's dashboard: riftlinux-notices draws it from these pieces as its values change.
# Its geometry is in riftlinux_notices.cpp (kTab*); the widths text is wrapped to here follow it.
TAB_CARD_W = 744        # kTabCardW
TAB_PAD = 32            # kTabPad: inside a card
TAB_SWITCH_ROOM = 150   # kTabSwitchW plus the gap before it: a switch at a row's right
TAB_BUTTON_ROOM = 280   # kTabButtonW plus the gap: a button at a row's right
TAB_INNER = TAB_CARD_W - 2 * TAB_PAD
TAB_WHITE, TAB_GREY, TAB_HEADING, TAB_AMBER = (232, 235, 240), (150, 156, 168), (128, 160, 215), (230, 170, 60)

# The cards, in the order riftlinux_notices.cpp places them (two columns), and their rows: (id, kind, name,
# explanation). Kinds: "slider", "switch", "button", "note" (an explanation alone). The ids name the pieces;
# riftlinux_notices.cpp lists the same rows.
TAB_CARDS = [
    ("sound", "Sound", [
        ("volume", "slider", "Headphones", "The Rift's built-in headphones."),
        ("mic", "slider", "Microphone", "The Rift's built-in microphone, for apps in VR."),
    ]),
    ("view", "View", [
        ("floor", "button", "Floor", "Lay a controller on the floor, then press Set floor."),
        ("height", "slider", "Height", "Raise or lower your view."),
    ]),
    ("picture", "Picture", [
        ("autohdr", "switch", "Auto-HDR", "Deeper darks, brighter highlights and richer colour."),
        ("strength", "slider", "Strength", "Contrast and highlights; 50 % is the default."),
        ("colour", "slider", "Colour", "How rich colours are; 50 % is the default."),
        ("bright", "switch", "HDR Bright", "Brighter HDR, like a TV's bright mode."),
    ]),
    ("motion", "Motion", [
        ("smoothing", "switch", "Motion smoothing", "Smooth motion when a game can't keep 90 frames a second."),
    ]),
    ("button", "Oculus button", [
        ("button", "note", None, "On the right controller: press it to open this tab, hold it for a second to "
                                 "recentre the view."),
    ]),
    # The tab's second page, "Display & status".
    ("status", "Status", [
        ("st-headset", "info", "Headset", None),
        ("st-left", "info", "Left controller", None),
        ("st-right", "info", "Right controller", None),
        ("st-sensors", "info", "Sensors", None),
        ("recalibrate", "button", "Measure the sensors manually", "Moved sensors are placed again "
                                                                  "automatically; use this only if tracking still "
                                                                  "seems off. Your floor is kept."),
        ("st-display", "info", "Display", None),
    ]),
    ("display", "Display", [
        ("directmode", "switch", "Direct mode", "RiftLinux's own compositor, for Auto-HDR, HDR and motion "
                                                "smoothing. Off: SteamVR's compositor."),
        ("renderscale", "slider", "Render scale", "How many pixels games render; 100 % is the CV1's own "
                                                  "sharpness, more is sharper and slower."),
        ("fastpath", "switch", "Fast path", "Games' images go straight to the lens correction, which is "
                                            "sharper. Off only to compare."),
    ]),
    ("perf", "Performance", [
        ("perf", "switch", "Performance numbers", "How fast the game runs, measured once a second."),
        ("pf-game", "info", "Game", None),
        ("pf-avg", "info", "Average, last 10 s", None),
        ("pf-smooth", "info", "Made by motion smoothing", None),
        ("pf-gpu", "info", "GPU time a frame", None),
        ("pf-dropped", "info", "Late, last 10 s", None),
    ]),
    # Only in RiftLinux's own runtime's panel: SteamVR's dashboard has its own Quit.
    ("app", "Application", [
        ("closeapp", "button", "Close the application", "Asks the running application to quit, and ends it if it "
                                                        "does not."),
    ]),
]
# Words the status card puts together with numbers (riftlinux_notices.cpp draw_card, the "info" rows).
TAB_WORDS = {
    "worn": "Worn", "not-worn": "Not worn", "no-data": "No data yet", "of": "of", "placed": "placed",
    "fps": "fps", "ms": "ms", "off": "Off", "frames": "frames",
    "measuring": "measuring", "direct-wayland": "Direct mode, Wayland", "direct-x11": "Direct mode, X11",
    "steamvr": "SteamVR's compositor", "sep": "\u00b7",
    # The same tab in RiftLinux's own runtime (native OpenXR applications, no SteamVR).
    "runtime": "RiftLinux runtime",
}


def wrap(text, f, width):
    lines, line = [], ""
    for word in text.split():
        t = word if not line else line + " " + word
        if not line or text_width(t, f) <= width:
            line = t
        else:
            lines.append(line)
            line = word
    return lines + ([line] if line else [])


def text_block(text, f, colour, width=None):
    """Text in lines of one height, its first line's top at the font's ascender: pieces drawn at the same y line up
    on their text whatever letters they have. Wrapped to @p width when given."""
    lines = wrap(text, f, width) if width else [text]
    ascent, descent = f.getmetrics()
    line_h = round((ascent + descent) * 1.12)
    img = Image.new("RGBA", (max(text_width(t, f) for t in lines) + 8, line_h * (len(lines) - 1) + ascent + descent),
                    colour + (0,))
    d = ImageDraw.Draw(img)
    for i, t in enumerate(lines):
        d.text((2, i * line_h), t, font=f, fill=colour + (255,), anchor="la")
    return img


def spaced_caps(text, f, colour, spacing=4):
    """A card's heading: capitals, letters a little apart."""
    text = text.upper()
    ascent, descent = f.getmetrics()
    widths = [text_width(c, f) if c != " " else f.getlength(" ") for c in text]
    img = Image.new("RGBA", (int(sum(widths) + spacing * len(text)) + 8, ascent + descent), colour + (0,))
    d = ImageDraw.Draw(img)
    x = 2
    for c, w in zip(text, widths):
        d.text((x, 0), c, font=f, fill=colour + (255,), anchor="la")
        x += w + spacing
    return img


def render_welcome(path):
    """The welcome panel: title, one line under it, four rows (bold name, what to do), a footer."""
    tf, sf = font("NotoSans-Bold.ttf", 72), font("NotoSans-Regular.ttf", 40)
    nf, bf = font("NotoSans-Bold.ttf", 44), font("NotoSans-Regular.ttf", 44)
    ff = font("NotoSans-Regular.ttf", 32)
    pad, name_w, row_h = 80, 300, 84
    text_w = max(text_width(t, bf) for _, t in WELCOME_ROWS)
    w = max(pad + 24 + name_w + text_w + pad, pad + text_width(WELCOME_SUB, sf) + pad)
    h = 62 + 104 + 64 + 40 + row_h * len(WELCOME_ROWS) + 30 + 22 + 60
    img = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    d.rounded_rectangle((0, 0, w - 1, h - 1), radius=44, fill=(18, 20, 26, 238), outline=(255, 255, 255, 40), width=2)
    d.rounded_rectangle((pad - 48, 74, pad - 34, 74 + 150), radius=7, fill=GREEN + (255,))
    y = 62
    d.text((pad, y), WELCOME_TITLE, font=tf, fill=(255, 255, 255, 255))
    y += 104
    d.text((pad, y), WELCOME_SUB, font=sf, fill=(200, 206, 216, 255))
    y += 64 + 40
    for name, text in WELCOME_ROWS:
        d.text((pad + 24, y), name, font=nf, fill=(255, 255, 255, 255))
        d.text((pad + 24 + name_w, y), text, font=bf, fill=(215, 220, 230, 255))
        y += row_h
    y += 30
    d.line((pad, y, w - pad, y), fill=(255, 255, 255, 40), width=2)
    d.text((pad, y + 22), WELCOME_FOOT, font=ff, fill=(160, 166, 178, 255))
    img.save(path, optimize=True)
    print(f"welcome panel {w}x{h}")


def render_tab(out):
    """The pieces of the Rift tab (headings, each control's name and explanation, button labels, digits) and the
    tab's icon."""
    bold = lambda size: font("NotoSans-Bold.ttf", size)
    regular = lambda size: font("NotoSans-Regular.ttf", size)
    piece = lambda img, name: write_raw(img, os.path.join(out, name + ".rgba"))
    piece(text_block("Rift CV1", bold(56), TAB_WHITE), "tab-title")
    piece(text_block("Settings for the headset; changes apply at once and are kept.", regular(30), TAB_GREY),
          "tab-subtitle")
    help_width = {"slider": TAB_INNER, "note": TAB_INNER, "switch": TAB_INNER - TAB_SWITCH_ROOM,
                  "button": TAB_INNER - TAB_BUTTON_ROOM, "info": TAB_INNER}
    for card, heading, rows in TAB_CARDS:
        piece(spaced_caps(heading, bold(28), TAB_HEADING), f"tab-card-{card}")
        for row, kind, name, help_text in rows:
            if kind == "info":
                # A status line: its name in grey, the value (put together in the helper) in white.
                piece(text_block(name, regular(29), TAB_GREY), f"tab-{row}-label")
                continue
            if name:
                piece(text_block(name, bold(34), TAB_WHITE), f"tab-{row}-label")
            piece(text_block(help_text, regular(27), TAB_GREY, help_width[kind]), f"tab-{row}-help")
    for key, text in TAB_WORDS.items():
        piece(text_block(text, bold(29), TAB_WHITE), f"tab-word-{key}")
    piece(text_block("Applies when SteamVR starts again.", regular(25), TAB_AMBER), "tab-restart")
    piece(text_block("Applies when the runtime starts again.", regular(25), TAB_AMBER), "tab-restart-runtime")
    for name, text in (("settings", "Settings"), ("display", "Display & status")):
        piece(text_block(text, bold(28), TAB_WHITE), f"tab-page-{name}")
    # The status card's numbers, smaller than the sliders' values.
    f = bold(29)
    chars = "0123456789%+-."
    cell = max(text_width(c, f) for c in chars) + 4
    ascent, descent = f.getmetrics()
    atlas = Image.new("RGBA", (cell * len(chars), ascent + descent), TAB_WHITE + (0,))
    d = ImageDraw.Draw(atlas)
    for i, c in enumerate(chars):
        d.text((i * cell + (cell - text_width(c, f)) // 2, 0), c, font=f, fill=TAB_WHITE + (255,), anchor="la")
    piece(atlas, "tab-digits-small")
    piece(text_block("Needs direct mode", bold(24), TAB_AMBER), "tab-needs-direct")
    # Under HDR Bright, from the compositor: whether it has anything to act on now (Home and most games are SDR).
    # Without this note, users would flick HDR Bright on in SteamVR Home and find it did nothing.
    piece(text_block("Requires HDR content \u2013 not related to Auto-HDR.", regular(27), TAB_WHITE, TAB_INNER),
          "tab-bright-note")
    piece(text_block("Now: HDR content on screen", regular(25), (150, 210, 160)), "tab-bright-status-on")
    piece(text_block("Now: no HDR content on screen", regular(25), TAB_GREY), "tab-bright-status-off")
    for name, text in (("mute", "Mute"), ("muted", "Muted"), ("setfloor", "Set floor"), ("reset", "Reset"),
                       ("measure", "Measure"), ("close", "Close")):
        piece(text_block(text, bold(30), TAB_WHITE), f"tab-btn-{name}")
    piece(text_block("cm", bold(30), TAB_WHITE), "tab-cm")
    # Digits, "%", "+", "-" and "." in cells of one width, side by side; the helper finds each one's ink.
    f = bold(40)
    chars = "0123456789%+-."
    cell = max(text_width(c, f) for c in chars) + 6
    ascent, descent = f.getmetrics()
    atlas = Image.new("RGBA", (cell * len(chars), ascent + descent), TAB_WHITE + (0,))
    d = ImageDraw.Draw(atlas)
    for i, c in enumerate(chars):
        d.text((i * cell + (cell - text_width(c, f)) // 2, 0), c, font=f, fill=TAB_WHITE + (255,), anchor="la")
    piece(atlas, "tab-digits")
    # The tab's icon: the headset seen from the front, a strap behind it.
    s = 256
    icon = Image.new("RGBA", (s, s), (0, 0, 0, 0))
    d = ImageDraw.Draw(icon)
    d.rounded_rectangle((8, 8, s - 9, s - 9), radius=44, fill=(8, 9, 12, 255))
    d.rounded_rectangle((28, 110, s - 29, 128), radius=9, fill=TAB_GREY + (255,))
    d.rounded_rectangle((52, 78, s - 53, 178), radius=34, fill=TAB_WHITE + (255,))
    for cx in (100, s - 101):
        d.ellipse((cx - 13, 114, cx + 13, 140), fill=(8, 9, 12, 255))
    icon.save(os.path.join(out, "quick-tab.png"), optimize=True)
    print(f"dashboard tab pieces in {os.path.normpath(out)}")


def render_hdr_chart(path):
    """The HDR test chart: HDR10 video as it is stored (16-bit PQ, BT.2020), labelled, with a line per row on
    what to look for. The SteamVR driver shows it 1.3 m ahead with display.hdr_test, described as MaxCLL 4000 nits
    ("static"), no metadata ("none") or a scene peak of 1000 nits ("scene"). File: "RLHD", width, height (32-bit, little
    end), then RGBA as 16-bit PQ, rows top down."""
    import numpy as np
    w, h = 1600, 1240
    white = 203.0  # nits of reference white (ITU-R BT.2408): 1.0 below
    lin = np.zeros((h, w, 3))  # linear light, BT.709 primaries, reference white 1.0
    is2020 = np.zeros((h, w), bool)  # pixels given in BT.2020 already
    text = Image.new("L", (w, h), 0)
    d = ImageDraw.Draw(text)
    head, label = font("NotoSans-Bold.ttf", 40), font("NotoSans-Regular.ttf", 32)

    def s2l(v):
        return v / 12.92 if v <= 0.04045 else ((v + 0.055) / 1.055) ** 2.4

    def centred(x0, x1, y, t, f=label):
        d.text(((x0 + x1 - text_width(t, f)) // 2, y), t, font=f, fill=255)

    d.text((30, 18), "HDR test chart: HDR10 video (PQ, BT.2020), up to 4000 nits", font=head, fill=255)
    # 1. Brightness.
    d.text((30, 92), "1  Brightness: each brighter than the last; 400, 1000, 4000 must not merge", font=head, fill=255)
    greys = [(0.5, "0.5"), (5, "5"), (50, "50"), (100, "100"), (203, "203 white"), (400, "400"), (1000, "1000"),
             (4000, "4000 nits")]
    for i, (n, t) in enumerate(greys):
        x0 = 30 + i * 194
        lin[150:290, x0:x0 + 176] = n / white
        centred(x0, x0 + 176, 296, t)
    # 2. Colours at reference white.
    d.text((30, 360), "2  Colours: natural; deep red and green richer than plain red and green", font=head, fill=255)
    colours = [((1, 0, 0), False, "Red"), ((0, 1, 0), False, "Green"), ((0, 0, 1), False, "Blue"),
               ((1, 1, 0), False, "Yellow"), ((1, 0, 0), True, "Deep red"), ((0, 1, 0), True, "Deep green"),
               ((s2l(224 / 255), s2l(172 / 255), s2l(105 / 255)), False, "Skin"),
               ((s2l(135 / 255), s2l(206 / 255), s2l(235 / 255)), False, "Sky")]
    for i, (c, wide, t) in enumerate(colours):
        x0 = 30 + i * 194
        lin[418:558, x0:x0 + 176] = c
        is2020[418:558, x0:x0 + 176] = wide
        centred(x0, x0 + 176, 564, t)
    # 3. Highlights: two to three times reference white, saturated.
    d.text((30, 628), "3  Highlights, 2-3x brighter than white: glow but keep their colour", font=head, fill=255)
    highlights = [((3, 0.79, 0), "Orange 3x"), ((3, 3, 0), "Yellow 3x"), ((2, 0, 0), "Red 2x"), ((0, 2, 2), "Cyan 2x")]
    for i, (c, t) in enumerate(highlights):
        x0 = 30 + i * 388
        lin[686:806, x0:x0 + 370] = c
        centred(x0, x0 + 370, 812, t)
    # 4. Neon: BT.2020 (beyond SDR's colours), 2.5-4 times reference white.
    d.text((30, 868), "4  Neon, HDR colours 2-4x brighter than white: vivid, glowing, none pale", font=head, fill=255)
    neon = [((1, 0, 0.02), 2.5, "Red"), ((1, 0.3, 0), 3, "Orange"), ((1, 1, 0), 2.5, "Yellow"),
            ((0, 1, 0.1), 2.5, "Green"), ((0, 1, 1), 2.5, "Cyan"), ((0.02, 0.05, 1), 4, "Blue"),
            ((0.6, 0, 1), 3, "Purple"), ((1, 0.05, 0.6), 2.5, "Pink")]
    for i, (c, k, t) in enumerate(neon):
        x0 = 30 + i * 194
        lin[926:1046, x0:x0 + 176] = np.array(c, float) * k
        is2020[926:1046, x0:x0 + 176] = True
        centred(x0, x0 + 176, 1052, t)
    # 5. Ramp: PQ signal 0 to 1 (0 to 10 000 nits), left to right.
    d.text((30, 1110), "5  Black to 10 000 nits: smooth, no steps or bands", font=head, fill=255)
    ramp_pq = np.linspace(0.0, 1.0, w - 60)

    # Labels in light grey, 0.7 x reference white, over black.
    t = np.asarray(text, float) / 255.0
    lin = lin * (1 - t[..., None]) + 0.7 * t[..., None]
    m709_2020 = np.array([[0.6274, 0.3293, 0.0433], [0.0691, 0.9195, 0.0114], [0.0164, 0.0880, 0.8956]])
    out = np.where(is2020[..., None], lin, lin @ m709_2020.T)
    m1, m2, c1, c2, c3 = 2610 / 16384, 2523 / 4096 * 128, 3424 / 4096, 2413 / 4096 * 32, 2392 / 4096 * 32
    y = np.power(np.clip(out * white, 0, None) / 10000.0, m1)
    pq = np.power((c1 + c2 * y) / (1 + c3 * y), m2)
    pq[1170:1225, 30:w - 30] = ramp_pq[None, :, None]
    rgba = np.ones((h, w, 4))
    rgba[..., :3] = pq
    with open(path, "wb") as f:
        f.write(b"RLHD" + w.to_bytes(4, "little") + h.to_bytes(4, "little"))
        f.write(np.round(rgba * 65535).astype("<u2").tobytes())
    print(f"HDR test chart {path}")


def render_dark_test(path):
    """Black level and shadow steps: pure black with patches 1-16 of 255 and a white reference,
    then a 0-64 ramp. In the headset the black must be as dark as the panel switched off, and the steps tell how
    finely the shadows grade."""
    w, h = 2000, 1100
    img = Image.new("RGB", (w, h), (0, 0, 0))
    d = ImageDraw.Draw(img)
    label = font("NotoSans-Regular.ttf", 34)
    levels = [1, 2, 3, 4, 6, 8, 12, 16]
    size, gap, top = 170, 60, 140
    left = (w - len(levels) * size - (len(levels) - 1) * gap) // 2
    d.text((left, 40), "Black level: the background must be fully dark. Which squares can you see?",
           font=label, fill=(160, 160, 160))
    for i, v in enumerate(levels):
        x = left + i * (size + gap)
        d.rectangle((x, top, x + size, top + size), fill=(v, v, v))
        d.text((x + size // 2 - 20, top + size + 12), str(v), font=label, fill=(120, 120, 120))
    d.rectangle((w - 140, 40, w - 60, 120), fill=(255, 255, 255))
    ramp_top = 560
    d.text((left, ramp_top - 60), "0 to 64: does it step or flow smoothly?", font=label, fill=(160, 160, 160))
    for x in range(left, w - left):
        v = round(64 * (x - left) / (w - 2 * left - 1))
        d.line((x, ramp_top, x, ramp_top + 300), fill=(v, v, v))
    img.save(path, optimize=True)
    print(f"test pattern {path}")


if __name__ == "__main__":
    main()
