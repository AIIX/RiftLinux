// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
// Author: Aditya Mehra <aix.m@outlook.com>
//
// Auto-HDR: OKLab, the lightness curve, colour and gamut. Used per layer by the layer squasher
// (layer.comp) and by the distortion pass (distortion.comp) on its fast path and for the panel's black level.

// OKLab (Björn Ottosson): a perceptual colour space, lightness and hue apart, from and to linear sRGB.
float cbrt(float x)
{
	return sign(x) * pow(abs(x), 1.0 / 3.0);
}

vec3 linear_srgb_to_oklab(vec3 c)
{
	float l = cbrt(0.4122214708 * c.r + 0.5363325363 * c.g + 0.0514459929 * c.b);
	float m = cbrt(0.2119034982 * c.r + 0.6806995451 * c.g + 0.1073969566 * c.b);
	float s = cbrt(0.0883024619 * c.r + 0.2817188376 * c.g + 0.6299787005 * c.b);
	return vec3(0.2104542553 * l + 0.7936177850 * m - 0.0040720468 * s,
	            1.9779984951 * l - 2.4285922050 * m + 0.4505937099 * s,
	            0.0259040371 * l + 0.7827717662 * m - 0.8086757660 * s);
}

vec3 oklab_to_linear_srgb(vec3 c)
{
	float l = c.x + 0.3963377774 * c.y + 0.2158037573 * c.z;
	float m = c.x - 0.1055613458 * c.y - 0.0638541728 * c.z;
	float s = c.x - 0.0894841775 * c.y - 1.2914855480 * c.z;
	l = l * l * l;
	m = m * m * m;
	s = s * s * s;
	return vec3(4.0767416621 * l - 3.3077115913 * m + 0.2309699292 * s,
	            -1.2684380046 * l + 2.6097574011 * m - 0.3413193965 * s,
	            -0.0041960863 * l - 0.7034186147 * m + 1.7076147010 * s);
}

bool in_gamut(vec3 c)
{
	return all(greaterThanEqual(c, vec3(-1e-4))) && all(lessThanEqual(c, vec3(1.0001)));
}

/*
 * Auto-HDR, inverse tone mapping: the HDR look on an SDR OLED, which has true blacks but a fixed
 * peak, over the whole picture. In OKLab, so lightness changes keep hues and colour changes keep lightness.
 * Lightness, one smooth curve (no kinks, so no new banding; never reverses):
 * - near black (below 0.2, about sRGB 13 of 255): lifted by @p black_lift so the levels the panel hides become
 *   visible, black itself stays black (black_lift 0 = as is; set from the test pattern);
 * - shadows to the knee: deepened, most at dark mid-tones (p.y), toward the OLED's black;
 * - mid-grey (the knee p.w) stays where it is;
 * - highlights above the knee: lifted, most toward the top, so lights, sky and reflections shine (p.x); white stays
 *   white and keeps a slope of 1 - p.x, so detail just below white is kept.
 * Colour: chroma raised by p.z, most for moderately coloured areas, little for near-greys (no tints), less for colours
 * already vivid (no clipping), much less for skin hues (over-saturated skin breaks realism in VR).
 * Gamut: what the panel cannot show gives up the lightness change first, then the extra chroma (a clip shifts hues).
 * Content past white (float swapchains) is left to the tone map.
 */
float auto_hdr_lightness(float L, vec4 p, float black_lift)
{
	const float floor_l = 0.2;
	if (L <= 0.0 || L >= 1.0) {
		return L;
	}
	if (L < floor_l) {
		float x = 1.0 - L / floor_l;
		return L + black_lift * x * x * L / (L + 0.01);
	}
	float knee = p.w;
	if (L < knee) {
		float v = (L - floor_l) / (knee - floor_l);
		return L - p.y * (knee - floor_l) * 4.0 * v * v * (1.0 - v) * (1.0 - v);
	}
	float u = (L - knee) / (1.0 - knee);
	return L + p.x * (1.0 - knee) * u * u * u * (1.0 - u);
}

/*
 * The most saturated colour of a hue on the panel (sRGB/BT.709), as OKLab lightness and chroma: Björn Ottosson's
 * compute_max_saturation and find_cusp ("sRGB gamut clipping", 2021; MIT License, LICENSES/MIT.txt). @p a, @p b: the
 * hue as a unit vector in OKLab.
 */
float oklab_max_saturation(float a, float b)
{
	float k0, k1, k2, k3, k4, wl, wm, ws;
	if (-1.88170328 * a - 0.80936493 * b > 1.0) {
		// Red runs out first.
		k0 = 1.19086277; k1 = 1.76576728; k2 = 0.59662641; k3 = 0.75515197; k4 = 0.56771245;
		wl = 4.0767416621; wm = -3.3077115913; ws = 0.2309699292;
	} else if (1.81444104 * a - 1.19445276 * b > 1.0) {
		// Green.
		k0 = 0.73956515; k1 = -0.45954404; k2 = 0.08285427; k3 = 0.12541070; k4 = 0.14503204;
		wl = -1.2684380046; wm = 2.6097574011; ws = -0.3413193965;
	} else {
		// Blue.
		k0 = 1.35733652; k1 = -0.00915799; k2 = -1.15130210; k3 = -0.50559606; k4 = 0.00692167;
		wl = -0.0041960863; wm = -0.7034186147; ws = 1.7076147010;
	}
	float S = k0 + k1 * a + k2 * b + k3 * a * a + k4 * a * b;
	float k_l = 0.3963377774 * a + 0.2158037573 * b;
	float k_m = -0.1055613458 * a - 0.0638541728 * b;
	float k_s = -0.0894841775 * a - 1.2914855480 * b;
	// One Halley step.
	float l_ = 1.0 + S * k_l, m_ = 1.0 + S * k_m, s_ = 1.0 + S * k_s;
	float l = l_ * l_ * l_, m = m_ * m_ * m_, s = s_ * s_ * s_;
	float l_dS = 3.0 * k_l * l_ * l_, m_dS = 3.0 * k_m * m_ * m_, s_dS = 3.0 * k_s * s_ * s_;
	float l_dS2 = 6.0 * k_l * k_l * l_, m_dS2 = 6.0 * k_m * k_m * m_, s_dS2 = 6.0 * k_s * k_s * s_;
	float f = wl * l + wm * m + ws * s;
	float f1 = wl * l_dS + wm * m_dS + ws * s_dS;
	float f2 = wl * l_dS2 + wm * m_dS2 + ws * s_dS2;
	return S - f * f1 / (f1 * f1 - 0.5 * f * f2);
}

vec2 find_cusp(float a, float b)
{
	float S_cusp = oklab_max_saturation(a, b);
	vec3 rgb_at_max = oklab_to_linear_srgb(vec3(1.0, S_cusp * a, S_cusp * b));
	float L_cusp = cbrt(1.0 / max(max(max(rgb_at_max.r, rgb_at_max.g), rgb_at_max.b), 1e-6));
	return vec2(L_cusp, L_cusp * S_cusp);
}

/*
 * Colour and gamut, shared by SDR and HDR: chroma raised by p.z (greys, vivid colours and skin spared), then what the
 * panel cannot show fitted. @p lab is the source in OKLab, @p L its new lightness. SDR: the colour first gives up its
 * lightness change and keeps its chroma (bright saturated colours would otherwise go pale), else at its own lightness
 * its extra chroma; the original fits, so nothing comes out less colourful than its source. HDR (@p keep_lightness):
 * the lightness is the tone mapping; out-of-range colours (BT.2020, bright highlights) are brought in toward the
 * lightness where their hue is most vivid, keeping as much colour as the panel can show.
 */
vec3 auto_hdr_colour_and_gamut(vec3 lab, float L, vec4 p, bool keep_lightness, bool source_fits)
{
	float C = length(lab.yz);
	float hue = degrees(atan(lab.z, lab.y));
	float coloured = smoothstep(0.01, 0.06, C) * (1.0 - 0.5 * smoothstep(0.15, 0.30, C));
	float skin = smoothstep(15.0, 35.0, hue) * (1.0 - smoothstep(75.0, 95.0, hue));
	vec2 ab = lab.yz * (1.0 + p.z * coloured * (1.0 - 0.75 * skin));

	vec3 out_rgb = oklab_to_linear_srgb(vec3(L, ab));
	if (in_gamut(out_rgb) || L > 1.0) {
		return max(out_rgb, vec3(0.0));
	}
	if (keep_lightness && source_fits) {
		/*
		 * HDR, a colour the panel shows (its intensity already scaled to the panel): only the extra chroma is
		 * given up, as far as needed, so it never comes out paler than it is and stays stable at the gamut's
		 * corners (pure blue flickered on a CV1 when it was refitted along its hue from scratch).
		 */
		float lo = 0.0, hi = 1.0;
		for (int i = 0; i < 8; i++) {
			float mid = 0.5 * (lo + hi);
			if (in_gamut(oklab_to_linear_srgb(vec3(L, mix(lab.yz, ab, mid))))) {
				lo = mid;
			} else {
				hi = mid;
			}
		}
		return max(oklab_to_linear_srgb(vec3(L, mix(lab.yz, ab, lo))), vec3(0.0));
	}
	if (keep_lightness) {
		/*
		 * HDR: bright colourful highlights moved part of the way to the lightness where their hue is most vivid on
		 * the panel (its cusp), further the more colourful they are, then given the most colour the panel has
		 * there: yellow and green stay bright, red, blue and purple come down a little and stay saturated (seen
		 * on the HDR chart: highlights went pale, yellow olive with a hue-blind method). Colours at or below
		 * their cusp keep their lightness.
		 */
		float Cn = length(ab);
		vec2 hab = ab / max(Cn, 1e-6);
		vec2 cusp = find_cusp(hab.x, hab.y);
		float s = 0.9 * clamp(Cn / max(cusp.y, 1e-4), 0.0, 1.0);
		float L2 = L > cusp.x ? mix(L, cusp.x, s) : L;
		float lo = 0.0, hi = Cn;
		for (int i = 0; i < 10; i++) {
			float mid = 0.5 * (lo + hi);
			if (in_gamut(oklab_to_linear_srgb(vec3(L2, hab * mid)))) {
				lo = mid;
			} else {
				hi = mid;
			}
		}
		return max(oklab_to_linear_srgb(vec3(L2, hab * lo)), vec3(0.0));
	}
	float L0 = lab.x;
	bool chroma_fits = in_gamut(oklab_to_linear_srgb(vec3(L0, ab)));
	float lo = 0.0, hi = 1.0;
	for (int i = 0; i < 6; i++) {
		float mid = 0.5 * (lo + hi);
		vec3 t = chroma_fits ? vec3(mix(L0, L, mid), ab) : vec3(L0, mix(lab.yz, ab, mid));
		if (in_gamut(oklab_to_linear_srgb(t))) {
			lo = mid;
		} else {
			hi = mid;
		}
	}
	out_rgb = oklab_to_linear_srgb(chroma_fits ? vec3(mix(L0, L, lo), ab) : vec3(L0, mix(lab.yz, ab, lo)));
	return max(out_rgb, vec3(0.0));
}

//! Auto-HDR for SDR content (and the panel's black level alone, p zero): inverse tone mapping.
vec3 auto_hdr(vec3 rgb, vec4 p, float black_lift)
{
	if (p.x == 0.0 && p.y == 0.0 && p.z == 0.0 && black_lift == 0.0) {
		return rgb;
	}
	vec3 lab = linear_srgb_to_oklab(max(rgb, vec3(0.0)));
	return auto_hdr_colour_and_gamut(lab, auto_hdr_lightness(lab.x, p, black_lift), p, false, false);
}

/*
 * HDR content: OKLab lightness @p x where 1.0 is the reference white (a grey's lightness is the cube
 * root of its luminance). Below white the reference white is put at the panel's lightness @p range.x, with the same
 * deeper mid-shadows as SDR (p.y); above white the highlights up to the scene's peak (@p range.y) go into the rest of
 * the panel's range along a rational (Reinhard-like) shoulder that starts with the slope at white (@p range.z, in
 * shoulder units) and keeps a slope to the peak, so the brightest highlights stay apart; a slope of 1 or less: the
 * peak fits as it is, nothing compressed (a dim scene keeps every highlight).
 */
float auto_hdr_hdr_lightness(float x, vec4 p, vec4 range)
{
	const float floor_l = 0.2;
	float white = range.x;
	if (x <= 0.0) {
		return x;
	}
	if (x <= 1.0) {
		float knee = p.w;
		float y = x;
		if (x > floor_l && x < knee) {
			float v = (x - floor_l) / (knee - floor_l);
			y = x - p.y * (knee - floor_l) * 4.0 * v * v * (1.0 - v) * (1.0 - v);
		}
		return white * y;
	}
	if (range.z <= 1.0) {
		return min(white * x, 1.0);
	}
	float u = clamp((x - 1.0) / max(range.y - 1.0, 1e-4), 0.0, 1.0);
	return white + (1.0 - white) * range.z * u / (1.0 + (range.z - 1.0) * u);
}

/*
 * Auto-HDR for HDR content: linear BT.709 (out-of-gamut colours negative), reference white 1.0. The colour's intensity
 * is scaled to its mapped lightness (all channels alike: its chromaticity and saturation kept), so a colour the panel
 * can show stays showable; then Auto-HDR's colour and, for what is still out of range (highlights, BT.2020 colours),
 * the hue-aware fit.
 */
vec3 auto_hdr_hdr(vec3 rgb, vec4 p, vec4 range)
{
	vec3 lab_in = linear_srgb_to_oklab(rgb);
	if (lab_in.x <= 0.0) {
		return vec3(0.0);
	}
	float r = auto_hdr_hdr_lightness(lab_in.x, p, range) / lab_in.x;
	vec3 scaled = rgb * (r * r * r);
	vec3 lab = linear_srgb_to_oklab(scaled);
	return auto_hdr_colour_and_gamut(lab, lab.x, p, true, in_gamut(scaled));
}

/*
 * A layer's colour, decoded to linear BT.709 (decode_hdr), through Auto-HDR, the same in the layer
 * squasher and on the fast path. @p source as decode_hdr's, w how much of the HDR mapping to use: PQ video 1; a float
 * layer (SteamVR game, linear, white at 1.0) by its measured peak, 0 (as SDR, the rest past white clipped) while
 * nothing much goes past white, 1 from a stop above it, so an SDR game drawing into a float image looks as it does in
 * an sRGB one. SDR layers (source.x 0) as SDR.
 */
vec3 auto_hdr_layer(vec3 c, vec4 p, vec4 source, vec4 range)
{
	if (source.x < 0.5) {
		return auto_hdr(c, p, 0.0);
	}
	if (source.w <= 0.0) {
		return auto_hdr(clamp(c, vec3(0.0), vec3(1.0)), p, 0.0);
	}
	vec3 h = auto_hdr_hdr(c, p, range);
	if (source.w < 1.0) {
		h = mix(auto_hdr(clamp(c, vec3(0.0), vec3(1.0)), p, 0.0), h, source.w);
	}
	return h;
}

//! SMPTE ST 2084 (PQ) to nits.
vec3 pq_to_nits(vec3 e)
{
	const float m1 = 2610.0 / 16384.0;
	const float m2 = 2523.0 / 4096.0 * 128.0;
	const float c1 = 3424.0 / 4096.0;
	const float c2 = 2413.0 / 4096.0 * 32.0;
	const float c3 = 2392.0 / 4096.0 * 32.0;
	vec3 p = pow(clamp(e, 0.0, 1.0), vec3(1.0 / m2));
	return 10000.0 * pow(max(p - c1, vec3(0.0)) / (c2 - c3 * p), vec3(1.0 / m1));
}

//! Linear BT.2020 to linear BT.709 (columns); colours outside BT.709 come out negative.
const mat3 k_bt2020_to_bt709 = mat3(1.6605, -0.1246, -0.0182, -0.5876, 1.1329, -0.1006, -0.0728, -0.0083, 1.1187);

/*!
 * An HDR layer's values to linear BT.709, reference white 1.0: @p source x the transfer (1 linear, 2 PQ), y the
 * primaries (1 BT.2020), z the reference white's nits (PQ).
 */
vec3 decode_hdr(vec3 c, vec4 source)
{
	if (source.x > 1.5) {
		c = pq_to_nits(c) / source.z;
	}
	if (source.y > 0.5) {
		/*
		 * Wide-gamut colours onto the panel's primaries by clipping what it cannot produce (negative channels),
		 * mpv's "clip": as saturated as the panel goes, continuous (no jumps), colours it can show untouched. Fitting
		 * by OKLab hue instead turned BT.2020's deep blue azure and its green mint (seen on the HDR chart).
		 */
		c = max(k_bt2020_to_bt709 * c, vec3(0.0));
	}
	return c;
}
