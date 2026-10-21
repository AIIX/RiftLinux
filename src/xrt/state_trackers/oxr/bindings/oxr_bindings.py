#!/usr/bin/env python3
# Copyright 2020-2025, Collabora, Ltd.
# Copyright 2024-2026, NVIDIA CORPORATION.
# SPDX-License-Identifier: BSL-1.0
"""Generate OpenXR binding code using the auxiliary bindings scripts."""

from bindings import *

import argparse
import os
from operator import attrgetter
from string import Template

def get_verify_switch_body(dict_of_lists, profile, profile_name, tab_char):
    """Generate function to check if a string is in a set of strings.
    Input is a dict where keys are length and
    the values are lists of strings of that length. And a suffix if any.
    Returns a list of lines."""
    ret = [ f"{tab_char}\tswitch (length) {{" ]
    for length in sorted(dict_of_lists.keys()):
        ret += [
            f'{tab_char}\tcase {str(length)}:',
        ]

        for i, path in enumerate(sorted(dict_of_lists[length])):
            if_or_else_if = 'if' if i == 0 else '} else if'
            ret += [
                f'{tab_char}\t\t{if_or_else_if} (strcmp(str, "{path}") == 0) {{',
                f'{tab_char}\t\t\treturn true;',
             ]
        ret += [
            f'{tab_char}\t\t}}',
            f'{tab_char}\t\tbreak;',
        ]
    ret += [
        f'{tab_char}\tdefault: break;',
        f'{tab_char}\t}}',
    ]
    return ret

def get_verify_func_switch(dict_of_lists, profile, profile_name, availability):
    """Generate function to check if a string is in a set of strings.
    Input is a dict where keys are length and
    the values are lists of strings of that length. And a suffix if any.
    Returns a list of lines."""
    if len(dict_of_lists) == 0:
        return ''

    ret = [
        f"\t// generated from: {profile_name}"
    ]

    # Example: pico neo 3 can be enabled by either enabling XR_BD_controller_interaction ext or using OpenXR 1.1+.
    # Disabling OXR_HAVE_BD_controller_interaction should NOT remove pico neo from OpenXR 1.1+ (it makes "exts->BD_controller_interaction" invalid C code).
    # Therefore separate code blocks for ext and version checks generated to avoid ifdef hell.
    feature_sets = sorted(availability.feature_sets, key=FeatureSet.as_tuple)
    for feature_set in feature_sets:
        requires_version = is_valid_version(feature_set.required_version)
        requires_extensions = bool(feature_set.required_extensions)

        tab_char = ''
        closing = []

        if requires_version:
            tab_char += '\t'
            ret += [
                f'{tab_char}if (openxr_version >= XR_MAKE_VERSION({feature_set.required_version["major"]}, {feature_set.required_version["minor"]}, 0)) {{',
            ]
            closing.append(f'{tab_char}}}\n')

        if requires_extensions:
            tab_char += '\t'
            exts = sorted(feature_set.required_extensions)
            ext_defines = ' && '.join(f'defined(OXR_HAVE_{ext})' for ext in exts)
            ret += [
                f'#if {ext_defines}',
                f'{tab_char}if ('+' && '.join(f'exts->{ext}' for ext in exts)+') {',
            ]
            closing.append(f'{tab_char}}}\n#endif // {ext_defines}\n\n')

        ret += get_verify_switch_body(dict_of_lists, profile, profile_name, tab_char)

        ret += reversed(closing)

    return ret

def get_verify_func_body(profile, dict_name, availability):
    """returns a list of lines"""
    ret = []
    if profile is None or dict_name is None or len(dict_name) == 0:
        return ret
    ret += get_verify_func_switch(getattr(
        profile, dict_name), profile, profile.name, availability)
    if profile.parent_profiles is None:
        return ret
    for pp in sorted(profile.parent_profiles, key=attrgetter("name")):
        ret += get_verify_func_body(pp, dict_name, availability.intersection(pp.availability()))

    return ret


def get_verify_func(profile, dict_name, suffix):
    """returns a list of lines"""
    name = f"oxr_verify_{profile.validation_func_name}{suffix}"

    ret = [
        'bool',
        '{name}(const struct oxr_extension_status *exts, XrVersion openxr_version, const char *str, size_t length)'.format(name=name),
        '{',

        *get_verify_func_body(profile, dict_name, profile.availability()),

        '\treturn false;',
        '}',
        ''
    ]

    return ret

def get_verify_functions(profile):
    """returns a list of lines"""
    ret = [
        *get_verify_func(profile, "subpaths_by_length", "_subpath"),
        *get_verify_func(profile, "dpad_paths_by_length", "_dpad_path"),
        *get_verify_func(profile, "dpad_emulators_by_length", "_dpad_emulator"),

        'void',
        f'oxr_verify_{profile.validation_func_name}_ext(const struct oxr_extension_status *extensions, XrVersion openxr_version, bool *out_supported, bool *out_enabled)',
        '{',
        '',
    ]

    is_promoted = is_valid_version(profile.openxr_version_promoted)
    if is_promoted:
        ret += [
            f'\tif (openxr_version >= XR_MAKE_VERSION({profile.openxr_version_promoted["major"]}, {profile.openxr_version_promoted["minor"]}, 0)) {{',
            '\t\t*out_supported = true;',
            '\t\t*out_enabled = true;',
            '\t\treturn;',
            '\t}',
            '',
        ]


    if profile.extension_name is not None:
        ret += [
            f'#ifdef OXR_HAVE_{profile.extension_name}',
            '\t*out_supported = true;',
            f'\t*out_enabled = extensions->{profile.extension_name};',
            '#else',
            '\t*out_supported = false;',
            '\t*out_enabled = false;',
            f'#endif // OXR_HAVE_{profile.extension_name}',
            '}',
            '',
        ]

    else:
        ret += [
            '\t*out_supported = true;',
            '\t*out_enabled = true;',
            '}',
            '',
        ]

    return ret


def generate_profile_template_entry(profile):
    """Generate a single profile_template entry. Returns a string."""
    lines = []
    hw_name = str(profile.name.split("/")[-1])
    vendor_name = str(profile.name.split("/")[-2])
    fname = vendor_name + "_" + hw_name + "_profile.json"
    controller_type = "monado_" + vendor_name + "_" + hw_name

    binding_count = len(profile.components)
    lines.extend([
        '\t{ // oxr_profile_template',
        f'\t\t.name = {profile.monado_device_enum},',
        f'\t\t.path = "{profile.name}",',
        f'\t\t.localized_name = "{profile.localized_name}",',
        f'\t\t.steamvr_input_profile_path = "{fname}",',
        f'\t\t.steamvr_controller_type = "{controller_type}",',
        f'\t\t.binding_count = {binding_count},',
        '\t\t.bindings = (struct binding_template[]){ // array of binding_template',
    ])

    component: Component
    for idx, component in enumerate(profile.components):
        # @todo Doesn't handle pose yet.
        steamvr_path = component.steamvr_path
        if component.component_name in ["click", "touch", "force", "value", "proximity"]:
            steamvr_path += "/" + component.component_name

        lines.extend([
            f'\t\t\t{{ // binding_template {idx}',
            f'\t\t\t\t.subaction_path = "{component.subaction_path}",',
            f'\t\t\t\t.steamvr_path = "{steamvr_path}",',
            f'\t\t\t\t.localized_name = "{component.subpath_localized_name}",',
            '',
            '\t\t\t\t.paths = { // array of paths',
        ])

        lines.extend([f'\t\t\t\t\t"{path}",' for path in component.get_full_openxr_paths()])

        lines.extend([
            '\t\t\t\t\tNULL,',
            '\t\t\t\t}, // /array of paths',
        ])

        # controllers can have input that we don't have bindings for
        if component.monado_binding:
            monado_binding = component.monado_binding

            # Input, dpad_activate and output default to 0.
            # If a binding specifies an actual binding value for them in json, those get overridden.
            input_value = '0'
            dpad_activate_value = '0'
            output_value = '0'

            if component.is_input() and monado_binding is not None:
                input_value = monado_binding

            if component.has_dpad_emulation() and "activate" in component.dpad_emulation:
                activate_component = find_component_in_list_by_name(
                    component.dpad_emulation["activate"], profile.components,
                    subaction_path=component.subaction_path,
                    identifier_json_path=component.identifier_json_path)
                dpad_activate_value = activate_component.monado_binding

            if component.is_output() and monado_binding is not None:
                output_value = monado_binding

            lines.extend([
                f'\t\t\t\t.input = {input_value},',
                f'\t\t\t\t.dpad_activate = {dpad_activate_value},',
                f'\t\t\t\t.output = {output_value},',
            ])

        lines.append(f'\t\t\t}}, // /binding_template {idx}')

    lines.append('\t\t}, // /array of binding_template')

    # Handle dpads
    dpads = []
    for identifier in profile.identifiers:
        if identifier.dpad:
            dpads.append(identifier)

    dpad_count = len(dpads)
    lines.append(f'\t\t.dpad_count = {dpad_count},')
    if len(dpads) == 0:
        lines.append('\t\t.dpads = NULL,')
    else:
        lines.append('\t\t.dpads = (struct dpad_emulation[]){ // array of dpad_emulation')
        for identifier in dpads:
            lines.append('\t\t\t{')
            lines.append(f'\t\t\t\t.subaction_path = "{identifier.subaction_path}",')
            lines.append('\t\t\t\t.paths = {')
            for path in identifier.dpad.paths:
                lines.append(f'\t\t\t\t\t"{path}",')
            lines.append('\t\t\t\t},')
            lines.append(f'\t\t\t\t.position = {identifier.dpad.position_component.monado_binding},')
            if identifier.dpad.activate_component:
                lines.append(f'\t\t\t\t.activate = {identifier.dpad.activate_component.monado_binding},')
            else:
                lines.append('\t\t\t\t.activate = 0')
            lines.append('\t\t\t},')
        lines.append('\t\t}, // /array of dpad_emulation')

    lines.append(f'\t\t.openxr_version.promoted.major = {profile.openxr_version_promoted["major"]},')
    lines.append(f'\t\t.openxr_version.promoted.minor = {profile.openxr_version_promoted["minor"]},')

    fn_prefixes = ["subpath", "dpad_path", "dpad_emulator"]
    for prefix in fn_prefixes:
        lines.append(f'\t\t.{prefix}_fn = oxr_verify_{profile.validation_func_name}_{prefix},')
    lines.append(f'\t\t.ext_verify_fn = oxr_verify_{profile.validation_func_name}_ext,')

    if profile.extension_name is None:
        lines.append('\t\t.extension_name = NULL,')
    else:
        lines.append(f'\t\t.extension_name = "{profile.extension_name}",')

    lines.append('\t}, // /profile_template')

    return '\n'.join(lines)


def generate_bindings_c(output, b):
    """Generate the file to verify subpaths on a interaction profile."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    template_file = os.path.join(script_dir, 'oxr_generated_bindings.c.template')

    with open(template_file, 'r') as f:
        src = Template(f.read())

    # Generate verify functions
    lines = []
    for profile in b.profiles:
        lines.extend(get_verify_functions(profile))
    verify_functions = '\n'.join(lines)

    # Generate profile templates
    profile_templates = '\n'.join(generate_profile_template_entry(profile) for profile in b.profiles)

    with open(output, "w", encoding="utf-8") as fp:
        fp.write(src.substitute(
            template_count=len(b.profiles),
            verify_functions=verify_functions,
            profile_templates=profile_templates
        ))


def generate_bindings_h(output, b):
    """Generate header for the verify subpaths functions."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    template_file = os.path.join(script_dir, 'oxr_generated_bindings.h.template')

    with open(template_file, 'r') as f:
        src = Template(f.read())

    # Generate function prototypes
    verify_protos = []
    fn_prefixes = ["_subpath", "_dpad_path", "_dpad_emulator"]
    for profile in b.profiles:
        for fn_suffix in fn_prefixes:
            verify_protos.extend([
                'bool',
                f'oxr_verify_{profile.validation_func_name}{fn_suffix}(const struct oxr_extension_status *extensions, XrVersion openxr_major_minor, const char *str, size_t length);',
                '',
            ])
        verify_protos.extend([
            'void',
            f'oxr_verify_{profile.validation_func_name}_ext(const struct oxr_extension_status *extensions, XrVersion openxr_version, bool *out_supported, bool *out_enabled);',
            '',
        ])

    with open(output, "w", encoding="utf-8") as fp:
        fp.write(src.substitute(
            template_count=len(b.profiles),
            verify_protos='\n'.join(verify_protos)
        ))


def main():
    """Handle command line and generate file(s)."""
    parser = argparse.ArgumentParser(description='OpenXR Bindings generator.')
    parser.add_argument(
        'bindings', help='Bindings file to use')
    parser.add_argument(
        'output', type=str, nargs='+',
        help='Output file, uses the name to choose output type')
    args = parser.parse_args()

    bindings = Bindings.load_and_parse(args.bindings)

    for output in args.output:
        if output.endswith("oxr_generated_bindings.c"):
            generate_bindings_c(output, bindings)
        elif output.endswith("oxr_generated_bindings.h"):
            generate_bindings_h(output, bindings)
        else:
            raise ValueError(f"Unknown output file: {output}")


if __name__ == "__main__":
    main()
