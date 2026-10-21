#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
# SPDX-License-Identifier: BSL-1.0
"""Turns the user guide (docs/) into the source of the project's website (GitHub Pages, built by Jekyll).

The pages stay plain Markdown that reads well on GitHub; the site needs a little more, made here in a copy:
- each page's title (its first heading, then left out of the text: the theme prints the title) and its description
  for search results (a "<!-- description: ... -->" line, invisible on GitHub; else its first paragraph), as Jekyll
  front matter;
- links to other pages of the guide pointing at the site's pages (faq.md -> faq.html, README.md -> the index);
- links out of docs/ (the README, CONTRIBUTING.md, ...) pointing at the repository on GitHub;
- the site's settings (_config.yml): its address, the theme, and the plugins that write the search metadata, the
  sitemap and robots.txt.
The licence header comments are left out of the copy; the pages are published under the same licence.

Usage: docs-site.py <docs dir> <output dir> [--repo-url https://github.com/AIIX/RiftLinux]
                    [--site-url https://aiix.github.io] [--base-url /RiftLinux]
"""

import argparse
import os
import re
import shutil
import sys

SITE_TITLE = "RiftLinux"
SITE_DESCRIPTION = ("RiftLinux, the open-source Linux driver for the Oculus Rift CV1: headset, Constellation sensors "
                    "and Touch controllers with full 6DoF tracking, for SteamVR and OpenXR.")
# The pages in the site's menu, in this order, with their short menu labels (the index is the home page; a page's
# full title is for search results and its own heading).
MENU = {"getting-started.md": "Getting started", "rift-tab.md": "Rift tab", "sensors.md": "Sensors",
        "controllers.md": "Controllers", "picture.md": "Picture", "openxr-without-steamvr.md": "OpenXR",
        "troubleshooting.md": "Troubleshooting", "faq.md": "FAQ"}
# The theme's header with the menu's labels from each page's "nav" (the theme shows titles, too long for a menu).
HEADER = """<header class="site-header" role="banner">
  <div class="wrapper">
    <a class="site-title" rel="author" href="{{ "/" | relative_url }}">{{ site.title | escape }}</a>
    <nav class="site-nav">
      <input type="checkbox" id="nav-trigger" class="nav-trigger" />
      <label for="nav-trigger"><span class="menu-icon"><svg viewBox="0 0 18 15" width="18px" height="15px" role="img" aria-label="Menu" fill="#424242"><rect width="18" height="3" rx="1.5"/><rect y="6" width="18" height="3" rx="1.5"/><rect y="12" width="18" height="3" rx="1.5"/></svg></span></label>
      <div class="trigger">
        {%- for path in site.header_pages -%}
          {%- assign item = site.pages | where: "path", path | first -%}
          {%- if item -%}
          <a class="page-link" href="{{ item.url | relative_url }}">{{ item.nav | default: item.title | escape | replace: " ", "&nbsp;" }}</a>
          {%- endif -%}
        {%- endfor -%}
      </div>
    </nav>
  </div>
</header>
"""
DESCRIPTION_MAX = 160


def plain(markdown):
    """A paragraph's text without Markdown: link texts kept, emphasis and code marks dropped, one line."""
    text = re.sub(r"!\[[^\]]*\]\([^)]*\)", "", markdown)
    text = re.sub(r"\[([^\]]+)\]\([^)]*\)", r"\1", text)
    text = re.sub(r"<(https?://[^>]+)>", r"\1", text)
    text = text.replace("**", "").replace("`", "")
    text = re.sub(r"(?<!\w)\*(\S[^*]*\S|\S)\*(?!\w)", r"\1", text)
    return " ".join(text.split())


def describe(paragraph):
    """Up to DESCRIPTION_MAX characters: whole sentences when they fit, else cut at a word."""
    text = plain(paragraph)
    if len(text) <= DESCRIPTION_MAX:
        return text
    out = ""
    for sentence in re.split(r"(?<=[.!?])\s+", text):
        if len(out) + len(sentence) + 1 > DESCRIPTION_MAX:
            break
        out = (out + " " + sentence).strip()
    if out:
        return out
    return text[:DESCRIPTION_MAX].rsplit(" ", 1)[0]


def yaml_quote(text):
    return '"' + text.replace("\\", "\\\\").replace('"', '\\"') + '"'


def convert(text, repo_url):
    """(front matter values, body) of one page."""
    # The comments before the title: the licence header, and the description line.
    given = None
    while True:
        m = re.match(r"\A\s*<!--(.*?)-->\s*", text, flags=re.S)
        if not m:
            break
        d = re.match(r"\s*description:\s*(.*\S)\s*\Z", m.group(1), flags=re.S)
        if d:
            given = " ".join(d.group(1).split())
        text = text[m.end():]
    lines = text.split("\n")
    title, start = None, 0
    for i, line in enumerate(lines):
        if line.startswith("# "):
            title, start = line[2:].strip(), i + 1
            break
    if title is None:
        raise ValueError("no title heading")
    body = "\n".join(lines[start:]).strip("\n") + "\n"
    # The first paragraph that is text (not a heading, table, list, quote, picture or code).
    description = given or ""
    for block in [] if given else re.split(r"\n\s*\n", body):
        b = block.strip()
        if b and not b.startswith(("#", "|", "-", ">", "!", "```", "1.", "<")):
            description = describe(b)
            break

    def link(m):
        target = m.group(1)
        if re.match(r"^[a-z]+:", target) or target.startswith("#"):
            return m.group(0)
        path, _, anchor = target.partition("#")
        anchor = "#" + anchor if anchor else ""
        if path.startswith("../"):
            return f"]({repo_url}/blob/main/{path[3:]}{anchor})"
        if path == "README.md":
            return f"](./{anchor})"
        if path.endswith(".md"):
            return f"]({path[:-3]}.html{anchor})"
        return m.group(0)

    # Code blocks are left alone.
    parts = re.split(r"(```.*?```)", body, flags=re.S)
    body = "".join(p if p.startswith("```") else re.sub(r"\]\(([^)\s]*)\)", link, p) for p in parts)
    return {"title": title, "description": description}, body


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("docs")
    ap.add_argument("out")
    ap.add_argument("--repo-url", default="https://github.com/AIIX/RiftLinux")
    ap.add_argument("--site-url", default="https://aiix.github.io")
    ap.add_argument("--base-url", default="/RiftLinux")
    a = ap.parse_args()

    if os.path.exists(a.out):
        shutil.rmtree(a.out)
    os.makedirs(a.out)
    pages = {}
    for name in sorted(os.listdir(a.docs)):
        src = os.path.join(a.docs, name)
        if os.path.isdir(src):
            shutil.copytree(src, os.path.join(a.out, name))
            continue
        if not name.endswith(".md"):
            shutil.copy2(src, os.path.join(a.out, name))
            continue
        try:
            meta, body = convert(open(src, encoding="utf-8").read(), a.repo_url)
        except ValueError as e:
            sys.exit(f"ERROR: {src}: {e}")
        target = "index.md" if name == "README.md" else name
        front = ["---", "layout: page", "title: " + yaml_quote(meta["title"])]
        if meta["description"]:
            front.append("description: " + yaml_quote(meta["description"]))
        if name in MENU:
            front.append("nav: " + yaml_quote(MENU[name]))
        front.append("---")
        with open(os.path.join(a.out, target), "w", encoding="utf-8") as f:
            f.write("\n".join(front) + "\n\n" + body)
        pages[name] = meta

    missing = [p for p in MENU if p not in pages]
    if missing:
        sys.exit("ERROR: menu pages missing from docs/: " + ", ".join(missing))
    config = [
        f"title: {yaml_quote(SITE_TITLE)}",
        f"description: {yaml_quote(SITE_DESCRIPTION)}",
        f"url: {yaml_quote(a.site_url)}",
        f"baseurl: {yaml_quote(a.base_url)}",
        "lang: en",
        "author: \"Aditya Mehra\"",
        "theme: minima",
        "plugins:",
        "  - jekyll-seo-tag",
        "  - jekyll-sitemap",
        "header_pages:",
    ] + [f"  - {p}" for p in MENU] + [
        "minima:",
        "  social_links:",
        f"    github: {a.repo_url.rstrip('/').split('/')[-2]}",
        "github_link: " + yaml_quote(a.repo_url),
    ]
    with open(os.path.join(a.out, "_config.yml"), "w", encoding="utf-8") as f:
        f.write("\n".join(config) + "\n")
    os.makedirs(os.path.join(a.out, "_includes"), exist_ok=True)
    with open(os.path.join(a.out, "_includes", "header.html"), "w", encoding="utf-8") as f:
        f.write(HEADER)
    for name, meta in pages.items():
        print(f"{name}: {meta['title']} | {meta['description']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
