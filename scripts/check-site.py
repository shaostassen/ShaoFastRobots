#!/usr/bin/env python3
"""Verify the built site after editing templates, docs.css, or lab frontmatter.

Two tiers of checks:

  static   Builds the site and inspects the generated HTML — every lab page has a
           sidebar, breadcrumbs, prev/next, no blog chrome; no broken <img> refs;
           the lab index and search index are complete. No dependencies.

  browser  Serves the site and drives real Chrome over the DevTools Protocol —
           console errors, sidebar overflow, horizontal scroll at desktop and
           phone widths, KaTeX rendering, and the sidebar search actually
           returning results. Needs `websocket-client` and Google Chrome; skipped
           automatically with a note if either is missing.

Usage:
    python3 scripts/check-site.py                # both tiers
    python3 scripts/check-site.py --static-only  # skip Chrome
    python3 scripts/check-site.py --shots out/   # also save screenshots

Exits non-zero if any check fails, so it works as a pre-deploy gate.
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import re
import shutil
import subprocess
import sys
import time
import urllib.parse
import urllib.request

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PUBLIC = os.path.join(REPO, "public")
LABS_GLOB = "labs"

CHROME = os.environ.get(
    "CHROME_BIN", "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
)

# Attribute values are unquoted when `minify_html = true` and quoted when it's
# off, so every pattern below has to tolerate both.
def attr(name: str, value: str) -> re.Pattern:
    return re.compile(rf'{name}=["\']?{re.escape(value)}[\s"\'>]')


class Results:
    def __init__(self) -> None:
        self.failures: list[str] = []
        self.notes: list[str] = []

    def check(self, ok: bool, label: str, detail: str = "") -> bool:
        if not ok:
            self.failures.append(f"{label}{': ' + detail if detail else ''}")
        return ok

    def note(self, msg: str) -> None:
        self.notes.append(msg)


# --------------------------------------------------------------------- static --

def lab_pages() -> list[str]:
    root = os.path.join(PUBLIC, LABS_GLOB)
    if not os.path.isdir(root):
        return []
    pages = []
    for entry in sorted(os.listdir(root)):
        index = os.path.join(root, entry, "index.html")
        if not (re.fullmatch(r"lab-?\d+", entry) and os.path.isfile(index)):
            continue
        # Skip `aliases` redirect stubs — Zola emits a ~500-byte page that just
        # bounces to the real URL, and it has none of the page furniture.
        if "<title>Redirect</title>" in open(index, encoding="utf-8").read(1000):
            continue
        pages.append(index)
    return sorted(pages, key=lambda p: int(re.search(r"lab-?(\d+)", p).group(1)))


PAGE_CHECKS = {
    "sidebar":      lambda h: "docs-sidebar" in h,
    "nav tree":     lambda h: "docs-nav" in h,
    "current item": lambda h: attr("class", "current").search(h) is not None,
    "sub-toc":      lambda h: "docs-nav-toc" in h,
    "breadcrumbs":  lambda h: "docs-breadcrumbs" in h,
    "edit link":    lambda h: "docs-edit-link" in h,
    "prev/next":    lambda h: "post-nav" in h,
    "docs.css":     lambda h: "docs.css" in h,
    "docs-nav.js":  lambda h: "docs-nav.js" in h,
    "one <head>":   lambda h: len(re.findall(r"<head\s*>", h)) == 1,
    # Duckquill picks singular/plural from i18n ("1 minute read" vs "6 minutes
    # read"), so match both — a bare "minute read" substring misses the common case.
    "no read time": lambda h: re.search(r"\d+\s+minutes?\s+read", h) is None,
    "no share btn": lambda h: "shareopenly" not in h.lower(),
    "no CDN katex": lambda h: "jsdelivr" not in h.lower(),
}


def check_static(r: Results) -> None:
    print("building site...")
    build = subprocess.run(["zola", "build"], cwd=REPO, capture_output=True, text=True)
    if not r.check(build.returncode == 0, "zola build", build.stderr.strip()[:300]):
        return
    print(f"  {build.stdout.strip().splitlines()[-2] if build.stdout.strip() else 'built'}")

    pages = lab_pages()
    if not r.check(bool(pages), "found lab pages", "none under public/" + LABS_GLOB):
        return
    print(f"\nstructural checks over {len(pages)} lab pages")

    for path in pages:
        html = open(path, encoding="utf-8").read()
        name = os.path.basename(os.path.dirname(path))
        bad = [label for label, test in PAGE_CHECKS.items() if not test(html)]
        r.check(not bad, f"{name}", ", ".join(bad))
        print(f"  {name:8s} {'ok' if not bad else 'FAIL ' + ', '.join(bad)}")

    # Colocated images are referenced relatively; a rename or a `path` override
    # silently breaks them, so resolve every one against the output directory.
    broken = []
    for path in pages:
        here = os.path.dirname(path)
        html = open(path, encoding="utf-8").read()
        for src in re.findall(r'<img[^>]+src=["\']?([^"\'\s>]+)', html):
            if src.startswith(("http", "data:", "//", "/")):
                continue
            if not os.path.exists(os.path.join(here, src)):
                broken.append(f"{os.path.basename(here)}/{src}")
    r.check(not broken, "image references", f"{len(broken)} broken: {broken[:5]}")
    print(f"\n  broken <img> refs: {len(broken)}")

    home = os.path.join(PUBLIC, "index.html")
    if os.path.isfile(home):
        # The home page lists labs as a timeline; the section page uses the flat
        # contents list. Either markup counts, as long as every lab is present.
        html = open(home, encoding="utf-8").read()
        entries = html.count("lab-timeline-title") or html.count("lab-index-title")
        r.check(entries == len(pages), "home lab index",
                f"lists {entries}, expected {len(pages)}")
        print(f"  home lab index:    {entries} entries")

    # A stale static/icons.css once shadowed the theme's generated one and silently
    # blanked 22 icons (copy button, go-to-top, anchor links). Any --icon-* the
    # stylesheets reference must actually be defined.
    sheets = [os.path.join(PUBLIC, name) for name in ("icons.css", "style.css", "docs.css")]
    sheets = [s for s in sheets if os.path.isfile(s)]
    if sheets:
        defined, used = set(), set()
        for sheet in sheets:  # docs.css defines its own icons, so scan every sheet for both
            css = open(sheet, encoding="utf-8").read()
            defined |= set(re.findall(r"(--icon-[a-z0-9-]+)\s*:", css))
            used |= set(re.findall(r"var\((--icon-[a-z0-9-]+)", css))
        missing = sorted(used - defined)
        r.check(not missing, "icon variables", f"{len(missing)} undefined: {missing[:6]}")
        print(f"  icons:             {len(defined)} defined, {len(missing)} referenced-but-missing")

    index_path = os.path.join(PUBLIC, "search_index.en.json")
    if os.path.isfile(index_path):
        data = json.load(open(index_path))
        docs = data if isinstance(data, list) else data.get("docs", [])
        titles = [d.get("title") or "" for d in docs]
        dupes = {t for t in titles if t and titles.count(t) > 1}
        r.check(len(docs) >= len(pages), "search index", f"only {len(docs)} docs")
        r.check(not dupes, "search index duplicates", ", ".join(sorted(dupes)[:3]))
        print(f"  search index:      {len(docs)} docs, {len(dupes)} duplicate titles")


# -------------------------------------------------------------------- browser --

class Chrome:
    """Minimal DevTools Protocol client: launch, navigate, evaluate, screenshot."""

    def __init__(self, port: int = 9222, profile: str = "/tmp/check-site-profile"):
        import websocket  # imported lazily so --static-only needs no dependency

        self._ws_lib = websocket
        self.port = port
        self.proc = subprocess.Popen(
            [
                CHROME, "--headless", "--disable-gpu", "--hide-scrollbars",
                f"--remote-debugging-port={port}",
                # Chrome 111+ rejects the WebSocket handshake with 403 without this.
                "--remote-allow-origins=*",
                f"--user-data-dir={profile}",
                "about:blank",
            ],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        self.ws = None
        self._id = 0
        for _ in range(60):
            try:
                targets = json.load(
                    urllib.request.urlopen(f"http://127.0.0.1:{port}/json", timeout=2)
                )
                url = next(t["webSocketDebuggerUrl"] for t in targets if t["type"] == "page")
                self.ws = websocket.create_connection(url, timeout=30)
                break
            except Exception:
                time.sleep(0.25)
        if self.ws is None:
            self.close()
            raise RuntimeError("could not attach to Chrome over CDP")
        self.send("Page.enable")
        self.send("Runtime.enable")
        self.send("Log.enable")

    def send(self, method: str, **params):
        self._id += 1
        self.ws.send(json.dumps({"id": self._id, "method": method, "params": params}))
        while True:  # async events arrive interleaved; skip until our reply
            msg = json.loads(self.ws.recv())
            if msg.get("id") == self._id:
                return msg

    def js(self, expression: str):
        reply = self.send("Runtime.evaluate", expression=expression,
                          awaitPromise=True, returnByValue=True)
        result = reply.get("result", {})
        if "exceptionDetails" in result:
            desc = result["exceptionDetails"].get("exception", {}).get("description", "")
            return f"<JS ERROR: {str(desc)[:160]}>"
        return result.get("result", {}).get("value")

    def viewport(self, width: int, height: int) -> None:
        self.send("Emulation.setDeviceMetricsOverride", width=width, height=height,
                  deviceScaleFactor=1, mobile=width < 700)

    def goto(self, url: str, settle: float = 3.5) -> None:
        self.send("Page.navigate", url=url)
        time.sleep(settle)

    def drain_console(self) -> list[str]:
        """Collect error-level console output buffered since the last call."""
        out = []
        self.ws.settimeout(0.5)
        try:
            while True:
                event = json.loads(self.ws.recv())
                method = event.get("method")
                if method == "Runtime.exceptionThrown":
                    text = event["params"]["exceptionDetails"].get("text", "")
                    out.append(f"exception: {text}"[:200])
                elif method == "Log.entryAdded":
                    entry = event["params"]["entry"]
                    if entry.get("level") == "error":
                        out.append(f"error: {entry.get('text', '')}"[:200])
        except Exception:
            pass
        finally:
            self.ws.settimeout(30)
        return out

    def screenshot(self, path: str) -> bool:
        data = self.send("Page.captureScreenshot", format="png").get("result", {}).get("data")
        if not data:
            return False
        with open(path, "wb") as fh:
            fh.write(base64.b64decode(data))
        return True

    def close(self) -> None:
        try:
            if self.ws:
                self.ws.close()
        except Exception:
            pass
        self.proc.terminate()


def serve(port: int) -> subprocess.Popen:
    proc = subprocess.Popen(
        ["zola", "serve", "--port", str(port)], cwd=REPO,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    for _ in range(60):
        try:
            urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=2).read()
            return proc
        except Exception:
            time.sleep(0.5)
    proc.terminate()
    raise RuntimeError(f"zola serve did not come up on port {port}")


def check_browser(r: Results, port: int, shots_dir: str | None) -> None:
    try:
        import websocket  # noqa: F401
    except ImportError:
        r.note("browser checks skipped: pip install websocket-client")
        return
    if not os.path.exists(CHROME):
        r.note(f"browser checks skipped: no Chrome at {CHROME} (set CHROME_BIN)")
        return

    pages = lab_pages()
    if not pages:
        return
    # Derive live URLs from the built output so this keeps working if URLs change.
    slugs = [os.path.basename(os.path.dirname(p)) for p in pages]
    base = f"http://127.0.0.1:{port}"
    section = urllib.parse.quote(LABS_GLOB)
    lab_url = f"{base}/{section}/{slugs[0]}/"
    math_url = f"{base}/{section}/{slugs[-1]}/"

    print(f"\nbrowser checks (zola serve :{port} + headless Chrome)")
    server = serve(port)
    chrome = None
    try:
        chrome = Chrome(port=port + 8000 if port + 8000 < 65535 else 9222)

        for label, url, (w, h) in [
            ("home desktop", base + "/", (1440, 1000)),
            ("lab desktop", lab_url, (1440, 1000)),
            ("lab phone", lab_url, (430, 900)),
        ]:
            chrome.viewport(w, h)
            chrome.goto(url)
            errors = chrome.drain_console()
            overflow = chrome.js(
                "document.documentElement.scrollWidth - document.documentElement.clientWidth")
            spill = chrome.js("""(() => {
                const rail = document.getElementById('docs-sidebar');
                const foot = document.querySelector('.docs-sidebar-foot');
                if (!rail || !foot) return 0;
                return Math.max(0, Math.round(
                    foot.getBoundingClientRect().bottom - window.innerHeight));
            })()""")
            r.check(not errors, f"{label} console", "; ".join(errors[:2]))
            r.check(overflow == 0, f"{label} h-scroll", f"{overflow}px")
            r.check(spill == 0, f"{label} sidebar spill", f"{spill}px past viewport")
            print(f"  {label:13s} console={len(errors)} h-scroll={overflow} spill={spill}")
            if shots_dir:
                chrome.screenshot(os.path.join(
                    shots_dir, f"{label.replace(' ', '-')}.png"))

        # The collapsed state has its own failure mode: if the expand control
        # loses its styling it stops being a fixed tab and becomes an invisible
        # full-width strip far down the page, stranding anyone whose stored
        # preference has the sidebar hidden. Checking only the visible state
        # missed exactly that.
        chrome.viewport(1440, 1000)
        chrome.goto(base + "/")
        chrome.js("localStorage.setItem('docs-sidebar-hidden','1')")
        chrome.goto(base + "/")
        hidden = chrome.js(
            "getComputedStyle(document.getElementById('docs-sidebar')).display")
        tab = chrome.js("""(() => {
            const b = document.querySelector('.docs-sidebar-expand');
            if (!b) return null;
            const r = b.getBoundingClientRect();
            const s = getComputedStyle(b);
            return {pos: s.position, w: Math.round(r.width), h: Math.round(r.height),
                    x: Math.round(r.x), y: Math.round(r.y),
                    onscreen: r.y >= 0 && r.y < window.innerHeight && r.width < 200};
        })()""")
        r.check(hidden == "none", "collapsed: sidebar hides", f"display={hidden}")
        r.check(bool(tab) and tab.get("pos") == "fixed",
                "collapsed: expand tab is fixed", str(tab))
        r.check(bool(tab) and tab.get("onscreen"),
                "collapsed: expand tab reachable", str(tab))
        print(f"  collapsed     sidebar={hidden} tab={tab.get('w') if tab else '?'}x"
              f"{tab.get('h') if tab else '?'} @{tab.get('y') if tab else '?'} "
              f"pos={tab.get('pos') if tab else '?'}")
        chrome.js("localStorage.removeItem('docs-sidebar-hidden')")

        # Sidebar search: type, then read the rendered results panel.
        chrome.viewport(1440, 1000)
        chrome.goto(lab_url)
        chrome.js("""(() => {
            const bar = document.getElementById('search-bar');
            if (!bar) return false;
            bar.value = 'kalman';
            // .focus() does not fire focus handlers in headless, so drive keyup.
            bar.dispatchEvent(new KeyboardEvent('keyup', {bubbles: true}));
            return true;
        })()""")
        time.sleep(2.5)
        hits = chrome.js("document.querySelectorAll('#search-results .item').length")
        first = chrome.js("document.querySelector('#search-results .item a')?.textContent")
        escapes = chrome.js("""(() => {
            const p = document.getElementById('search-results');
            const s = document.getElementById('docs-sidebar');
            if (!p || !s) return false;
            return p.getBoundingClientRect().right > s.getBoundingClientRect().right;
        })()""")
        r.check(isinstance(hits, int) and hits > 0, "sidebar search", f"{hits} results")
        r.check(escapes is True, "search panel not clipped", "panel confined to sidebar")
        print(f"  search        hits={hits} first={first!r}")
        if shots_dir:
            chrome.screenshot(os.path.join(shots_dir, "search.png"))

        # KaTeX on the math-heavy final lab.
        chrome.goto(math_url)
        rendered = chrome.js("document.querySelectorAll('.katex').length")
        errs = chrome.js("document.querySelectorAll('.katex-error').length")
        raw = chrome.js("(document.body.innerText.match(/\\$\\$/g) || []).length")
        if rendered:
            r.check(errs == 0, "katex errors", f"{errs}")
            r.check(raw == 0, "unrendered math", f"{raw} stray '$$'")
        print(f"  katex         rendered={rendered} errors={errs} unrendered={raw}")

        # Light theme, where low-contrast regressions show up first.
        chrome.goto(lab_url)
        chrome.js("document.documentElement.setAttribute('data-theme','light')")
        time.sleep(1)
        if shots_dir:
            chrome.screenshot(os.path.join(shots_dir, "light.png"))
            print(f"\n  screenshots -> {shots_dir}")
    finally:
        if chrome:
            chrome.close()
        # Wait for the server to actually exit. `zola serve` writes into public/,
        # and a still-running instance makes the *next* build fail with
        # "Couldn't delete output directory / Directory not empty".
        server.terminate()
        try:
            server.wait(timeout=10)
        except subprocess.TimeoutExpired:
            server.kill()
            server.wait(timeout=5)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--static-only", action="store_true", help="skip the Chrome tier")
    ap.add_argument("--port", type=int, default=1111, help="port for zola serve")
    ap.add_argument("--shots", metavar="DIR", help="write screenshots to DIR")
    args = ap.parse_args()

    if not shutil.which("zola"):
        print("error: zola not on PATH", file=sys.stderr)
        return 2
    if not os.path.isdir(os.path.join(REPO, "themes", "duckquill", "sass")):
        print("error: theme submodule missing — run:\n"
              "  git submodule update --init --recursive", file=sys.stderr)
        return 2

    shots_dir = None
    if args.shots:
        shots_dir = os.path.abspath(args.shots)
        os.makedirs(shots_dir, exist_ok=True)

    r = Results()
    check_static(r)
    if not args.static_only and not r.failures:
        check_browser(r, args.port, shots_dir)
    elif r.failures and not args.static_only:
        r.note("browser checks skipped: static checks already failed")

    print()
    for note in r.notes:
        print(f"note: {note}")
    if r.failures:
        print(f"\nFAILED ({len(r.failures)}):")
        for f in r.failures:
            print(f"  - {f}")
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
