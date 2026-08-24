#!/usr/bin/env python3
"""Shared resolution/bootstrap of pyright for the static-analysis gate.

pyright_check.py needs two things: the Fusion API stub defs (fusion_stubs.py provisions those)
and pyright itself. This module is the pyright half, and it follows the same rule: provision
into ~/.cache/fusion360-gear-generator/ rather than ask a human to run pip.

The `pyright` PyPI distribution is a pure-Python WRAPPER (pyright-python) around the real
Node-based checker. Installing it is quick, but its FIRST execution locates node — bootstrapping
one via nodeenv when there is none — and downloads the pinned pyright npm bundle into the
wrapper's own cache. So a fresh provision costs a second, slower download at first run.

Resolution precedence (see resolve_pyright):
    1. importable module         `import pyright` works -> run it as-is (today's behaviour)
    2. PATH binary               a `pyright` executable on $PATH (npm CLI or console script)
    3. cached --target install   a previous bootstrap under the cache, put on PYTHONPATH
    4. install                   else `pip install --target <cache>` pyright, then as (3)

`--no-install` (threaded through as no_install=True) forbids step 4 and raises instead. The
install is always `--target` under the cache, never `--break-system-packages`: the gate must
not modify the user's Python environment.
"""
import importlib.util
import os
import shutil
import subprocess
import sys

MANUAL_HINT = "python3 -m pip install --break-system-packages pyright"


class PyrightUnavailable(Exception):
    """Raised when pyright cannot be resolved or bootstrapped. The message is
    human-readable; callers map it to their own exit code / fallback."""


def cache_pkg_dir():
    """Target dir for the cached `pip install --target` of the pyright wrapper. The env var
    is read at call time so a caller (or a test) can redirect the cache."""
    cache = os.environ.get("XDG_CACHE_HOME") or os.path.join(os.path.expanduser("~"), ".cache")
    return os.path.join(cache, "fusion360-gear-generator", "pyright-pkg")


def _pythonpath_with(target):
    """`target` prepended to any existing $PYTHONPATH — never dropping it — or `target` alone."""
    existing = os.environ.get("PYTHONPATH")
    if existing:
        return target + os.pathsep + existing
    return target


def _cached_install_present(target):
    return os.path.isfile(os.path.join(target, "pyright", "__init__.py"))


def _pip_install(target):
    """Install the pyright wrapper (and its deps) into `target`. Module-level so tests can
    stub it. Raises PyrightUnavailable on any failure, leaving no half-install behind."""
    try:
        subprocess.run(
            [sys.executable, "-m", "pip", "install", "--quiet", "--target", target, "pyright"],
            check=True, capture_output=True, text=True, timeout=600)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired,
            FileNotFoundError) as e:
        # A partial target dir would be trusted by the cached-install branch on the next run.
        shutil.rmtree(target, ignore_errors=True)
        detail = getattr(e, "stderr", None) or str(e)
        raise PyrightUnavailable(
            f"failed to install pyright into {target} (offline? pip missing?): "
            f"{str(detail).strip()[:300]} — install it yourself with `{MANUAL_HINT}` and retry")


def resolve_pyright(no_install=False, quiet=False):
    """Resolve pyright per the precedence documented at module level.

    Returns `(argv_prefix, extra_env)`: the caller appends its own pyright arguments to
    `argv_prefix` and merges `extra_env` over os.environ for the subprocess. Raises
    PyrightUnavailable (with a descriptive message) on any failure, so each caller maps it
    to its own exit code instead of exiting here."""
    if importlib.util.find_spec("pyright") is not None:
        return [sys.executable, "-m", "pyright"], {}

    on_path = shutil.which("pyright")
    if on_path:
        return [on_path], {}

    target = cache_pkg_dir()
    if _cached_install_present(target):
        return [sys.executable, "-m", "pyright"], {"PYTHONPATH": _pythonpath_with(target)}

    if no_install:
        raise PyrightUnavailable(
            f"pyright not found and --no-install given; install it with `{MANUAL_HINT}` "
            f"or unset --no-install")

    if not quiet:
        print(f"pyright not found; installing the pyright wrapper into {target}")
        print("  (first pyright run may also download node / the pyright npm bundle into "
              "its own cache)")
    _pip_install(target)
    return [sys.executable, "-m", "pyright"], {"PYTHONPATH": _pythonpath_with(target)}
