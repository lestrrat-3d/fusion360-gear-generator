#!/usr/bin/env python3
"""Shared resolution/clone of the Autodesk FusionAPIReference Python stub defs.

One clone, one resolution policy, used by both:
  - pyright_check.py     (static-analysis gate — needs the stubs on pyright's path)
  - build_fusion_index.py (API lookup index — parses the stubs into a grep-able index)

The FusionAPIReference repo is 338M; the Python defs are ~4M, so the clone is sparse,
shallow, and blobless (~13M on disk) and cached under ~/.cache/fusion360-gear-generator/
so the two tools share it.

Resolution precedence (see resolve_defs):
    1. explicit arg (e.g. --stubs)   authoritative; a wrong path raises, no fallback
    2. $FUSION_API_STUBS             authoritative if set; a wrong path raises
    3. cached clone                  else clone the repo (sparse/shallow/blobless) and reuse

Any of these may point at the `defs` dir itself OR the FusionAPIReference checkout root;
the dir that contains `adsk/core.py` is located under it.
"""
import os
import shutil
import subprocess

REPO_URL = "https://github.com/AutodeskFusion360/FusionAPIReference.git"
DEFS_REL = os.path.join("Fusion_API_Python_Reference", "defs")  # defs path within the repo


class StubsUnavailable(Exception):
    """Raised when the Fusion API stub defs cannot be resolved or cloned. The message
    is human-readable; callers map it to their own exit code / fallback."""


def defs_at(base):
    """Return the abs defs dir (containing adsk/core.py) at `base` or under its
    `Fusion_API_Python_Reference/defs` subpath, or None. Accepts either form so a path
    may point at the defs dir itself or at the FusionAPIReference checkout root."""
    if not base:
        return None
    base = os.path.abspath(base)
    for cand in (base, os.path.join(base, DEFS_REL)):
        if os.path.isfile(os.path.join(cand, "adsk", "core.py")):
            return cand
    return None


def cache_repo_dir():
    cache = os.environ.get("XDG_CACHE_HOME") or os.path.join(os.path.expanduser("~"), ".cache")
    return os.path.join(cache, "fusion360-gear-generator", "FusionAPIReference")


def clone_stubs(quiet=False):
    """Clone (sparse, shallow, blobless) just the Python defs into the cache; return the
    defs dir, or None on failure (offline / git missing)."""
    repo = cache_repo_dir()
    existing = defs_at(repo)
    if existing:
        return existing
    if not quiet:
        print(f"Fusion API stubs not found; cloning {REPO_URL}")
        print(f"  -> {repo} (sparse: {DEFS_REL}, shallow, blobless; ~13M of 338M)")
    os.makedirs(os.path.dirname(repo), exist_ok=True)
    try:
        subprocess.run(
            ["git", "clone", "--depth", "1", "--filter=blob:none", "--sparse",
             REPO_URL, repo],
            check=True, capture_output=True, text=True, timeout=600)
        subprocess.run(
            ["git", "-C", repo, "sparse-checkout", "set", DEFS_REL],
            check=True, capture_output=True, text=True, timeout=600)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired,
            FileNotFoundError) as e:
        if os.path.isdir(repo):
            shutil.rmtree(repo, ignore_errors=True)  # don't leave a half-clone behind
        detail = getattr(e, "stderr", None) or str(e)
        if not quiet:
            print("ERROR: failed to clone Fusion API stubs (offline? git missing?).")
            print("  " + str(detail).strip()[:300])
        return None
    return defs_at(repo)


def resolve_defs(explicit=None, env_var="FUSION_API_STUBS", quiet=False):
    """Resolve the abs defs dir per the precedence documented at module level.
    Returns the defs dir. Raises StubsUnavailable (with a descriptive message) on any
    failure, so each caller maps it to its own exit code instead of exiting here."""
    if explicit:
        d = defs_at(explicit)
        if not d:
            raise StubsUnavailable(
                f"--stubs {explicit} has no adsk/core.py "
                f"(point at the defs dir or the FusionAPIReference checkout)")
        return d
    env = os.environ.get(env_var)
    if env:
        d = defs_at(env)
        if not d:
            raise StubsUnavailable(
                f"${env_var}={env} has no adsk/core.py "
                f"(point at the defs dir or the FusionAPIReference checkout)")
        return d
    d = clone_stubs(quiet=quiet)
    if not d:
        raise StubsUnavailable(
            f"could not resolve Fusion API stubs (offline? git missing?). "
            f"Set ${env_var} to a local FusionAPIReference checkout to skip cloning.")
    return d
