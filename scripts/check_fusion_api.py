#!/usr/bin/env python3
"""Check every add-in file's Fusion API names and call signatures with pyright and the Fusion API stubs.

The add-in's tests stub `adsk` with MagicMock, so any attribute resolves there and a misspelled Fusion call only fails
inside Fusion. This runs pyright (standard mode) over `lib/`, `commands/` and the Python files at the repository root,
with Autodesk's FusionAPIReference stubs on the search path, and fails on these findings only:

  syntax error                       a file that does not parse
  undefined name                     reportUndefinedVariable
  unknown module attribute           `adsk.fusion.SurfaceTypes`: no such name in that module
  unknown attribute of a Fusion type `component.constructionPlanez`: the stub class has no such member, and no stub
                                     class derived from it has one. A member that only a derived class has
                                     (`CommandInput.value`, which `ValueCommandInput` has) is a missing cast, not a
                                     misspelling, and is not reported.
  wrong call signature               reportCallIssue for a wrong argument count, an unknown keyword or a missing
                                     argument

pyright names a class without its module, and local classes share names with stub classes (`Sketch`, `Profile`). So
pyright runs twice, with and without the stubs, and an attribute finding counts as one about a Fusion type only when it
appears with the stubs alone.

Everything else pyright reports (Optional access, enum members typed as int in the stubs, untyped helpers) is noise for
this purpose and is ignored. scripts/fusion-api-allow.txt lists findings that are known and still work in Fusion, one
`<path>: <message>` per line, without the line number; each entry is a finding to fix, not a rule.

Usage, from anywhere:
    scripts/fetch-fusion-api-stubs.sh       # once: puts the pinned stubs in .tmp/FusionAPIReference
    scripts/check_fusion_api.py [--stubs DIR] [--root DIR] [--verbose]

--stubs defaults to $FUSION_API_STUBS, then to .tmp/FusionAPIReference under the root. --root defaults to this
script's repository. pyright is the `pyright` on PATH, else `python3 -m pyright`. The run writes its pyright configs
to <root>/.tmp/pyright-fusion-api*.json.

Exit status: 0 no findings, 1 findings, 2 setup error (no stubs, no pyright, or pyright analysed no files).
"""
import argparse
import ast
import glob
import json
import os
import re
import shutil
import subprocess
import sys
import warnings

ACCESS_RE = re.compile(r'^Cannot (?:access|assign to) attribute "([^"]+)" for class "([^"]+)"')
MODULE_RE = re.compile(r'^"([^"]+)" is not a known attribute of module "([^"]+)"')
CALL_RE = re.compile(
    r'^(Expected \d+ positional argument|No parameter named|Argument missing for parameter'
    r'|Arguments missing for parameters|Multiple values for parameter)')


class SetupError(Exception):
    pass


def find_stubs(path):
    """Return the directory that holds adsk/core.py at or under path, or None."""
    if not path or not os.path.isdir(path):
        return None
    for candidate in (path, os.path.join(path, "Fusion_API_Python_Reference", "defs")):
        if os.path.isfile(os.path.join(candidate, "adsk", "core.py")):
            return os.path.abspath(candidate)
    return None


def stub_index(stubs):
    """Map each stub class name to (base names, member names)."""
    classes = {}
    for path in sorted(glob.glob(os.path.join(stubs, "adsk", "*.py"))):
        with open(path, encoding="utf-8") as handle:
            text = handle.read()
        with warnings.catch_warnings():
            # The stubs' docstrings hold invalid escape sequences such as "\S".
            warnings.simplefilter("ignore", SyntaxWarning)
            tree = ast.parse(text, filename=path)
        for node in tree.body:
            if not isinstance(node, ast.ClassDef):
                continue
            bases = {b.id if isinstance(b, ast.Name) else b.attr
                     for b in node.bases if isinstance(b, (ast.Name, ast.Attribute))}
            members = set()
            for item in node.body:
                if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    members.add(item.name)
                elif isinstance(item, ast.AnnAssign) and isinstance(item.target, ast.Name):
                    members.add(item.target.id)
                elif isinstance(item, ast.Assign):
                    members.update(t.id for t in item.targets if isinstance(t, ast.Name))
            known_bases, known_members = classes.get(node.name, (set(), set()))
            classes[node.name] = (known_bases | bases, known_members | members)
    return classes


def derived_have(classes, name, attribute):
    """Whether the class or any stub class derived from it has the attribute."""
    children = {}
    for child, (bases, _) in classes.items():
        for base in bases:
            children.setdefault(base, set()).add(child)
    seen, todo = set(), [name]
    while todo:
        current = todo.pop()
        if current in seen:
            continue
        seen.add(current)
        if attribute in classes.get(current, (set(), set()))[1]:
            return True
        todo.extend(children.get(current, ()))
    return False


def first_line(diagnostic):
    message = diagnostic.get("message") or ""
    return message.splitlines()[0] if message else ""


def location(diagnostic, root):
    path = os.path.relpath(diagnostic.get("file", ""), root).replace(os.sep, "/")
    start = diagnostic.get("range", {}).get("start", {})
    return path, start.get("line", 0) + 1, start.get("character", 0)


def classify(diagnostic, classes, without_stubs, root):
    """Return the reason a diagnostic is a finding, or None when it is ignored."""
    rule = diagnostic.get("rule")
    message = first_line(diagnostic)
    if rule is None:
        return "syntax error" if diagnostic.get("severity") == "error" else None
    if rule == "reportUndefinedVariable":
        return "undefined name"
    if rule == "reportCallIssue" and CALL_RE.match(message):
        return "wrong call signature"
    if rule != "reportAttributeAccessIssue":
        return None
    if MODULE_RE.match(message):
        return "unknown module attribute"
    match = ACCESS_RE.match(message)
    if not match:
        return None
    attribute, cls = match.group(1), match.group(2)
    if cls.startswith("type[") and cls.endswith("]"):
        cls = cls[len("type["):-1]
    if cls not in classes or (location(diagnostic, root), message) in without_stubs:
        return None
    if derived_have(classes, cls, attribute):
        return None
    return "unknown attribute of a Fusion type"


def load_allow(path):
    entries = set()
    if os.path.isfile(path):
        with open(path, encoding="utf-8") as handle:
            for line in handle:
                line = line.strip()
                if line and not line.startswith("#"):
                    entries.add(line)
    return entries


def run_pyright(root, targets, stubs, name):
    """Run pyright over targets with stubs (or none) on the search path and return its diagnostics."""
    tmp = os.path.join(root, ".tmp")
    os.makedirs(tmp, exist_ok=True)
    config_path = os.path.join(tmp, name)
    # Paths in a pyright config are relative to the config file; absolute ones in "include" are ignored. "exclude" is
    # set because the default excludes every path with a component that starts with a dot, which is every path inside
    # a .worktrees/ checkout.
    config = {
        "typeCheckingMode": "standard",
        "pythonVersion": "3.12",
        "include": [os.path.relpath(os.path.join(root, t), tmp) for t in targets],
        "exclude": ["**/__pycache__"],
        "executionEnvironments": [{
            "root": os.path.relpath(root, tmp),
            "extraPaths": [os.path.relpath(stubs, tmp)] if stubs else [],
        }],
        "reportMissingModuleSource": "none",
    }
    with open(config_path, "w", encoding="utf-8") as handle:
        json.dump(config, handle, indent=2)
    found = shutil.which("pyright")
    command = ([found] if found else [sys.executable, "-m", "pyright"]) + ["-p", config_path, "--outputjson"]
    env = dict(os.environ, PYRIGHT_PYTHON_IGNORE_WARNINGS="1")
    try:
        proc = subprocess.run(command, cwd=root, capture_output=True, text=True, env=env)
    except OSError as error:
        raise SetupError("cannot run pyright: %s" % error)
    if proc.returncode not in (0, 1):
        raise SetupError("pyright exited %d: %s" % (proc.returncode, proc.stderr.strip()[:500]))
    try:
        data = json.loads(proc.stdout)
    except json.JSONDecodeError as error:
        raise SetupError("pyright returned malformed JSON: %s" % error)
    if data.get("summary", {}).get("filesAnalyzed", 0) == 0:
        raise SetupError("pyright analysed no files; check %s" % config_path)
    diagnostics = data.get("generalDiagnostics", [])
    if stubs and any(d.get("rule") == "reportMissingImports" and "adsk" in d.get("message", "")
                     for d in diagnostics):
        raise SetupError("pyright cannot resolve adsk from %s" % stubs)
    return data["summary"]["filesAnalyzed"], diagnostics


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--stubs", help="FusionAPIReference checkout or its defs directory")
    parser.add_argument("--root", help="repository root to check")
    parser.add_argument("--verbose", action="store_true", help="also list allowed findings")
    args = parser.parse_args()

    root = os.path.abspath(args.root or os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    stubs_arg = args.stubs or os.environ.get("FUSION_API_STUBS") or os.path.join(root, ".tmp", "FusionAPIReference")
    stubs = find_stubs(stubs_arg)
    if stubs is None:
        print("check_fusion_api: no Fusion API stubs (adsk/core.py) at %s; run scripts/fetch-fusion-api-stubs.sh"
              % stubs_arg, file=sys.stderr)
        return 2

    targets = ["lib", "commands"] + sorted(os.path.basename(p) for p in glob.glob(os.path.join(root, "*.py")))
    targets = [t for t in targets if os.path.exists(os.path.join(root, t))]
    try:
        analysed, diagnostics = run_pyright(root, targets, stubs, "pyright-fusion-api.json")
        _, bare = run_pyright(root, targets, None, "pyright-fusion-api-nostubs.json")
    except SetupError as error:
        print("check_fusion_api: %s" % error, file=sys.stderr)
        return 2
    without_stubs = {(location(d, root), first_line(d)) for d in bare}

    classes = stub_index(stubs)
    allow_path = os.path.join(root, "scripts", "fusion-api-allow.txt")
    allow = load_allow(allow_path)
    findings, allowed, used = [], [], set()
    for diagnostic in diagnostics:
        reason = classify(diagnostic, classes, without_stubs, root)
        if reason is None:
            continue
        (path, line, _), message = location(diagnostic, root), first_line(diagnostic)
        key = "%s: %s" % (path, message)
        record = "%s:%d: %s [%s]" % (path, line, message, reason)
        if key in allow:
            used.add(key)
            allowed.append(record)
        else:
            findings.append(record)

    for record in sorted(findings):
        print(record)
    if args.verbose:
        for record in sorted(allowed):
            print("allowed: " + record)
    allow_name = os.path.relpath(allow_path, root)
    for entry in sorted(allow - used):
        print("check_fusion_api: warning: %s matches nothing; remove it from %s" % (entry, allow_name))
    print("check_fusion_api: %d files, %d findings, %d allowed by %s"
          % (analysed, len(findings), len(allowed), allow_name))
    return 1 if findings else 0


if __name__ == "__main__":
    sys.exit(main())
