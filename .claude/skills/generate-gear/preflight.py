#!/usr/bin/env python3
"""One early verdict on whether this environment can run a gear stage.

Each of the three skills opens with a setup step that asks the orchestrating model to verify
the environment by hand: a worktree rather than the root checkout, `.tmp/`, the sketch and
decad engine checkouts, a go toolchain, pyright, the Fusion API database. Nothing checked it,
so a missing piece surfaced later as an exit-2 out of `proof/run.sh`, `pyright_check.py` or
`check_compile.py`, after a drafting round had already been spent. This script asks every
question at once, before the round starts.

It reports, it does not repair. The only thing it writes is `.tmp/` under the root, because
every stage needs that directory and creating it is cheaper than reporting it missing. It
never clones the stubs, never installs pyright, never runs `go test`, and never touches the
network. A check that cannot be answered without doing one of those says so and leaves the
work to the tool that owns it. The one heavier thing it runs is `check_compile.py`, on the emit
stage only, because emitting from a stale step list wastes a whole drafting round and the tool
that owns that verdict is cheap enough to consult here — it still needs no network and runs no
`go test`.

Resolution rules are borrowed, never re-derived: the engine directories resolve exactly as
`proof/run.sh` resolves them ($SKETCH_DIR / $DECAD_DIR, else siblings of the main checkout),
and the API database and the stubs are resolved by importing the sibling modules that own
those policies (`fusion_api`, `fusion_stubs`).

Usage:
    python3 preflight.py <gear> [--stage {compile,emit,generate,all}]
                                [--root DIR] [--format {text,json}]

Exit codes:
    0  ready (warnings are allowed; they name work to do, not a broken environment)
    1  at least one FAIL
    2  usage/setup error (bad gear name, unknown stage, --root is not the repo root)
"""
import argparse
import importlib.util
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)  # fusion_api / fusion_stubs / pick_model are siblings, imported
    #                           lazily below

GEAR_NAME = re.compile(r'[a-z][a-z0-9_]*\Z')  # same spelling stage.py accepts
TIMEOUT = 10  # seconds; every subprocess here is a version probe
# The one exception: check_compile.py parses the proof and queries the API database.
CHECK_COMPILE_TIMEOUT = 300

OK = 'ok'
WARN = 'warn'
FAIL = 'fail'
SKIP = 'skip'

# The files the drafter is told to read before writing a gear. Root-relative.
FRAMEWORK = (
    os.path.join('lib', 'geargen', 'base.py'),
    os.path.join('lib', 'geargen', 'misc.py'),
    os.path.join('lib', 'geargen', 'utilities.py'),
    os.path.join('lib', 'geargen', 'spurproxy.py'),
    os.path.join('lib', 'fusion360utils'),
)

# What makes a directory the repo root, and so what --root must contain.
ROOT_MARKERS = (
    'spec',
    os.path.join('lib', 'geargen'),
    os.path.join('.claude', 'skills', 'generate-gear'),
)


class Usage(Exception):
    """A bad invocation. Printed to stderr; always exit 2."""


class Context(object):
    """What every check is handed: the absolute root and the gear under test.

    `default_model` is the session's default model when the caller passed one.
    It is the only field no check can derive for itself — the orchestrating
    agent is the only thing that knows it — so it stays None when unstated and
    the row that reads it skips.
    """

    def __init__(self, root, gear, default_model=None):
        self.root = root
        self.gear = gear
        self.default_model = default_model

    def path(self, *parts):
        return os.path.join(self.root, *parts)

    def spec(self, *parts):
        return self.path('spec', self.gear, *parts)


# --- subprocess helpers --------------------------------------------------------------------
def _run(argv, cwd=None, timeout=TIMEOUT):
    """Run a probe. Returns (ok, first line of output or the reason it failed)."""
    try:
        proc = subprocess.run(argv, capture_output=True, text=True, timeout=timeout, cwd=cwd)
    except FileNotFoundError:
        return False, '%s is not on PATH' % argv[0]
    except OSError as exc:
        return False, '%s could not be run (%s)' % (argv[0], exc)
    except subprocess.TimeoutExpired:
        return False, '%s did not answer within %ds' % (argv[0], timeout)
    if proc.returncode != 0:
        detail = (proc.stderr or proc.stdout or '').strip().splitlines()
        return False, '%s exited %d%s' % (
            ' '.join(argv), proc.returncode, ' (%s)' % detail[0] if detail else '')
    out = (proc.stdout or '').strip()
    return True, out.splitlines()[0] if out else ''


def _git_paths(root):
    """(git-dir, git-common-dir) as absolute paths, or None when git cannot answer."""
    ok, out = _run(['git', '-C', root, 'rev-parse', '--path-format=absolute',
                    '--git-dir', '--git-common-dir'])
    if not ok:
        return None
    lines = out.splitlines()
    if len(lines) != 2:
        # One line means an older git ignored --path-format; re-ask without it.
        ok, git_dir = _run(['git', '-C', root, 'rev-parse', '--absolute-git-dir'])
        if not ok:
            return None
        ok, common = _run(['git', '-C', root, 'rev-parse', '--git-common-dir'])
        if not ok:
            return None
        if not os.path.isabs(common):
            common = os.path.join(root, common)
        return os.path.abspath(git_dir), os.path.abspath(common)
    return os.path.abspath(lines[0]), os.path.abspath(lines[1])


def main_checkout(root):
    """The main working tree, which is what `proof/run.sh` hangs the engine siblings off."""
    paths = _git_paths(root)
    if paths is None:
        return None
    return os.path.abspath(os.path.dirname(paths[1]))


def _engine(ctx, name, env_var, sibling):
    """Resolve one engine the way proof/run.sh does, and say whether it holds a module."""
    override = os.environ.get(env_var)
    if override:
        resolved, source = os.path.abspath(override), '$%s' % env_var
    else:
        main = main_checkout(ctx.root)
        if main is None:
            return FAIL, ('cannot locate the main checkout (git failed), so %s falls back to '
                          'nothing; set $%s to the %s checkout' % (name, env_var, name))
        resolved = os.path.abspath(os.path.join(main, os.pardir, sibling))
        source = 'sibling of %s' % main
    if os.path.isfile(os.path.join(resolved, 'go.mod')):
        return OK, '%s engine at %s (%s)' % (name, resolved, source)
    return FAIL, ('no go.mod at %s (%s), which is where proof/run.sh looks for the %s engine; '
                  'set $%s to a checkout' % (resolved, source, name, env_var))


def _missing(ctx, paths):
    return [p for p in paths if not os.path.exists(ctx.path(p))]


# --- checks --------------------------------------------------------------------------------
# Every check takes the context and returns (status, one-clause reason).
def check_repo_root(ctx):
    return OK, 'repository root at %s' % ctx.root


def check_git(ctx):
    ok, detail = _run(['git', '--version'])
    if ok:
        return OK, detail
    return FAIL, ('%s; provenance hashing and the tracked-file gates need git' % detail)


def check_tmp_dir(ctx):
    tmp = ctx.path('.tmp')
    if os.path.isdir(tmp):
        return OK, '.tmp/ exists'
    if os.path.exists(tmp):
        return FAIL, '%s exists but is not a directory' % tmp
    try:
        os.makedirs(tmp)
    except OSError as exc:
        return FAIL, 'could not create %s (%s)' % (tmp, exc)
    return OK, '.tmp/ (created)'


def check_worktree(ctx):
    paths = _git_paths(ctx.root)
    if paths is None:
        return WARN, 'git cannot say whether %s is a worktree' % ctx.root
    git_dir, common = paths
    if git_dir != common:
        return OK, 'linked worktree (git dir %s)' % git_dir
    return WARN, ('running in the root checkout; editing stages must run in a worktree '
                  '(`$PROJECT/.worktrees/<branch>`), while read-only work may stay here')


def check_spec(ctx):
    path = ctx.spec('instructions.md')
    if os.path.isfile(path):
        return OK, 'spec at spec/%s/instructions.md' % ctx.gear
    return FAIL, 'no spec at %s' % path


def check_go(ctx):
    ok, detail = _run(['go', 'version'])
    if ok:
        return OK, detail
    return FAIL, '%s; the proof cannot be run without a go toolchain' % detail


def check_sketch_engine(ctx):
    return _engine(ctx, 'sketch', 'SKETCH_DIR', 'sketch')


def check_decad_engine(ctx):
    return _engine(ctx, 'decad', 'DECAD_DIR', 'decad')


def check_proof_harness(ctx):
    wanted = (os.path.join('proof', 'run.sh'), os.path.join('proof', 'proofkit'),
              os.path.join('proof', 'proofkit3d'), os.path.join('proof', 'involute'))
    missing = _missing(ctx, wanted)
    if missing:
        return FAIL, 'the proof harness is incomplete: %s missing' % ', '.join(missing)
    return OK, 'proof harness present (run.sh, proofkit, proofkit3d, involute)'


def check_revision_pin(ctx):
    if os.environ.get('PROOF_VERIFY_REVISIONS') != '1':
        return OK, 'PROOF_VERIFY_REVISIONS is unset, so proof/run.sh pins no revision'
    unset = [v for v in ('SKETCH_COMMIT', 'DECAD_COMMIT') if not os.environ.get(v)]
    if unset:
        return FAIL, ('PROOF_VERIFY_REVISIONS=1 requires %s, which proof/run.sh reads with a `:?` '
                      'guard' % ' and '.join('$' + v for v in unset))
    return OK, 'revisions pinned to $SKETCH_COMMIT and $DECAD_COMMIT'


def check_model_tiers(ctx):
    """Record which model each role resolves to for this session.

    Informational only. An off-ladder default is legal (MODELS.md), so there is
    nothing here that can fail a stage; the row exists so a run's report says
    which tier its drafter ran on instead of leaving it to be reconstructed.
    """
    if not ctx.default_model:
        return SKIP, 'no --default-model given, so the tiers cannot be resolved'
    try:
        import pick_model
    except ImportError as exc:
        return SKIP, 'the sibling module pick_model could not be imported (%s)' % exc
    design, _ = pick_model.resolve('design', ctx.default_model)
    mechanical, reason = pick_model.resolve('mechanical', ctx.default_model)
    return OK, 'design=%s, mechanical=%s (%s)' % (design, mechanical, reason)


def check_api_db(ctx):
    try:
        import fusion_api
    except ImportError as exc:
        return FAIL, 'the sibling module fusion_api could not be imported (%s)' % exc
    # The module memoises the resolved script; preflight must report the environment as it is
    # now rather than as an earlier import in this process saw it.
    fusion_api._script = None
    try:
        return OK, 'API database query script at %s' % fusion_api.query_script()
    except fusion_api.Unavailable as exc:
        return FAIL, str(exc)


def check_steps(ctx):
    path = ctx.spec('steps.md')
    if os.path.isfile(path):
        return OK, 'step list at spec/%s/steps.md' % ctx.gear
    return FAIL, 'no step list at %s; run /compile-gear %s first' % (path, ctx.gear)


def _first_line_with(text, needle):
    """The first line holding `needle`, else the last nonempty line, else ''."""
    lines = [l.strip() for l in (text or '').splitlines() if l.strip()]
    for line in lines:
        if needle in line:
            return line
    return lines[-1] if lines else ''


def _run_check_compile(ctx):
    """Run the freshness gate the way `/emit-gear` used to run it by hand.

    `check_compile.py` resolves every path it reads against the working directory, so it must
    be started from the repo root. It parses the proof and queries the API database, which the
    10s probe timeout does not cover; this one is generous enough to fail rather than hang.
    Kept as its own function so a test can stand in for the whole subprocess."""
    return subprocess.run(
        [sys.executable, os.path.join(HERE, 'check_compile.py'), ctx.gear],
        capture_output=True, text=True, timeout=CHECK_COMPILE_TIMEOUT, cwd=ctx.root)


def check_steps_current(ctx):
    """Whether the step list still matches the spec it was compiled from.

    This is the one row that runs a real tool rather than a version probe. It stands where
    `/emit-gear` step 2 stood, and it costs the round nothing: that step ran the same command
    a moment later anyway."""
    if not os.path.isfile(ctx.spec('steps.md')):
        return SKIP, 'no step list, which the steps row already reports'

    try:
        proc = _run_check_compile(ctx)
    except subprocess.TimeoutExpired:
        return FAIL, ('check_compile.py did not finish within %ds; run it by hand to see where it '
                      'stopped' % CHECK_COMPILE_TIMEOUT)
    except OSError as exc:
        return FAIL, 'check_compile.py could not be run (%s)' % exc

    if proc.returncode == 0:
        return OK, 'check_compile.py: step list is current'
    if proc.returncode == 1:
        return FAIL, ('check_compile.py found BLOCKING findings (%s); run /compile-gear %s first'
                      % (_first_line_with(proc.stdout, 'BLOCKING'), ctx.gear))
    if proc.returncode == 2:
        return FAIL, ('check_compile.py exit 2 (%s): an input is missing or unreadable'
                      % _first_line_with(proc.stderr or proc.stdout, ''))
    return FAIL, ('check_compile.py exited %d (%s), which is not one of its documented codes'
                  % (proc.returncode, _first_line_with(proc.stderr or proc.stdout, '')))


def check_proof_dir(ctx):
    path = ctx.path('proof', ctx.gear)
    if not os.path.isdir(path):
        return FAIL, 'no proof at %s; run /compile-gear %s first' % (path, ctx.gear)
    sources = [n for n in sorted(os.listdir(path)) if n.endswith('.go')]
    if not sources:
        return FAIL, '%s holds no .go file, so there is no proof to read' % path
    return OK, 'proof/%s/ holds %d .go file(s)' % (ctx.gear, len(sources))


def check_contract_manifest(ctx):
    path = ctx.spec('contract.json')
    if os.path.isfile(path):
        return OK, 'contract manifest at spec/%s/contract.json' % ctx.gear
    return SKIP, 'no spec/%s/contract.json, so the contract gate will be prose-checked' % ctx.gear


def check_framework(ctx):
    missing = _missing(ctx, FRAMEWORK)
    if missing:
        return FAIL, 'the drafter must read these, and they are missing: %s' % ', '.join(missing)
    return OK, 'framework sources present (%s)' % ', '.join(os.path.basename(p.rstrip(os.sep))
                                                            for p in FRAMEWORK)


def check_pyright(ctx):
    if importlib.util.find_spec('pyright') is not None:
        return OK, 'pyright is importable'
    return WARN, ('pyright is not installed; install it with '
                  '`python3 -m pip install --break-system-packages pyright`')


def check_stubs(ctx):
    try:
        import fusion_stubs
    except ImportError as exc:
        return FAIL, 'the sibling module fusion_stubs could not be imported (%s)' % exc
    env = os.environ.get('FUSION_API_STUBS')
    if env:
        # $FUSION_API_STUBS is authoritative for pyright_check.py: a wrong path there is an
        # exit 2, never a fallback, so it is a failure here too.
        defs = fusion_stubs.defs_at(env)
        if defs:
            return OK, 'Fusion API stubs at %s ($FUSION_API_STUBS)' % defs
        return FAIL, ('$FUSION_API_STUBS=%s has no adsk/core.py, and pyright_check.py will exit 2 '
                      'on it rather than fall back' % env)
    cached = fusion_stubs.defs_at(fusion_stubs.cache_repo_dir())
    if cached:
        return OK, 'Fusion API stubs cached at %s' % cached
    return WARN, ('Fusion API stubs are not cached; pyright_check.py will auto-clone them '
                  '(~13M) on its first run')


def check_sketch_bench(ctx):
    path = ctx.spec('sketch', 'run.sh')
    if os.path.isfile(path):
        return OK, 'sketch bench at spec/%s/sketch/run.sh' % ctx.gear
    return WARN, ('no sketch bench at %s; the skill builds one from the spec\'s recipes, so this '
                  'is work rather than a broken environment' % path)


def has_sketch_bench(ctx):
    return os.path.isfile(ctx.spec('sketch', 'run.sh'))


# --- the stage matrix ----------------------------------------------------------------------
# (key, check, downgrade) per stage. `downgrade` says whether a FAIL from the check is only a
# warning for this stage; it is a bool, or a predicate on the context for the cases where the
# stage's own state decides (the generate stage needs an engine only once a bench exists).
# Checks that grade themselves (stubs) are never downgraded.
COMMON = (
    ('repo-root', check_repo_root, False),
    ('git', check_git, False),
    ('tmp-dir', check_tmp_dir, False),
    ('worktree', check_worktree, False),
    ('model-tiers', check_model_tiers, False),
)

STAGES = {
    'compile': COMMON + (
        ('spec', check_spec, False),
        ('go', check_go, False),
        ('sketch-engine', check_sketch_engine, False),
        ('decad-engine', check_decad_engine, False),
        ('proof-harness', check_proof_harness, False),
        ('revision-pin', check_revision_pin, False),
        ('api-db', check_api_db, False),
    ),
    'emit': COMMON + (
        ('steps', check_steps, False),
        ('proof-dir', check_proof_dir, False),
        ('steps-current', check_steps_current, False),
        ('api-db', check_api_db, False),
        ('contract-manifest', check_contract_manifest, False),
        ('framework', check_framework, False),
        ('pyright', check_pyright, False),
        ('stubs', check_stubs, False),
    ),
    'generate': COMMON + (
        ('spec', check_spec, False),
        ('sketch-bench', check_sketch_bench, False),
        ('sketch-engine', check_sketch_engine, lambda ctx: not has_sketch_bench(ctx)),
        ('go', check_go, lambda ctx: not has_sketch_bench(ctx)),
        ('framework', check_framework, False),
        ('pyright', check_pyright, False),
        ('stubs', check_stubs, False),
        # The database is the documented fallback for this stage's API questions, not an input
        # to a gate, so its absence costs accuracy rather than the run.
        ('api-db', check_api_db, True),
    ),
}

STAGE_ORDER = ('compile', 'emit', 'generate')


def plan(stage):
    """The checks to run, in order. `all` is the union, each key once, graded as strictly as
    any stage that asks for it."""
    if stage != 'all':
        return list(STAGES[stage])
    merged = []
    index = {}
    for name in STAGE_ORDER:
        for key, check, downgrade in STAGES[name]:
            if key in index:
                # A key any stage grades strictly is graded strictly for the union.
                position = index[key]
                if merged[position][2] is not False and downgrade is False:
                    merged[position] = (key, check, False)
                continue
            index[key] = len(merged)
            merged.append((key, check, downgrade))
    return merged


def run_checks(ctx, stage):
    """Every check for the stage, as [{'key', 'status', 'detail'}]."""
    results = []
    for key, check, downgrade in plan(stage):
        status, detail = check(ctx)
        if status == FAIL and (downgrade(ctx) if callable(downgrade) else downgrade):
            status = WARN
        results.append({'key': key, 'status': status, 'detail': detail})
    return results


# --- reporting -----------------------------------------------------------------------------
MARK = {OK: '[ok]', WARN: '[warn]', FAIL: '[FAIL]', SKIP: '[skip]'}


def failures(results):
    return [r for r in results if r['status'] == FAIL]


def render_text(gear, stage, results):
    lines = ['%-6s %s: %s' % (MARK[r['status']], r['key'], r['detail']) for r in results]
    broken = failures(results)
    if broken:
        lines.append('preflight %s (%s): NOT READY (%d failure%s)'
                     % (gear, stage, len(broken), '' if len(broken) == 1 else 's'))
    else:
        lines.append('preflight %s (%s): READY' % (gear, stage))
    return '\n'.join(lines)


def render_json(gear, stage, results):
    return json.dumps({'gear': gear, 'stage': stage, 'ready': not failures(results),
                       'checks': results})


# --- entry point ---------------------------------------------------------------------------
def repo_root():
    """Three levels above this file, the same rule run_gates.repo_root() uses."""
    return os.path.abspath(os.path.join(HERE, os.pardir, os.pardir, os.pardir))


def parse_args(argv):
    parser = argparse.ArgumentParser(
        prog='preflight.py',
        description='Check that this environment can run a gear stage, before the round starts.')
    parser.add_argument('gear')
    parser.add_argument('--stage', choices=['compile', 'emit', 'generate', 'all'], default='all')
    parser.add_argument('--root', default=None)
    parser.add_argument('--format', choices=['text', 'json'], default='text')
    parser.add_argument(
        '--default-model', default=None, metavar='MODEL',
        help="the session's default model; makes the model-tiers row report which model each "
             'role resolves to (see MODELS.md)')
    return parser.parse_args(argv)


def resolve_root(root_arg):
    root = os.path.abspath(root_arg) if root_arg else repo_root()
    missing = [m for m in ROOT_MARKERS if not os.path.isdir(os.path.join(root, m))]
    if missing:
        raise Usage('%s is not the repository root: %s missing' % (root, ', '.join(missing)))
    return root


def main(argv):
    args = parse_args(argv)
    if not GEAR_NAME.match(args.gear):
        raise Usage("'%s' is not a gear name; expected %s" % (args.gear, GEAR_NAME.pattern))
    ctx = Context(resolve_root(args.root), args.gear, args.default_model)
    results = run_checks(ctx, args.stage)
    render = render_json if args.format == 'json' else render_text
    print(render(args.gear, args.stage, results))
    return 1 if failures(results) else 0


def cli(argv):
    """main() plus the usage-error mapping, so the exit contract is testable as one call."""
    try:
        return main(argv)
    except Usage as error:
        sys.stderr.write('preflight.py: %s\n' % error)
        return 2


if __name__ == '__main__':
    sys.exit(cli(sys.argv[1:]))
