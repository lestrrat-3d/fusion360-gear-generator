# Early documentation-query environment evidence

This records one observed drafting invocation for fusion360-gear-generator from this agent's tool transcript.
It does not rerun the command or infer its inherited environment.

## Observed invocation

The exact `tools.exec_command` command string was:

```text
go doc github.com/lestrrat-3d/sketch.Sketch.CreateSpline; go doc github.com/lestrrat-3d/sketch.Sketch.CreateArc; go doc github.com/lestrrat-3d/sketch.Profile; go doc github.com/lestrrat-3d/sketch.Sketch.AddConstraint
```

The explicitly supplied working directory was:

```text
/home/lestrrat/dev/src/github.com/lestrrat-3d/sketch
```

The other explicitly supplied argument was `max_output_tokens: 4000`.
The invocation did not supply shell, login, sandbox-permission, or environment overrides.

The tool response reported `chunk_id: df062a`, `exit_code: 0`,
`wall_time_seconds: 1.398618278`, and `original_token_count: 1243`.
It returned no ongoing session identifier.
The reported exit code belongs to the combined shell command; separate exit codes for its first three
`go doc` processes were not captured.

Absolute start and end timestamps were not included in the available tool response.
They are unavailable here and have not been reconstructed from duration or conversation order.

## Original output storage

The original output was returned directly in the tool transcript under chunk `df062a`.
The invocation did not redirect output to a file, and no original output/log path was recorded.
The text below transcribes that captured output; it is not a new command result.

```text
package sketch // import "github.com/lestrrat-3d/sketch"

func (s *Sketch) CreateSpline(control ...*Point) (*Spline, error)
    CreateSpline adds a cubic B-spline over the given control points and returns
    its handle. Share control points with other geometry to relate them.
    It returns ErrInvalidShape with fewer than 4 control points.

package sketch // import "github.com/lestrrat-3d/sketch"

func (s *Sketch) CreateArc(center, start, end *Point) *Arc
    CreateArc adds an arc swept counter-clockwise from start to end about
    center, and the internal radius-consistency constraint. Returns its handle.

package sketch // import "github.com/lestrrat-3d/sketch"

type Profile struct {
	// Entities is the de-duplicated set of distinct sketch entities on the
	// OUTER boundary, in first-seen walk order. For a boundary with no bare
	// crossings this is the historical contract unchanged (a rectangle is four
	// lines, a circle is one circle); a curve split at a crossing appears once.
	Entities []Entity
	// Outer is the ordered outer-boundary edge loop, counter-clockwise. Each
	// edge is a whole entity or a fragment of one.
	Outer []BoundaryEdge
	// Holes are inner boundary loops (each clockwise), nil when the region is
	// simply connected. A hole is a void in this region — a separate region may
	// also occupy it.
	Holes [][]BoundaryEdge
	// Area is the net region area (outer minus holes) in base units (mm²),
	// >= 0 for a clean region, 0 for a degenerate one.
	Area float64
	// Valid is false when the region cannot be trusted as an extrudable profile:
	// a self-intersecting or zero-area boundary, or an unresolvable (degenerate)
	// arrangement condition that REACHES this region — one involving a curve its own
	// boundary is built from, or one no curve could be blamed for at all.
	//
	// An ATTRIBUTABLE condition, on curves this region's boundary does not use,
	// leaves this region valid, so a sketch can hold both valid and invalid
	// profiles. An unattributable one has no such reach limit and invalidates every
	// region detected. Whether the sketch as a whole is verifiable is a different
	// question, answered by [VerificationReport.ProfilesValid], which also covers a
	// condition that produced no region to report.
	Valid bool
	// SelfIntersecting marks the specific invalidity that the boundary the
	// region derives from crosses or touches itself.
	SelfIntersecting bool

	// Has unexported fields.
}
    Profile is a closed planar region detected in a sketch. Its outer boundary
    is an ordered loop of edges; each edge is a whole sketch entity or a
    fragment of one produced by splitting at a bare crossing. A region may
    enclose holes. A single closed primitive (circle or ellipse) is a region on
    its own.

    Profiles are what downstream operations (eventually extrude/revolve)
    consume, so an Entity's shape, closure, area and validity are observable
    here.

func (p *Profile) IsStale() bool
func (p *Profile) Revision() uint64
func (p *Profile) Sketch() *Sketch
package sketch // import "github.com/lestrrat-3d/sketch"

func (s *Sketch) AddConstraint(cs ...Constraint)
    AddConstraint commits one or more constraints to the sketch. Constraints
    reference solver-bound geometry (the Point/Line/Circle handles returned
    by the Add methods), which is therefore already committed. Dimensional
    constraints created from a bare float adopt the sketch's default unit for
    their kind here.

    A constraint referencing another sketch's geometry is committed as written
    but never parameterized (see foreignConstraint), so committing it cannot
    corrupt the sketch that owns the geometry. This sketch then reports it
    exactly as before: Sketch.Verify flags ForeignHandles and Sketch.MarshalJSON
    refuses to write it.

    A constraint still HOLDING auxiliary solver variables another
    sketch allocated is instead ignored entirely — not committed (see
    foreignAllocation). Its indices address that sketch's variable vector,
    so an appended row would read across sketches at every residual call.

    A constraint that merely records such a sketch while holding no live
    allocation — a driven dimension owns no auxiliary variable — is committed
    normally, since it addresses no other vector.

    A nil candidate, or a typed nil (a nil pointer of a concrete constraint
    type boxed in the interface), is DROPPED rather than committed — it
    names no geometry, so no report could describe it, and committing it
    would panic every later pass that reads it (residuals, Solve, Verify,
    Diagnose, RedundantConstraints) far from the call that made the mistake.
    A live constraint holding a nil point or entity operand IS still committed,
    the same treatment a reference-foreign constraint gets, so Sketch.Verify
    stays loud about it; only its resolveUnit/allocVars hooks are skipped,
    since they dereference the operands' coordinates.

```

## Recorded environment not explicitly applied

The invocation did not load `.tmp/t8/environment.json` or explicitly apply any of its seven settings:

```json
{
  "GOTOOLCHAIN": "go1.26.8",
  "GOMAXPROCS": "8",
  "GOCACHE": "/home/lestrrat/dev/src/github.com/lestrrat-3d/fusion360-gear-generator/.worktrees/docs-prose-aligned-measurements/.tmp/t8-cache/candidate/go-build",
  "TMPDIR": "/home/lestrrat/dev/src/github.com/lestrrat-3d/fusion360-gear-generator/.worktrees/docs-prose-aligned-measurements/.tmp/t8-cache/candidate/tmp",
  "GIT_CEILING_DIRECTORIES": "/home/lestrrat/dev/src/github.com/lestrrat-3d/fusion360-gear-generator/.worktrees/docs-prose-aligned-measurements/.tmp/t8-cache/candidate/tmp",
  "SKETCH_DIR": "/home/lestrrat/dev/src/github.com/lestrrat-3d/sketch",
  "DECAD_DIR": "/home/lestrrat/dev/src/github.com/lestrrat-3d/decad/.worktrees/chore-gear-pipeline-pin"
}
```

These are the recorded intended values, not observed values from that process.
The explicitly supplied working directory matches the recorded SKETCH_DIR path, but the invocation
did not set the SKETCH_DIR variable.
The inherited values of all seven variables were not captured; they could have matched some recorded
values, differed, or been unset. The transcript cannot distinguish those possibilities.

## Unknown effects and clarification scope

The invocation did not report its selected Go executable or effective toolchain version.
It did not report effective cache or temporary-directory paths, cache hits, cache writes, or created files.
No before/after cache snapshot or tracing result is available for it.
Consequently, this evidence does not establish either cache modification or absence of cache modification,
and it does not establish whether later measurement caches were affected.

This clarification only read the shell rules and recorded environment file, then wrote this report.
It did not edit the generated drafts, run a new Go command, or run a gate.
