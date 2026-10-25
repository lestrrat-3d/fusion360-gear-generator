# Gallery port pipeline

Every gear add-in under `lib/geargen/` is generated from a model in
[decad-gallery](https://github.com/lestrrat-3d/decad-gallery). The gallery builds each gear as Go code on top of
[decad](https://github.com/lestrrat-3d/decad) and [sketch](https://github.com/lestrrat-3d/sketch) and proves its
geometry in its own repository. This repository translates that Go code into Fusion 360 Python and holds what the
gallery cannot say: the dialog, the Fusion runtime and what Fusion taught us.

`docs/gallery-translation-goal.md` owns the goal, the translation rules and the definition of done. `docs/fusion/`
holds the Fusion findings. This file describes how the pipeline works and where it stops.

## Layout

| Path | Role |
| --- | --- |
| `port/go.mod` | Pins the gallery, decad and sketch revisions as pseudo-versions. `go get` is the only thing that moves a pin. |
| `port/gallery` | Reads the pinned gallery source: `Params`, defaults, the parameter table, the operations a build performs, and a digest of named functions. |
| `port/go2py` | Translates gallery Go into Python. `api.go` is the table of operations the translator accepts. |
| `port/fusionexpr` | Rewrites a gallery parameter expression into a Fusion expression. It refuses a function it has no rule for. |
| `port/<gear>/` | One package per gear (`spur`, `helical`, `herringbone`, `cycloidal`, `bevel`): the mapping from gallery names to Fusion names and dialog inputs (`mapping.go`), the generator, and `reviewed.json`, which records what the gallery's build looked like when the translation was reviewed. |
| `port/<gear>/fragments/` | The stored Python that wraps the translation: dialog, command handlers, selections and anchor. It holds marker lines where generated code goes. |
| `port/cmd/gearport` | The command. |
| `lib/geargen/decadfusion.py` | The Fusion implementation of the API table. It stands in for the gallery's Document, World, Sketch and Body. Written by hand. |
| `lib/geargen/<gear>.py` | Build output. The next `gearport gen` overwrites it. |

## Commands

Build once with `go build -C port -o ../.tmp/gearport ./cmd/gearport`, then run from the repository root:

| Command | Effect |
| --- | --- |
| `gearport gen <gear>` | Writes `lib/geargen/<gear>.py`. |
| `gearport check <gear>` | Exits non-zero when that file differs from a fresh generation. |
| `gearport accept <gear>` | Records the digest of the gallery functions the fragment was reviewed against. |
| `gearport id <gear>` | Prints the build id the file's contents carry. |

The gears are `spurgear`, `helicalgear`, `herringbonegear`, `bevelgear` and `cycloidal`.

Every generated file starts with a header naming the gallery revision and carries `GEAR_BUILD_ID`, the first 12 hex
digits of the SHA-256 of the file without its three stamp lines. Fusion logs `<gear> build <id>` when it imports the
module, so a match with `gearport id` shows that Fusion ran that file.

## How a gear is generated

1. The gallery reader loads the pinned model package.
2. The generator fills the marker lines of the stored fragment: dialog inputs with the gallery defaults, the
   parameter names, and the translated Python.
3. `port/go2py` translates the gallery functions the gear's `Spec` lists (`FeatureSpec` in the gear's `mapping.go`).
   The `Spec` names the structs to render, the functions to translate whole, the drawing steps to translate up to a
   stop call, package values, and the points where the Fusion runtime replaces gallery code.
4. Each call of `decad`, `sketch`, `r3`, `units` or `sketchcheck` in the translated code maps to an entry of the API
   table, which `decadfusion.py` implements. `port/spur/runtime_test.go` fails when the table has an entry the module
   does not define.
5. Each call that creates a Fusion object needs a role name from the gear's `Roles` table, so every sketch, plane,
   feature and body is named by what it is for (see `CLAUDE.md`, "Naming").

What the translator does with Go that Python reads differently: it drops error checks, because the runtime raises
instead; it drops the context argument; it folds branches on an unset options struct; it copies a struct that the
translated code assigns field by field (`Mutable`); and it reads a missing map key as the zero value. A call of
anything not in the API table stops the port. Nothing is skipped silently.

A function the runtime implements instead of the translator (`Bound`), a function of another gallery package
(`External`), a function the add-in does itself (`Provided`), and a retry loop that runs once (`FirstOnly`, see
`docs/gallery-translation-goal.md`) are the only places where gallery code is not translated. One gallery error value,
`decad.ErrDegenerate`, is translated as an exception (`Catch`): a split sheet that misses the tooth is a skip in the
gallery, so the runtime raises `decaderrors.Degenerate` and the translated `errors.Is` becomes `isinstance`. Every other
`errors.Is` is false.

The translator refuses Go it would turn into different Python: a redeclared name that shadows one read again, mixed
integer and float literals, float loop variables, a loop condition the body changes, `%` on operands that may be
negative (Go truncates, Python floors), a `break` inside a `switch` in a loop, copies of structs it cannot copy, and
`Sprintf` verbs it does not render as Go does. One known difference remains: Python's `math.acos`, `math.sqrt` and
division raise where Go returns NaN or Inf.

### What stops `gearport gen`

- A gallery `Params` field that is in neither the gear's inputs nor its gallery-only list.
- A parameter table entry the mapping does not list.
- A gallery call, option or struct the translator has no rule for.
- A creating call with no role name.
- A `Roles`, `FirstOnly`, `Bound`, `External`, `Provided` or `Catch` entry that the translation never uses.
- A change in the ordered list of creating calls of a translated function, or in the code of the functions a gear
  reviews by digest. Both are recorded in `reviewed.json`. A person reviews the translation, then runs
  `gearport accept <gear>`. Because each entry carries the call's text, two swapped calls of one operation stop the port
  too, so role names cannot swap silently.
- A gallery module that is not in the Go module cache and cannot be downloaded, or one that a `replace` directive
  points at a local checkout.

## The Fusion runtime

`lib/geargen/decadfusion.py` implements the API table on Fusion's API. These parts matter when reading a failure:

- A `Runtime(component, plane, anchor, parameters, sketch_only, scale)` is one build. The gallery's coordinates are in
  the build's local frame; `to_world` and `to_local` convert. Spur, helical and herringbone run in centimetres;
  cycloidal and bevel run in millimetres with a scale of 0.1.
- Each sketch records the gallery's calls and draws them in the order Fusion needs (`[PB-SPLINE-RING]`,
  `[PB-DIMENSION-COST]`), then anchors its local origin. A sketch on a plane across the base plane gets an axis map
  (`[PB-FRAME-PLANE]`).
- A gallery container becomes a Fusion component made after the build (`[PB-CONTAINER-BODIES]`).
- A union and a rotated copy are recorded and made as one combine and one circular pattern when something first reads
  the body (`[PB-LAZY-JOIN]`).
- A model that only reads its parameters back keeps them in a Python table (`[PB-PARAM-TABLE]`). The bevel gear does
  this and registers no Fusion user parameter. The other gears register user parameters from the gallery table.
- A stale module guard compares the runtime's source hash with the file on disk, and the command reloads the runtime
  when they differ (`[PB-STALE-MODULE]`). The error dialog shows the build line. A running build writes a progress
  file (`[PB-PROGRESS-FILE]`).

## Checks

`.github/workflows/checks.yml` runs on every pull request and on pushes to `main`:

- `port` runs `go mod tidy` (and fails on a diff), `go vet`, golangci-lint and `go test ./...` in `port/`, then
  `scripts/check-generated-gears.sh`, which names every gear whose file differs from a fresh generation. A pin bump with no
  regeneration, a hand edit of a generated file, a stale fragment and a translator change all fail here.
- `addins` runs `python3 -m unittest discover -s tests` and `scripts/check_fusion_api.py`. The tests compile and import
  the add-in, its entry file and the shipped diag scripts with the Fusion modules stubbed, require every `_fx.<name>` a
  generated file uses to exist in the runtime, fail on an unnamed sketch, plane, feature or body, run the translated
  bevel build against a fake Fusion, and test the runtime's bookkeeping (containers, lazy joins and copies, split, scale,
  planes, units). The API check runs pyright against the FusionAPIReference stubs, pinned in the workflow, and fails on an
  unknown Fusion member, a wrong argument count or an undefined name. `scripts/fusion-api-allow.txt` lists the retired
  members the code still uses, which work in Fusion today. `scripts/check-deploy.sh` runs `deploy.sh` into a temporary
  folder, compiles what it copied and fails if it shipped a test, a Go file or a document.

The gallery proves each gear's geometry. This repository does not repeat it.

What nothing checks: the degrees of freedom of the add-in's own sketches, the Fusion API calls on values whose type
pyright cannot see, and the numeric agreement of a Fusion body with the gallery body. Only a person can run Fusion, so
the Fusion load stays a manual step. Its findings go where `CLAUDE.md`, "When Fusion gives a verdict", says.

Bumping a pin is its own change, and the Fusion load decides whether the bump is accepted.

## Gears

| Gear | Translated | Fusion check |
| --- | --- | --- |
| Spur | Whole build, with the root fillet and the embedded tooth | Merged in PRs #196 and #197 |
| Helical | The spur pipeline with its own tooth loft and root fillet | Merged in PRs #196 and #197 |
| Herringbone | The helical pipeline with a mirrored half | Merged in PRs #196 and #197 |
| Cycloidal | Whole build, one lobe and a circular pattern | Default drive loaded and confirmed by the user (PR #198); not checked with two discs and a disc gap of zero |
| Bevel | Straight and spiral pairs, one Tooth Type drop-down | Straight pair and the default spiral pair loaded and confirmed by the user (PR #199) |

The screw gear has no gallery model and is not part of the port.

### Where the add-in differs from the gallery

- Cycloidal: the drive is grouped into sub-components, which the gallery does not have. The dialog checks the inputs
  as you type with the gallery's own `Problems` function, translated.
- Spur, helical and herringbone: the gallery's own checks (`validate`, `ValidateHelixAngle`) run, translated, before
  anything is built, so a bad input raises the gallery's message. The dialog has no live check for them, because they
  need the derived parameters.
- Bevel: the dialog has a Tooth Type drop-down that the gallery does not have. Straight is the default and builds
  with a spiral angle of zero; Spiral shows Mean Spiral Angle, Hand of Spiral and Cutter Radius. The gallery default is
  also straight, and the dialog starts Mean Spiral Angle at 35° because a spiral pair needs an angle above zero. The
  add-in places the pair on a Target Plane at a Center Point (`[PB-PAIR-PLANE]`) and leaves the gallery's `Placement`
  empty.
- Bevel: the gallery retries a spiral build with other overshoots and join orders when decad refuses a boolean. The
  translation runs the first try only, because Fusion's kernel is exact (`docs/gallery-translation-goal.md`).
- All gears: a failed generation deletes the new component and the user parameters it registered (`lib/geargen/base.py`)
  and reports the error in a dialog.
