The proof files are `proof/screwgear/sketches_test.go`, `proof/screwgear/construction_test.go`, `proof/screwgear/solids_test.go`, `proof/screwgear/channels_test.go`, `proof/screwgear/zz_registrations_test.go`.

<!-- step-metadata: 2 -->

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/screwgear/instructions.md` | `65c0a702310ed893f36a1af3ce1d56f25872c32f` |
| `spec/screwgear/fusion.md` | `44259a5d084b537745c4d168ba9fd734e3f4b4fb` |
| `CLAUDE.md` | `916e8624ca88af226c264c21f295c14a9fb9e901` |
| `proof/screwgear/README.md` | `4407c8cc550abd64b4096efe9198abe58b39b560` |
| `spec/cycloidal/fusion.md` | `afa5a99986f2e0d9f82fb5e21591553cdc54aac4` |
| `spec/screwgear/mesh-search.md` | `245bc5c833387a83598ee6c8a7971e8efd5825be` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `cdd32545b0f8c651752827c6697601f1e32b4d39` |

## S01 `[PROSE]` Create the Screw Gearing occurrence and configure the command

Carry all dialog values, selection entities, range checks, searches and method contracts before creating geometry.
The inherited method `getOccurrence` creates the top occurrence under `self.parentComponent`.
Name it Screw Gearing. No engine model exists for command dialogs or Fusion occurrence ownership.
[PB-DIALOG-DEFAULT-UNITS] [PB-SELECTION-DECL] [PB-SELECTION-FILTER-ENUM]
[PB-AUTOFOCUS-FIRST] [PB-SELECTION-STASH] [PB-EVAL-EXPRESSION] [PB-PRECOMPUTED-MODE]
[PB-OCCURRENCE-TREE] [PB-NEVER-ACTIVATE] [PB-NO-CROSS-SIBLING] [PB-USE-SELECTED-PLANE]
Compute Lambda=TwistLead/(2*pi), A=W-Engagement, L=N*P, and Beta=atan(pi*W/TwistLead).
Phi_g is that gear’s mounting angle in radians, and Z0 is 0 for A or assemblyPhase for B.
Both gears have the same positive screw hand. The toothed edge is the leaned cosine of the Cell Sections step.
The framework calls the configurator method configure and generator method generate; prefixBase returns ScrewGear.
Preserve every named method in the call graph below. No additional GenerationContext class is defined.

### Architecture

The module lib/geargen/screwgear.py defines exactly these public classes (the command wiring binds
to them by name; exported via lib/geargen/__init__.py):

- **ScrewGearCommandInputsConfigurator** — classmethod configure(cls, command) adds the dialog
  inputs in the order and groups of "Variables" below. No conditional visibility.
- **ScrewGearGenerator(base.Generator)** — 1-arg constructor (design) (inherited); implements
  generate(self, inputs) and the call graph below; relies on inherited deleteComponent() for
  error cleanup. Overrides prefixBase() to return 'ScrewGear'.

**Generation Context: none.** Carry handles on self: self.designOcc, self.gearOccs (list of
two), self.cageOcc, self.gearBodies (list of two), self.cageBody, self.pathLines (list of
two dicts, one per gear, keyed 'bore-' and 'bore+', each the sketch line of §1 that one bore's
sweep of §4 runs along), and self.windows (the windows the window search of §4 finds room for,
zero to two, each its facing direction and its corners).

**Dependencies: none.** Imports only the framework (base, misc, utilities, solids,
fusion360utils). Use solids.hide_construction_geometry(component) for the final cleanup — do
NOT re-implement it.

**Entry wiring:** commands/screwgear/entry.py constructs `GearCommand(gear_type='ScrewGear',
name='Screw Gear Generator', …)`, binding the two classes above by name (PLAYBOOK.md
"Command-entry wiring").

**Parameter mode: all-Python-precomputed** ([PB-PRECOMPUTED-MODE]). Every value is computed in
Python in internal cm and written numerically; the generator registers **no** named user parameters.
The trigonometry here (per-station rotation angles, the screw step matrix) has no useful expression
form in the parameter table. The two searches of §4 that run before any feature — the check on
the wall between the bores and the window search — work in millimetres, with the step sizes §4
states in millimetres, and every length they hand on is divided by 10.

### Component Setup

One command invocation creates a **Screw Gearing** component under the user-selected Parent
Component, holding four sub-components ([PB-OCCURRENCE-TREE]). The top occurrence is the
inherited self.getOccurrence(), which creates it under self.parentComponent and is what the
inherited deleteComponent() deletes on failure; the build names its component and never calls
addNewComponent for it itself. The four children are each
occurrences.addNewComponent(adsk.core.Matrix3D.create()) under that component:

- **Design** — every sketch, construction plane and feature runs here.
- **Gear A**, **Gear B**, **Cage** — empty until the end, when the finished bodies are
  relocated into them with body.moveToComponent ([PB-NO-CROSS-SIBLING]).

**NEVER call occurrence.activate()** ([PB-NEVER-ACTIVATE]). Place sketches directly on the
user-selected plane ([PB-USE-SELECTED-PLANE]).

The user selects a **plane** and a **point**. The plane's normal is the cage axis and the common
perpendicular n̂; the point is the mechanism's centre C.

### Variables

User inputs in dialog order: first by importance, then in named groups. All linear inputs are
mm; the tooth slant and the crossing and mounting angles are degrees; the tooth bow is a bare
number in mm⁻¹.

The frame's inputs keep the names the video frame gave them, and mean this for the sleeve:
cageRadius is where each bore sits on its gear's own axis and the middle of the sleeve's wall;
cageRise is half the sleeve's height, to its flat end faces; collarHalf is half the wall's
thickness, so the sleeve runs from radius cageRadius - collarHalf to cageRadius + collarHalf;
collarWall is the least material the build leaves round every bore and every window, at the
end faces, between two bores and beside a window; clearance is added all round the bore's
rectangle; roofAllowance is added on one face of one bore of each gear, the roof the printer
bridges (§4, "The roof allowance"). The video frame's ringRadius, ringWire and rodDiameter are gone with its ring,
loop and rods.

The dialog opens with where the mechanism goes, because nothing can be built without it and
Fusion focuses the first selection input ([PB-AUTOFOCUS-FIRST]). Then three groups, each a
GroupCommandInput made with command.commandInputs.addGroupCommandInput(groupId, groupLabel),
whose inputs are added to that group's children collection instead of the top-level
commandInputs. The groups run from what a user changes most to what they should rarely touch:
the ribbon's size, then the frame, then the mesh arrangement, whose values come from the meshing
search (mesh-search.md) and jam when changed carelessly, so that group starts collapsed
(isExpanded = False); the other two start expanded. Within a group the rows run from most to
least important, as listed.

| Group (group id, label) | Dialog label | input id | unit | default |
|---|---|---|---|---|
| none (top level) | Target Plane | plane | selection | — |
| none (top level) | Centre Point | point | selection | — |
| none (top level) | Parent Component | parent | selection | — |
| ribbonGroup, Ribbon | Ribbon Width | ribbonWidth | mm | 15 |
| ribbonGroup, Ribbon | Tooth Count | toothCount | — | 68 |
| ribbonGroup, Ribbon | Twist Lead | twistLead | mm | 49.5 |
| ribbonGroup, Ribbon | Ribbon Thickness | ribbonThickness | mm | 3.75 |
| ribbonGroup, Ribbon | Tooth Pitch | toothPitch | mm | 2.625 |
| ribbonGroup, Ribbon | Tooth Height | toothHeight | mm | 2.625 |
| ribbonGroup, Ribbon | Tooth Slant | toothSlant | deg | 25.8 |
| ribbonGroup, Ribbon | Tooth Bow | toothBow | — (mm⁻¹) | 0.048 |
| frameGroup, Frame | Cage Radius | cageRadius | mm | 15 |
| frameGroup, Frame | Cage Rise | cageRise | mm | 18.75 |
| frameGroup, Frame | Clearance | clearance | mm | 0.20 |
| frameGroup, Frame | Roof Allowance | roofAllowance | mm | 0.30 |
| frameGroup, Frame | Collar Half Length | collarHalf | mm | 3 |
| frameGroup, Frame | Collar Wall | collarWall | mm | 3 |
| meshGroup, Mesh (from the mesh search) | Crossing Angle | crossAngle | deg | 80 |
| meshGroup, Mesh (from the mesh search) | Engagement | engagement | mm | 1.05 |
| meshGroup, Mesh (from the mesh search) | Mounting Angle A | mountAngleA | deg | 0 |
| meshGroup, Mesh (from the mesh search) | Mounting Angle B | mountAngleB | deg | 0 |
| meshGroup, Mesh (from the mesh search) | Assembly Phase | assemblyPhase | mm | −1.31 |

Every input is read back by id with inputs.itemById(id) on the command's top-level
commandInputs, grouped or not; input ids are unique across the whole command, which is what
lets that lookup reach into a group. processInputs raises naming the id if any lookup returns
None, so a lookup that does not reach into a group fails at once and by name.

Module-level constants for every input id: INPUT_ID_RIBBON_WIDTH = 'ribbonWidth' and so on.
Two more module-level constants are not dialog inputs. TOOTH_SPLINE_POINTS = 11 is how many
points each section's toothed side is fitted through (§2). CELL_TEETH = 4 is the number of teeth
the lofted cell of §2 holds, written cellTeeth in this spec. Every count that depends on it is
derived from it in processInputs, and this spec states those counts for cellTeeth = 4 and,
as the fallback the first cell measurement was made at, for cellTeeth = 1.
Value inputs are addValueInput(id, label, unit, ValueInput.createByReal(default)) with the
default in internal units ([PB-DIALOG-DEFAULT-UNITS]): mm/10 for a length, radians for an
angle, the bare count for toothCount and the bare number for toothBow. toothBow has the unit
'', as toothCount does, and is read the same way; its value is in mm⁻¹, and the build, which
works in cm, lowers the edge by 10*toothBow*v^2 cm for v in cm. The two tooth inputs sit in
the Ribbon group after Tooth Height because they shape the ribbon the printer makes; their
values come from the search for line contact (mesh-search.md) and are fitted to the default
crossing angle and lead ("Why the ridges lean").

The three selection inputs are addSelectionInput(id, label, tooltip), each with these filters
([PB-SELECTION-FILTER-ENUM]) and setSelectionLimits(1, 1) ([PB-SELECTION-DECL]):

- Target Plane: ConstructionPlanes + PlanarFaces; tooltip Plane the cage's axis is normal to.
- Centre Point: ConstructionPoints + SketchPoints; tooltip Centre of the mechanism.
- Parent Component: Occurrences + RootComponents; pre-selects get_design().rootComponent;
  tooltip Component the mechanism is created under.

Read raw values with design.unitsManager.evaluateExpression(input.expression, units), which
returns internal units — cm for length and **radians** for angle ([PB-EVAL-EXPRESSION]).

**Range checks, all raised as a clear error naming the offending field and the bound:**

- ribbonWidth, ribbonThickness, toothPitch, twistLead, collarHalf, collarWall and
  clearance must be > 0; roofAllowance must be >= 0, zero cutting every bore to the
  clearance alone; toothCount must be a whole number >= 4.
- toothHeight must be > 0 and < ribbonWidth/2.
- toothSlant must lie strictly between −90° and 90°, where its tangent is finite. Zero is the
  straight ridge; the sign is the frame's ("The part"), and the build checks it (§2).
- toothBow must be >= 0, zero leaving the ridge unstraightened, and
  toothHeight + toothBow*(ribbonThickness/2)^2 must be < ribbonWidth/2, the root at the faces
  staying on the toothed side of the axis as toothHeight < ribbonWidth/2 keeps it on the mid
  plane. The message names toothBow. At the defaults the left side is 2.79 mm against 7.5 mm.
- engagement must be > 0 and <= toothHeight. Past the tooth height the two blanks foul each
  other rather than meshing.
- twistLead has no upper bound: a very long lead approaches two straight racks pushing each
  other, which is the degenerate case Segerman names, and nothing here forbids it. The section
  floor in §2 is what keeps the tooth at a long lead.
- crossAngle must lie strictly between 0° and 180°. At either end the axes are parallel and the
  engaged zone below has no length. 80° is the angle the tooth's lean is fitted to, and 87.2° and
  90° have been measured to drive with it; the search covered 38.5°–100° at the straight tooth
  (mesh-search.md), and nothing here clamps to it.
- assemblyPhase must lie strictly within ±toothPitch. A phase a pitch further on is the same
  phase with the ribbon's ends moved a pitch. Only the default has been measured to sit in the
  free window.
- Both bores, each cut a millimetre past the wall (§4), have to lie within the ribbon's length:
  cageRadius + collarHalf + 1 mm < toothCount*toothPitch/2, the millimetre being the cut's
  margin. The message names cageRadius.

The sleeve's own four checks come next, in this order, each naming the field given and the
bound; sleeveRefusal in sleeve_test.go is the same four in the same order, and
TestSleeveInputsAreChecked reaches each with an input that passes every check before it.
Ri = cageRadius - collarHalf is the sleeve's inner radius and `c = hypot(W/2 + clearance,
T/2 + clearance + roofAllowance)` the corner radius of the level bore's roof side, the furthest
any bore's corner stands from its axis, 8.058 mm at the defaults; every check takes it for all
four bores.

- **The channel starts in the hollow**, naming cageRadius: hypot(c, 1 mm) < Ri. Each bore's
  cut starts a millimetre before the channel's corner first reaches the inner face, at station
  sIn = sqrt(Ri^2 - c^2) - 1 mm (§4), and the check is sIn > 0. That needs the corner inside
  the inner radius by more than the millimetre allows: with c < Ri alone, a corner within about
  0.04 mm of a 12 mm inner radius would put sIn at or before the middle, so a gear's bore- and
  bore+ lines would overlap or run backwards and the build would fail inside Fusion. A 4 mm
  clearance fails it, and so does a cage radius 0.02 mm past collarHalf + c, where sIn is
  −0.44 mm.
- **The mesh stays visible along the axis**, naming cageRadius:
  hypot(axialWindow, hypot(W/2, T/2)) + clearance <= Ri, with
  axialWindow = 1.5 * sqrt(W^2 - A^2) / sin(Sigma), 8.40 mm at the defaults, the reach the
  proof's axialWindow samples. The left side bounds how far the engaged zone's footprint along
  the frame's axis reaches from it, 11.41 mm, and with the clearance 11.61 mm against the 12 mm
  inner radius at the defaults; TestSleeveKeepsTheMeshVisibleAlongTheAxis walks the footprint
  itself, 11.24 mm. At a 1.18 mm engagement the check still passes, by 0.025 mm, while the
  test's walk of the footprint grown by the clearance as a box already finds 16 points hidden;
  the box reaches past the clearance at its corners, which the closed form does not count. At
  1.20 mm the check refuses. The bound is at least axialWindow, so it also keeps the engaged zone
  inside the hollow. A 1.5 mm engagement fails it.
- **The end faces keep collarWall**, naming cageRise:
  cageRise >= A/2 + c + collarWall, 18.03 mm at the defaults. No point of a channel is further
  from the middle plane than its axis, A/2, plus c. That is a sufficient bound, not the gap:
  the channels reach 13.81 mm from the middle inside the wall, which leaves 4.94 mm of end wall
  at the 18.75 mm default, and a 16.5 mm rise, which this check refuses, leaves 2.69 mm. The
  check also refuses rises from about 16.81 mm up to 18.03 mm, which the channels would allow.
- **The wall between two neighbouring bores keeps collarWall**, naming collarWall: the
  separation §4 computes under "The wall between the bores" must be at least collarWall; the
  message names the two bores and the separation. It is 5.067 mm at the defaults, across the gap
  between the two -R bores. It depends on nearly every input at once, so it is computed rather
  than bounded by a closed form: TestSleeveWindowsFollowTheSize finds it refusing a sleeve
  scaled to 2/3 with the 3 mm wall kept (2.931 mm) and a 4 mm collarWall with a 0.55 mm
  clearance (3.946 mm), and accepting a sleeve scaled to 0.75 (3.457 mm), a 5 mm collarWall
  (5.067 mm), and a 4 mm collarWall at the defaults (5.067 mm), at a 70° crossing (4.463 mm) or
  at a 14.75 mm cage radius (4.769 mm).

collarHalf trades grip against travel. A thicker wall holds the gear closer to its screw motion
and shortens the travel by twice its own growth, because a ribbon runs
toothCount*toothPitch - 2*(cageRadius + collarHalf) through its bores.

After the four checks the window search of §4 runs. It refuses nothing: a gap it finds no room in
gets no window, and the build logs which and why (§4, "When a gap has no room").

**No range is enforced on either Mounting Angle, and none is asserted here.** At the defaults equal
angles from −12° to +12° drive and 0° is the default; what happens elsewhere is the proof's to
map, and this spec does not clamp what has not been measured. Nor is a range enforced on the
slant and the bow past the checks above: the defaults are fitted to the default crossing angle
and lead, and away from them the contact shortens by an amount nothing here measures.

### Sketch Discipline

Every sketch must report isFullyConstrained before it is consumed, and the build raises naming the
sketch when one does not ([PB-SKETCH-FIRST], [PB-FULL-CONSTRAINT]). No sketch here carries text,
so none is exempt. Six rules hold in every sketch of this build:

- **Only the Anchor sketch projects anything, and every other sketch's references are fixed
  points of its own** ([SCREW-F-REFERENCES], [PB-PROJECT-NOT-FIXED]). The Anchor sketch
  projects the selected point and binds the Anchor Line to the projection exactly as the bevel
  gear's Anchor sketch does, which reads fully constrained in Fusion. Every later sketch takes
  its references — the ends of a sweep path, the axis point of a section, the corners of a cell
  section, the corners of a window — as
  **reference points**: each is a world point of the frame of §1, mapped in with
  sketch.modelToSketchSpace, given z = 0 when it is meant to lie on the sketch's plane
  ([PB-SKETCH-ZERO-Z]; the one exception is the next rule) and added with
  sketch.sketchPoints.add. The sketch's curves are drawn **sharing** those points
  ([PB-SHARE-XOR-COINCIDENT]), and after the last curve that uses a reference point is drawn,
  and before any constraint or dimension is added, the point is set isFixed = True. A
  **reference line** is a construction line between two reference points, so it has no freedom
  and carries no dimension. Nothing is projected into these sketches and
  intersectWithSketchPlane is not called: the point where a sweep path pierces its section
  plane is the path's own start, origin_g + s*dir_g for the span's first station s, and
  that is what the reference point is placed at. A circle centred on a reference point is
  instead created at that point's position with its centerSketchPoint set isFixed = True
  ([PB-CIRCLE-CENTER]), and no reference point is added for it.
- **The Cell Sections sketch is the one sketch whose points lie off its plane, and it keeps
  their z** ([SCREW-F-CELL-LOFT], [PB-3D-SKETCH-SECTIONS]). Its points are the corners and
  the toothed side's fit points of every section of the tooth cell (§2), which stand at every
  height above and below the Gear Axis Plane the sketch is drawn on. [PB-SKETCH-ZERO-Z] zeroes
  the z of a point that is meant to lie on the plane and that rounding has pushed off it; a
  point meant to lie off the plane keeps the z that modelToSketchSpace gives it. Fusion
  accepted such a sketch of rectangles on 2026-09-28, read it fully constrained at 11, 41 and 81
  sections, and found one profile per section; the sketch whose toothed sides are fitted splines
  has not been loaded ("What the proof cannot reach").
- **Every angular dimension is taken against whichever of two perpendicular reference lines puts
  it between 45° and 135°** ([PB-ANGULAR-DIM]). Fusion cannot dimension the angle between two
  lines that are nearly parallel, and the one angle this build dimensions — a section's twist at
  its station, in the rectangle scheme of §4 — runs through 0° and 180° along the ribbon. The
  scheme names its two references; the build computes the angle it is about to seed, takes the
  reference that keeps it inside that range, and puts the dimension's text point inside the
  wedge it measures. The value written is the angle between two **rays**, each named in the
  step, from the point where the two lines meet; the text point sits inside that wedge.
- **Every seed is the solved position** ([PB-SEED-NEAR]), computed in Python in the frame of §1
  and mapped in with modelToSketchSpace, so the solver has nothing to move and a dimension's
  side is the seed's ([PB-DIM-VALUE-SEMANTICS]). **Every point mapped in that is meant to lie
  on the plane has its z set to 0 before it is used** — a reference point, a raw seed, an
  arc's through point, a circle's centre and a dimension's text point alike
  ([PB-SKETCH-ZERO-Z]). Fusion's section planes do not sit exactly where this build's
  arithmetic puts them, and the first Fusion load failed on a collar section whose points all
  landed 4.4e-6 cm off its plane.
- **No addPerpendicular on a line that only one of its ends anchors.** A line drawn from a
  fixed point, given a length and made perpendicular to a reference has two solutions, one each
  side of the reference, and the proof's sketch gate refuses a sketch that admits a mirror
  image. addPerpendicular is used only in the rectangle scheme of §4, where the line it turns
  already has both ends tied to other lines.
- **Sketch computing is deferred while the two heavy kinds of sketch are drawn, and nowhere
  else** ([PB-SKETCH-DEFER], [SCREW-F-DEFER]): the Cell Sections sketch of §2, and the four
  bore section sketches of §4. In those, set
  sketch.isComputeDeferred = True right after the sketch is created and named and before its
  first point, and set it back to False after the last curve, constraint, dimension or
  isFixed, before isFullyConstrained or profiles is read. Measured in Fusion on
  2026-09-28: a collar section fell from 0.45 s to 0.23 s and a one-tooth Cell Sections
  sketch from 0.88 s to 0.09 s, each still reading fully constrained with its profiles found.
  The Anchor sketch does not defer, because deferral has not been measured under a projection;
  the Paths, Sleeve and Window sketches do not, because they are a few fixed points with lines
  or circles and were not measured.

The four bore section sketches of §4 share one rectangle scheme, stated there, that leaves no
freedom, so there is no under-constrained case to exempt, unlike the bevel tooth profile. Every
other sketch is built from reference points alone: lines between them and circles centred on
them, with nothing left to solve.

### Method contract — call graph

```
generate(inputs)
  → processInputs(inputs)                    # read, check, precompute; each gear's level bore, the bore-wall check and the window search run here
  → buildComponentTree()                     # Screw Gearing + Design + 3 children
  → buildAnchor()                            # anchor sketch, centre point, reference direction, axis planes, n̂
  → buildGear(index)   x2                    # per gear: paths → cell → repeat → one body
      → buildSweepPaths(index)               # Paths sketch: the two bore-path lines on the gear's axis
      → buildToothCell(index)                # one Cell Sections sketch (41 sections at the defaults) → one loft → the slant's sign check
      → repeatCellByDoubling(index)          # copy + screw-move + join, in cells; a remainder cell when N is not a multiple of cellTeeth
  → buildCage()                              # the sleeve, one twisted sweep cut per bore, one extrude cut per window; logs the end to print on
  → relocateBodies()                         # moveToComponent into Gear A / Gear B / Cage
  → solids.hide_construction_geometry(self.designOcc.component)
```

### What the build makes

Counted at the defaults, in timeline entries, so a Fusion load can be checked against it. The
first two Fusion loads ([SCREW-F-FIRST-LOAD]) built the video frame's geometry as 136 sketches,
134 construction planes and 79 features, one sketch and one plane per loft section, and took
about five minutes; the third built it as 23 sketches, 12 planes and 56 features by replacing
every per-section sketch and plane with one sweep or one sketch. The gears' rows below are that
construction's, which the diagnostics of 2026-09-28 measured piece by piece; the sleeve's rows
replace the ring, rods, loop and collars and have not been built in Fusion.

| Part | Sketches | Planes | Features, cellTeeth = 4 | Features, cellTeeth = 1 |
|---|---|---|---|---|
| Anchor (§1) | 1 | 0 | 0 | 0 |
| Gear axis planes (§1) | 0 | 2 | 0 | 0 |
| Per gear: Paths sketch (§1) | 1 | 0 | 0 | 0 |
| Per gear: Cell Sections sketch and loft (§2) | 1 | 0 | 1 | 1 |
| Per gear: doubling (§3), 3 features a round | 0 | 0 | 15 (5 rounds) | 21 (7 rounds) |
| Both gears, subtotal | 4 | 0 | 32 | 44 |
| Sleeve (§4): Sleeve sketch, extrude | 1 | 0 | 1 | 1 |
| Bores (§4): 4 planes, 4 sketches, 4 sweep cuts | 4 | 4 | 4 | 4 |
| Windows and marks (§4): 2 planes, 6 sketches, 2 cuts, 4 extrudes, 4 joins | 6 | 2 | 10 | 10 |
| Relocate (§5): 3 moveToComponent | 0 | 0 | 3 | 3 |
| **Total** | **16** | **8** | **50** | **62** |

That is 70 timeline entries at cellTeeth = 4 and 82 at cellTeeth = 1, plus the five
component creations: 75 and 87 in all, against 96 and 108 for the video frame. The third Fusion
load ([SCREW-F-FIRST-LOAD]) counted exactly 96 timeline items for that frame. Check a load
against the timeline count, not against Component.features.count: at that load, summed over
the five components, it read 62, six more than the 56 features the timeline held, all six in
Design, for a reason not yet measured. A remainder cell (§3) adds one sketch, one loft and one
join per gear that needs one; the defaults need none. A window the search finds no room for
(§4) takes one sketch and one feature off the count, and when neither window has room the
Window Plane is not made either. The heaviest sketches measured are the bore sections of §4, at
0.23 s each for the video frame's collar sections drawn by the same scheme, and the two Cell
Sections sketches at 0.37 s each with computing deferred; each collar sweep took 0.04 s, each
cell loft 0.23 s, and the doubling's copies, moves and joins were not timed on their own. The
two searches of §4 that run before any feature took 2.2 s together at the defaults in a CPython
port of them, run outside Fusion, at the defaults before 2026-10-02, and 4.4 s at every length
scaled by 1.5; Fusion has not run
them.

The required Fusion calls for this timeline entry are:

- `command.commandInputs.addSelectionInput(id, label, tooltip)`.
- `selectionInput.addSelectionFilter(filterConstant)`.
- `selectionInput.setSelectionLimits(1, 1)`.
- `parentInput.addSelection(rootComponent)`.
- `command.commandInputs.addGroupCommandInput(groupId, groupLabel)`.
- `group.children.addValueInput(id, label, unit, defaultValue)`.
- `adsk.core.ValueInput.createByReal(defaultInInternalUnits)`.
- `inputs.itemById(id)`.
- `selectionInput.selection(0)`.
- `self.design.unitsManager.evaluateExpression(input.expression, units)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "addSelectionInput",
      "owner": "adsk.core.CommandInputs",
      "reason": null,
      "receiver": "command.commandInputs",
      "role": "required",
      "span": "command.commandInputs.addSelectionInput(id, label, tooltip)"
    },
    {
      "condition": null,
      "name": "addSelectionFilter",
      "owner": "adsk.core.SelectionCommandInput",
      "reason": null,
      "receiver": "selectionInput",
      "role": "required",
      "span": "selectionInput.addSelectionFilter(filterConstant)"
    },
    {
      "condition": null,
      "name": "setSelectionLimits",
      "owner": "adsk.core.SelectionCommandInput",
      "reason": null,
      "receiver": "selectionInput",
      "role": "required",
      "span": "selectionInput.setSelectionLimits(1, 1)"
    },
    {
      "condition": null,
      "name": "addSelection",
      "owner": "adsk.core.SelectionCommandInput",
      "reason": null,
      "receiver": "parentInput",
      "role": "required",
      "span": "parentInput.addSelection(rootComponent)"
    },
    {
      "condition": null,
      "name": "addGroupCommandInput",
      "owner": "adsk.core.CommandInputs",
      "reason": null,
      "receiver": "command.commandInputs",
      "role": "required",
      "span": "command.commandInputs.addGroupCommandInput(groupId, groupLabel)"
    },
    {
      "condition": null,
      "name": "addValueInput",
      "owner": "adsk.core.CommandInputs",
      "reason": null,
      "receiver": "group.children",
      "role": "required",
      "span": "group.children.addValueInput(id, label, unit, defaultValue)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(defaultInInternalUnits)"
    },
    {
      "condition": null,
      "name": "itemById",
      "owner": "adsk.core.CommandInputs",
      "reason": null,
      "receiver": "inputs",
      "role": "required",
      "span": "inputs.itemById(id)"
    },
    {
      "condition": null,
      "name": "selection",
      "owner": "adsk.core.SelectionCommandInput",
      "reason": null,
      "receiver": "selectionInput",
      "role": "required",
      "span": "selectionInput.selection(0)"
    },
    {
      "condition": null,
      "name": "evaluateExpression",
      "owner": "adsk.core.UnitsManager",
      "reason": null,
      "receiver": "self.design.unitsManager",
      "role": "required",
      "span": "self.design.unitsManager.evaluateExpression(input.expression, units)"
    }
  ],
  "citations": [
    {
      "first": 566,
      "last": 910,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L566–910.

## S02 `[PROSE]` Create the Design child occurrence

Create exactly one child named Design under Screw Gearing. This is one timeline entry.
[PB-OCCURRENCE-TREE] [PB-NEVER-ACTIVATE]

### Component Setup

One command invocation creates a **Screw Gearing** component under the user-selected Parent
Component, holding four sub-components ([PB-OCCURRENCE-TREE]). The top occurrence is the
inherited self.getOccurrence(), which creates it under self.parentComponent and is what the
inherited deleteComponent() deletes on failure; the build names its component and never calls
addNewComponent for it itself. The four children are each
occurrences.addNewComponent(adsk.core.Matrix3D.create()) under that component:

- **Design** — every sketch, construction plane and feature runs here.
- **Gear A**, **Gear B**, **Cage** — empty until the end, when the finished bodies are
  relocated into them with body.moveToComponent ([PB-NO-CROSS-SIBLING]).

**NEVER call occurrence.activate()** ([PB-NEVER-ACTIVATE]). Place sketches directly on the
user-selected plane ([PB-USE-SELECTED-PLANE]).

The user selects a **plane** and a **point**. The plane's normal is the cage axis and the common
perpendicular n̂; the point is the mechanism's centre C.


The required Fusion calls for this timeline entry are:

- `adsk.core.Matrix3D.create()`.
- `parent.occurrences.addNewComponent(identityMatrix)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Matrix3D",
      "reason": null,
      "receiver": "adsk.core.Matrix3D",
      "role": "required",
      "span": "adsk.core.Matrix3D.create()"
    },
    {
      "condition": null,
      "name": "addNewComponent",
      "owner": "adsk.fusion.Occurrences",
      "reason": null,
      "receiver": "parent.occurrences",
      "role": "required",
      "span": "parent.occurrences.addNewComponent(identityMatrix)"
    }
  ],
  "citations": [
    {
      "first": 598,
      "last": 616,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L598–616.

## S03 `[PROSE]` Create the Gear A child occurrence

Create exactly one child named Gear A under Screw Gearing. This is one timeline entry.
[PB-OCCURRENCE-TREE] [PB-NEVER-ACTIVATE]

### Component Setup

One command invocation creates a **Screw Gearing** component under the user-selected Parent
Component, holding four sub-components ([PB-OCCURRENCE-TREE]). The top occurrence is the
inherited self.getOccurrence(), which creates it under self.parentComponent and is what the
inherited deleteComponent() deletes on failure; the build names its component and never calls
addNewComponent for it itself. The four children are each
occurrences.addNewComponent(adsk.core.Matrix3D.create()) under that component:

- **Design** — every sketch, construction plane and feature runs here.
- **Gear A**, **Gear B**, **Cage** — empty until the end, when the finished bodies are
  relocated into them with body.moveToComponent ([PB-NO-CROSS-SIBLING]).

**NEVER call occurrence.activate()** ([PB-NEVER-ACTIVATE]). Place sketches directly on the
user-selected plane ([PB-USE-SELECTED-PLANE]).

The user selects a **plane** and a **point**. The plane's normal is the cage axis and the common
perpendicular n̂; the point is the mechanism's centre C.


The required Fusion calls for this timeline entry are:

- `adsk.core.Matrix3D.create()`.
- `parent.occurrences.addNewComponent(identityMatrix)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Matrix3D",
      "reason": null,
      "receiver": "adsk.core.Matrix3D",
      "role": "required",
      "span": "adsk.core.Matrix3D.create()"
    },
    {
      "condition": null,
      "name": "addNewComponent",
      "owner": "adsk.fusion.Occurrences",
      "reason": null,
      "receiver": "parent.occurrences",
      "role": "required",
      "span": "parent.occurrences.addNewComponent(identityMatrix)"
    }
  ],
  "citations": [
    {
      "first": 598,
      "last": 616,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L598–616.

## S04 `[PROSE]` Create the Gear B child occurrence

Create exactly one child named Gear B under Screw Gearing. This is one timeline entry.
[PB-OCCURRENCE-TREE] [PB-NEVER-ACTIVATE]

### Component Setup

One command invocation creates a **Screw Gearing** component under the user-selected Parent
Component, holding four sub-components ([PB-OCCURRENCE-TREE]). The top occurrence is the
inherited self.getOccurrence(), which creates it under self.parentComponent and is what the
inherited deleteComponent() deletes on failure; the build names its component and never calls
addNewComponent for it itself. The four children are each
occurrences.addNewComponent(adsk.core.Matrix3D.create()) under that component:

- **Design** — every sketch, construction plane and feature runs here.
- **Gear A**, **Gear B**, **Cage** — empty until the end, when the finished bodies are
  relocated into them with body.moveToComponent ([PB-NO-CROSS-SIBLING]).

**NEVER call occurrence.activate()** ([PB-NEVER-ACTIVATE]). Place sketches directly on the
user-selected plane ([PB-USE-SELECTED-PLANE]).

The user selects a **plane** and a **point**. The plane's normal is the cage axis and the common
perpendicular n̂; the point is the mechanism's centre C.


The required Fusion calls for this timeline entry are:

- `adsk.core.Matrix3D.create()`.
- `parent.occurrences.addNewComponent(identityMatrix)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Matrix3D",
      "reason": null,
      "receiver": "adsk.core.Matrix3D",
      "role": "required",
      "span": "adsk.core.Matrix3D.create()"
    },
    {
      "condition": null,
      "name": "addNewComponent",
      "owner": "adsk.fusion.Occurrences",
      "reason": null,
      "receiver": "parent.occurrences",
      "role": "required",
      "span": "parent.occurrences.addNewComponent(identityMatrix)"
    }
  ],
  "citations": [
    {
      "first": 598,
      "last": 616,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L598–616.

## S05 `[PROSE]` Create the Cage child occurrence

Create exactly one child named Cage under Screw Gearing. This is one timeline entry.
[PB-OCCURRENCE-TREE] [PB-NEVER-ACTIVATE]

### Component Setup

One command invocation creates a **Screw Gearing** component under the user-selected Parent
Component, holding four sub-components ([PB-OCCURRENCE-TREE]). The top occurrence is the
inherited self.getOccurrence(), which creates it under self.parentComponent and is what the
inherited deleteComponent() deletes on failure; the build names its component and never calls
addNewComponent for it itself. The four children are each
occurrences.addNewComponent(adsk.core.Matrix3D.create()) under that component:

- **Design** — every sketch, construction plane and feature runs here.
- **Gear A**, **Gear B**, **Cage** — empty until the end, when the finished bodies are
  relocated into them with body.moveToComponent ([PB-NO-CROSS-SIBLING]).

**NEVER call occurrence.activate()** ([PB-NEVER-ACTIVATE]). Place sketches directly on the
user-selected plane ([PB-USE-SELECTED-PLANE]).

The user selects a **plane** and a **point**. The plane's normal is the cage axis and the common
perpendicular n̂; the point is the mechanism's centre C.


The required Fusion calls for this timeline entry are:

- `adsk.core.Matrix3D.create()`.
- `parent.occurrences.addNewComponent(identityMatrix)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Matrix3D",
      "reason": null,
      "receiver": "adsk.core.Matrix3D",
      "role": "required",
      "span": "adsk.core.Matrix3D.create()"
    },
    {
      "condition": null,
      "name": "addNewComponent",
      "owner": "adsk.fusion.Occurrences",
      "reason": null,
      "receiver": "parent.occurrences",
      "role": "required",
      "span": "parent.occurrences.addNewComponent(identityMatrix)"
    }
  ],
  "citations": [
    {
      "first": 598,
      "last": 616,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L598–616.

## S06 `[GO]` Anchor sketch

The proof function is `stepAnchor`.

<!-- proof-run: proofkit.Run(anchorCases, stepAnchor) -->

[PB-SKETCH-FIRST] [PB-FULL-CONSTRAINT] [PB-REFLINE-DIRECTION] [PB-WORLDGEO-CONSTRAINED]
[PB-WORLD-FRAME] [PB-DIM-VALUE-SEMANTICS] [PB-SEED-NEAR] [SCREW-F-REFERENCES]
The API database cannot verify Sketch.project. Keep the source requirement and report it for source review.

### 1: Anchor and frame

Create the anchor sketch, named Anchor, on the user-selected plane and project the selected
point into it, sketch.project(point).item(0); this is the one projection in the build
([SCREW-F-REFERENCES]). Draw the **Anchor Line** through it: a line from two raw Point3D
seeds 0.5 cm either side of the projected point along the sketch's own x axis, so it is 10 mm
long with its end to the right of its start, and constrain it with four things and nothing else,
which is the bevel gear's Anchor sketch and reads fully constrained in Fusion:

- addCoincident(projectedPoint, line) — the centre lies on the line — **and**
  addMidPoint(projectedPoint, line) — the centre bisects it. Both, not the midpoint alone,
  as the bevel spec requires of its own Anchor sketch. The proof's sketch engine emits the
  coincident row as part of its midpoint constraint, so the compiled proof writes the midpoint
  alone, as proof/bevelgear does.
- addHorizontal(line) — sketch-local, so it survives a tilted plane ([PB-REFLINE-DIRECTION]).
- A **horizontal** distance dimension from the line's start to its end
  (addDistanceDimension(start, end, HorizontalDimensionOrientation, textPoint)), value 10 mm.
  Not an aligned one: midpoint, horizontal and an aligned length are satisfied by the line in
  either of its two end-for-end orientations, and the proof's sketch gate refuses that as
  ambiguous; a horizontal distance from start to end runs one way, so only the seeded orientation
  satisfies it.

Midpoint, horizontal and the horizontal distance take the line's four degrees of freedom, so it
has none, and its absolute direction is arbitrary. The build raises unless the sketch reports
isFullyConstrained, and only then reads the frame from world geometry
([PB-WORLDGEO-CONSTRAINED], [PB-WORLD-FRAME]): C is the projected point's
worldGeometry, and ê is the unit vector from the line's startSketchPoint.worldGeometry to
its endSketchPoint.worldGeometry. The plane's normal is n̂, read in the next paragraph but
one. No later sketch projects the point or the line: every other sketch takes C and ê as
numbers, and the Anchor Line itself is passed once more, as the line the Window Plane of §4 is
built on.

The required Fusion calls for this timeline entry are:

- `component.sketches.add(targetPlane)`.
- `sketch.project(point)`.
- `sketch.sketchCurves.sketchLines.addByTwoPoints(startSeed, endSeed)`.
- `sketch.geometricConstraints.addCoincident(projectedPoint, anchorLine)`.
- `sketch.geometricConstraints.addMidPoint(projectedPoint, anchorLine)`.
- `sketch.geometricConstraints.addHorizontal(anchorLine)`.
- `sketch.sketchDimensions.addDistanceDimension(start, end, horizontalOrientation, textPoint)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "component.sketches",
      "role": "required",
      "span": "component.sketches.add(targetPlane)"
    },
    {
      "condition": null,
      "name": "project",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.project(point)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "sketch.sketchCurves.sketchLines.addByTwoPoints(startSeed, endSeed)"
    },
    {
      "condition": null,
      "name": "addCoincident",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addCoincident(projectedPoint, anchorLine)"
    },
    {
      "condition": null,
      "name": "addMidPoint",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addMidPoint(projectedPoint, anchorLine)"
    },
    {
      "condition": null,
      "name": "addHorizontal",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addHorizontal(anchorLine)"
    },
    {
      "condition": null,
      "name": "addDistanceDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addDistanceDimension(start, end, horizontalOrientation, textPoint)"
    }
  ],
  "citations": [
    {
      "first": 914,
      "last": 944,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L914–944.

## S07 `[PROSE]` Gear A Axis Plane

Create exactly one plane named Gear A Axis Plane at -A/2 from the selected plane.
Use its geometry origin and normal to check the signed offset and determine nHat; no engine sketch owns a Fusion plane.
[PB-CONSTRUCTION-PLANES] [PB-USE-SELECTED-PLANE] [SCREW-F-NORMAL-SIGN]

Compute both axes from ê and n̂:

```
dirA = rotate(ê, +Sigma/2, about n̂)      originA = C - (A/2)*n̂
dirB = rotate(ê, -Sigma/2, about n̂)      originB = C + (A/2)*n̂
```

Gear A's cross-section frame is û_A = +n̂, v̂_A = dirA × û_A; gear B's is û_B = -n̂,
v̂_B = dirB × û_B. **û points at the other gear** — that is what makes Phi mean the same thing
for both parts, and it is why the two gears are the same part rather than mirror images. A point
of gear g at station s with section coordinates (u, v) is
origin_g + s*dir_g + (u*cos theta - v*sin theta)*û_g + (u*sin theta + v*cos theta)*v̂_g, with
theta = s/Lambda + Phi_g; every seed below is that point. The third direction of the frame is
k̂ = n̂ × ê, level and square to ê; the proof's pair puts ê, k̂ and n̂ on its X, Y
and Z axes with C at the origin, and §4 names directions by them.

**The axis planes, and the sign of n̂.** Two construction planes are offset from the selected
plane itself ([PB-USE-SELECTED-PLANE], [PB-CONSTRUCTION-PLANES]): Gear A Axis Plane by
-A/2 and Gear B Axis Plane by +A/2, both with setByOffset. Fusion offsets along the
selected entity's own normal, and the build does not assume its sign: n̂ is read from the Gear
A Axis Plane after it is made — the unit normal of its geometry, signed so that C lies +A/2
along it — and the build checks that C is A/2 from each of the two planes
([SCREW-F-NORMAL-SIGN]). No later plane is offset from the selected plane.


The required Fusion calls for this timeline entry are:

- `component.constructionPlanes.createInput()`.
- `adsk.core.ValueInput.createByReal(offset)`.
- `planeInput.setByOffset(targetPlane, offsetValue)`.
- `component.constructionPlanes.add(planeInput)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "component.constructionPlanes",
      "role": "required",
      "span": "component.constructionPlanes.createInput()"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(offset)"
    },
    {
      "condition": null,
      "name": "setByOffset",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByOffset(targetPlane, offsetValue)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "component.constructionPlanes",
      "role": "required",
      "span": "component.constructionPlanes.add(planeInput)"
    }
  ],
  "citations": [
    {
      "first": 946,
      "last": 969,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L946–969.

## S08 `[PROSE]` Gear B Axis Plane

Create exactly one plane named Gear B Axis Plane at +A/2 from the selected plane.
Use its geometry origin and normal to check the signed offset and determine nHat; no engine sketch owns a Fusion plane.
[PB-CONSTRUCTION-PLANES] [PB-USE-SELECTED-PLANE] [SCREW-F-NORMAL-SIGN]

Compute both axes from ê and n̂:

```
dirA = rotate(ê, +Sigma/2, about n̂)      originA = C - (A/2)*n̂
dirB = rotate(ê, -Sigma/2, about n̂)      originB = C + (A/2)*n̂
```

Gear A's cross-section frame is û_A = +n̂, v̂_A = dirA × û_A; gear B's is û_B = -n̂,
v̂_B = dirB × û_B. **û points at the other gear** — that is what makes Phi mean the same thing
for both parts, and it is why the two gears are the same part rather than mirror images. A point
of gear g at station s with section coordinates (u, v) is
origin_g + s*dir_g + (u*cos theta - v*sin theta)*û_g + (u*sin theta + v*cos theta)*v̂_g, with
theta = s/Lambda + Phi_g; every seed below is that point. The third direction of the frame is
k̂ = n̂ × ê, level and square to ê; the proof's pair puts ê, k̂ and n̂ on its X, Y
and Z axes with C at the origin, and §4 names directions by them.

**The axis planes, and the sign of n̂.** Two construction planes are offset from the selected
plane itself ([PB-USE-SELECTED-PLANE], [PB-CONSTRUCTION-PLANES]): Gear A Axis Plane by
-A/2 and Gear B Axis Plane by +A/2, both with setByOffset. Fusion offsets along the
selected entity's own normal, and the build does not assume its sign: n̂ is read from the Gear
A Axis Plane after it is made — the unit normal of its geometry, signed so that C lies +A/2
along it — and the build checks that C is A/2 from each of the two planes
([SCREW-F-NORMAL-SIGN]). No later plane is offset from the selected plane.


The required Fusion calls for this timeline entry are:

- `component.constructionPlanes.createInput()`.
- `adsk.core.ValueInput.createByReal(offset)`.
- `planeInput.setByOffset(targetPlane, offsetValue)`.
- `component.constructionPlanes.add(planeInput)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "component.constructionPlanes",
      "role": "required",
      "span": "component.constructionPlanes.createInput()"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(offset)"
    },
    {
      "condition": null,
      "name": "setByOffset",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByOffset(targetPlane, offsetValue)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "component.constructionPlanes",
      "role": "required",
      "span": "component.constructionPlanes.add(planeInput)"
    }
  ],
  "citations": [
    {
      "first": 946,
      "last": 969,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L946–969.

## S09 `[GO]` Per-gear Paths sketch

The proof function is `stepPaths`.

<!-- proof-run: proofkit.Run(pathCases, stepPaths) -->

Instantiate this sketch once in each gear build, first Gear A then Gear B.
Both bore path lines belong to that one sketch; point fixation follows line creation.
[PB-PROJECT-NOT-FIXED] [PB-SHARE-XOR-COINCIDENT] [PB-SKETCH-ZERO-Z]
[PB-WORLDGEO-CONSTRAINED] [PB-PATH-FROM-SKETCH] [SCREW-F-REFERENCES]

**The Paths sketches.** Gear A Paths on the Gear A Axis Plane and Gear B Paths on gear B's,
each holding the two lines that gear's bores are swept along (§4), both on the gear's axis,
which lies in that plane. The +R bore's cut spans stations sIn to sOut of the gear's axis
and the -R bore's -sOut to -sIn, with sIn = sqrt(Ri^2 - c^2) - 1 mm and
sOut = cageRadius + collarHalf + 1 mm (§4): 7.892 and 19 mm at the defaults. The sketch holds
four reference points (Sketch Discipline), at origin_g + s*dir_g for s = -sOut, -sIn,
sIn and sOut, all on the plane, and two solid lines — bore- from -sOut to -sIn and
bore+ from sIn to sOut — each drawn from its negative station to its positive one, sharing
both points, so its start is its negative end and it runs along +dir_g; then all four points
are set isFixed = True. The lines carry no dimension and no constraint, and nothing else is in
the sketch. Both lie on one line with a gap between them; Fusion read a sketch of four such
lines, overlapping, fully constrained on 2026-09-28, and a path made from one of its lines with
chaining off held that one line ([PB-PATH-FROM-SKETCH]). A line whose two ends are fixed has a
worldGeometry the build can trust ([PB-WORLDGEO-CONSTRAINED]), and a line held by dimensions
from a projected point does not. The section plane of each bore is setByDistanceOnPath on its
own line at fraction 0 ([PB-CONSTRUCTION-PLANES]; pass the line directly), so it stands at
the span's negative end, square to the axis; the point where the line pierces it is the span's
first station, origin_g + s*dir_g, and Fusion put that plane's origin on the station to four
decimals of a millimetre. The sweep's path is features.createPath(line, False) on the same
line ([SCREW-F-TWISTED-SLOT]).


The required Fusion calls for this timeline entry are:

- `component.sketches.add(axisPlane)`.
- `sketch.modelToSketchSpace(worldPoint)`.
- `sketch.sketchPoints.add(localPoint)`.
- `sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "component.sketches",
      "role": "required",
      "span": "component.sketches.add(axisPlane)"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.modelToSketchSpace(worldPoint)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SketchPoints",
      "reason": null,
      "receiver": "sketch.sketchPoints",
      "role": "required",
      "span": "sketch.sketchPoints.add(localPoint)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)"
    }
  ],
  "citations": [
    {
      "first": 970,
      "last": 990,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L970–990.

## S10 `[GO]` Per-gear Cell Sections sketch

The proof function is `stepCellSections`.

<!-- proof-run: proofkit.Run(sectionCases, stepCellSections) -->

One timeline sketch holds all sections per gear. The proof tests planar polygonal samples;
Fusion alone verifies the complete off-plane sketch and fitted splines.
[PB-SKETCH-FIRST] [PB-3D-SKETCH-SECTIONS] [PB-SKETCH-DEFER] [PB-SHARE-XOR-COINCIDENT]
[PB-SKETCHCURVES] [PB-PROJECT-NOT-FIXED] [SCREW-F-CELL-LOFT] [SCREW-F-DEFER]

### 2: The tooth cell

One cell is c tooth pitches of the finished ribbon, at the ribbon's negative end: stations s
from s0 = Z0 - L/2 to s0 + c*P, where Z0 is the gear's tooth phase — 0 for gear A and the
assemblyPhase input for gear B ("Defaults") — and c = min(cellTeeth, N) is the number of
teeth the cell holds, 4 at the defaults (cellTeeth, "Variables"). Gear B's whole ribbon is
gear A's advanced by Z0 under its own screw motion, ends and teeth alike, so the same cell
built Z0 further along its axis and repeated the same way is gear B. Build it once per gear.

**The section count is derived, not pinned, and it is not user-configurable.** The cell is lofted
through c*n + 1 sections, n steps to the tooth, where n is the larger of two counts:

- the smallest that keeps the twist between neighbouring sections under 2°,
  ceil((P/Lambda) / 2°), 10 at the defaults. A surface ruled straight between two sections
  cuts the corner of the true helicoid at the crest by (W/2)*(1 - cos(dtheta/2)), dtheta
  the twist per step: 1.0 µm at the defaults' 1.91°, under a hundredth of the 1.08 mm backlash,
  which is the bound the proof holds it to (the shortfall grows with the width and the backlash
  with the pitch, so it is not held to a fixed number of microns);
- eight. A straight chord of the cosine between two sections falls (H/2)*(1 - cos(pi/n)) short
  of it at the deepest point whatever the twist: 0.064 mm at the defaults' ten steps, 6% of the
  backlash, and 0.100 mm, 3.8% of the tooth height, at eight. The proof holds it under a fifth of
  the backlash; it was 7.8% of the 0.82 mm backlash before 2026-10-02 and 14% of the 0.46 mm one
  until 2026-10-03, when the leaned tooth widened the backlash. The chord runs along s at every
  v alike, so the lean does not change it. That floor is what keeps a slow
  twist from lofting the tooth through two or three sections and losing it: at a 400 mm lead the
  twist alone asks for two sections, and the cell is built through nine.

That is 10 steps to the tooth at the defaults: 41 sections in a four-tooth cell, 11 in a
one-tooth cell. TestLoftSectionCountHoldsTheHelicoid holds both bounds over leads from 20 mm
to 400 mm.

**What the loft is, and what the count guarantees for it.** Both bounds are arithmetic about a
*ruled* loft, whose surface runs straight between neighbouring sections. Fusion builds a ruled
loft only through exactly two sections; a loft through more passes through every section and is
fitted smoothly between them, and LoftFeatureInput has no option to make it ruled — its
sections carry only end conditions ([SCREW-F-CELL-LOFT]). The cell is **one loft through all
c*n + 1 sections**, so it is the smooth kind. What the count guarantees for it is that the
built cell carries the exact rotated section at each of its stations, dtheta apart, its
toothed side the spline through the edge's points, and that between stations its surface
interpolates those sections; the chord figures above are the
departure of the ruled loft through the same sections, and they are the only figures the proof
has. What the smooth surface does between sections was measured in Fusion on 2026-09-28, at
the defaults of that day, the 10 mm ribbon ([SCREW-F-CELL-LOFT]): at one, four and eight
teeth, probes 0.04 mm inside and 0.04 mm outside the toothed edge and a face, at the midpoint
between every pair of sections, all fell on the right side of the built surface — 40 of 40,
160 of 160 and 320 of 320 — and the built volume was 0.06% to 0.23% under the ruled loft's. So
at that size the built surface stayed within 0.04 mm of the helicoid between sections, under a
tenth of the 0.44 mm backlash of that day, with straight-ridge rectangles for sections; the 1.5×
ribbon has the same section spacing, and neither it nor the spline sections have been
measured. The proof cannot measure that ("What the proof cannot reach"), and a Fusion
load is what checks it at other inputs. A chain of c*n two-section lofts would be ruled and would carry the bounds
literally, at the cost of that many lofts and joins per cell, and this spec keeps the one loft.

**The Cell Sections sketch.** All the cell's sections go in **one sketch**, named
{gearLabel} Cell Sections, on the gear's Axis Plane ([SCREW-F-CELL-LOFT],
[PB-3D-SKETCH-SECTIONS]), drawn with sketch computing deferred (Sketch Discipline,
[PB-SKETCH-DEFER]). No section plane is made. Section k, for k from 0 to c*n, is the
cross-section at station s_k = s0 + k*P/n, turned about the axis by theta_k. Its back side
runs along u = -W/2, its faces along v = ±T/2, and its toothed side follows the edge of "The
part" across the thickness, through M points:

```
theta_k        = s_k/Lambda + Phi
v_j            = -T/2 + j*T/(M - 1),    j = 0 … M - 1,    M = TOOTH_SPLINE_POINTS = 11
Utooth(v, s_k) = W/2 - H/2 + (H/2)*cos(2*pi*(s_k + tan(Slant)*v - Z0)/P) - Bow*v^2
```

M is a module constant, beside CELL_TEETH ("Variables"): eleven points, T/10 apart, 0.375 mm
at the defaults, both faces included. TestToothSplineHoldsTheEdge fits a natural cubic spline
through them at every station of a pitch and finds it within 0.018 mm of the edge along u;
through nine points it departs by 0.030 mm, through seven by 0.058 mm, because across the
thickness the edge runs through 0.69 of a pitch of phase.

Each section has M + 2 points: the two back corners B0 at (uB, -hv) and B1 at (uB, hv),
and the toothed points F_j at (Utooth(v_j, s_k), v_j), of which F_0 and F_(M-1) are the
toothed corners; uB = -W/2 and hv = T/2. Each is the world point of §1 at those section
coordinates and station s_k, mapped in with sketch.modelToSketchSpace(point) and added with
sketch.sketchPoints.add(point) **with its z kept**: the points lie off the sketch's plane on
purpose (Sketch Discipline, [PB-SKETCH-ZERO-Z] not applied). Four curves share them, in this
order ([PB-SHARE-XOR-COINCIDENT], [PB-SKETCHCURVES]):

- L1 = sketch.sketchCurves.sketchLines.addByTwoPoints(B0, F_0), the face at v = -hv.
- S = sketch.sketchCurves.sketchFittedSplines.add(fitPoints), the toothed side, where
  fitPoints = adsk.core.ObjectCollection.create() holds the sketch points F_0 to F_(M-1),
  each put in with fitPoints.add(F_j) in order of j. The spline runs from F_0, the end of
  L1, to F_(M-1), the start of L3. Raise naming the section unless S is not None and
  S.fitPoints.count is M.
- L3 = sketch.sketchCurves.sketchLines.addByTwoPoints(F_(M-1), B1), the face at v = +hv.
- L4 = sketch.sketchCurves.sketchLines.addByTwoPoints(B1, B0), the back side.

The build keeps each section's four curves together, in that order, because they are what the
loft is fed. After the last curve of the last section is drawn, every point in the sketch is set
isFixed = True: every point the build added, and every item of every spline's fitPoints, by
S.fitPoints.item(i) for i below S.fitPoints.count. The API reference says the spline takes
existing sketch points as fit points and does not say whether it keeps those objects or makes
its own, so both are fixed. Nothing else is in the sketch: no construction line, no constraint,
no dimension, and no tangent or curvature handle (activateTangentHandle and
activateCurvatureHandle are not called). Then set isComputeDeferred back to False, raise
unless the sketch reports isFullyConstrained, and raise unless sketch.profiles.count is
c*n + 1, one profile per section. The profiles are not what the loft is fed, because nothing
says which profile is which station.

The calls and their signatures, as the Fusion API reference database (fusion:query-api) gives
them:

| Call | Signature |
|---|---|
| SketchPoints.add | (point: core.Point3D) -> SketchPoint |
| Sketch.modelToSketchSpace | (modelCoordinate: core.Point3D) -> core.Point3D |
| SketchLines.addByTwoPoints | (startPoint: core.Base, endPoint: core.Base) -> SketchLine, either a SketchPoint or a Point3D |
| SketchFittedSplines.add | (fitPoints: core.ObjectCollection) -> SketchFittedSpline; "any combination of existing SketchPoint or Point3D objects"; None if it failed |
| ObjectCollection.create | () -> ObjectCollection, static |
| ObjectCollection.add | (item: Base) -> bool |
| SketchFittedSpline.fitPoints | SketchPointList, read-only, start point first and end point last; count: int, item(index: int) -> SketchPoint |
| SketchPoint.isFixed | bool, read/write, declared on SketchEntity |
| Sketch.isComputeDeferred | bool, read/write |
| Features.createPath | (curve: core.Base, isChain: bool = True) -> Path |
| LoftFeatures.createInput | (operation: FeatureOperations) -> LoftFeatureInput |
| LoftSections.add | (entity: core.Base) -> LoftSection; a Path is one of the entities it takes |
| LoftFeatures.add | (input: LoftFeatureInput) -> LoftFeature |
| BRepBody.pointContainment | (point: core.Point3D) -> PointContainment |

**What the loft sees.** Every section is one closed loop of the same four curves in the same
order — a line, a fitted spline, a line, a line — so every section has the same number of curves
and the same number of corners, and the loft meets the spline of one section with the spline of
the next. A section with a different count would leave the loft to pair unlike curves; the build
raises before the loft unless every section holds exactly the four curves above.

The required Fusion calls for this timeline entry are:

- `component.sketches.add(axisPlane)`.
- `sketch.modelToSketchSpace(worldPoint)`.
- `sketch.sketchPoints.add(localPoint)`.
- `sketch.sketchCurves.sketchLines.addByTwoPoints(B0, F0)`.
- `adsk.core.ObjectCollection.create()`.
- `fitPoints.add(fitPoint)`.
- `sketch.sketchCurves.sketchFittedSplines.add(fitPoints)`.
- `sketch.sketchCurves.sketchLines.addByTwoPoints(FLast, B1)`.
- `sketch.sketchCurves.sketchLines.addByTwoPoints(B1, B0)`.
- `spline.fitPoints.item(i)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "component.sketches",
      "role": "required",
      "span": "component.sketches.add(axisPlane)"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.modelToSketchSpace(worldPoint)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SketchPoints",
      "reason": null,
      "receiver": "sketch.sketchPoints",
      "role": "required",
      "span": "sketch.sketchPoints.add(localPoint)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "sketch.sketchCurves.sketchLines.addByTwoPoints(B0, F0)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "adsk.core.ObjectCollection",
      "role": "required",
      "span": "adsk.core.ObjectCollection.create()"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "fitPoints",
      "role": "required",
      "span": "fitPoints.add(fitPoint)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SketchFittedSplines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchFittedSplines",
      "role": "required",
      "span": "sketch.sketchCurves.sketchFittedSplines.add(fitPoints)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "sketch.sketchCurves.sketchLines.addByTwoPoints(FLast, B1)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "sketch.sketchCurves.sketchLines.addByTwoPoints(B1, B0)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.SketchPointList",
      "reason": null,
      "receiver": "spline.fitPoints",
      "role": "required",
      "span": "spline.fitPoints.item(i)"
    }
  ],
  "citations": [
    {
      "first": 991,
      "last": 1117,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L991–1117.

## S11 `[GO]` Per-gear tooth cell loft

The proof function is `stepCellLoft`.

<!-- proof-run: proofkit3d.RunSolid(cellCases, stepCellLoft, assertCellLoft) -->

One Fusion loft consumes all section paths in station order; the proof substitutes stitched, triangulated ruled walls.
The straight-ridge solid merges collinear tooth chords without changing its outline; the sketch retains all fit points.
[PB-LOFT] [PB-PATH-FROM-SKETCH] [PB-EMPTY-RESULT] [PB-SELF-DIAGNOSING] [PB-LOGGING]
[SCREW-F-CELL-LOFT]

**Loft the sections in station order** ([PB-LOFT], [SCREW-F-CELL-LOFT]). Create the loft
with loftFeatures.createInput(NewBodyFeatureOperation); for each section in order of k, put
its four curves in an ObjectCollection in the order L1, S, L3, L4, make its path with
features.createPath(collection, False) ([PB-PATH-FROM-SKETCH]; never Path.create, which
raises in this component), and loftSections.add(path); then loftFeatures.add(input). The
result is c pitches of the twisted toothed ribbon in one body; raise unless the feature leaves
exactly one body and that body isSolid ([PB-EMPTY-RESULT]). Measured on 2026-09-28 at the
10 mm ribbon, with four-line sections: the four-tooth loft took 0.23 s and held 0.164158 cm³
against the ruled loft's 0.164470. The loft through spline sections has not been timed.

**The screw step still holds.** Utooth(v, s + P) = Utooth(v, s) at every v: the slant moves
the cosine's phase by an amount that depends on v alone and the bow lowers the edge by an
amount that depends on v alone, so the section n steps on, one pitch along the axis, is
section k carried by Step(1) of §3, its toothed points included. The cell is still one
pitch-periodic piece of the ribbon, and the doubling of §3 and the remainder cell are unchanged;
TestRibbonIsInvariantUnderItsScrewStep carries every section's corners and toothed points
through one screw step and finds them on the next pitch's to 1e-9 mm.

**Checking the slant's sign** ([PB-SELF-DIAGNOSING]). Nothing in the sketch shows which way a
ridge leans, and a slant of the wrong sign builds a cell whose ridges cross the other ribbon's at
61° where the axes cross instead of lying along them ("The part"). So after the loft, before §3,
the build probes the cell with cellBody.pointContainment(point) at four points. Let
sc = Z0 + P*ceil((s0 + P/2 - Z0)/P), the first crest station at least half a pitch into the
cell. For each face sign σ of -1 and +1, let v_p = σ*(T/2 - 0.25 mm) and
u_p = W/2 - Bow*v_p^2 - 0.25 mm, a quarter millimetre inside the face and under the crest. The
**on-ridge** probe is the world point of §1 at (u_p, v_p) and station sc - tan(Slant)*v_p,
where the ridge through sc crosses v_p; the **off-ridge** probe is the same at station
sc + tan(Slant)*v_p, where that ridge would cross under the opposite sign. For each probe the
build evaluates m = Utooth(v_p, s) - u_p at the probe's station under the input slant and m'
under its negation. A probe is used when |m| and |m'| are both at least 0.1 mm and their signs
differ. A used probe must read PointInsidePointContainment when m > 0 and
PointOutsidePointContainment when m < 0; otherwise the build raises naming the gear, the
probe and what it read. When no probe is used, as with a slant near zero, the build logs with
futil.log ([PB-LOGGING]) that the sign was not checked. At the defaults all four are used: on
the ridge each stands 0.25 mm inside the tooth under the right sign and 2.13 mm outside under the
wrong one, and off the ridge the reverse; gear A's stand 1.84 and 3.41 mm from the cell's start,
inside its 10.5 mm (TestToothSlantProbesTellTheHand). Lengths are in cm in the build, as
everywhere.

The required Fusion calls for this timeline entry are:

- `component.features.loftFeatures.createInput(newBodyOperation)`.
- `adsk.core.ObjectCollection.create()`.
- `curves.add(curve)`.
- `component.features.createPath(curves, False)`.
- `loftInput.loftSections.add(sectionPath)`.
- `component.features.loftFeatures.add(loftInput)`.
- `cellBody.pointContainment(probe)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.LoftFeatures",
      "reason": null,
      "receiver": "component.features.loftFeatures",
      "role": "required",
      "span": "component.features.loftFeatures.createInput(newBodyOperation)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "adsk.core.ObjectCollection",
      "role": "required",
      "span": "adsk.core.ObjectCollection.create()"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "curves",
      "role": "required",
      "span": "curves.add(curve)"
    },
    {
      "condition": null,
      "name": "createPath",
      "owner": "adsk.fusion.Features",
      "reason": null,
      "receiver": "component.features",
      "role": "required",
      "span": "component.features.createPath(curves, False)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.LoftSections",
      "reason": null,
      "receiver": "loftInput.loftSections",
      "role": "required",
      "span": "loftInput.loftSections.add(sectionPath)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.LoftFeatures",
      "reason": null,
      "receiver": "component.features.loftFeatures",
      "role": "required",
      "span": "component.features.loftFeatures.add(loftInput)"
    },
    {
      "condition": null,
      "name": "pointContainment",
      "owner": "adsk.fusion.BRepBody",
      "reason": null,
      "receiver": "cellBody",
      "role": "required",
      "span": "cellBody.pointContainment(probe)"
    }
  ],
  "citations": [
    {
      "first": 1119,
      "last": 1156,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1119–1156.

## S12 `[GO]` Copy a ribbon cell block

The proof function is `stepCellCopy`.

<!-- proof-run: proofkit3d.RunSolid(cellCases, stepCellCopy, assertCellCopy) -->

Each copy is one timeline entry. Apply this entry for asides and doubling according to the binary schedule.
The proof displaces its actual duplicate by (1000,1000,1000) mm to avoid unsupported coincident pair contact.
It checks volume identity but cannot verify the initially coincident placement.
[SCREW-F-COPY-BODY] [PB-EMPTY-RESULT]

### 3: Repeat the cell by doubling

The finished ribbon is the cell, c teeth, repeated under the **screw step**

```
Step(k) = translate(k*P along the axis) ∘ rotate(k*P/Lambda about the axis)
```

Write N = q*c + r, with q = floor(N/c) whole cells and a remainder of r = N mod c teeth:
q = 17 and r = 0 at the defaults, q = 68 at a one-tooth cell. Build the q cells by
doubling rather than by q separate placements. A **round** is copy, move, join: copy the
current body, move the copy by Step(m*c) where m is the number of cells the body holds, join
the two, and the body holds 2m cells. Doubling alone reaches only a power of two, and copying
the whole body for the rest would overlap it, so the rest is made of unmoved copies put aside on
the way up:

- Write q in binary. Start with the cell, m = 1.
- For each bit of q below its top bit, lowest first: if that bit is set, take a copy of the
  body and keep it unmoved — an **aside** of m cells; then double.
- After the last doubling m is the top bit's power of two. Move each aside into place, largest
  first, by Step(m*c), join it, and add its cells to m. The last join brings m to q.

That is floor(log2 q) + popcount(q) - 1 rounds and as many joins: 5 at the defaults — q = 17,
one aside of the single cell, taken before the first doubling, then four doublings to 16 cells,
and the aside moved by Step(64) — and 7 at a one-tooth cell — q = 68, one aside of 4 cells,
taken when the body held 4, six doublings to 64, and the aside moved by Step(64) — against 67
placements. When q = 1 there is no round. Every placement is
exact, because the body genuinely is invariant under Step
(TestRibbonIsInvariantUnderItsScrewStep). Every move is by a whole number of teeth already
built, so each join meets its neighbour at one shared planar cross-section and leaves no sliver
and no overlap; TestDoublingScheduleCoversTheRibbon runs the schedule at every count from 4 to
512 with cells of one, three and four teeth.

**The remainder.** When r > 0, the last r teeth are a second, shorter cell, built where they
belong rather than moved there: a sketch {gearLabel} Cell Remainder and a loft by the recipe of
§2 with c replaced by r, so r*n + 1 sections at stations from s0 + q*c*P to s0 + N*P,
joined into the body after the last round. Its first section is the body's last, so the join
meets at a shared cross-section like every other. The defaults have none; 69 teeth in
four-tooth cells would have one of one tooth.

Copy with copyPasteBodies.add(body) and take the copy from the feature's bodies
([SCREW-F-COPY-BODY]); move with moveFeatures.createInput2(bodies) → defineAsFreeMove(matrix)
→ add(input) ([PB-MOVE-ROTATE], [SCREW-F-SCREW-STEP]); join with a combineFeatures join and
check that it leaves exactly one body: the combine feature's own bodies.count is 1, and that
body is the ribbon from then on ([PB-EMPTY-RESULT], [SCREW-F-JOIN]). After the last
join name the body Gear A or Gear B.

**A zero-angle matrix is a no-op that Fusion rejects** ([PB-MOVE-ROTATE]). A screw step is never
zero for k >= 1, so no guard is needed here, but do not "optimize" a k = 0 case into the loop.

The required Fusion calls for this timeline entry are:

- `component.features.copyPasteBodies.add(sourceBody)`.
- `copyFeature.bodies.item(0)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CopyPasteBodies",
      "reason": null,
      "receiver": "component.features.copyPasteBodies",
      "role": "required",
      "span": "component.features.copyPasteBodies.add(sourceBody)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "copyFeature.bodies",
      "role": "required",
      "span": "copyFeature.bodies.item(0)"
    }
  ],
  "citations": [
    {
      "first": 1158,
      "last": 1206,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1158–1206.

## S13 `[GO]` Screw-move a copied block

The proof function is `stepCellMove`.

<!-- proof-run: proofkit3d.RunSolid(cellCases, stepCellMove, assertCellMove) -->

Each move is one timeline entry. k is m*c teeth; never a zero displacement.
Construct a second identity matrix, assign its translation to a copied axis vector scaled by k*pitch,
and compose it with the axis rotation. Never overwrite the rotation matrix translation.
[PB-MOVE-ROTATE] [SCREW-F-SCREW-STEP]

### 3: Repeat the cell by doubling

The finished ribbon is the cell, c teeth, repeated under the **screw step**

```
Step(k) = translate(k*P along the axis) ∘ rotate(k*P/Lambda about the axis)
```

Write N = q*c + r, with q = floor(N/c) whole cells and a remainder of r = N mod c teeth:
q = 17 and r = 0 at the defaults, q = 68 at a one-tooth cell. Build the q cells by
doubling rather than by q separate placements. A **round** is copy, move, join: copy the
current body, move the copy by Step(m*c) where m is the number of cells the body holds, join
the two, and the body holds 2m cells. Doubling alone reaches only a power of two, and copying
the whole body for the rest would overlap it, so the rest is made of unmoved copies put aside on
the way up:

- Write q in binary. Start with the cell, m = 1.
- For each bit of q below its top bit, lowest first: if that bit is set, take a copy of the
  body and keep it unmoved — an **aside** of m cells; then double.
- After the last doubling m is the top bit's power of two. Move each aside into place, largest
  first, by Step(m*c), join it, and add its cells to m. The last join brings m to q.

That is floor(log2 q) + popcount(q) - 1 rounds and as many joins: 5 at the defaults — q = 17,
one aside of the single cell, taken before the first doubling, then four doublings to 16 cells,
and the aside moved by Step(64) — and 7 at a one-tooth cell — q = 68, one aside of 4 cells,
taken when the body held 4, six doublings to 64, and the aside moved by Step(64) — against 67
placements. When q = 1 there is no round. Every placement is
exact, because the body genuinely is invariant under Step
(TestRibbonIsInvariantUnderItsScrewStep). Every move is by a whole number of teeth already
built, so each join meets its neighbour at one shared planar cross-section and leaves no sliver
and no overlap; TestDoublingScheduleCoversTheRibbon runs the schedule at every count from 4 to
512 with cells of one, three and four teeth.

**The remainder.** When r > 0, the last r teeth are a second, shorter cell, built where they
belong rather than moved there: a sketch {gearLabel} Cell Remainder and a loft by the recipe of
§2 with c replaced by r, so r*n + 1 sections at stations from s0 + q*c*P to s0 + N*P,
joined into the body after the last round. Its first section is the body's last, so the join
meets at a shared cross-section like every other. The defaults have none; 69 teeth in
four-tooth cells would have one of one tooth.

Copy with copyPasteBodies.add(body) and take the copy from the feature's bodies
([SCREW-F-COPY-BODY]); move with moveFeatures.createInput2(bodies) → defineAsFreeMove(matrix)
→ add(input) ([PB-MOVE-ROTATE], [SCREW-F-SCREW-STEP]); join with a combineFeatures join and
check that it leaves exactly one body: the combine feature's own bodies.count is 1, and that
body is the ribbon from then on ([PB-EMPTY-RESULT], [SCREW-F-JOIN]). After the last
join name the body Gear A or Gear B.

**A zero-angle matrix is a no-op that Fusion rejects** ([PB-MOVE-ROTATE]). A screw step is never
zero for k >= 1, so no guard is needed here, but do not "optimize" a k = 0 case into the loop.

The required Fusion calls for this timeline entry are:

- `adsk.core.Matrix3D.create()`.
- `rotation.setToRotation(k * pitch / lam, axisVector, axisPoint)`.
- `axisVector.copy()`.
- `shift.scaleBy(k * pitch)`.
- `rotation.transformBy(translationMatrix)`.
- `adsk.core.ObjectCollection.create()`.
- `bodies.add(copiedBody)`.
- `component.features.moveFeatures.createInput2(bodies)`.
- `moveInput.defineAsFreeMove(rotation)`.
- `component.features.moveFeatures.add(moveInput)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Matrix3D",
      "reason": null,
      "receiver": "adsk.core.Matrix3D",
      "role": "required",
      "span": "adsk.core.Matrix3D.create()"
    },
    {
      "condition": null,
      "name": "setToRotation",
      "owner": "adsk.core.Matrix3D",
      "reason": null,
      "receiver": "rotation",
      "role": "required",
      "span": "rotation.setToRotation(k * pitch / lam, axisVector, axisPoint)"
    },
    {
      "condition": null,
      "name": "copy",
      "owner": "adsk.core.Vector3D",
      "reason": null,
      "receiver": "axisVector",
      "role": "required",
      "span": "axisVector.copy()"
    },
    {
      "condition": null,
      "name": "scaleBy",
      "owner": "adsk.core.Vector3D",
      "reason": null,
      "receiver": "shift",
      "role": "required",
      "span": "shift.scaleBy(k * pitch)"
    },
    {
      "condition": null,
      "name": "transformBy",
      "owner": "adsk.core.Matrix3D",
      "reason": null,
      "receiver": "rotation",
      "role": "required",
      "span": "rotation.transformBy(translationMatrix)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "adsk.core.ObjectCollection",
      "role": "required",
      "span": "adsk.core.ObjectCollection.create()"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "bodies",
      "role": "required",
      "span": "bodies.add(copiedBody)"
    },
    {
      "condition": null,
      "name": "createInput2",
      "owner": "adsk.fusion.MoveFeatures",
      "reason": null,
      "receiver": "component.features.moveFeatures",
      "role": "required",
      "span": "component.features.moveFeatures.createInput2(bodies)"
    },
    {
      "condition": null,
      "name": "defineAsFreeMove",
      "owner": "adsk.fusion.MoveFeatureInput",
      "reason": null,
      "receiver": "moveInput",
      "role": "required",
      "span": "moveInput.defineAsFreeMove(rotation)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.MoveFeatures",
      "reason": null,
      "receiver": "component.features.moveFeatures",
      "role": "required",
      "span": "component.features.moveFeatures.add(moveInput)"
    }
  ],
  "citations": [
    {
      "first": 1158,
      "last": 1206,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1158–1206.

## S14 `[GO]` Join a moved block to the ribbon

The proof function is `stepCellJoin`.

<!-- proof-run: proofkit3d.RunSolid(cellCases, stepCellJoin, assertCellJoin) -->

Each join is one timeline entry. Set JoinFeatureOperation and isKeepToolBodies=False.
Require bodies.count=1 and take that feature body as the next ribbon.
Decad refuses booleans on stitched boundaries; the proof builds the combined sampled outer boundary directly.
It verifies doubled volume and one solid lump, but cannot verify tool consumption or target identity.
When r>0, instantiate Cell Sections and Cell Loft with r teeth at s0+q*c*P, then instantiate this join.
[PB-EMPTY-RESULT] [PB-SELF-DIAGNOSING] [SCREW-F-JOIN]

### 3: Repeat the cell by doubling

The finished ribbon is the cell, c teeth, repeated under the **screw step**

```
Step(k) = translate(k*P along the axis) ∘ rotate(k*P/Lambda about the axis)
```

Write N = q*c + r, with q = floor(N/c) whole cells and a remainder of r = N mod c teeth:
q = 17 and r = 0 at the defaults, q = 68 at a one-tooth cell. Build the q cells by
doubling rather than by q separate placements. A **round** is copy, move, join: copy the
current body, move the copy by Step(m*c) where m is the number of cells the body holds, join
the two, and the body holds 2m cells. Doubling alone reaches only a power of two, and copying
the whole body for the rest would overlap it, so the rest is made of unmoved copies put aside on
the way up:

- Write q in binary. Start with the cell, m = 1.
- For each bit of q below its top bit, lowest first: if that bit is set, take a copy of the
  body and keep it unmoved — an **aside** of m cells; then double.
- After the last doubling m is the top bit's power of two. Move each aside into place, largest
  first, by Step(m*c), join it, and add its cells to m. The last join brings m to q.

That is floor(log2 q) + popcount(q) - 1 rounds and as many joins: 5 at the defaults — q = 17,
one aside of the single cell, taken before the first doubling, then four doublings to 16 cells,
and the aside moved by Step(64) — and 7 at a one-tooth cell — q = 68, one aside of 4 cells,
taken when the body held 4, six doublings to 64, and the aside moved by Step(64) — against 67
placements. When q = 1 there is no round. Every placement is
exact, because the body genuinely is invariant under Step
(TestRibbonIsInvariantUnderItsScrewStep). Every move is by a whole number of teeth already
built, so each join meets its neighbour at one shared planar cross-section and leaves no sliver
and no overlap; TestDoublingScheduleCoversTheRibbon runs the schedule at every count from 4 to
512 with cells of one, three and four teeth.

**The remainder.** When r > 0, the last r teeth are a second, shorter cell, built where they
belong rather than moved there: a sketch {gearLabel} Cell Remainder and a loft by the recipe of
§2 with c replaced by r, so r*n + 1 sections at stations from s0 + q*c*P to s0 + N*P,
joined into the body after the last round. Its first section is the body's last, so the join
meets at a shared cross-section like every other. The defaults have none; 69 teeth in
four-tooth cells would have one of one tooth.

Copy with copyPasteBodies.add(body) and take the copy from the feature's bodies
([SCREW-F-COPY-BODY]); move with moveFeatures.createInput2(bodies) → defineAsFreeMove(matrix)
→ add(input) ([PB-MOVE-ROTATE], [SCREW-F-SCREW-STEP]); join with a combineFeatures join and
check that it leaves exactly one body: the combine feature's own bodies.count is 1, and that
body is the ribbon from then on ([PB-EMPTY-RESULT], [SCREW-F-JOIN]). After the last
join name the body Gear A or Gear B.

**A zero-angle matrix is a no-op that Fusion rejects** ([PB-MOVE-ROTATE]). A screw step is never
zero for k >= 1, so no guard is needed here, but do not "optimize" a k = 0 case into the loop.

The required Fusion calls for this timeline entry are:

- `adsk.core.ObjectCollection.create()`.
- `tools.add(toolBody)`.
- `component.features.combineFeatures.createInput(targetBody, tools)`.
- `component.features.combineFeatures.add(combineInput)`.
- `combineFeature.bodies.item(0)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "adsk.core.ObjectCollection",
      "role": "required",
      "span": "adsk.core.ObjectCollection.create()"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "tools",
      "role": "required",
      "span": "tools.add(toolBody)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "component.features.combineFeatures",
      "role": "required",
      "span": "component.features.combineFeatures.createInput(targetBody, tools)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "component.features.combineFeatures",
      "role": "required",
      "span": "component.features.combineFeatures.add(combineInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "combineFeature.bodies",
      "role": "required",
      "span": "combineFeature.bodies.item(0)"
    }
  ],
  "citations": [
    {
      "first": 1158,
      "last": 1206,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1158–1206.

## S15 `[GO]` Sleeve sketch

The proof function is `stepSleeveSketch`.

<!-- proof-run: proofkit.Run(sleeveCases, stepSleeveSketch) -->

[PB-USE-SELECTED-PLANE] [PB-SKETCH-ZERO-Z] [PB-CIRCLE-CENTER] [PB-RADIAL-DIM]
[PB-EMPTY-RESULT] [SCREW-F-SLEEVE]

### 4: The cage

The cage is the **sleeve**: one tube about the frame's axis, the four bores cut through its wall
by twisted sweeps, and up to two windows cut through the wall by extrusions. Heights are along
n̂ from the centre C; gear B's axis is on the +n̂ side and gear A's on the -n̂ side.
proof/screwgear/sleeve_test.go proves the shape, and where the build follows one of its
functions step for step the function is named. Its lengths, with their values at the defaults:

```
Ri   = cageRadius - collarHalf                inner radius, 12 mm
Ro   = cageRadius + collarHalf                outer radius, 18 mm
hw   = W/2 + clearance                        the bore's half-width, 7.70 mm
ht   = T/2 + clearance                        the bore's half-thickness, 2.075 mm
a    = roofAllowance                          the roof allowance, 0.30 mm
c    = hypot(hw, ht + a)                      the furthest any bore's corner stands from its axis, 8.058 mm
sIn  = sqrt(Ri^2 - c^2) - 1 mm                where a +R bore's cut starts on its axis, 7.892 mm
sOut = Ro + 1 mm                              where it ends, 19 mm
```

The sleeve prints standing on its **−n̂ end**, the end below the selected plane, with no
support. Every outside face is a vertical cylinder or a level end face and every face of a
window is upright or at 45°, so the only material the printer lays on air is the roofs of the
four bores: 1429 cells of the proof's 0.25 mm grid, 89 mm², standing on that end
(TestSleevePrintsStandingOnEitherEnd). Standing on the other end it prints too, 1428 cells,
but the roof allowance is then on the floors and the bridged roofs keep only the clearance.

**The tube** ([SCREW-F-SLEEVE]). One sketch, Sleeve, on the selected plane
([PB-USE-SELECTED-PLANE]): two circles addByCenterRadius at C, mapped in with
modelToSketchSpace and given z = 0 ([PB-SKETCH-ZERO-Z]), of radii Ri and Ro, each with
its centerSketchPoint.isFixed = True ([PB-CIRCLE-CENTER]) and a diameter dimension of 2*Ri
or 2*Ro whose text point sits on its circle, off the centre ([PB-RADIAL-DIM]). Nothing else is
in the sketch. It has two profiles, the inner disc and the ring between the circles; the ring is
the profile whose profileLoops.count is 2, and the build raises unless exactly one profile has
two loops ([PB-EMPTY-RESULT]). find_profile_by_curve_counts cannot pick it, since it treats a
circle as a curve type that disqualifies a loop. Extrude the ring as a new body with
setSymmetricExtent(ValueInput.createByReal(cageRise), False), False making the value each
side's length ([PB-THROUGH-CUT] for the argument), so the tube runs from -cageRise to
+cageRise: 37.5 mm tall with a flat 565.5 mm² ring at each end. Raise unless the feature's
bodies.count is 1. That body is the cage from here on.

The required Fusion calls for this timeline entry are:

- `component.sketches.add(targetPlane)`.
- `sketch.modelToSketchSpace(C)`.
- `sketch.sketchCurves.sketchCircles.addByCenterRadius(localCentre, radius)`.
- `sketch.sketchDimensions.addDiameterDimension(circle, textPoint)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "component.sketches",
      "role": "required",
      "span": "component.sketches.add(targetPlane)"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.modelToSketchSpace(C)"
    },
    {
      "condition": null,
      "name": "addByCenterRadius",
      "owner": "adsk.fusion.SketchCircles",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchCircles",
      "role": "required",
      "span": "sketch.sketchCurves.sketchCircles.addByCenterRadius(localCentre, radius)"
    },
    {
      "condition": null,
      "name": "addDiameterDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addDiameterDimension(circle, textPoint)"
    }
  ],
  "citations": [
    {
      "first": 1208,
      "last": 1246,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1208–1246.

## S16 `[GO]` Extrude the sleeve ring

The proof function is `stepSleeveExtrude`.

<!-- proof-run: proofkit3d.RunSolid(tubeCases, stepSleeveExtrude, assertSleeveExtrude) -->

Select the unique profile with profileLoops.count=2.
[PB-THROUGH-CUT] [PB-EMPTY-RESULT] [SCREW-F-SLEEVE]

**The tube** ([SCREW-F-SLEEVE]). One sketch, Sleeve, on the selected plane
([PB-USE-SELECTED-PLANE]): two circles addByCenterRadius at C, mapped in with
modelToSketchSpace and given z = 0 ([PB-SKETCH-ZERO-Z]), of radii Ri and Ro, each with
its centerSketchPoint.isFixed = True ([PB-CIRCLE-CENTER]) and a diameter dimension of 2*Ri
or 2*Ro whose text point sits on its circle, off the centre ([PB-RADIAL-DIM]). Nothing else is
in the sketch. It has two profiles, the inner disc and the ring between the circles; the ring is
the profile whose profileLoops.count is 2, and the build raises unless exactly one profile has
two loops ([PB-EMPTY-RESULT]). find_profile_by_curve_counts cannot pick it, since it treats a
circle as a curve type that disqualifies a loop. Extrude the ring as a new body with
setSymmetricExtent(ValueInput.createByReal(cageRise), False), False making the value each
side's length ([PB-THROUGH-CUT] for the argument), so the tube runs from -cageRise to
+cageRise: 37.5 mm tall with a flat 565.5 mm² ring at each end. Raise unless the feature's
bodies.count is 1. That body is the cage from here on.

The required Fusion calls for this timeline entry are:

- `component.features.extrudeFeatures.createInput(ring, newBodyOperation)`.
- `adsk.core.ValueInput.createByReal(cageRise)`.
- `extrudeInput.setSymmetricExtent(riseValue, False)`.
- `component.features.extrudeFeatures.add(extrudeInput)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "component.features.extrudeFeatures",
      "role": "required",
      "span": "component.features.extrudeFeatures.createInput(ring, newBodyOperation)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(cageRise)"
    },
    {
      "condition": null,
      "name": "setSymmetricExtent",
      "owner": "adsk.fusion.ExtrudeFeatureInput",
      "reason": null,
      "receiver": "extrudeInput",
      "role": "required",
      "span": "extrudeInput.setSymmetricExtent(riseValue, False)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "component.features.extrudeFeatures",
      "role": "required",
      "span": "component.features.extrudeFeatures.add(extrudeInput)"
    }
  ],
  "citations": [
    {
      "first": 1234,
      "last": 1246,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1234–1246.

## S17 `[PROSE]` Per-bore plane

Instantiate one plane per bore in order A -R, A +R, B -R, B +R.
Its boreLine is pathLines[g][bore-] or pathLines[g][bore+] for that precise crossing.
[PB-CONSTRUCTION-PLANES] [SCREW-F-TWISTED-SLOT]

**The bores.** One per crossing: a gear's +R bore runs through the wall where its axis crosses
the circle of radius cageRadius, at station +cageRadius of the axis, and its -R bore at
-cageRadius. Each is the crest rectangle plus the clearance, 2*hw by 2*ht, 15.4 by 4.15 mm,
turned at every station s to the ribbon's own angle s/Lambda + Phi_g. That is the channel the
video frame's collars were cut with, and TestSleeveBoresAreTheSameChannels pins it: the
rectangle, the stations, the twist, the angles at the crossings, 109.09° and −109.09° (123.1°
and −95.1° at the 14° mounting angles before 2026-10-03), and which
bore carries the roof allowance on which face. Only the span the cut covers is the sleeve's own.
The tube's inside is a cylinder, not a plane square to the ribbon, so the channel's corners reach
into the wall before its centre line does: a corner first touches the inner face by station
sqrt(Ri^2 - c^2), 8.892 mm. The cut runs from a millimetre before that, sIn, where the whole
section is in the hollow, to a millimetre past the outer face, sOut, where the whole section is
outside: [sIn, sOut] for a +R bore and [-sOut, -sIn] for a -R bore, 11.108 mm of axis and
80.78° of turn. The assembly phase moves
gear B's ribbon along its axis and not its bores: any stretch of the ribbon fits a bore, and the
ribbon can go in at any phase a pitch apart, so the frame has no way to know the phase.

**The roof allowance** (levelBore, roofSide, boreOpening). With the sleeve standing on its
−n̂ end, a bore whose long faces pass through level inside the wall has its roof bridged across
the hole by the printer. At the 14° mounting angles of the second sleeve that was each gear's
-R bore alone, whose long faces lay flat at station −14.30 mm, and the second sleeve printed
those two bores too tight and the two +R bores not ([SCREW-F-PRINT-2]). So each gear's
**level bore** has its roof face moved out by roofAllowance, and every other face of every bore
stays at the clearance. At the defaults' zero mounting angles **both** bores of each gear pass
through level, at stations ±12.37 mm, so the two tie and the -R bore takes the allowance; the
+R bores' roofs are bridged 15.4 mm across with the clearance alone, as the second sleeve's
tight roofs were ("What the print showed", "What the proof cannot reach"):

- The level bore is, of the gear's two bores, the one whose long faces come nearest level over
  the wall's span on its centre line. A long face runs along the section's u, which stands
  theta from û_g, and û_g is along ±n̂, so a long face is level where cos(theta) is zero.
  For the bore at σ*cageRadius, σ being -1 or +1, take theta at the two stations
  σ*(cageRadius - collarHalf) and σ*(cageRadius + collarHalf); when some pi/2 + k*pi lies
  between them its tilt is 0, and otherwise it is the smaller |cos theta| of the two. The bore
  with the smaller tilt is the level bore, the -R bore when they tie. At the defaults both
  bores' tilt is 0, so the -R bore is the level bore; at the 14° of the third print the -R
  bores' tilt was 0 and the +R bores' |cos 101.3°|, 0.195.
- The roof face is the long face that is up when the sleeve stands on its −n̂ end: the +v
  face when v̂(sc)·n̂ = -sin(theta(sc))*(û_g·n̂) is positive at the bore's crossing station
  sc = σ*cageRadius, and the -v face otherwise. v̂_g·n̂ is zero, so that is the whole
  product. At the defaults it is gear A's +v face and gear B's -v face. The sign holds over
  the whole cut: v̂·n̂ changes sign only where a long face stands upright, 90° of twist, 12.4 mm
  of axis, from where it lies level.
- The level bore's rectangle spans v from vLo = -ht to vHi = ht + a when its roof is the
  +v face, and from vLo = -ht - a to vHi = ht otherwise: 4.45 mm through at the defaults.
  Every other bore spans -ht to ht, and every bore spans u from -hw to hw.

The allowance adds no move of the ribbon along or across the axes and no roll: the other bore and
the level bore's floor hold the ribbon to the clearance there. It lets the ribbon tilt, its level
bore's end rising into the room while the other bore holds, which carries the crossing 0.248 mm
instead of 0.200 mm toward the other ribbon for gear A and away from it for gear B
(TestRoofAllowanceAddsOnlyATilt). After the sleeve is built the build logs, with futil.log
([PB-LOGGING]), `Print the cage standing on its end below the selected plane: the roof
allowance is on the bridged roofs that way up.`, so the print orientation is explicit even when
the bore marks are hard to see.

**Cut each bore with a twisted sweep** ([SCREW-F-TWISTED-SLOT], [PB-SWEEP-TWIST]): the bore's
rectangle, drawn by the rectangle scheme below on the bore's own plane,
{gearLabel} Bore {-R|+R} Plane, setByDistanceOnPath at fraction 0 of the bore's bore- or
bore+ line of §1, in the sketch {gearLabel} Bore {-R|+R}, at that plane's station, -sOut or
sIn; then swept along that line as `sweepFeatures.createInput(profile, path,
CutFeatureOperation), with path = features.createPath(line, False), twistAngle` set to
ValueInput.createByReal(+(sOut - sIn)/Lambda), participantBodies set to a list holding the
cage body alone, and nothing else set; then sweepFeatures.add. The sign is positive: the path
runs along +dir_g, the section's angle s/Lambda + Phi grows with s, and a positive
twistAngle turns the profile that way, as measured on 2026-09-28 on the video frame's collars
([PB-SWEEP-TWIST]). Each profile starts in air, in the hollow for a +R bore and outside the
tube for a -R bore, and each cut turns 80°; the cuts measured in Fusion turned 58–65° and
started on a collar's face. The add-in at c8a63b5 built such cuts at the earlier 82.31°, and both
printed ribbons screwed through them ([SCREW-F-PRINT-MESH]). The ribbons pass through the
channel and are not on the participant list, so the cut leaves them whole: measured on
2026-09-28, a body wholly inside the channel and off the list kept its volume to the last digit
while the cut body lost the channel's. The bore is the exact helicoid — the rectangle turning
rigidly at 1/Lambda about the axis, the channel TestRibbonsStayInsideTheirBoresOverTheTravel
and TestSleeveAdmitsOnlyTheScrewMotion measure against — with no facets and no fit between
samples. The two bores of one gear stand 2*cageRadius apart and are two sweeps on two lines, as
§1 draws them.

The required Fusion calls for this timeline entry are:

- `component.constructionPlanes.createInput()`.
- `adsk.core.ValueInput.createByReal(0)`.
- `planeInput.setByDistanceOnPath(boreLine, zeroValue)`.
- `component.constructionPlanes.add(planeInput)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "component.constructionPlanes",
      "role": "required",
      "span": "component.constructionPlanes.createInput()"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(0)"
    },
    {
      "condition": null,
      "name": "setByDistanceOnPath",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByDistanceOnPath(boreLine, zeroValue)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "component.constructionPlanes",
      "role": "required",
      "span": "component.constructionPlanes.add(planeInput)"
    }
  ],
  "citations": [
    {
      "first": 1248,
      "last": 1324,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1248–1324.

## S18 `[GO]` Per-bore rectangle sketch

The proof function is `stepBoreSketch`.

<!-- proof-run: proofkit.Run(boreCases, stepBoreSketch) -->

Each sketch is one entry and follows its own plane immediately before its sweep.
Seed every corner at its exact rotated coordinates, set local z=0, and use no shared-point coincidences.
Angle wedge rays are O→Cp and O→E when K is the angular operand. For L2 use its +v ray and the
ray of Ru from the intersection with L2 toward Cp or O that keeps the wedge in 45°–135°.
Place the text point strictly inside that seeded wedge and assign its positive angle magnitude.
[PB-SKETCH-FIRST] [PB-ANGULAR-DIM] [PB-OFFSET-DIM] [PB-NO-OVERCONSTRAIN]
[PB-SEED-NEAR] [PB-DIM-VALUE-SEMANTICS] [PB-PROFILE-MATCH] [PB-SHARE-XOR-COINCIDENT]
[PB-SKETCH-ZERO-Z] [PB-SKETCH-DEFER] [SCREW-F-DEFER] [SCREW-F-TWISTED-SLOT]

**The rectangle scheme.** Each of the four bore section sketches is a rectangle spanning v from
vLo to vHi and u from uB to uF, turned by theta = s/Lambda + Phi_g about the axis point
O, which lies on the line v = 0; for a bore other than a level one, at its centre, and for a
level bore a/2 off it across the thickness. Four lines, two
dimensions, one coincidence and one angle cannot fix O inside such a rectangle; a construction
spine through O can, and this is the scheme:

- **References.** Two reference points (Sketch Discipline): O, the point where the sweep
  path pierces the sketch plane, at origin_g + s*dir_g for the plane's station s, and
  Cp, at origin_g + s*dir_g + (A/2)*û_g, which is A/2 from O along the gear's
  unrotated û and lies on the plane because û_g does. The construction line Ru from O
  to Cp, sharing both, is the sketch's zero of rotation; it has no freedom once its ends are
  fixed. Both points are set isFixed = True after Ru and the spine K are drawn and before
  any dimension is added. Nothing is projected into a section sketch: the Anchor Line's
  projection would run through Cp across the rectangle at u = A/2, which at the defaults
  is inside the rectangle and would split the profile, and [PB-PROJECT-NOT-FIXED] rules a
  projected point out as an anchor in any case.
- **The spine.** A construction line K from O to E, seeded at (uF, 0) turned by
  theta, with a distance dimension O–E of uF.
- **The angle.** An angular dimension between Ru and K when |sin theta| >= sqrt(1/2),
  where the angle is theta folded into 45°–135°, and otherwise between Ru and the toothed
  side L2, which stands at theta + 90° (Sketch Discipline).
- **The rectangle.** Four lines sharing their corners: L1 from (uB, vLo) to (uF, vLo),
  L2 on to (uF, vHi), L3 on to (uB, vHi), L4 back to the start, every seed the solved
  point ([PB-SHARE-XOR-COINCIDENT]: shared, no coincident on a corner). Then L1 parallel to
  K with an offset dimension of -vLo, and L3 on the other side with one of vHi; E coincident on
  L2, and L2 perpendicular to K; L4 parallel to L2 with an offset dimension of
  uF - uB ([PB-OFFSET-DIM], [PB-NO-OVERCONSTRAIN]).

Ten degrees of freedom — E and the four corners — against ten rows: five dimensions (the
length, the angle, three offsets) and five constraints (three parallels, a perpendicular, a
coincidence), so the sketch closes fully constrained. In every one of the four sketches
uB = -hw, uF = hw, and vLo and vHi are the bore's own from "The roof allowance", and its
four lines are solid and are
the profile, the one loop of four lines, find_profile_by_curve_counts(sketch, lines=4)
([PB-PROFILE-MATCH]). The construction lines bound nothing. The scheme is drawn with sketch
computing deferred (Sketch Discipline); the video frame's collar sections, drawn by the same
scheme, read fully constrained in Fusion on 2026-09-28.


The required Fusion calls for this timeline entry are:

- `component.sketches.add(borePlane)`.
- `sketch.modelToSketchSpace(worldPoint)`.
- `sketch.sketchPoints.add(localPoint)`.
- `sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)`.
- `sketch.sketchDimensions.addDistanceDimension(O, E, alignedOrientation, lengthText)`.
- `sketch.sketchDimensions.addAngularDimension(Ru, K, angleText)`.
- `sketch.sketchDimensions.addAngularDimension(Ru, L2, angleText)`.
- `sketch.geometricConstraints.addParallel(L1, K)`.
- `sketch.geometricConstraints.addParallel(L3, K)`.
- `sketch.geometricConstraints.addParallel(L4, L2)`.
- `sketch.geometricConstraints.addCoincident(E, L2)`.
- `sketch.geometricConstraints.addPerpendicular(L2, K)`.
- `sketch.sketchDimensions.addOffsetDimension(K, L1, text1)`.
- `sketch.sketchDimensions.addOffsetDimension(K, L3, text3)`.
- `sketch.sketchDimensions.addOffsetDimension(L2, L4, text4)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "component.sketches",
      "role": "required",
      "span": "component.sketches.add(borePlane)"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.modelToSketchSpace(worldPoint)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SketchPoints",
      "reason": null,
      "receiver": "sketch.sketchPoints",
      "role": "required",
      "span": "sketch.sketchPoints.add(localPoint)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)"
    },
    {
      "condition": null,
      "name": "addDistanceDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addDistanceDimension(O, E, alignedOrientation, lengthText)"
    },
    {
      "condition": null,
      "name": "addAngularDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addAngularDimension(Ru, K, angleText)"
    },
    {
      "condition": null,
      "name": "addAngularDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addAngularDimension(Ru, L2, angleText)"
    },
    {
      "condition": null,
      "name": "addParallel",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addParallel(L1, K)"
    },
    {
      "condition": null,
      "name": "addParallel",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addParallel(L3, K)"
    },
    {
      "condition": null,
      "name": "addParallel",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addParallel(L4, L2)"
    },
    {
      "condition": null,
      "name": "addCoincident",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addCoincident(E, L2)"
    },
    {
      "condition": null,
      "name": "addPerpendicular",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addPerpendicular(L2, K)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addOffsetDimension(K, L1, text1)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addOffsetDimension(K, L3, text3)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addOffsetDimension(L2, L4, text4)"
    }
  ],
  "citations": [
    {
      "first": 1326,
      "last": 1364,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1326–1364.

## S19 `[GO]` Per-bore twisted sweep cut

The proof function is `stepBoreSweep`.

<!-- proof-run: proofkit3d.RunSolid(boreSolidCases, stepBoreSweep, assertBoreSweep) -->

Use exactly cageBody in participantBodies. Set twistAngle to the positive span/lam ValueInput.
Set no orientation, solid twist axis, rail or guide surface.
The proof substitutes a chorded, rotated-rectangle tool and cuts each bore from a fresh sleeve.
It verifies material removal and solid topology; it does not verify cumulative four-bore clearance or containment probes.
[PB-SWEEP-TWIST] [PB-PATH-FROM-SKETCH] [PB-SELF-DIAGNOSING] [PB-EMPTY-RESULT]
[SCREW-F-TWISTED-SLOT] [SCREW-F-SWEEP-CHECK]

proof builds each bore's channel as a chain of two-section lofts through rotated rectangles ("What the
proof checks"), and the count it lofts through is derived from the turn and the clearance: no two
neighbouring sections more than 5° of twist apart, nor more than the angle at which the facets between them
take 4% of the clearance, 2*acos(1 - 0.04*clearance/c), and the count is ceil(turn / step) + 1 for the
smaller step. At the defaults the turn is 80.78°, the facet bound 5.1° and 5° governs: 18 sections. At a
clearance of 0.05 mm the turn is 79.58°, the facet bound 2.6° governs, and the count is 32. Those facets
are a ruled wall's: flat between sections, inside the true channel, and 0.007 mm of the 0.20 mm clearance
at the derived count, which TestSleeveBoreSubstituteKeepsItsClearance holds at 95% of the clearance from
0.05 mm to 0.9 mm. decad does not build that wall: it walls each cell with two flat triangles, which
depart from it by up to a quarter of the cell's twist, 0.32 mm on the long faces at the defaults, more than
the clearance (the same test logs it). So the compiled proof reads the cage's volume and the build's probes
off the stand-in, and no clearance. The build derives no section count: its channel has none.

**The bore has to twist; a straight hole binds.** Over the wall's 2*collarHalf the ribbon turns
by 2*collarHalf/Lambda, so its corner sweeps (W/2)*(2*collarHalf/Lambda) across the opening.
At the defaults that is 5.7 mm against 0.20 mm of clearance.

**The bore is cut to the crest rectangle, never to a tooth.** The crests are the ribbon's outer
edge, so that rectangle holds the whole ribbon, teeth included, and its toothed side bears on the
crests ("Why the cage needs no tooth-shaped cut"); nothing in the frame is shaped like a tooth.

**The sweep's sense is checked, not assumed** ([SCREW-F-SWEEP-CHECK], [PB-SELF-DIAGNOSING]).
After each bore cut the build requires the sweep feature to leave exactly one body and probes the
cage with pointContainment at the two points origin_g + sc*dir_g ± (W/2 + clearance/2)*û(sc)
at the crossing itself, sc = ±cageRadius, where û(s) is the section's turned u direction,
cos(theta)*û_g + sin(theta)*v̂_g with theta = s/Lambda + Phi_g. Both must be
PointOutsidePointContainment, the channel being open there, and the build raises naming the
bore otherwise. The probes stand 16.63 mm from the frame's axis at all four bores,
inside the wall, and clear of the ribbon and of the channel's wall by clearance/2. They tell
the two senses apart on their own. The profile sits at the cut's first station s0, sIn or
-sOut, and under the wrong sense the channel at the crossing is turned 2*(sc - s0)/Lambda
from the right one — 103° for a +R bore, 58° for a -R bore — which puts the probes 7.39 and
6.46 mm across a channel 2.075 mm half thick, or 2.375 mm on the level bore's roof side, in the
wall
(TestSleeveBoreProbesTellTheTwistSense). With no collar body there is no end-vertex check; the
probes are the whole of the sense check.

The required Fusion calls for this timeline entry are:

- `component.features.createPath(boreLine, False)`.
- `component.features.sweepFeatures.createInput(profile, path, cutOperation)`.
- `adsk.core.ValueInput.createByReal((sOut - sIn) / lam)`.
- `component.features.sweepFeatures.add(sweepInput)`.
- `cageBody.pointContainment(probe)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createPath",
      "owner": "adsk.fusion.Features",
      "reason": null,
      "receiver": "component.features",
      "role": "required",
      "span": "component.features.createPath(boreLine, False)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.SweepFeatures",
      "reason": null,
      "receiver": "component.features.sweepFeatures",
      "role": "required",
      "span": "component.features.sweepFeatures.createInput(profile, path, cutOperation)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal((sOut - sIn) / lam)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SweepFeatures",
      "reason": null,
      "receiver": "component.features.sweepFeatures",
      "role": "required",
      "span": "component.features.sweepFeatures.add(sweepInput)"
    },
    {
      "condition": null,
      "name": "pointContainment",
      "owner": "adsk.fusion.BRepBody",
      "reason": null,
      "receiver": "cageBody",
      "role": "required",
      "span": "cageBody.pointContainment(probe)"
    }
  ],
  "citations": [
    {
      "first": 1366,
      "last": 1401,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1366–1401.

## S20 `[PROSE]` Window Plane

Create one Window Plane only when at least one searched gap has room. The searches below run
before all geometry in processInputs, not during this plane entry.
[PB-CONSTRUCTION-PLANES] [PB-LOGGING] [SCREW-F-SLEEVE]

**The wall between the bores** (channelSeparation). This is the fourth of the sleeve's range
checks ("Variables"), computed in processInputs before any feature. Round the tube the bores
alternate between the gears: gear A's +R bore at azimuth Sigma/2 about +n̂ from ê, gear
B's -R at 180° - Sigma/2, gear A's -R at 180° + Sigma/2 and gear B's +R at -Sigma/2,
which is 40°, 140°, 220° and 320° at the defaults. So the four gaps between neighbours face +k̂
(gear A's +R and gear B's -R bores), -ê (the two -R bores), -k̂ (gear A's -R and gear
B's +R) and +ê (the two +R bores).

1. Sample each bore's outline (channelOutline): at stations every 0.1 mm from σ*sIn outward
   while |s| <= sOut, σ being +1 for a +R bore and -1 for a -R bore, take seventeen
   points on each side of the bore's rectangle — (hw, v) and (-hw, v) for
   v = vLo + (vHi - vLo)*i/16, (u, vHi) and (u, vLo) for u = -hw + 2*hw*i/16,
   i = 0 … 16 —
   turned to the station's angle and placed as the world point of §1. Keep a point when its
   distance from the frame's axis, hypot((P - C)·ê, (P - C)·k̂), lies within Ri - 0.5 mm to
   Ro + 0.5 mm.
2. For each gap, facing d: with across = n̂ × d, project the kept points of the gap's two bores
   to ((P - C)·across, (P - C)·n̂), and take the convex hull of each bore's projections by
   Andrew's monotone chain.
3. For every edge of either hull, with m the unit vector square to it, the separation along
   m is the larger of min(m·q) - max(m·p) and min(m·p) - max(m·q), p over the first
   hull's corners and q over the second's. The gap's separation is the largest over all the
   edges; it is negative when the hulls overlap.
4. The least of the four gaps' separations must be at least collarWall.

Projecting onto a plane and then onto a line brings no two points nearer, and the hull holds
every projected point, so the separation never exceeds the least distance between the two
outlines, which nearestChannels in the proof measures and TestSleevePrintsStandingOnEitherEnd
holds to collarWall. At the defaults it is 5.067 mm, across the -ê gap, where
nearestChannels measures 5.074 mm; at the 14° mounting angles before 2026-10-03 they were 5.078
and 5.187 mm.

**The windows.** Down the hollow is one way to see the mesh; two windows through the wall show it
from the side. The windows go across the two wider gaps between neighbouring bores
(windowFacings): they face +k̂ and -k̂ when crossAngle <= 90°, where those gaps are
180° - Sigma wide against Sigma across ±ê (100° against 80° at the defaults), and +ê and
-ê past it. Across a wide gap one flanking bore is low and the other high, and the wall between
them is a band running at about 45° from above the low bore down to below the high one; each
window is cut along that band. Each window's shape is found in processInputs, before any
feature, by the search below, which is newWindow in the proof step for step; each window is
found the same way from its own facing direction. At equal mounting angles and no roof
allowance the -k̂ window comes out as the +k̂ one turned half a turn about ê, as the bores
do; the allowance on gear A's -R bore, which flanks the -k̂ window, makes that window's band
narrower, so the two differ. Neither is a shortcut the build takes.

*The window's plane.* For a window facing the level unit direction d, let across = n̂ × d,
which is -ê for d = +k̂. A point P has plane coordinates t = (P - C)·across and
z = (P - C)·n̂, and depth a = (P - C)·d. The window is the set of points with a > 0 whose
(t, z) lie in its hexagon: a prism pushed straight out through the wall from the plane through
the frame's axis. At t the wall runs from a0(t) = sqrt(max(0, Ri^2 - t^2)) to
a1(t) = sqrt(max(0, Ro^2 - t^2)).

*A bore's section in the wall* (sectionInWall). For a bore of gear g at station s with
|s| < Ro, in the section plane's coordinates x along û_g and y along v̂_g: take the
rectangle's corners (-hw, vLo), (hw, vLo), (hw, vHi) and (-hw, vHi), in that order, with
the bore's own vLo and vHi ("The roof allowance"), which runs counter-clockwise, each turned by theta = s/Lambda + Phi_g to
(u*cos theta - v*sin theta, u*sin theta + v*cos theta). Clip it to the heights inside the end
faces: keep the x for which (origin_g + x*û_g - C)·n̂ lies within ±cageRise. A point at
(x, y) stands hypot(s, y) from the frame's axis, so clip what is left twice more, once to
near <= y <= far and once to -far <= y <= -near, with near = sqrt(max(0, Ri^2 - s^2)) and
far = sqrt(Ro^2 - s^2): those are the section's two **pieces** in the wall, either of which may
be empty. Each clip keeps the part of a convex polygon on one side of a line, walking its edges
in order, keeping each corner on the kept side and adding the point where an edge crosses the
line. At |s| >= Ro the section has no piece.

*The long sides.* The two bores whose crossings lie on the window's side, (crossing - C)·d > 0,
**flank** the window; the other two are its **far** bores. The low flanking bore is the one whose
crossing is lower along n̂, and lean is +1 when the high one's crossing has the larger t
and -1 otherwise. The long sides are the 45° lines z + lean*t = lo and z + lean*t = hi.
Walk each flanking bore's cut span at stations every 0.001 mm from its lower end, and take every
corner of every piece as the world point origin_g + x*û_g + y*v̂_g + s*dir_g, with its
m = z + lean*t (wallCorners). The low bore's largest m is lowReach and the high bore's
least is highReach; then lo = lowReach + sqrt(2)*collarWall and
hi = highReach - sqrt(2)*collarWall. m grows by sqrt(2) per millimetre across the band, so
each long side stands collarWall beyond its bore's channel measured on the plane, and a point
of the channel that far from the hexagon on the plane is at least that far from the prism. A
linear measure of a piece is extreme at a corner, so the corners are all the walk needs.

*The trims.* zLimit is the furthest any channel reaches from the middle plane in the wall, up or
down (channelTop): over all four bores, at stations from σ*sIn outward every 0.01 mm while
|s| <= sOut, wherever the section reaches the wall — |s| <= Ro and hypot(s, y) >= Ri for the
largest y = |u*sin theta + v*cos theta| over the bore's four corners (u, v) — it is the largest
|(origin_g - C)·n̂ + (u*cos theta - v*sin theta)*(û_g·n̂)| over those corners, 13.81 mm at the
defaults. Then

```
top    = min(2*zLimit - hi, hi + sqrt(2)*Ri)
bottom = max(-2*zLimit - lo, lo - sqrt(2)*Ri)
```

and the trims are the 45° lines z - lean*t = top and z - lean*t = bottom. The side hi meets
top at t = lean*(hi - top)/2, and lo meets bottom at t = lean*(lo - bottom)/2. The first
term keeps each long corner inside the height the channels already reach, so both end bands keep
the end wall the channels leave. The second cuts each long side off where it would meet the inner
face more than 45° round from d, at |t| = Ri/sqrt(2): where a 45° roof meets the curved inner
face, the line they meet on descends at atan(cos psi), psi being how far round the face the
line is from d, and past psi = 45° that line steps further along the face per layer than the
proof's one-cell rule accepts.

*The ends* (windowEnd). Each window has two upright ends, t = right and t = left, standing
as far out as the far bores allow. For each side δ = +1 (right) and δ = -1 (left), bisect
te 24 times between 0 and min(Ri, Ro/sqrt(2))*(1 - 1e-9): take the middle, and make it the
new lower end when clear(te) holds and the new upper end when it does not. The end is the final
lower end: right = end(+1) and left = -end(-1). The limit Ro/sqrt(2) keeps a window's roofs
off the outer face where they would meet it more than 45° round. clear(te), at t = δ*te:

1. zLow = max(lo - lean*t, bottom + lean*t) and zHigh = min(hi - lean*t, top + lean*t), the
   end's two corners; not clear when zLow > zHigh.
2. need = collarWall + 0.1 mm/sqrt(2) + 0.005 mm, 3.076 mm at the defaults: collarWall plus
   the most the true least distance can fall under the sampled one, which is half the diagonal
   of a 0.1 mm sample cell and 5 µm for the stations the channel is walked at (windowSlack).
3. The points, each C + t*across + z*n̂ + a*d, are: the end's two corner lines through the wall,
   at z = zLow and at z = zHigh, each at a = a0, a0 + 0.1 mm, … and last a1, a step that
   would pass a1 landing on it; and, on the inner face a = a0(t) and on the outer face
   a = a1(t), the end's upright edge between the corners, at z = zLow + (zHigh - zLow)*k/n
   for k = 1 … n - 1 with n = ceil((zHigh - zLow)/0.1 mm), and at
   z = -zLimit + j*0.1 mm for every j >= 0 with z <= zLimit and zLow < z < zHigh.
4. Not clear as soon as any point's distance to either far bore's channel in the wall, by the
   walk below with reach = need, is under need; clear otherwise.

The upright edges are sampled at the heights the proof's full check of a window samples an end
at: its walk along the edge, and its walk of the openings, which steps up from -zLimit. So the
check that places the ends reads the same points the proof then holds to need. The corner lines
alone, as the proof placed the ends before this search was written, held at the 0.45 mm clearance
the defaults had until 2026-10-02 and gave the same window there, but at a 0.2 mm clearance, with
the mounting angles and engagement of that time, they let an end's edge on the inner face come
2.60 mm from a far bore against the 3 mm collarWall.

*The distance to a channel* (wallGap). For a point P and one bore of gear g: in the gear's
frame x = (P - origin_g)·û_g, y = (P - origin_g)·v̂_g and sq = (P - origin_g)·dir_g. Every
point of a section lies within c of the gear's axis, so when `hypot(hypot(x, y), sq -
min(max(sq, span start), span end)) - c >= reach the distance is taken as reach` and nothing is
walked. Otherwise walk the bore's **station table**, built once per bore before the first walk:
stations span start + k*0.002 mm for k = 0, 1, … while inside the span, each with its two
pieces, and, wherever the set of non-empty pieces differs between two neighbouring stations,
stations every 0.0001 mm strictly between those two, all in order of s. For each piece keep its
**circle**: the average of its corners, and the largest distance from that to a corner. Start with
best = reach^2; walk up from the first station at or above sq, then down from the one before
it, and stop each way at the first station with (sq - s)^2 >= best. At each station, for each
non-empty piece, pass over the piece when o = hypot(x - cx, y - cy) - radius is positive and
(sq - s)^2 + o^2 >= best; otherwise set best = min(best, (sq - s)^2 + d2), where d2 is 0
when (x, y) is inside the piece — on the inner side of every edge of the counter-clockwise
polygon, which a piece of fewer than three corners never is — and otherwise the least squared
distance from (x, y) to the piece's edges. The distance is sqrt(best). A section's points move
at most 1.42 mm per millimetre of station, and the tube's faces clip them faster only near the few
stations where a face is tangent to the section's plane, so between two 2 µm stations the distance
dips under the nearer by at most 4 µm, which the 5 µm in need covers; where a piece appears or
vanishes, the 0.1 µm stations follow the channel's corner first reaching into the wall. The walk
stops only where no station further on could come nearer and passes over only pieces that could
not, so it finds the least a walk of every station would.

*The hexagon.* Its corners are the square -2*Ro <= t, z <= 2*Ro clipped, in this order, to
lean*t + z <= hi, -lean*t - z <= -lo, -lean*t + z <= top, lean*t - z <= -bottom,
t <= right and -t <= -left (clipCorners), so every edge is upright or at 45°. Drop a corner
that lies within 0.001 mm of the one before it, since a sketch line cannot have zero length. At
the defaults the +k̂ window has lo = -5.180, hi = 5.180, bottom = -22.151, top = 22.151,
left = -11.570 and right = 11.570, and its six corners in (t, z) are (11.57, -6.39),
(-8.49, 13.67), (-11.57, 10.58), (-11.57, 6.39), (8.49, -13.67) and (11.57, -10.58): long
sides 7.33 mm apart across the band, ends 23.14 mm apart, 220.7 mm² on the plane, reaching
13.67 mm from the middle against the channels' 13.81 mm. The -k̂ window has lo = -4.886,
hi = 5.180, bottom = -21.857, top = 22.151, left = -11.536 and right = 11.570, and its
corners are (11.57, -6.39), (-8.49, 13.67), (-11.54, 10.61), (-11.54, 6.65),
(8.49, -13.37) and (11.57, -10.29): long sides 7.12 mm apart, ends 23.11 mm apart, 213.8 mm².

*When a gap has no room* (room). A window is not cut when hi <= lo, the flanking bores leaving
no band between them, or when right <= left or the hexagon has fewer than three distinct
corners or no area, the far bores leaving the band no length. The build then logs
No window facing {d}: {reason} with futil.log ([PB-LOGGING]), {d} being +k, -k, +e
or -e, and builds the sleeve without that window; a window only takes material away, so every
check the sleeve passes still holds without it. No input TestSleeveWindowsFollowTheSize
accepts reaches this rule. It leaves both windows out at a 7 mm collarWall, where the flanking
bores leave no band; the build refuses that input for the wall between its bores anyway.

*What the search keeps.* TestSleeveWindowsKeepTheirWalls holds the default windows: each
flanking bore's channel lies exactly 3.000 mm beyond a long side, measured on the plane; every
face of the cut keeps 3.076 mm from the far bores' channels, sampled every 0.1 mm; every edge
rises at 45° or more; the faces meet the tube at edges of 50.0° or more; and the lines where the
roofs meet the tube descend at 35.3° or more. TestSleeveWindowPostsStandFirm holds every post
beside a window to at least collarWall wide, 3.73 mm at the narrowest at the defaults, and any
stretch of post under two collarWalls wide to at most twice as tall as it is wide, 0.70 times
at the defaults. Through either window 81.6% of the mesh zone can be seen past both ribbons
(TestSleeveWindowsShowTheMeshFromTheSide).
TestSleeveWindowsFollowTheSize runs all of those checks, with the one-piece and printing checks,
at 33 inputs across the dialog's ranges, and every window it cuts passes.

*Cutting a window in Fusion* ([SCREW-F-SLEEVE]). Both windows lie on one plane through the
frame's axis square to d, Window Plane ([PB-CONSTRUCTION-PLANES]). For ±k̂ windows it is
setByAngle(anchorLine, ValueInput.createByString('90 deg'), targetPlane), the plane through the
Anchor Line square to the selected plane, which holds C, ê and n̂; for ±ê windows it is
setByDistanceOnPath(anchorLine, ValueInput.createByReal(0.5)), the plane square to the Anchor
Line through its midpoint, C. It is not made when neither window has room. Each window is a
sketch Window {d} on that plane holding one reference point per corner at
C + t*across + z*n̂, mapped in with modelToSketchSpace and given z = 0
([PB-SKETCH-ZERO-Z]), and one solid line from each corner to the next, the last back to the
first, sharing the points ([PB-SHARE-XOR-COINCIDENT]); then every point is set isFixed = True.
Nothing else is in the sketch, and its profile is its one loop: raise unless profiles.count is
1, then take profiles.item(0) ([PB-SINGLE-PROFILE]). The two windows are two sketches because
on the shared plane their hexagons cross.

Cut each window with one extrude: extrudeFeatures.createInput(profile, CutFeatureOperation),
then `setOneSideExtent(DistanceExtentDefinition.create(ValueInput.createByReal(Ro + 1 mm)),
direction), participantBodies` set to a list holding the cage body alone so that the ribbons in
the hollow are left whole ([PB-THROUGH-CUT]), then extrudeFeatures.add. direction is
PositiveExtentDirection when sketch.modelToSketchSpace(C + d) has a positive z, that is when
d points to the sketch's positive side, and NegativeExtentDirection otherwise. The cut runs
one way only: the same hexagon on the far side of the axis is the other window's side of the

The required Fusion calls for this timeline entry are:

- `component.constructionPlanes.createInput()`.
- `adsk.core.ValueInput.createByString("90 deg")`.
- `planeInput.setByAngle(anchorLine, rightAngleValue, targetPlane)`.
- `adsk.core.ValueInput.createByReal(0.5)`.
- `planeInput.setByDistanceOnPath(anchorLine, halfValue)`.
- `component.constructionPlanes.add(planeInput)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "component.constructionPlanes",
      "role": "required",
      "span": "component.constructionPlanes.createInput()"
    },
    {
      "condition": null,
      "name": "createByString",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByString(\"90 deg\")"
    },
    {
      "condition": null,
      "name": "setByAngle",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByAngle(anchorLine, rightAngleValue, targetPlane)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(0.5)"
    },
    {
      "condition": null,
      "name": "setByDistanceOnPath",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByDistanceOnPath(anchorLine, halfValue)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "component.constructionPlanes",
      "role": "required",
      "span": "component.constructionPlanes.add(planeInput)"
    }
  ],
  "citations": [
    {
      "first": 1403,
      "last": 1608,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1403–1608.

## S21 `[GO]` Per-window hexagon sketch

The proof function is `stepWindowSketch`.

<!-- proof-run: proofkit.Run(windowCases, stepWindowSketch) -->

Instantiate one sketch for each searched window, d before -d. Their crossing polygons require separate sketches.
[PB-SHARE-XOR-COINCIDENT] [PB-SKETCH-ZERO-Z] [PB-SINGLE-PROFILE] [SCREW-F-SLEEVE]

*The hexagon.* Its corners are the square -2*Ro <= t, z <= 2*Ro clipped, in this order, to
lean*t + z <= hi, -lean*t - z <= -lo, -lean*t + z <= top, lean*t - z <= -bottom,
t <= right and -t <= -left (clipCorners), so every edge is upright or at 45°. Drop a corner
that lies within 0.001 mm of the one before it, since a sketch line cannot have zero length. At
the defaults the +k̂ window has lo = -5.180, hi = 5.180, bottom = -22.151, top = 22.151,
left = -11.570 and right = 11.570, and its six corners in (t, z) are (11.57, -6.39),
(-8.49, 13.67), (-11.57, 10.58), (-11.57, 6.39), (8.49, -13.67) and (11.57, -10.58): long
sides 7.33 mm apart across the band, ends 23.14 mm apart, 220.7 mm² on the plane, reaching
13.67 mm from the middle against the channels' 13.81 mm. The -k̂ window has lo = -4.886,
hi = 5.180, bottom = -21.857, top = 22.151, left = -11.536 and right = 11.570, and its
corners are (11.57, -6.39), (-8.49, 13.67), (-11.54, 10.61), (-11.54, 6.65),
(8.49, -13.37) and (11.57, -10.29): long sides 7.12 mm apart, ends 23.11 mm apart, 213.8 mm².

*When a gap has no room* (room). A window is not cut when hi <= lo, the flanking bores leaving
no band between them, or when right <= left or the hexagon has fewer than three distinct
corners or no area, the far bores leaving the band no length. The build then logs
No window facing {d}: {reason} with futil.log ([PB-LOGGING]), {d} being +k, -k, +e
or -e, and builds the sleeve without that window; a window only takes material away, so every
check the sleeve passes still holds without it. No input TestSleeveWindowsFollowTheSize
accepts reaches this rule. It leaves both windows out at a 7 mm collarWall, where the flanking
bores leave no band; the build refuses that input for the wall between its bores anyway.

*What the search keeps.* TestSleeveWindowsKeepTheirWalls holds the default windows: each
flanking bore's channel lies exactly 3.000 mm beyond a long side, measured on the plane; every
face of the cut keeps 3.076 mm from the far bores' channels, sampled every 0.1 mm; every edge
rises at 45° or more; the faces meet the tube at edges of 50.0° or more; and the lines where the
roofs meet the tube descend at 35.3° or more. TestSleeveWindowPostsStandFirm holds every post
beside a window to at least collarWall wide, 3.73 mm at the narrowest at the defaults, and any
stretch of post under two collarWalls wide to at most twice as tall as it is wide, 0.70 times
at the defaults. Through either window 81.6% of the mesh zone can be seen past both ribbons
(TestSleeveWindowsShowTheMeshFromTheSide).
TestSleeveWindowsFollowTheSize runs all of those checks, with the one-piece and printing checks,
at 33 inputs across the dialog's ranges, and every window it cuts passes.

*Cutting a window in Fusion* ([SCREW-F-SLEEVE]). Both windows lie on one plane through the
frame's axis square to d, Window Plane ([PB-CONSTRUCTION-PLANES]). For ±k̂ windows it is
setByAngle(anchorLine, ValueInput.createByString('90 deg'), targetPlane), the plane through the
Anchor Line square to the selected plane, which holds C, ê and n̂; for ±ê windows it is
setByDistanceOnPath(anchorLine, ValueInput.createByReal(0.5)), the plane square to the Anchor
Line through its midpoint, C. It is not made when neither window has room. Each window is a
sketch Window {d} on that plane holding one reference point per corner at
C + t*across + z*n̂, mapped in with modelToSketchSpace and given z = 0
([PB-SKETCH-ZERO-Z]), and one solid line from each corner to the next, the last back to the
first, sharing the points ([PB-SHARE-XOR-COINCIDENT]); then every point is set isFixed = True.
Nothing else is in the sketch, and its profile is its one loop: raise unless profiles.count is
1, then take profiles.item(0) ([PB-SINGLE-PROFILE]). The two windows are two sketches because
on the shared plane their hexagons cross.

Cut each window with one extrude: extrudeFeatures.createInput(profile, CutFeatureOperation),
then `setOneSideExtent(DistanceExtentDefinition.create(ValueInput.createByReal(Ro + 1 mm)),
direction), participantBodies` set to a list holding the cage body alone so that the ribbons in
the hollow are left whole ([PB-THROUGH-CUT]), then extrudeFeatures.add. direction is
PositiveExtentDirection when sketch.modelToSketchSpace(C + d) has a positive z, that is when
d points to the sketch's positive side, and NegativeExtentDirection otherwise. The cut runs
one way only: the same hexagon on the far side of the axis is the other window's side of the

The required Fusion calls for this timeline entry are:

- `component.sketches.add(windowPlane)`.
- `sketch.modelToSketchSpace(worldPoint)`.
- `sketch.sketchPoints.add(localPoint)`.
- `sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)`.
- `sketch.profiles.item(0)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "component.sketches",
      "role": "required",
      "span": "component.sketches.add(windowPlane)"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.modelToSketchSpace(worldPoint)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SketchPoints",
      "reason": null,
      "receiver": "sketch.sketchPoints",
      "role": "required",
      "span": "sketch.sketchPoints.add(localPoint)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.Profiles",
      "reason": null,
      "receiver": "sketch.profiles",
      "role": "required",
      "span": "sketch.profiles.item(0)"
    }
  ],
  "citations": [
    {
      "first": 1554,
      "last": 1608,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1554–1608.

## S22 `[GO]` Per-window extrude cut

The proof function is `stepWindowCut`.

<!-- proof-run: proofkit3d.RunSolid(windowSolidCases, stepWindowCut, assertWindowCut) -->

The millimetre margin is 0.1 cm in the Fusion call. Use only cageBody as participant.
The proof cuts each window independently from an uncut sleeve, avoiding the cumulative boolean engine limit.
[PB-THROUGH-CUT] [PB-SELF-DIAGNOSING] [PB-EMPTY-RESULT] [SCREW-F-SLEEVE]

1, then take profiles.item(0) ([PB-SINGLE-PROFILE]). The two windows are two sketches because
on the shared plane their hexagons cross.

Cut each window with one extrude: extrudeFeatures.createInput(profile, CutFeatureOperation),
then `setOneSideExtent(DistanceExtentDefinition.create(ValueInput.createByReal(Ro + 1 mm)),
direction), participantBodies` set to a list holding the cage body alone so that the ribbons in
the hollow are left whole ([PB-THROUGH-CUT]), then extrudeFeatures.add. direction is
PositiveExtentDirection when sketch.modelToSketchSpace(C + d) has a positive z, that is when
d points to the sketch's positive side, and NegativeExtentDirection otherwise. The cut runs
one way only: the same hexagon on the far side of the axis is the other window's side of the
wall, where the bores stand.

*Checking a window's cut* ([PB-SELF-DIAGNOSING]). Before each cut, cageBody.pointContainment
at the probe C + tc*across + zc*n̂ + ((a0(tc) + a1(tc))/2)*d, with (tc, zc) the average of the
hexagon's corners, must be PointInsidePointContainment: the middle of the wall where the window
goes, which keeps collarWall from every bore. After the cut the same point must be
PointOutsidePointContainment, and the feature's bodies.count must be 1 ([PB-EMPTY-RESULT]).
The build raises naming the window and what it read otherwise. A cut extruded the wrong way
leaves the probe inside.

**Order.** The tube is the first body. The four bores are cut from it, each with the cage as its
only participant, in the order gear A -R, gear A +R, gear B -R, gear B +R; then the
windows, the one facing d before the one facing -d. Every cut has to leave exactly one body,
counted as the feature's bodies.count, and that body is the cage from then on; the build raises
with the piece's name otherwise. The marks are joined after the cuts. TestSleeveIsOnePiece
holds the unmarked sleeve is one piece: 16,602 mm³ at the defaults, about 21 g of PLA.

The required Fusion calls for this timeline entry are:

- `component.features.extrudeFeatures.createInput(profile, cutOperation)`.
- `adsk.core.ValueInput.createByReal(Ro + 0.1)`.
- `adsk.fusion.DistanceExtentDefinition.create(lengthValue)`.
- `extrudeInput.setOneSideExtent(distanceExtent, direction)`.
- `sketch.modelToSketchSpace(directionProbe)`.
- `component.features.extrudeFeatures.add(extrudeInput)`.
- `cageBody.pointContainment(probe)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "component.features.extrudeFeatures",
      "role": "required",
      "span": "component.features.extrudeFeatures.createInput(profile, cutOperation)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(Ro + 0.1)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.fusion.DistanceExtentDefinition",
      "reason": null,
      "receiver": "adsk.fusion.DistanceExtentDefinition",
      "role": "required",
      "span": "adsk.fusion.DistanceExtentDefinition.create(lengthValue)"
    },
    {
      "condition": null,
      "name": "setOneSideExtent",
      "owner": "adsk.fusion.ExtrudeFeatureInput",
      "reason": null,
      "receiver": "extrudeInput",
      "role": "required",
      "span": "extrudeInput.setOneSideExtent(distanceExtent, direction)"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.modelToSketchSpace(directionProbe)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "component.features.extrudeFeatures",
      "role": "required",
      "span": "component.features.extrudeFeatures.add(extrudeInput)"
    },
    {
      "condition": null,
      "name": "pointContainment",
      "owner": "adsk.fusion.BRepBody",
      "reason": null,
      "receiver": "cageBody",
      "role": "required",
      "span": "cageBody.pointContainment(probe)"
    }
  ],
  "citations": [
    {
      "first": 1599,
      "last": 1624,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1599–1624.

## S23 `[PROSE]` Marker Plane

Create one marker plane after all bore and window cuts. Its signed world height is cageRise-inset.
[PB-CONSTRUCTION-PLANES] [PB-SELF-DIAGNOSING] [SCREW-F-BORE-MARKS]

After the four bore cuts and any window cuts, raise one mark for each bore on the top end at
+cageRise along nHat. The opposite end stays flat on the print bed. Follow the bore-cut
order: Gear A -R, Gear A +R, Gear B -R, Gear B +R. Use the bore's gear index g and
sign sigma: a **circle** identifies +R (sigma = +1),
and a **square** identifies -R (sigma = -1). Both gears get both marks. The marks join only
to the sleeve, never to either ribbon.

Use halfSize = min(1 mm, collarHalf/2) and inset = min(0.1 mm, collarWall/2) in the same
length units as the sleeve. At the defaults, halfSize = 1 mm and inset = 0.1 mm. Make one
construction plane parallel to Gear B Axis Plane. That plane is +A/2 along nHat from C,
so offset it by cageRise - inset - A/2 toward +nHat. Choose the signed Fusion offset from
the dot product of Gear B Axis Plane's normal with nHat. Check the new plane's origin's
signed offset from C along nHat against cageRise - inset. The end-wall range
check keeps this plane inside solid sleeve material. For each bore, create a separate sketch on
that plane, named Gear A Bore +R Circle Marker, Gear A Bore -R Square Marker, and likewise
for Gear B. Its centre in world coordinates is
C + (cageRise - inset)*nHat + sigma*cageRadius*dirVecs[g]. Map every point into the sketch
and set its local z to zero before drawing.

The required Fusion calls for this timeline entry are:

- `component.constructionPlanes.createInput()`.
- `adsk.core.ValueInput.createByReal(signedOffset)`.
- `planeInput.setByOffset(gearBAxisPlane, offsetValue)`.
- `component.constructionPlanes.add(planeInput)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "component.constructionPlanes",
      "role": "required",
      "span": "component.constructionPlanes.createInput()"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(signedOffset)"
    },
    {
      "condition": null,
      "name": "setByOffset",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByOffset(gearBAxisPlane, offsetValue)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "component.constructionPlanes",
      "role": "required",
      "span": "component.constructionPlanes.add(planeInput)"
    }
  ],
  "citations": [
    {
      "first": 2016,
      "last": 2033,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 657,
      "last": 667,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L2016–2033; `spec/screwgear/fusion.md` L657–667.

## S24 `[GO]` Per-bore marker sketch

The proof function is `stepMarkerSketch`.

<!-- proof-run: proofkit.Run(markerCases, stepMarkerSketch) -->

Instantiate the marker sketch, extrude and join triplet four times in bore order A -R, A +R, B -R, B +R.
Each +R sketch has only a circle and each -R sketch has only a square.
[PB-SKETCH-FIRST] [PB-CIRCLE-CENTER] [PB-RADIAL-DIM] [PB-SKETCH-ZERO-Z]
[PB-SHARE-XOR-COINCIDENT] [PB-SINGLE-PROFILE] [PB-EMPTY-RESULT] [SCREW-F-BORE-MARKS]

After the four bore cuts and any window cuts, raise one mark for each bore on the top end at
+cageRise along nHat. The opposite end stays flat on the print bed. Follow the bore-cut
order: Gear A -R, Gear A +R, Gear B -R, Gear B +R. Use the bore's gear index g and
sign sigma: a **circle** identifies +R (sigma = +1),
and a **square** identifies -R (sigma = -1). Both gears get both marks. The marks join only
to the sleeve, never to either ribbon.

Use halfSize = min(1 mm, collarHalf/2) and inset = min(0.1 mm, collarWall/2) in the same
length units as the sleeve. At the defaults, halfSize = 1 mm and inset = 0.1 mm. Make one
construction plane parallel to Gear B Axis Plane. That plane is +A/2 along nHat from C,
so offset it by cageRise - inset - A/2 toward +nHat. Choose the signed Fusion offset from
the dot product of Gear B Axis Plane's normal with nHat. Check the new plane's origin's
signed offset from C along nHat against cageRise - inset. The end-wall range
check keeps this plane inside solid sleeve material. For each bore, create a separate sketch on
that plane, named Gear A Bore +R Circle Marker, Gear A Bore -R Square Marker, and likewise
for Gear B. Its centre in world coordinates is
C + (cageRise - inset)*nHat + sigma*cageRadius*dirVecs[g]. Map every point into the sketch
and set its local z to zero before drawing.

For a +R mark, draw one circle of radius halfSize, fix its centre point, and dimension its
diameter to 2*halfSize. For a -R mark, draw four lines joining four fixed sketch points in
counter-clockwise order. Their world coordinates are the centre plus
(-halfSize,-halfSize), (halfSize,-halfSize), (halfSize,halfSize), and
(-halfSize,halfSize) in the (eHat,kHat) basis. Do not draw a circle and a square in the same
sketch. Each sketch must be fully constrained and have exactly one closed profile; raise with
its name and the observed profile count or constraint status otherwise. For every accepted
collarHalf, each profile lies strictly inside the annular top face: the square corner's radial
offset from its centre is at most sqrt(2)*halfSize < collarHalf, and the circle is smaller.

The required Fusion calls for this timeline entry are:

- `component.sketches.add(markerPlane)`.
- `sketch.modelToSketchSpace(worldPoint)`.
- `sketch.sketchCurves.sketchCircles.addByCenterRadius(localCentre, halfSize)`.
- `sketch.sketchDimensions.addDiameterDimension(circle, textPoint)`.
- `sketch.sketchPoints.add(localPoint)`.
- `sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)`.
- `sketch.profiles.item(0)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "component.sketches",
      "role": "required",
      "span": "component.sketches.add(markerPlane)"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.modelToSketchSpace(worldPoint)"
    },
    {
      "condition": null,
      "name": "addByCenterRadius",
      "owner": "adsk.fusion.SketchCircles",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchCircles",
      "role": "required",
      "span": "sketch.sketchCurves.sketchCircles.addByCenterRadius(localCentre, halfSize)"
    },
    {
      "condition": null,
      "name": "addDiameterDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addDiameterDimension(circle, textPoint)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SketchPoints",
      "reason": null,
      "receiver": "sketch.sketchPoints",
      "role": "required",
      "span": "sketch.sketchPoints.add(localPoint)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.Profiles",
      "reason": null,
      "receiver": "sketch.profiles",
      "role": "required",
      "span": "sketch.profiles.item(0)"
    }
  ],
  "citations": [
    {
      "first": 2016,
      "last": 2043,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 657,
      "last": 667,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L2016–2043; `spec/screwgear/fusion.md` L657–667.

## S25 `[GO]` Per-bore marker extrude

The proof function is `stepMarkerExtrude`.

<!-- proof-run: proofkit3d.RunSolid(markerSolidCases, stepMarkerExtrude, assertMarkerExtrude) -->

The visible height is 0.04 cm; inset is also in cm. Set directionProbe to the world centre of this bore mark plus nHat, then map it to select the outward extent.
[PB-EMPTY-RESULT] [PB-SELF-DIAGNOSING] [SCREW-F-BORE-MARKS]

Extrude that profile as a **new body** by inset + 0.4 mm, toward +nHat, selecting the
sketch's positive or negative extent from the mapped position of the mark centre plus nHat.
The start lies inside the sleeve and the visible part rises exactly 0.4 mm above the end face. Require the
extrude feature to have one body. Join that body to the current cage body using
[SCREW-F-JOIN], require the combine feature to have one body, and use that body as the cage
for the next mark. Include the gear and bore sign in either failure. At the defaults the two
visible circles add 2*pi*(1 mm)^2*(0.4 mm) and the two visible squares add
2*(2 mm)^2*(0.4 mm), for about 5.713 mm³ above the original end face.

The required Fusion calls for this timeline entry are:

- `component.features.extrudeFeatures.createInput(profile, newBodyOperation)`.
- `adsk.core.ValueInput.createByReal(inset + 0.04)`.
- `adsk.fusion.DistanceExtentDefinition.create(lengthValue)`.
- `extrudeInput.setOneSideExtent(distanceExtent, direction)`.
- `sketch.modelToSketchSpace(directionProbe)`.
- `component.features.extrudeFeatures.add(extrudeInput)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "component.features.extrudeFeatures",
      "role": "required",
      "span": "component.features.extrudeFeatures.createInput(profile, newBodyOperation)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(inset + 0.04)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.fusion.DistanceExtentDefinition",
      "reason": null,
      "receiver": "adsk.fusion.DistanceExtentDefinition",
      "role": "required",
      "span": "adsk.fusion.DistanceExtentDefinition.create(lengthValue)"
    },
    {
      "condition": null,
      "name": "setOneSideExtent",
      "owner": "adsk.fusion.ExtrudeFeatureInput",
      "reason": null,
      "receiver": "extrudeInput",
      "role": "required",
      "span": "extrudeInput.setOneSideExtent(distanceExtent, direction)"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.modelToSketchSpace(directionProbe)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "component.features.extrudeFeatures",
      "role": "required",
      "span": "component.features.extrudeFeatures.add(extrudeInput)"
    }
  ],
  "citations": [
    {
      "first": 2045,
      "last": 2052,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 657,
      "last": 667,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L2045–2052; `spec/screwgear/fusion.md` L657–667.

## S26 `[GO]` Per-bore marker join

The proof function is `stepMarkerJoin`.

<!-- proof-run: proofkit3d.RunSolid(markerSolidCases, stepMarkerJoin, assertMarkerJoin) -->

Set targetBody to the current cageBody and toolBody to the new mark body only.
Set JoinFeatureOperation and isKeepToolBodies=False; require one body and use it for the next mark.
The proof joins each mark to an uncut sleeve as the spec permits.
For half-thickness below 0.5 mm, the proof substitutes a 128-sided annulus; its maximum chord error is 0.0046 mm.
This bounds the thin-sleeve faceting substitute; it does not prove a circular boolean at that thickness.
[PB-EMPTY-RESULT] [PB-SELF-DIAGNOSING] [SCREW-F-JOIN] [SCREW-F-BORE-MARKS]

Extrude that profile as a **new body** by inset + 0.4 mm, toward +nHat, selecting the
sketch's positive or negative extent from the mapped position of the mark centre plus nHat.
The start lies inside the sleeve and the visible part rises exactly 0.4 mm above the end face. Require the
extrude feature to have one body. Join that body to the current cage body using
[SCREW-F-JOIN], require the combine feature to have one body, and use that body as the cage
for the next mark. Include the gear and bore sign in either failure. At the defaults the two
visible circles add 2*pi*(1 mm)^2*(0.4 mm) and the two visible squares add
2*(2 mm)^2*(0.4 mm), for about 5.713 mm³ above the original end face.

The proof builds the mark sketches and the extruded marks against an uncut annular sleeve,
since the bore and window proof cannot chain all six cuts in one body. It checks each sketch's
constraint and profile result, the four marks' positions and shapes, their containment in the
annular end face, the 0.4 mm visible height, and one connected sleeve body after each join.
The full sleeve surface and pictures may continue to omit marks, but they do not replace these
marker checks. The proof cannot test Fusion's choice of extrusion direction or a printer's
result; the mapped mark-centre-plus-nHat direction check belongs in the Add-In.

The required Fusion calls for this timeline entry are:

- `adsk.core.ObjectCollection.create()`.
- `tools.add(toolBody)`.
- `component.features.combineFeatures.createInput(targetBody, tools)`.
- `component.features.combineFeatures.add(combineInput)`.
- `combineFeature.bodies.item(0)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "adsk.core.ObjectCollection",
      "role": "required",
      "span": "adsk.core.ObjectCollection.create()"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "tools",
      "role": "required",
      "span": "tools.add(toolBody)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "component.features.combineFeatures",
      "role": "required",
      "span": "component.features.combineFeatures.createInput(targetBody, tools)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "component.features.combineFeatures",
      "role": "required",
      "span": "component.features.combineFeatures.add(combineInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "combineFeature.bodies",
      "role": "required",
      "span": "combineFeature.bodies.item(0)"
    }
  ],
  "citations": [
    {
      "first": 2045,
      "last": 2060,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 657,
      "last": 667,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L2045–2060; `spec/screwgear/fusion.md` L657–667.

## S27 `[PROSE]` Relocate Gear A

Relocate exactly the Gear A body to its named output occurrence, preserving world position.
After the last relocation use the framework helper solids.hide_construction_geometry on self.designOcc.component.
[PB-NO-CROSS-SIBLING] [PB-TREE-CLEANUP] [PB-HIDE-AFTER-USE]

### 5: Relocate the bodies

Name the cage body Cage, then move the two gear bodies and the cage body into their
sub-components with body.moveToComponent, which preserves world position and needs no
activation. Then solids.hide_construction_geometry(self.designOcc.component): the Design
sub-component is where every sketch and construction plane of this build was made, and the helper
walks it and anything under it.

The required Fusion calls for this timeline entry are:

- `body.moveToComponent(targetOccurrence)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "moveToComponent",
      "owner": "adsk.fusion.BRepBody",
      "reason": null,
      "receiver": "body",
      "role": "required",
      "span": "body.moveToComponent(targetOccurrence)"
    }
  ],
  "citations": [
    {
      "first": 1626,
      "last": 1632,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1626–1632.

## S28 `[PROSE]` Relocate Gear B

Relocate exactly the Gear B body to its named output occurrence, preserving world position.
After the last relocation use the framework helper solids.hide_construction_geometry on self.designOcc.component.
[PB-NO-CROSS-SIBLING] [PB-TREE-CLEANUP] [PB-HIDE-AFTER-USE]

### 5: Relocate the bodies

Name the cage body Cage, then move the two gear bodies and the cage body into their
sub-components with body.moveToComponent, which preserves world position and needs no
activation. Then solids.hide_construction_geometry(self.designOcc.component): the Design
sub-component is where every sketch and construction plane of this build was made, and the helper
walks it and anything under it.

The required Fusion calls for this timeline entry are:

- `body.moveToComponent(targetOccurrence)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "moveToComponent",
      "owner": "adsk.fusion.BRepBody",
      "reason": null,
      "receiver": "body",
      "role": "required",
      "span": "body.moveToComponent(targetOccurrence)"
    }
  ],
  "citations": [
    {
      "first": 1626,
      "last": 1632,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1626–1632.

## S29 `[PROSE]` Relocate Cage

Relocate exactly the Cage body to its named output occurrence, preserving world position.
After the last relocation use the framework helper solids.hide_construction_geometry on self.designOcc.component.
[PB-NO-CROSS-SIBLING] [PB-TREE-CLEANUP] [PB-HIDE-AFTER-USE]

### 5: Relocate the bodies

Name the cage body Cage, then move the two gear bodies and the cage body into their
sub-components with body.moveToComponent, which preserves world position and needs no
activation. Then solids.hide_construction_geometry(self.designOcc.component): the Design
sub-component is where every sketch and construction plane of this build was made, and the helper
walks it and anything under it.

The required Fusion calls for this timeline entry are:

- `body.moveToComponent(targetOccurrence)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "moveToComponent",
      "owner": "adsk.fusion.BRepBody",
      "reason": null,
      "receiver": "body",
      "role": "required",
      "span": "body.moveToComponent(targetOccurrence)"
    }
  ],
  "citations": [
    {
      "first": 1626,
      "last": 1632,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1626–1632.
