The runnable proof files are `proof/screwgear/sketches_test.go`, `proof/screwgear/solids_test.go`, `proof/screwgear/timeline_test.go`, and `proof/screwgear/zz_registrations_test.go`.

<!-- step-metadata: 2 -->

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/screwgear/instructions.md` | `7f645db3c91a00a85a30686448a55c399b50a9b1` |
| `spec/screwgear/fusion.md` | `04129978aea51d49ff8363c00f93955ce5d11ac5` |
| `CLAUDE.md` | `916e8624ca88af226c264c21f295c14a9fb9e901` |
| `proof/screwgear/README.md` | `4bf5085d20c63599a401cb86952fc45bd40b4da1` |
| `spec/cycloidal/fusion.md` | `afa5a99986f2e0d9f82fb5e21591553cdc54aac4` |
| `spec/screwgear/mesh-search.md` | `245bc5c833387a83598ee6c8a7971e8efd5825be` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `cdd32545b0f8c651752827c6697601f1e32b4d39` |

## 00 `[PROSE]` Create Screw Gearing component and resolve inputs

## Architecture

The module 'lib/geargen/screwgear.py' defines exactly these public classes (the command wiring binds
to them by name; exported via 'lib/geargen/__init__.py'):

- **'ScrewGearCommandInputsConfigurator'** — classmethod 'configure(cls, command)' adds the dialog
  inputs in the order and groups of "Variables" below. No conditional visibility.
- **'ScrewGearGenerator(base.Generator)'** — 1-arg constructor '(design)' (inherited); implements
  'generate(self, inputs)' and the call graph below; relies on inherited 'deleteComponent()' for
  error cleanup. Overrides 'prefixBase()' to return ''ScrewGear''.

**Generation Context: none.** Carry handles on 'self': 'self.designOcc', 'self.gearOccs' (list of
two), 'self.cageOcc', 'self.gearBodies' (list of two), 'self.cageBody', 'self.pathLines' (list of
two dicts, one per gear, keyed ''bore-'' and ''bore+'', each the sketch line of §1 that one bore's
sweep of §4 runs along), and 'self.windows' (the windows the window search of §4 finds room for,
zero to two, each its facing direction and its corners).

**Dependencies: none.** Imports only the framework ('base', 'misc', 'utilities', 'solids',
'fusion360utils'). Use 'solids.hide_construction_geometry(component)' for the final cleanup — do
NOT re-implement it.

**Entry wiring:** 'commands/screwgear/entry.py' constructs 'GearCommand(gear_type='ScrewGear',
name='Screw Gear Generator', …)', binding the two classes above by name (PLAYBOOK.md
"Command-entry wiring").

**Parameter mode: all-Python-precomputed** ('[PB-PRECOMPUTED-MODE]'). Every value is computed in
Python in internal cm and written numerically; the generator registers **no** named user parameters.
The trigonometry here (per-station rotation angles, the screw step matrix) has no useful expression
form in the parameter table. The two searches of §4 that run before any feature — the check on
the wall between the bores and the window search — work in millimetres, with the step sizes §4
states in millimetres, and every length they hand on is divided by 10.

## Component Setup

One command invocation creates a **'Screw Gearing'** component under the user-selected Parent
Component, holding four sub-components ('[PB-OCCURRENCE-TREE]'). The top occurrence is the
inherited 'self.getOccurrence()', which creates it under 'self.parentComponent' and is what the
inherited 'deleteComponent()' deletes on failure; the build names its component and never calls
'addNewComponent' for it itself. The four children are each
'occurrences.addNewComponent(adsk.core.Matrix3D.create())' under that component:

- **'Design'** — every sketch, construction plane and feature runs here.
- **'Gear A'**, **'Gear B'**, **'Cage'** — empty until the end, when the finished bodies are
  relocated into them with 'body.moveToComponent' ('[PB-NO-CROSS-SIBLING]').

**NEVER call 'occurrence.activate()'** ('[PB-NEVER-ACTIVATE]'). Place sketches directly on the
user-selected plane ('[PB-USE-SELECTED-PLANE]').

The user selects a **plane** and a **point**. The plane's normal is the cage axis and the common
perpendicular 'n̂'; the point is the mechanism's centre 'C'.

## Variables

User inputs in dialog order: first by importance, then in named groups. All linear inputs are
mm; the tooth slant and the crossing and mounting angles are degrees; the tooth bow is a bare
number in mm⁻¹.

The frame's inputs keep the names the video frame gave them, and mean this for the sleeve:
'cageRadius' is where each bore sits on its gear's own axis and the middle of the sleeve's wall;
'cageRise' is half the sleeve's height, to its flat end faces; 'collarHalf' is half the wall's
thickness, so the sleeve runs from radius 'cageRadius - collarHalf' to 'cageRadius + collarHalf';
'collarWall' is the least material the build leaves round every bore and every window, at the
end faces, between two bores and beside a window; 'clearance' is added all round the bore's
rectangle; 'roofAllowance' is added on one face of one bore of each gear, the roof the printer
bridges (§4, "The roof allowance"). The video frame's 'ringRadius', 'ringWire' and 'rodDiameter' are gone with its ring,
loop and rods.

The dialog opens with where the mechanism goes, because nothing can be built without it and
Fusion focuses the first selection input ('[PB-AUTOFOCUS-FIRST]'). Then three groups, each a
'GroupCommandInput' made with 'command.commandInputs.addGroupCommandInput(groupId, groupLabel)',
whose inputs are added to that group's 'children' collection instead of the top-level
'commandInputs'. The groups run from what a user changes most to what they should rarely touch:
the ribbon's size, then the frame, then the mesh arrangement, whose values come from the meshing
search ('mesh-search.md') and jam when changed carelessly, so that group starts collapsed
('isExpanded = False'); the other two start expanded. Within a group the rows run from most to
least important, as listed.

| Group (group id, label) | Dialog label | input id | unit | default |
|---|---|---|---|---|
| none (top level) | Target Plane | 'plane' | selection | — |
| none (top level) | Centre Point | 'point' | selection | — |
| none (top level) | Parent Component | 'parent' | selection | — |
| 'ribbonGroup', Ribbon | Ribbon Width | 'ribbonWidth' | mm | 15 |
| 'ribbonGroup', Ribbon | Tooth Count | 'toothCount' | — | 68 |
| 'ribbonGroup', Ribbon | Twist Lead | 'twistLead' | mm | 49.5 |
| 'ribbonGroup', Ribbon | Ribbon Thickness | 'ribbonThickness' | mm | 3.75 |
| 'ribbonGroup', Ribbon | Tooth Pitch | 'toothPitch' | mm | 2.625 |
| 'ribbonGroup', Ribbon | Tooth Height | 'toothHeight' | mm | 2.625 |
| 'ribbonGroup', Ribbon | Tooth Slant | 'toothSlant' | deg | 25.8 |
| 'ribbonGroup', Ribbon | Tooth Bow | 'toothBow' | — (mm⁻¹) | 0.048 |
| 'frameGroup', Frame | Cage Radius | 'cageRadius' | mm | 15 |
| 'frameGroup', Frame | Cage Rise | 'cageRise' | mm | 18.75 |
| 'frameGroup', Frame | Clearance | 'clearance' | mm | 0.20 |
| 'frameGroup', Frame | Roof Allowance | 'roofAllowance' | mm | 0.60 |
| 'frameGroup', Frame | Collar Half Length | 'collarHalf' | mm | 3 |
| 'frameGroup', Frame | Collar Wall | 'collarWall' | mm | 3 |
| 'meshGroup', Mesh (from the mesh search) | Crossing Angle | 'crossAngle' | deg | 80 |
| 'meshGroup', Mesh (from the mesh search) | Engagement | 'engagement' | mm | 1.05 |
| 'meshGroup', Mesh (from the mesh search) | Mounting Angle A | 'mountAngleA' | deg | 0 |
| 'meshGroup', Mesh (from the mesh search) | Mounting Angle B | 'mountAngleB' | deg | 0 |
| 'meshGroup', Mesh (from the mesh search) | Assembly Phase | 'assemblyPhase' | mm | −1.31 |

Every input is read back by id with 'inputs.itemById(id)' on the command's top-level
'commandInputs', grouped or not; input ids are unique across the whole command, which is what
lets that lookup reach into a group. 'processInputs' raises naming the id if any lookup returns
'None', so a lookup that does not reach into a group fails at once and by name.

Module-level constants for every input id: 'INPUT_ID_RIBBON_WIDTH = 'ribbonWidth'' and so on.
Two more module-level constants are not dialog inputs. 'TOOTH_SPLINE_POINTS = 11' is how many
points each section's toothed side is fitted through (§2). 'CELL_TEETH = 4' is the number of teeth
the lofted cell of §2 holds, written 'cellTeeth' in this spec. Every count that depends on it is
derived from it in 'processInputs', and this spec states those counts for 'cellTeeth = 4' and,
as the fallback the first cell measurement was made at, for 'cellTeeth = 1'.
Value inputs are 'addValueInput(id, label, unit, ValueInput.createByReal(default))' with the
default in internal units ('[PB-DIALOG-DEFAULT-UNITS]'): 'mm/10' for a length, radians for an
angle, the bare count for 'toothCount' and the bare number for 'toothBow'. 'toothBow' has the unit
'''', as 'toothCount' does, and is read the same way; its value is in mm⁻¹, and the build, which
works in cm, lowers the edge by '10*toothBow*v^2' cm for 'v' in cm. The two tooth inputs sit in
the Ribbon group after Tooth Height because they shape the ribbon the printer makes; their
values come from the search for line contact ('mesh-search.md') and are fitted to the default
crossing angle and lead ("Why the ridges lean").

The three selection inputs are 'addSelectionInput(id, label, tooltip)', each with these filters
('[PB-SELECTION-FILTER-ENUM]') and 'setSelectionLimits(1, 1)' ('[PB-SELECTION-DECL]'):

- Target Plane: 'ConstructionPlanes' + 'PlanarFaces'; tooltip 'Plane the cage's axis is normal to'.
- Centre Point: 'ConstructionPoints' + 'SketchPoints'; tooltip 'Centre of the mechanism'.
- Parent Component: 'Occurrences' + 'RootComponents'; pre-selects 'get_design().rootComponent';
  tooltip 'Component the mechanism is created under'.

Read raw values with 'design.unitsManager.evaluateExpression(input.expression, units)', which
returns internal units — cm for length and **radians** for angle ('[PB-EVAL-EXPRESSION]').

**Range checks, all raised as a clear error naming the offending field and the bound:**

- 'ribbonWidth', 'ribbonThickness', 'toothPitch', 'twistLead', 'collarHalf', 'collarWall' and
  'clearance' must be '> 0'; 'roofAllowance' must be '>= 0', zero cutting every bore to the
  clearance alone; 'toothCount' must be a whole number '>= 4'.
- 'toothHeight' must be '> 0' and '< ribbonWidth/2'.
- 'toothSlant' must lie strictly between −90° and 90°, where its tangent is finite. Zero is the
  straight ridge; the sign is the frame's ("The part"), and the build checks it (§2).
- 'toothBow' must be '>= 0', zero leaving the ridge unstraightened, and
  'toothHeight + toothBow*(ribbonThickness/2)^2' must be '< ribbonWidth/2', the root at the faces
  staying on the toothed side of the axis as 'toothHeight < ribbonWidth/2' keeps it on the mid
  plane. The message names 'toothBow'. At the defaults the left side is 2.79 mm against 7.5 mm.
- 'engagement' must be '> 0' and '<= toothHeight'. Past the tooth height the two blanks foul each
  other rather than meshing.
- 'twistLead' has no upper bound: a very long lead approaches two straight racks pushing each
  other, which is the degenerate case Segerman names, and nothing here forbids it. The section
  floor in §2 is what keeps the tooth at a long lead.
- 'crossAngle' must lie strictly between 0° and 180°. At either end the axes are parallel and the
  engaged zone below has no length. 80° is the angle the tooth's lean is fitted to, and 87.2° and
  90° have been measured to drive with it; the search covered 38.5°–100° at the straight tooth
  ('mesh-search.md'), and nothing here clamps to it.
- 'assemblyPhase' must lie strictly within '±toothPitch'. A phase a pitch further on is the same
  phase with the ribbon's ends moved a pitch. Only the default has been measured to sit in the
  free window.
- Both bores, each cut a millimetre past the wall (§4), have to lie within the ribbon's length:
  'cageRadius + collarHalf + 1 mm < toothCount*toothPitch/2', the millimetre being the cut's
  margin. The message names 'cageRadius'.

The sleeve's own four checks come next, in this order, each naming the field given and the
bound; 'sleeveRefusal' in 'sleeve_test.go' is the same four in the same order, and
'TestSleeveInputsAreChecked' reaches each with an input that passes every check before it.
'Ri = cageRadius - collarHalf' is the sleeve's inner radius and 'c = hypot(W/2 + clearance,
T/2 + clearance + roofAllowance)' the corner radius of the level bore's roof side, the furthest
any bore's corner stands from its axis, 8.151 mm at the defaults; every check takes it for all
four bores.

- **The channel starts in the hollow**, naming 'cageRadius': 'hypot(c, 1 mm) < Ri'. Each bore's
  cut starts a millimetre before the channel's corner first reaches the inner face, at station
  'sIn = sqrt(Ri^2 - c^2) - 1 mm' (§4), and the check is 'sIn > 0'. That needs the corner inside
  the inner radius by more than the millimetre allows: with 'c < Ri' alone, a corner within about
  0.04 mm of a 12 mm inner radius would put 'sIn' at or before the middle, so a gear's 'bore-' and
  'bore+' lines would overlap or run backwards and the build would fail inside Fusion. A 4 mm
  clearance fails it, and so does a cage radius 0.02 mm past 'collarHalf + c', where 'sIn' is
  −0.44 mm.
- **The mesh stays visible along the axis**, naming 'cageRadius':
  'hypot(axialWindow, hypot(W/2, T/2)) + clearance <= Ri', with
  'axialWindow = 1.5 * sqrt(W^2 - A^2) / sin(Sigma)', 8.40 mm at the defaults, the reach the
  proof's 'axialWindow' samples. The left side bounds how far the engaged zone's footprint along
  the frame's axis reaches from it, 11.41 mm, and with the clearance 11.61 mm against the 12 mm
  inner radius at the defaults; 'TestSleeveKeepsTheMeshVisibleAlongTheAxis' walks the footprint
  itself, 11.24 mm. At a 1.18 mm engagement the check still passes, by 0.025 mm, while the
  test's walk of the footprint grown by the clearance as a box already finds 16 points hidden;
  the box reaches past the clearance at its corners, which the closed form does not count. At
  1.20 mm the check refuses. The bound is at least 'axialWindow', so it also keeps the engaged zone
  inside the hollow. A 1.5 mm engagement fails it.
- **The end faces keep 'collarWall'**, naming 'cageRise':
  'cageRise >= A/2 + c + collarWall', 18.13 mm at the defaults. No point of a channel is further
  from the middle plane than its axis, 'A/2', plus 'c'. That is a sufficient bound, not the gap:
  the channels reach 14.04 mm from the middle inside the wall, which leaves 4.71 mm of end wall
  at the 18.75 mm default, and a 16.5 mm rise, which this check refuses, leaves 2.46 mm. The
  check also refuses rises from about 17.04 mm up to 18.13 mm, which the channels would allow.
- **The wall between two neighbouring bores keeps 'collarWall'**, naming 'collarWall': the
  separation §4 computes under "The wall between the bores" must be at least 'collarWall'; the
  message names the two bores and the separation. It is 4.967 mm at the defaults, across the gap
  between the two '-R' bores. It depends on nearly every input at once, so it is computed rather
  than bounded by a closed form: 'TestSleeveWindowsFollowTheSize' finds it refusing a sleeve
  scaled to 2/3 with the 3 mm wall kept (2.759 mm), a 4 mm 'collarWall' with a 0.55 mm
  clearance (3.828 mm), and a 5 mm 'collarWall' (4.967 mm), while accepting a sleeve scaled to
  0.75 (3.344 mm).

'collarHalf' trades grip against travel. A thicker wall holds the gear closer to its screw motion
and shortens the travel by twice its own growth, because a ribbon runs
'toothCount*toothPitch - 2*(cageRadius + collarHalf)' through its bores.

After the four checks the window search of §4 runs. It refuses nothing: a gap it finds no room in
gets no window, and the build logs which and why (§4, "When a gap has no room").

**No range is enforced on either Mounting Angle, and none is asserted here.** At the defaults equal
angles from −12° to +12° drive and 0° is the default; what happens elsewhere is the proof's to
map, and this spec does not clamp what has not been measured. Nor is a range enforced on the
slant and the bow past the checks above: the defaults are fitted to the default crossing angle
and lead, and away from them the contact shortens by an amount nothing here measures.


## Sketch Discipline

Every sketch must report 'isFullyConstrained' before it is consumed, and the build raises naming the
sketch when one does not ('[PB-SKETCH-FIRST]', '[PB-FULL-CONSTRAINT]'). No sketch here carries text,
so none is exempt. Six rules hold in every sketch of this build:

- **Only the Anchor sketch projects anything, and every other sketch's references are fixed
  points of its own** ('[SCREW-F-REFERENCES]', '[PB-PROJECT-NOT-FIXED]'). The Anchor sketch
  projects the selected point and binds the Anchor Line to the projection exactly as the bevel
  gear's Anchor sketch does, which reads fully constrained in Fusion. Every later sketch takes
  its references — the ends of a sweep path, the axis point of a section, the corners of a cell
  section, the corners of a window — as
  **reference points**: each is a world point of the frame of §1, mapped in with
  'sketch.modelToSketchSpace', given 'z = 0' when it is meant to lie on the sketch's plane
  ('[PB-SKETCH-ZERO-Z]'; the one exception is the next rule) and added with
  'sketch.sketchPoints.add'. The sketch's curves are drawn **sharing** those points
  ('[PB-SHARE-XOR-COINCIDENT]'), and after the last curve that uses a reference point is drawn,
  and before any constraint or dimension is added, the point is set 'isFixed = True'. A
  **reference line** is a construction line between two reference points, so it has no freedom
  and carries no dimension. Nothing is projected into these sketches and
  'intersectWithSketchPlane' is not called: the point where a sweep path pierces its section
  plane is the path's own start, 'origin_g + s*dir_g' for the span's first station 's', and
  that is what the reference point is placed at. A circle centred on a reference point is
  instead created at that point's position with its 'centerSketchPoint' set 'isFixed = True'
  ('[PB-CIRCLE-CENTER]'), and no reference point is added for it.
- **The Cell Sections sketch is the one sketch whose points lie off its plane, and it keeps
  their 'z'** ('[SCREW-F-CELL-LOFT]', '[PB-3D-SKETCH-SECTIONS]'). Its points are the corners and
  the toothed side's fit points of every section of the tooth cell (§2), which stand at every
  height above and below the Gear Axis Plane the sketch is drawn on. '[PB-SKETCH-ZERO-Z]' zeroes
  the 'z' of a point that is meant to lie on the plane and that rounding has pushed off it; a
  point meant to lie off the plane keeps the 'z' that 'modelToSketchSpace' gives it. Fusion
  accepted such a sketch of rectangles on 2026-09-28, read it fully constrained at 11, 41 and 81
  sections, and found one profile per section; the sketch whose toothed sides are fitted splines
  has not been loaded ("What the proof cannot reach").
- **Every angular dimension is taken against whichever of two perpendicular reference lines puts
  it between 45° and 135°** ('[PB-ANGULAR-DIM]'). Fusion cannot dimension the angle between two
  lines that are nearly parallel, and the one angle this build dimensions — a section's twist at
  its station, in the rectangle scheme of §4 — runs through 0° and 180° along the ribbon. The
  scheme names its two references; the build computes the angle it is about to seed, takes the
  reference that keeps it inside that range, and puts the dimension's text point inside the
  wedge it measures. The value written is the angle between two **rays**, each named in the
  step, from the point where the two lines meet; the text point sits inside that wedge.
- **Every seed is the solved position** ('[PB-SEED-NEAR]'), computed in Python in the frame of §1
  and mapped in with 'modelToSketchSpace', so the solver has nothing to move and a dimension's
  side is the seed's ('[PB-DIM-VALUE-SEMANTICS]'). **Every point mapped in that is meant to lie
  on the plane has its 'z' set to 0 before it is used** — a reference point, a raw seed, an
  arc's through point, a circle's centre and a dimension's text point alike
  ('[PB-SKETCH-ZERO-Z]'). Fusion's section planes do not sit exactly where this build's
  arithmetic puts them, and the first Fusion load failed on a collar section whose points all
  landed '4.4e-6' cm off its plane.
- **No 'addPerpendicular' on a line that only one of its ends anchors.** A line drawn from a
  fixed point, given a length and made perpendicular to a reference has two solutions, one each
  side of the reference, and the proof's sketch gate refuses a sketch that admits a mirror
  image. 'addPerpendicular' is used only in the rectangle scheme of §4, where the line it turns
  already has both ends tied to other lines.
- **Sketch computing is deferred while the two heavy kinds of sketch are drawn, and nowhere
  else** ('[PB-SKETCH-DEFER]', '[SCREW-F-DEFER]'): the Cell Sections sketch of §2, and the four
  bore section sketches of §4. In those, set
  'sketch.isComputeDeferred = True' right after the sketch is created and named and before its
  first point, and set it back to 'False' after the last curve, constraint, dimension or
  'isFixed', before 'isFullyConstrained' or 'profiles' is read. Measured in Fusion on
  2026-09-28: a collar section fell from 0.45 s to 0.23 s and a one-tooth Cell Sections
  sketch from 0.88 s to 0.09 s, each still reading fully constrained with its profiles found.
  The Anchor sketch does not defer, because deferral has not been measured under a projection;
  the Paths, Sleeve and Window sketches do not, because they are a few fixed points with lines
  or circles and were not measured.

The four bore section sketches of §4 share one rectangle scheme, stated there, that leaves no
freedom, so there is no under-constrained case to exempt, unlike the bevel tooth profile. Every
other sketch is built from reference points alone: lines between them and circles centred on
them, with nothing left to solve.

## Method contract — call graph

'''
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
'''


Read selections before any occurrence creation [PB-SELECTION-STASH]. Compute all lengths in cm except the two searches, whose mm outputs are divided by ten. Preserve the method names and call graph above. All subsequent steps run in Design, with no activation [PB-NEVER-ACTIVATE]. The authoritative precompute search is carried below in full.

**The wall between the bores** ('channelSeparation'). This is the fourth of the sleeve's range
checks ("Variables"), computed in 'processInputs' before any feature. Round the tube the bores
alternate between the gears: gear A's '+R' bore at azimuth 'Sigma/2' about '+n̂' from 'ê', gear
B's '-R' at '180° - Sigma/2', gear A's '-R' at '180° + Sigma/2' and gear B's '+R' at '-Sigma/2',
which is 40°, 140°, 220° and 320° at the defaults. So the four gaps between neighbours face '+k̂'
(gear A's '+R' and gear B's '-R' bores), '-ê' (the two '-R' bores), '-k̂' (gear A's '-R' and gear
B's '+R') and '+ê' (the two '+R' bores).

1. Sample each bore's outline ('channelOutline'): at stations every 0.1 mm from 'σ*sIn' outward
   while '|s| <= sOut', 'σ' being '+1' for a '+R' bore and '-1' for a '-R' bore, take seventeen
   points on each side of the bore's rectangle — '(hw, v)' and '(-hw, v)' for
   'v = vLo + (vHi - vLo)*i/16', '(u, vHi)' and '(u, vLo)' for 'u = -hw + 2*hw*i/16',
   'i = 0 … 16' —
   turned to the station's angle and placed as the world point of §1. Keep a point when its
   distance from the frame's axis, 'hypot((P - C)·ê, (P - C)·k̂)', lies within 'Ri - 0.5 mm' to
   'Ro + 0.5 mm'.
2. For each gap, facing 'd': with 'across = n̂ × d', project the kept points of the gap's two bores
   to '((P - C)·across, (P - C)·n̂)', and take the convex hull of each bore's projections by
   Andrew's monotone chain.
3. For every edge of either hull, with 'm' the unit vector square to it, the separation along
   'm' is the larger of 'min(m·q) - max(m·p)' and 'min(m·p) - max(m·q)', 'p' over the first
   hull's corners and 'q' over the second's. The gap's separation is the largest over all the
   edges; it is negative when the hulls overlap.
4. The least of the four gaps' separations must be at least 'collarWall'.

Projecting onto a plane and then onto a line brings no two points nearer, and the hull holds
every projected point, so the separation never exceeds the least distance between the two
outlines, which 'nearestChannels' in the proof measures and 'TestSleevePrintsStandingOnEitherEnd'
holds to 'collarWall'. At the defaults it is 4.967 mm, across the '-ê' gap, where
'nearestChannels' measures 4.968 mm; at the 14° mounting angles before 2026-10-03 they were 5.078
and 5.187 mm.

**The windows.** Down the hollow is one way to see the mesh; two windows through the wall show it
from the side. The windows go across the two wider gaps between neighbouring bores
('windowFacings'): they face '+k̂' and '-k̂' when 'crossAngle <= 90°', where those gaps are
'180° - Sigma' wide against 'Sigma' across '±ê' (100° against 80° at the defaults), and '+ê' and
'-ê' past it. Across a wide gap one flanking bore is low and the other high, and the wall between
them is a band running at about 45° from above the low bore down to below the high one; each
window is cut along that band. Each window's shape is found in 'processInputs', before any
feature, by the search below, which is 'newWindow' in the proof step for step; each window is
found the same way from its own facing direction. At equal mounting angles and no roof
allowance the '-k̂' window comes out as the '+k̂' one turned half a turn about 'ê', as the bores
do; the allowance on gear A's '-R' bore, which flanks the '-k̂' window, makes that window's band
narrower, so the two differ. Neither is a shortcut the build takes.

*The window's plane.* For a window facing the level unit direction 'd', let 'across = n̂ × d',
which is '-ê' for 'd = +k̂'. A point 'P' has plane coordinates 't = (P - C)·across' and
'z = (P - C)·n̂', and depth 'a = (P - C)·d'. The window is the set of points with 'a > 0' whose
'(t, z)' lie in its hexagon: a prism pushed straight out through the wall from the plane through
the frame's axis. At 't' the wall runs from 'a0(t) = sqrt(max(0, Ri^2 - t^2))' to
'a1(t) = sqrt(max(0, Ro^2 - t^2))'.

*A bore's section in the wall* ('sectionInWall'). For a bore of gear 'g' at station 's' with
'|s| < Ro', in the section plane's coordinates 'x' along 'û_g' and 'y' along 'v̂_g': take the
rectangle's corners '(-hw, vLo)', '(hw, vLo)', '(hw, vHi)' and '(-hw, vHi)', in that order, with
the bore's own 'vLo' and 'vHi' ("The roof allowance"), which runs counter-clockwise, each turned by 'theta = s/Lambda + Phi_g' to
'(u*cos theta - v*sin theta, u*sin theta + v*cos theta)'. Clip it to the heights inside the end
faces: keep the 'x' for which '(origin_g + x*û_g - C)·n̂' lies within '±cageRise'. A point at
'(x, y)' stands 'hypot(s, y)' from the frame's axis, so clip what is left twice more, once to
'near <= y <= far' and once to '-far <= y <= -near', with 'near = sqrt(max(0, Ri^2 - s^2))' and
'far = sqrt(Ro^2 - s^2)': those are the section's two **pieces** in the wall, either of which may
be empty. Each clip keeps the part of a convex polygon on one side of a line, walking its edges
in order, keeping each corner on the kept side and adding the point where an edge crosses the
line. At '|s| >= Ro' the section has no piece.

*The long sides.* The two bores whose crossings lie on the window's side, '(crossing - C)·d > 0',
**flank** the window; the other two are its **far** bores. The low flanking bore is the one whose
crossing is lower along 'n̂', and 'lean' is '+1' when the high one's crossing has the larger 't'
and '-1' otherwise. The long sides are the 45° lines 'z + lean*t = lo' and 'z + lean*t = hi'.
Walk each flanking bore's cut span at stations every 0.001 mm from its lower end, and take every
corner of every piece as the world point 'origin_g + x*û_g + y*v̂_g + s*dir_g', with its
'm = z + lean*t' ('wallCorners'). The low bore's largest 'm' is 'lowReach' and the high bore's
least is 'highReach'; then 'lo = lowReach + sqrt(2)*collarWall' and
'hi = highReach - sqrt(2)*collarWall'. 'm' grows by 'sqrt(2)' per millimetre across the band, so
each long side stands 'collarWall' beyond its bore's channel measured on the plane, and a point
of the channel that far from the hexagon on the plane is at least that far from the prism. A
linear measure of a piece is extreme at a corner, so the corners are all the walk needs.

*The trims.* 'zLimit' is the furthest any channel reaches from the middle plane in the wall, up or
down ('channelTop'): over all four bores, at stations from 'σ*sIn' outward every 0.01 mm while
'|s| <= sOut', wherever the section reaches the wall — '|s| <= Ro' and 'hypot(s, y) >= Ri' for the
largest 'y = |u*sin theta + v*cos theta|' over the bore's four corners '(u, v)' — it is the largest
'|(origin_g - C)·n̂ + (u*cos theta - v*sin theta)*(û_g·n̂)|' over those corners, 14.04 mm at the
defaults. Then

'''
top    = min(2*zLimit - hi, hi + sqrt(2)*Ri)
bottom = max(-2*zLimit - lo, lo - sqrt(2)*Ri)
'''

and the trims are the 45° lines 'z - lean*t = top' and 'z - lean*t = bottom'. The side 'hi' meets
'top' at 't = lean*(hi - top)/2', and 'lo' meets 'bottom' at 't = lean*(lo - bottom)/2'. The first
term keeps each long corner inside the height the channels already reach, so both end bands keep
the end wall the channels leave. The second cuts each long side off where it would meet the inner
face more than 45° round from 'd', at '|t| = Ri/sqrt(2)': where a 45° roof meets the curved inner
face, the line they meet on descends at 'atan(cos psi)', 'psi' being how far round the face the
line is from 'd', and past 'psi = 45°' that line steps further along the face per layer than the
proof's one-cell rule accepts.

*The ends* ('windowEnd'). Each window has two upright ends, 't = right' and 't = left', standing
as far out as the far bores allow. For each side 'δ = +1' (right) and 'δ = -1' (left), bisect
'te' 24 times between 0 and 'min(Ri, Ro/sqrt(2))*(1 - 1e-9)': take the middle, and make it the
new lower end when 'clear(te)' holds and the new upper end when it does not. The end is the final
lower end: 'right = end(+1)' and 'left = -end(-1)'. The limit 'Ro/sqrt(2)' keeps a window's roofs
off the outer face where they would meet it more than 45° round. 'clear(te)', at 't = δ*te':

1. 'zLow = max(lo - lean*t, bottom + lean*t)' and 'zHigh = min(hi - lean*t, top + lean*t)', the
   end's two corners; not clear when 'zLow > zHigh'.
2. 'need = collarWall + 0.1 mm/sqrt(2) + 0.005 mm', 3.076 mm at the defaults: 'collarWall' plus
   the most the true least distance can fall under the sampled one, which is half the diagonal
   of a 0.1 mm sample cell and 5 µm for the stations the channel is walked at ('windowSlack').
3. The points, each 'C + t*across + z*n̂ + a*d', are: the end's two corner lines through the wall,
   at 'z = zLow' and at 'z = zHigh', each at 'a = a0, a0 + 0.1 mm, …' and last 'a1', a step that
   would pass 'a1' landing on it; and, on the inner face 'a = a0(t)' and on the outer face
   'a = a1(t)', the end's upright edge between the corners, at 'z = zLow + (zHigh - zLow)*k/n'
   for 'k = 1 … n - 1' with 'n = ceil((zHigh - zLow)/0.1 mm)', and at
   'z = -zLimit + j*0.1 mm' for every 'j >= 0' with 'z <= zLimit' and 'zLow < z < zHigh'.
4. Not clear as soon as any point's distance to either far bore's channel in the wall, by the
   walk below with 'reach = need', is under 'need'; clear otherwise.

The upright edges are sampled at the heights the proof's full check of a window samples an end
at: its walk along the edge, and its walk of the openings, which steps up from '-zLimit'. So the
check that places the ends reads the same points the proof then holds to 'need'. The corner lines
alone, as the proof placed the ends before this search was written, held at the 0.45 mm clearance
the defaults had until 2026-10-02 and gave the same window there, but at a 0.2 mm clearance, with
the mounting angles and engagement of that time, they let an end's edge on the inner face come
2.60 mm from a far bore against the 3 mm 'collarWall'.

*The distance to a channel* ('wallGap'). For a point 'P' and one bore of gear 'g': in the gear's
frame 'x = (P - origin_g)·û_g', 'y = (P - origin_g)·v̂_g' and 'sq = (P - origin_g)·dir_g'. Every
point of a section lies within 'c' of the gear's axis, so when 'hypot(hypot(x, y), sq -
min(max(sq, span start), span end)) - c >= reach' the distance is taken as 'reach' and nothing is
walked. Otherwise walk the bore's **station table**, built once per bore before the first walk:
stations 'span start + k*0.002 mm' for 'k = 0, 1, …' while inside the span, each with its two
pieces, and, wherever the set of non-empty pieces differs between two neighbouring stations,
stations every 0.0001 mm strictly between those two, all in order of 's'. For each piece keep its
**circle**: the average of its corners, and the largest distance from that to a corner. Start with
'best = reach^2'; walk up from the first station at or above 'sq', then down from the one before
it, and stop each way at the first station with '(sq - s)^2 >= best'. At each station, for each
non-empty piece, pass over the piece when 'o = hypot(x - cx, y - cy) - radius' is positive and
'(sq - s)^2 + o^2 >= best'; otherwise set 'best = min(best, (sq - s)^2 + d2)', where 'd2' is 0
when '(x, y)' is inside the piece — on the inner side of every edge of the counter-clockwise
polygon, which a piece of fewer than three corners never is — and otherwise the least squared
distance from '(x, y)' to the piece's edges. The distance is 'sqrt(best)'. A section's points move
at most 1.42 mm per millimetre of station, and the tube's faces clip them faster only near the few
stations where a face is tangent to the section's plane, so between two 2 µm stations the distance
dips under the nearer by at most 4 µm, which the 5 µm in 'need' covers; where a piece appears or
vanishes, the 0.1 µm stations follow the channel's corner first reaching into the wall. The walk
stops only where no station further on could come nearer and passes over only pieces that could
not, so it finds the least a walk of every station would.

*The hexagon.* Its corners are the square '-2*Ro <= t, z <= 2*Ro' clipped, in this order, to
'lean*t + z <= hi', '-lean*t - z <= -lo', '-lean*t + z <= top', 'lean*t - z <= -bottom',
't <= right' and '-t <= -left' ('clipCorners'), so every edge is upright or at 45°. Drop a corner
that lies within 0.001 mm of the one before it, since a sketch line cannot have zero length. At
the defaults the '+k̂' window has 'lo = -5.180', 'hi = 5.180', 'bottom = -22.151', 'top = 22.151',
'left = -11.570' and 'right = 11.570', and its six corners in '(t, z)' are '(11.57, -6.39)',
'(-8.49, 13.67)', '(-11.57, 10.58)', '(-11.57, 6.39)', '(8.49, -13.67)' and '(11.57, -10.58)': long
sides 7.33 mm apart across the band, ends 23.14 mm apart, 220.7 mm² on the plane, reaching
13.67 mm from the middle against the channels' 14.04 mm. The '-k̂' window has 'lo = -4.587',
'hi = 5.180', 'bottom = -21.558', 'top = 22.151', 'left = -11.500' and 'right = 11.570', and its
corners are '(11.57, -6.39)', '(-8.49, 13.67)', '(-11.50, 10.65)', '(-11.50, 6.91)',
'(8.49, -13.07)' and '(11.57, -9.99)': long sides 6.91 mm apart, ends 23.07 mm apart, 206.7 mm².

*When a gap has no room* ('room'). A window is not cut when 'hi <= lo', the flanking bores leaving
no band between them, or when 'right <= left' or the hexagon has fewer than three distinct
corners or no area, the far bores leaving the band no length. The build then logs
'No window facing {d}: {reason}' with 'futil.log' ('[PB-LOGGING]'), '{d}' being '+k', '-k', '+e'
or '-e', and builds the sleeve without that window; a window only takes material away, so every
check the sleeve passes still holds without it. No input 'TestSleeveWindowsFollowTheSize'
accepts reaches this rule. It leaves both windows out at a 7 mm 'collarWall', where the flanking
bores leave no band; the build refuses that input for the wall between its bores anyway.

*What the search keeps.* 'TestSleeveWindowsKeepTheirWalls' holds the default windows: each
flanking bore's channel lies exactly 3.000 mm beyond a long side, measured on the plane; every
face of the cut keeps 3.076 mm from the far bores' channels, sampled every 0.1 mm; every edge
rises at 45° or more; the faces meet the tube at edges of 50.0° or more; and the lines where the
roofs meet the tube descend at 35.3° or more. 'TestSleeveWindowPostsStandFirm' holds every post
beside a window to at least 'collarWall' wide, 3.77 mm at the narrowest at the defaults, and any
stretch of post under two 'collarWall's wide to at most twice as tall as it is wide, 0.69 times
at the defaults. Through either window 80.6% of the mesh zone can be seen past both ribbons
('TestSleeveWindowsShowTheMeshFromTheSide').
'TestSleeveWindowsFollowTheSize' runs all of those checks, with the one-piece and printing checks,
at 33 inputs across the dialog's ranges, and every window it cuts passes.


The 'show Sketch.project' query has no match; the spec deliberately retains the historically measured call. The API docs forbid parent preselection with addSelection during commandCreated; the configure recipe does not state an activate hook. Both are spec gaps; preserve the requirements for source review.

Make these required Fusion calls:

- `command.commandInputs.addGroupCommandInput(groupId, groupLabel)`.
- `command.commandInputs.addSelectionInput(id, label, tooltip)`.
- `selectionInput.addSelectionFilter(filterConstant)`.
- `selectionInput.setSelectionLimits(1, 1)`.
- `parentInput.addSelection(rootComponent)`.
- `group.children.addValueInput(id, label, unit, initialValue)`.
- `adsk.core.ValueInput.createByReal(value)`.
- `inputs.itemById(id)`.
- `selectionInput.selection(0).entity`.
- `self.design.unitsManager.evaluateExpression(input.expression, units)`.

<!-- step-meta
{
  "calls": [
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
      "name": "addValueInput",
      "owner": "adsk.core.CommandInputs",
      "reason": null,
      "receiver": "group.children",
      "role": "required",
      "span": "group.children.addValueInput(id, label, unit, initialValue)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(value)"
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
      "span": "selectionInput.selection(0).entity"
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
      "first": 576,
      "last": 879,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L576–879.

## 01 `[PROSE]` Create Design component

The top Screw Gearing occurrence comes from inherited getOccurrence. Create the Design occurrence beneath it at the identity transform. Name its component 'Design'. [PB-OCCURRENCE-TREE] [PB-NO-CROSS-SIBLING] [PB-NEVER-ACTIVATE].

Make these required Fusion calls:

- `adsk.core.Matrix3D.create()`.
- `topComponent.occurrences.addNewComponent(identity)`.

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
      "receiver": "topComponent.occurrences",
      "role": "required",
      "span": "topComponent.occurrences.addNewComponent(identity)"
    }
  ],
  "citations": [
    {
      "first": 608,
      "last": 625,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L608–625.

## 02 `[PROSE]` Create Gear A component

The top Screw Gearing occurrence comes from inherited getOccurrence. Create the Gear A occurrence beneath it at the identity transform. Name its component 'Gear A'. [PB-OCCURRENCE-TREE] [PB-NO-CROSS-SIBLING] [PB-NEVER-ACTIVATE].

Make these required Fusion calls:

- `adsk.core.Matrix3D.create()`.
- `topComponent.occurrences.addNewComponent(identity)`.

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
      "receiver": "topComponent.occurrences",
      "role": "required",
      "span": "topComponent.occurrences.addNewComponent(identity)"
    }
  ],
  "citations": [
    {
      "first": 608,
      "last": 625,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L608–625.

## 03 `[PROSE]` Create Gear B component

The top Screw Gearing occurrence comes from inherited getOccurrence. Create the Gear B occurrence beneath it at the identity transform. Name its component 'Gear B'. [PB-OCCURRENCE-TREE] [PB-NO-CROSS-SIBLING] [PB-NEVER-ACTIVATE].

Make these required Fusion calls:

- `adsk.core.Matrix3D.create()`.
- `topComponent.occurrences.addNewComponent(identity)`.

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
      "receiver": "topComponent.occurrences",
      "role": "required",
      "span": "topComponent.occurrences.addNewComponent(identity)"
    }
  ],
  "citations": [
    {
      "first": 608,
      "last": 625,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L608–625.

## 04 `[PROSE]` Create Cage component

The top Screw Gearing occurrence comes from inherited getOccurrence. Create the Cage occurrence beneath it at the identity transform. Name its component 'Cage'. [PB-OCCURRENCE-TREE] [PB-NO-CROSS-SIBLING] [PB-NEVER-ACTIVATE].

Make these required Fusion calls:

- `adsk.core.Matrix3D.create()`.
- `topComponent.occurrences.addNewComponent(identity)`.

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
      "receiver": "topComponent.occurrences",
      "role": "required",
      "span": "topComponent.occurrences.addNewComponent(identity)"
    }
  ],
  "citations": [
    {
      "first": 608,
      "last": 625,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L608–625.

## 05 `[GO]` Anchor sketch

### 1: Anchor and frame

Create the anchor sketch, named 'Anchor', on the user-selected plane and project the selected
point into it, 'sketch.project(point).item(0)'; this is the one projection in the build
('[SCREW-F-REFERENCES]'). Draw the **Anchor Line** through it: a line from two raw 'Point3D'
seeds 0.5 cm either side of the projected point along the sketch's own x axis, so it is 10 mm
long with its end to the right of its start, and constrain it with four things and nothing else,
which is the bevel gear's Anchor sketch and reads fully constrained in Fusion:

- 'addCoincident(projectedPoint, line)' — the centre lies on the line — **and**
  'addMidPoint(projectedPoint, line)' — the centre bisects it. Both, not the midpoint alone,
  as the bevel spec requires of its own Anchor sketch. The proof's sketch engine emits the
  coincident row as part of its midpoint constraint, so the compiled proof writes the midpoint
  alone, as 'proof/bevelgear' does.
- 'addHorizontal(line)' — sketch-local, so it survives a tilted plane ('[PB-REFLINE-DIRECTION]').
- A **horizontal** distance dimension from the line's start to its end
  ('addDistanceDimension(start, end, HorizontalDimensionOrientation, textPoint)'), value 10 mm.
  Not an aligned one: midpoint, horizontal and an aligned length are satisfied by the line in
  either of its two end-for-end orientations, and the proof's sketch gate refuses that as
  ambiguous; a horizontal distance from start to end runs one way, so only the seeded orientation
  satisfies it.

Midpoint, horizontal and the horizontal distance take the line's four degrees of freedom, so it
has none, and its absolute direction is arbitrary. The build raises unless the sketch reports
'isFullyConstrained', and only then reads the frame from world geometry
('[PB-WORLDGEO-CONSTRAINED]', '[PB-WORLD-FRAME]'): 'C' is the projected point's
'worldGeometry', and 'ê' is the unit vector from the line's 'startSketchPoint.worldGeometry' to
its 'endSketchPoint.worldGeometry'. The plane's normal is 'n̂', read in the next paragraph but
one. No later sketch projects the point or the line: every other sketch takes 'C' and 'ê' as
numbers, and the Anchor Line itself is passed once more, as the line the Window Plane of §4 is
built on.

Compute both axes from 'ê' and 'n̂':

'''
dirA = rotate(ê, +Sigma/2, about n̂)      originA = C - (A/2)*n̂
dirB = rotate(ê, -Sigma/2, about n̂)      originB = C + (A/2)*n̂
'''

Gear A's cross-section frame is 'û_A = +n̂', 'v̂_A = dirA × û_A'; gear B's is 'û_B = -n̂',
'v̂_B = dirB × û_B'. **'û' points at the other gear** — that is what makes 'Phi' mean the same thing
for both parts, and it is why the two gears are the same part rather than mirror images. A point
of gear 'g' at station 's' with section coordinates '(u, v)' is
'origin_g + s*dir_g + (u*cos theta - v*sin theta)*û_g + (u*sin theta + v*cos theta)*v̂_g', with
'theta = s/Lambda + Phi_g'; every seed below is that point. The third direction of the frame is
'k̂ = n̂ × ê', level and square to 'ê'; the proof's 'pair' puts 'ê', 'k̂' and 'n̂' on its X, Y
and Z axes with 'C' at the origin, and §4 names directions by them.


[PB-SKETCH-FIRST] [PB-FULL-CONSTRAINT] [PB-WORLD-FRAME] [SCREW-F-REFERENCES].

The proof function is `stepEntry05Anchorsketch`.

<!-- proof-run: proofkit.Run(anchorCases, stepEntry05Anchorsketch) -->

Make these required Fusion calls:

- `design.sketches.add(targetPlane)`.
- `sketch.project(selectedPoint)`.
- `projected.item(0)`.
- `adsk.core.Point3D.create(x, y, z)`.
- `sketch.sketchCurves.sketchLines.addByTwoPoints(startSeed, endSeed)`.
- `sketch.geometricConstraints.addCoincident(projectedPoint, anchorLine)`.
- `sketch.geometricConstraints.addMidPoint(projectedPoint, anchorLine)`.
- `sketch.geometricConstraints.addHorizontal(anchorLine)`.
- `sketch.sketchDimensions.addDistanceDimension(anchorLine.startSketchPoint, anchorLine.endSketchPoint, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "design.sketches",
      "role": "required",
      "span": "design.sketches.add(targetPlane)"
    },
    {
      "condition": null,
      "name": "project",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.project(selectedPoint)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "projected",
      "role": "required",
      "span": "projected.item(0)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "adsk.core.Point3D.create(x, y, z)"
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
      "span": "sketch.sketchDimensions.addDistanceDimension(anchorLine.startSketchPoint, anchorLine.endSketchPoint, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)"
    }
  ],
  "citations": [
    {
      "first": 923,
      "last": 978,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L923–978.

## 06 `[PROSE]` Gear A axis plane

Create Gear A Axis Plane, offset from targetPlane by -A/2. Read Gear A's geometry normal and sign nHat so C is A/2 above it. Check both plane distances equal A/2. [PB-USE-SELECTED-PLANE] [PB-CONSTRUCTION-PLANES] [SCREW-F-NORMAL-SIGN].

Make these required Fusion calls:

- `design.constructionPlanes.createInput()`.
- `adsk.core.ValueInput.createByReal(value)`.
- `design.constructionPlanes.add(planeInput)`.
- `planeInput.setByOffset(targetPlane, offsetValue)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "design.constructionPlanes",
      "role": "required",
      "span": "design.constructionPlanes.createInput()"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(value)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "design.constructionPlanes",
      "role": "required",
      "span": "design.constructionPlanes.add(planeInput)"
    },
    {
      "condition": null,
      "name": "setByOffset",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByOffset(targetPlane, offsetValue)"
    }
  ],
  "citations": [
    {
      "first": 971,
      "last": 978,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L971–978.

## 07 `[PROSE]` Gear B axis plane

Create Gear B Axis Plane, offset from targetPlane by +A/2. Read Gear A's geometry normal and sign nHat so C is A/2 above it. Check both plane distances equal A/2. [PB-USE-SELECTED-PLANE] [PB-CONSTRUCTION-PLANES] [SCREW-F-NORMAL-SIGN].

Make these required Fusion calls:

- `design.constructionPlanes.createInput()`.
- `adsk.core.ValueInput.createByReal(value)`.
- `design.constructionPlanes.add(planeInput)`.
- `planeInput.setByOffset(targetPlane, offsetValue)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "design.constructionPlanes",
      "role": "required",
      "span": "design.constructionPlanes.createInput()"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(value)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "design.constructionPlanes",
      "role": "required",
      "span": "design.constructionPlanes.add(planeInput)"
    },
    {
      "condition": null,
      "name": "setByOffset",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByOffset(targetPlane, offsetValue)"
    }
  ],
  "citations": [
    {
      "first": 971,
      "last": 978,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L971–978.

## 08 `[GO]` Gear A paths sketch

**The Paths sketches.** 'Gear A Paths' on the Gear A Axis Plane and 'Gear B Paths' on gear B's,
each holding the two lines that gear's bores are swept along (§4), both on the gear's axis,
which lies in that plane. The '+R' bore's cut spans stations 'sIn' to 'sOut' of the gear's axis
and the '-R' bore's '-sOut' to '-sIn', with 'sIn = sqrt(Ri^2 - c^2) - 1 mm' and
'sOut = cageRadius + collarHalf + 1 mm' (§4): 7.806 and 19 mm at the defaults. The sketch holds
four reference points (Sketch Discipline), at 'origin_g + s*dir_g' for 's' = '-sOut', '-sIn',
'sIn' and 'sOut', all on the plane, and two solid lines — 'bore-' from '-sOut' to '-sIn' and
'bore+' from 'sIn' to 'sOut' — each drawn from its negative station to its positive one, sharing
both points, so its start is its negative end and it runs along '+dir_g'; then all four points
are set 'isFixed = True'. The lines carry no dimension and no constraint, and nothing else is in
the sketch. Both lie on one line with a gap between them; Fusion read a sketch of four such
lines, overlapping, fully constrained on 2026-09-28, and a path made from one of its lines with
chaining off held that one line ('[PB-PATH-FROM-SKETCH]'). A line whose two ends are fixed has a
'worldGeometry' the build can trust ('[PB-WORLDGEO-CONSTRAINED]'), and a line held by dimensions
from a projected point does not. The section plane of each bore is 'setByDistanceOnPath' on its
own line at fraction '0' ('[PB-CONSTRUCTION-PLANES]'; pass the line directly), so it stands at
the span's negative end, square to the axis; the point where the line pierces it is the span's
first station, 'origin_g + s*dir_g', and Fusion put that plane's origin on the station to four
decimals of a millimetre. The sweep's path is 'features.createPath(line, False)' on the same
line ('[SCREW-F-TWISTED-SLOT]').

[PB-PATH-FROM-SKETCH] [PB-WORLDGEO-CONSTRAINED].

The proof function is `stepEntry08GearApathssketch`.

<!-- proof-run: proofkit.Run(pathCases, stepEntry08GearApathssketch) -->

Make these required Fusion calls:

- `design.sketches.add(plane)`.
- `adsk.core.Point3D.create(x, y, z)`.
- `sketch.modelToSketchSpace(worldPoint)`.
- `sketch.sketchPoints.add(local)`.
- `sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "design.sketches",
      "role": "required",
      "span": "design.sketches.add(plane)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "adsk.core.Point3D.create(x, y, z)"
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
      "span": "sketch.sketchPoints.add(local)"
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
      "first": 979,
      "last": 998,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L979–998.

## 9 `[GO]` Gear A cell sections sketch

### 2: The tooth cell

One cell is 'c' tooth pitches of the finished ribbon, at the ribbon's negative end: stations 's'
from 's0 = Z0 - L/2' to 's0 + c*P', where 'Z0' is the gear's tooth phase — '0' for gear A and the
'assemblyPhase' input for gear B ("Defaults") — and 'c = min(cellTeeth, N)' is the number of
teeth the cell holds, 4 at the defaults ('cellTeeth', "Variables"). Gear B's whole ribbon is
gear A's advanced by 'Z0' under its own screw motion, ends and teeth alike, so the same cell
built 'Z0' further along its axis and repeated the same way is gear B. Build it once per gear.

**The section count is derived, not pinned, and it is not user-configurable.** The cell is lofted
through 'c*n + 1' sections, 'n' steps to the tooth, where 'n' is the larger of two counts:

- the smallest that keeps the twist between neighbouring sections under 2°,
  'ceil((P/Lambda) / 2°)', 10 at the defaults. A surface ruled straight between two sections
  cuts the corner of the true helicoid at the crest by '(W/2)*(1 - cos(dtheta/2))', 'dtheta'
  the twist per step: 1.0 µm at the defaults' 1.91°, under a hundredth of the 1.08 mm backlash,
  which is the bound the proof holds it to (the shortfall grows with the width and the backlash
  with the pitch, so it is not held to a fixed number of microns);
- eight. A straight chord of the cosine between two sections falls '(H/2)*(1 - cos(pi/n))' short
  of it at the deepest point whatever the twist: 0.064 mm at the defaults' ten steps, 6% of the
  backlash, and 0.100 mm, 3.8% of the tooth height, at eight. The proof holds it under a fifth of
  the backlash; it was 7.8% of the 0.82 mm backlash before 2026-10-02 and 14% of the 0.46 mm one
  until 2026-10-03, when the leaned tooth widened the backlash. The chord runs along 's' at every
  'v' alike, so the lean does not change it. That floor is what keeps a slow
  twist from lofting the tooth through two or three sections and losing it: at a 400 mm lead the
  twist alone asks for two sections, and the cell is built through nine.

That is 10 steps to the tooth at the defaults: 41 sections in a four-tooth cell, 11 in a
one-tooth cell. 'TestLoftSectionCountHoldsTheHelicoid' holds both bounds over leads from 20 mm
to 400 mm.

**What the loft is, and what the count guarantees for it.** Both bounds are arithmetic about a
*ruled* loft, whose surface runs straight between neighbouring sections. Fusion builds a ruled
loft only through exactly two sections; a loft through more passes through every section and is
fitted smoothly between them, and 'LoftFeatureInput' has no option to make it ruled — its
sections carry only end conditions ('[SCREW-F-CELL-LOFT]'). The cell is **one loft through all
'c*n + 1' sections**, so it is the smooth kind. What the count guarantees for it is that the
built cell carries the exact rotated section at each of its stations, 'dtheta' apart, its
toothed side the spline through the edge's points, and that between stations its surface
interpolates those sections; the chord figures above are the
departure of the ruled loft through the same sections, and they are the only figures the proof
has. What the smooth surface does between sections was measured in Fusion on 2026-09-28, at
the defaults of that day, the 10 mm ribbon ('[SCREW-F-CELL-LOFT]'): at one, four and eight
teeth, probes 0.04 mm inside and 0.04 mm outside the toothed edge and a face, at the midpoint
between every pair of sections, all fell on the right side of the built surface — 40 of 40,
160 of 160 and 320 of 320 — and the built volume was 0.06% to 0.23% under the ruled loft's. So
at that size the built surface stayed within 0.04 mm of the helicoid between sections, under a
tenth of the 0.44 mm backlash of that day, with straight-ridge rectangles for sections; the 1.5×
ribbon has the same section spacing, and neither it nor the spline sections have been
measured. The proof cannot measure that ("What the proof cannot reach"), and a Fusion
load is what checks it at other inputs. A chain of 'c*n' two-section lofts would be ruled and would carry the bounds
literally, at the cost of that many lofts and joins per cell, and this spec keeps the one loft.

**The Cell Sections sketch.** All the cell's sections go in **one sketch**, named
'{gearLabel} Cell Sections', on the gear's Axis Plane ('[SCREW-F-CELL-LOFT]',
'[PB-3D-SKETCH-SECTIONS]'), drawn with sketch computing deferred (Sketch Discipline,
'[PB-SKETCH-DEFER]'). No section plane is made. Section 'k', for 'k' from '0' to 'c*n', is the
cross-section at station 's_k = s0 + k*P/n', turned about the axis by 'theta_k'. Its back side
runs along 'u = -W/2', its faces along 'v = ±T/2', and its toothed side follows the edge of "The
part" across the thickness, through 'M' points:

'''
theta_k        = s_k/Lambda + Phi
v_j            = -T/2 + j*T/(M - 1),    j = 0 … M - 1,    M = TOOTH_SPLINE_POINTS = 11
Utooth(v, s_k) = W/2 - H/2 + (H/2)*cos(2*pi*(s_k + tan(Slant)*v - Z0)/P) - Bow*v^2
'''

'M' is a module constant, beside 'CELL_TEETH' ("Variables"): eleven points, 'T/10' apart, 0.375 mm
at the defaults, both faces included. 'TestToothSplineHoldsTheEdge' fits a natural cubic spline
through them at every station of a pitch and finds it within 0.018 mm of the edge along 'u';
through nine points it departs by 0.030 mm, through seven by 0.058 mm, because across the
thickness the edge runs through 0.69 of a pitch of phase.

Each section has 'M + 2' points: the two back corners 'B0' at '(uB, -hv)' and 'B1' at '(uB, hv)',
and the toothed points 'F_j' at '(Utooth(v_j, s_k), v_j)', of which 'F_0' and 'F_(M-1)' are the
toothed corners; 'uB = -W/2' and 'hv = T/2'. Each is the world point of §1 at those section
coordinates and station 's_k', mapped in with 'sketch.modelToSketchSpace(point)' and added with
'sketch.sketchPoints.add(point)' **with its 'z' kept**: the points lie off the sketch's plane on
purpose (Sketch Discipline, '[PB-SKETCH-ZERO-Z]' not applied). Four curves share them, in this
order ('[PB-SHARE-XOR-COINCIDENT]', '[PB-SKETCHCURVES]'):

- 'L1 = sketch.sketchCurves.sketchLines.addByTwoPoints(B0, F_0)', the face at 'v = -hv'.
- 'S = sketch.sketchCurves.sketchFittedSplines.add(fitPoints)', the toothed side, where
  'fitPoints = adsk.core.ObjectCollection.create()' holds the sketch points 'F_0' to 'F_(M-1)',
  each put in with 'fitPoints.add(F_j)' in order of 'j'. The spline runs from 'F_0', the end of
  'L1', to 'F_(M-1)', the start of 'L3'. Raise naming the section unless 'S' is not 'None' and
  'S.fitPoints.count' is 'M'.
- 'L3 = sketch.sketchCurves.sketchLines.addByTwoPoints(F_(M-1), B1)', the face at 'v = +hv'.
- 'L4 = sketch.sketchCurves.sketchLines.addByTwoPoints(B1, B0)', the back side.

The build keeps each section's four curves together, in that order, because they are what the
loft is fed. After the last curve of the last section is drawn, every point in the sketch is set
'isFixed = True': every point the build added, and every item of every spline's 'fitPoints', by
'S.fitPoints.item(i)' for 'i' below 'S.fitPoints.count'. The API reference says the spline takes
existing sketch points as fit points and does not say whether it keeps those objects or makes
its own, so both are fixed. Nothing else is in the sketch: no construction line, no constraint,
no dimension, and no tangent or curvature handle ('activateTangentHandle' and
'activateCurvatureHandle' are not called). Then set 'isComputeDeferred' back to 'False', raise
unless the sketch reports 'isFullyConstrained', and raise unless 'sketch.profiles.count' is
'c*n + 1', one profile per section. The profiles are not what the loft is fed, because nothing
says which profile is which station.

The calls and their signatures, as the Fusion API reference database ('fusion:query-api') gives
them:

| Call | Signature |
|---|---|
| 'SketchPoints.add' | '(point: core.Point3D) -> SketchPoint' |
| 'Sketch.modelToSketchSpace' | '(modelCoordinate: core.Point3D) -> core.Point3D' |
| 'SketchLines.addByTwoPoints' | '(startPoint: core.Base, endPoint: core.Base) -> SketchLine', either a 'SketchPoint' or a 'Point3D' |
| 'SketchFittedSplines.add' | '(fitPoints: core.ObjectCollection) -> SketchFittedSpline'; "any combination of existing SketchPoint or Point3D objects"; 'None' if it failed |
| 'ObjectCollection.create' | '() -> ObjectCollection', static |
| 'ObjectCollection.add' | '(item: Base) -> bool' |
| 'SketchFittedSpline.fitPoints' | 'SketchPointList', read-only, start point first and end point last; 'count: int', 'item(index: int) -> SketchPoint' |
| 'SketchPoint.isFixed' | 'bool', read/write, declared on 'SketchEntity' |
| 'Sketch.isComputeDeferred' | 'bool', read/write |
| 'Features.createPath' | '(curve: core.Base, isChain: bool = True) -> Path' |
| 'LoftFeatures.createInput' | '(operation: FeatureOperations) -> LoftFeatureInput' |
| 'LoftSections.add' | '(entity: core.Base) -> LoftSection'; a 'Path' is one of the entities it takes |
| 'LoftFeatures.add' | '(input: LoftFeatureInput) -> LoftFeature' |
| 'BRepBody.pointContainment' | '(point: core.Point3D) -> PointContainment' |

**What the loft sees.** Every section is one closed loop of the same four curves in the same
order — a line, a fitted spline, a line, a line — so every section has the same number of curves
and the same number of corners, and the loft meets the spline of one section with the spline of
the next. A section with a different count would leave the loft to pair unlike curves; the build
raises before the loft unless every section holds exactly the four curves above.

**Loft the sections in station order** ('[PB-LOFT]', '[SCREW-F-CELL-LOFT]'). Create the loft
with 'loftFeatures.createInput(NewBodyFeatureOperation)'; for each section in order of 'k', put
its four curves in an 'ObjectCollection' in the order 'L1', 'S', 'L3', 'L4', make its path with

Keep every off-plane z; never flatten this sketch. Set isComputeDeferred True before points, fix original and spline fit points after all curves, set it False before profiles or constraints are read. No tangent or curvature handle is activated. [PB-3D-SKETCH-SECTIONS] [PB-SKETCH-DEFER].

The proof function is `stepEntry9GearAcellsectionssketch`.

<!-- proof-run: proofkit.Run(sectionCases, stepEntry9GearAcellsectionssketch) -->

Make these required Fusion calls:

- `design.sketches.add(plane)`.
- `adsk.core.Point3D.create(x, y, z)`.
- `sketch.modelToSketchSpace(worldPoint)`.
- `sketch.sketchPoints.add(local)`.
- `sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)`.
- `adsk.core.ObjectCollection.create()`.
- `fitPoints.add(sectionPoint)`.
- `sketch.sketchCurves.sketchFittedSplines.add(fitPoints)`.
- `spline.fitPoints.item(i)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "design.sketches",
      "role": "required",
      "span": "design.sketches.add(plane)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "adsk.core.Point3D.create(x, y, z)"
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
      "span": "sketch.sketchPoints.add(local)"
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
      "span": "fitPoints.add(sectionPoint)"
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
      "first": 1000,
      "last": 1130,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1000–1130.

## 10 `[GO]` Gear A cell loft

'features.createPath(collection, False)' ('[PB-PATH-FROM-SKETCH]'; never 'Path.create', which
raises in this component), and 'loftSections.add(path)'; then 'loftFeatures.add(input)'. The
result is 'c' pitches of the twisted toothed ribbon in one body; raise unless the feature leaves
exactly one body and that body 'isSolid' ('[PB-EMPTY-RESULT]'). Measured on 2026-09-28 at the
10 mm ribbon, with four-line sections: the four-tooth loft took 0.23 s and held 0.164158 cm³
against the ruled loft's 0.164470. The loft through spline sections has not been timed.

**The screw step still holds.** 'Utooth(v, s + P) = Utooth(v, s)' at every 'v': the slant moves
the cosine's phase by an amount that depends on 'v' alone and the bow lowers the edge by an
amount that depends on 'v' alone, so the section 'n' steps on, one pitch along the axis, is
section 'k' carried by 'Step(1)' of §3, its toothed points included. The cell is still one
pitch-periodic piece of the ribbon, and the doubling of §3 and the remainder cell are unchanged;
'TestRibbonIsInvariantUnderItsScrewStep' carries every section's corners and toothed points
through one screw step and finds them on the next pitch's to 1e-9 mm.

**Checking the slant's sign** ('[PB-SELF-DIAGNOSING]'). Nothing in the sketch shows which way a
ridge leans, and a slant of the wrong sign builds a cell whose ridges cross the other ribbon's at
61° where the axes cross instead of lying along them ("The part"). So after the loft, before §3,
the build probes the cell with 'cellBody.pointContainment(point)' at four points. Let
'sc = Z0 + P*ceil((s0 + P/2 - Z0)/P)', the first crest station at least half a pitch into the
cell. For each face sign 'σ' of '-1' and '+1', let 'v_p = σ*(T/2 - 0.25 mm)' and
'u_p = W/2 - Bow*v_p^2 - 0.25 mm', a quarter millimetre inside the face and under the crest. The
**on-ridge** probe is the world point of §1 at '(u_p, v_p)' and station 'sc - tan(Slant)*v_p',
where the ridge through 'sc' crosses 'v_p'; the **off-ridge** probe is the same at station
'sc + tan(Slant)*v_p', where that ridge would cross under the opposite sign. For each probe the
build evaluates 'm = Utooth(v_p, s) - u_p' at the probe's station under the input slant and 'm''
under its negation. A probe is used when '|m|' and '|m'|' are both at least 0.1 mm and their signs
differ. A used probe must read 'PointInsidePointContainment' when 'm > 0' and
'PointOutsidePointContainment' when 'm < 0'; otherwise the build raises naming the gear, the
probe and what it read. When no probe is used, as with a slant near zero, the build logs with
'futil.log' ('[PB-LOGGING]') that the sign was not checked. At the defaults all four are used: on
the ridge each stands 0.25 mm inside the tooth under the right sign and 2.13 mm outside under the
wrong one, and off the ridge the reverse; gear A's stand 1.84 and 3.41 mm from the cell's start,
inside its 10.5 mm ('TestToothSlantProbesTellTheHand'). Lengths are in cm in the build, as
everywhere.


Loft all sections through one feature, in ascending station order, from four-curve paths L1, S, L3, L4. Do not feed unordered profiles. Require one solid body. [PB-LOFT] [PB-PATH-FROM-SKETCH] [PB-EMPTY-RESULT].

The proof function is `stepEntry10GearAcellloft`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry10GearAcellloft, assertCellSegmentLoft) -->

Make these required Fusion calls:

- `design.features.loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`.
- `adsk.core.ObjectCollection.create()`.
- `curves.add(sectionCurve)`.
- `design.features.createPath(curves, False)`.
- `loftInput.loftSections.add(sectionPath)`.
- `design.features.loftFeatures.add(loftInput)`.
- `loftFeature.bodies.item(0)`.
- `cellBody.pointContainment(probePoint)`.

The supported proof substitution is explicit:

The proof builds one real two-section interval of this cell, using its actual sampled sections. The evaluator
refuses unions of adjacent intervals at their tangent triangulated caps. The sketch proof checks every sampled
section. This solid substitute checks the representative interval's solidity and prismoid volume, and establishes
no whole-cell smooth surface or volume.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.LoftFeatures",
      "reason": null,
      "receiver": "design.features.loftFeatures",
      "role": "required",
      "span": "design.features.loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)"
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
      "span": "curves.add(sectionCurve)"
    },
    {
      "condition": null,
      "name": "createPath",
      "owner": "adsk.fusion.Features",
      "reason": null,
      "receiver": "design.features",
      "role": "required",
      "span": "design.features.createPath(curves, False)"
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
      "receiver": "design.features.loftFeatures",
      "role": "required",
      "span": "design.features.loftFeatures.add(loftInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "loftFeature.bodies",
      "role": "required",
      "span": "loftFeature.bodies.item(0)"
    },
    {
      "condition": null,
      "name": "pointContainment",
      "owner": "adsk.fusion.BRepBody",
      "reason": null,
      "receiver": "cellBody",
      "role": "required",
      "span": "cellBody.pointContainment(probePoint)"
    }
  ],
  "citations": [
    {
      "first": 1131,
      "last": 1166,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1131–1166.

## 11 `[GO]` Gear A aside copy

At q=17, copy the one-cell body before doubling. Keep that copy unmoved as the one-cell aside. ### 3: Repeat the cell by doubling

The finished ribbon is the cell, 'c' teeth, repeated under the **screw step**

'''
Step(k) = translate(k*P along the axis) ∘ rotate(k*P/Lambda about the axis)
'''

Write 'N = q*c + r', with 'q = floor(N/c)' whole cells and a remainder of 'r = N mod c' teeth:
'q = 17' and 'r = 0' at the defaults, 'q = 68' at a one-tooth cell. Build the 'q' cells by
doubling rather than by 'q' separate placements. A **round** is copy, move, join: copy the
current body, move the copy by 'Step(m*c)' where 'm' is the number of cells the body holds, join
the two, and the body holds '2m' cells. Doubling alone reaches only a power of two, and copying
the whole body for the rest would overlap it, so the rest is made of unmoved copies put aside on
the way up:

- Write 'q' in binary. Start with the cell, 'm = 1'.
- For each bit of 'q' below its top bit, lowest first: if that bit is set, take a copy of the
  body and keep it unmoved — an **aside** of 'm' cells; then double.
- After the last doubling 'm' is the top bit's power of two. Move each aside into place, largest
  first, by 'Step(m*c)', join it, and add its cells to 'm'. The last join brings 'm' to 'q'.

That is 'floor(log2 q) + popcount(q) - 1' rounds and as many joins: 5 at the defaults — 'q = 17',
one aside of the single cell, taken before the first doubling, then four doublings to 16 cells,
and the aside moved by 'Step(64)' — and 7 at a one-tooth cell — 'q = 68', one aside of 4 cells,
taken when the body held 4, six doublings to 64, and the aside moved by 'Step(64)' — against 67
placements. When 'q = 1' there is no round. Every placement is
exact, because the body genuinely is invariant under 'Step'
('TestRibbonIsInvariantUnderItsScrewStep'). Every move is by a whole number of teeth already
built, so each join meets its neighbour at one shared planar cross-section and leaves no sliver
and no overlap; 'TestDoublingScheduleCoversTheRibbon' runs the schedule at every count from 4 to
512 with cells of one, three and four teeth.

**The remainder.** When 'r > 0', the last 'r' teeth are a second, shorter cell, built where they
belong rather than moved there: a sketch '{gearLabel} Cell Remainder' and a loft by the recipe of
§2 with 'c' replaced by 'r', so 'r*n + 1' sections at stations from 's0 + q*c*P' to 's0 + N*P',
joined into the body after the last round. Its first section is the body's last, so the join
meets at a shared cross-section like every other. The defaults have none; 69 teeth in
four-tooth cells would have one of one tooth.

Copy with 'copyPasteBodies.add(body)' and take the copy from the feature's 'bodies'
('[SCREW-F-COPY-BODY]'); move with 'moveFeatures.createInput2(bodies)' → 'defineAsFreeMove(matrix)'
→ 'add(input)' ('[PB-MOVE-ROTATE]', '[SCREW-F-SCREW-STEP]'); join with a 'combineFeatures' join and
check that it leaves exactly one body: the combine feature's own 'bodies.count' is 1, and that
body is the ribbon from then on ('[PB-EMPTY-RESULT]', '[SCREW-F-JOIN]'). After the last
join name the body 'Gear A' or 'Gear B'.

**A zero-angle matrix is a no-op that Fusion rejects** ('[PB-MOVE-ROTATE]'). A screw step is never
zero for 'k >= 1', so no guard is needed here, but do not "optimize" a 'k = 0' case into the loop.


[SCREW-F-COPY-BODY] [PB-EMPTY-RESULT].

The proof function is `stepEntry11GearAasidecopy`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry11GearAasidecopy, assertCopyCell) -->

The proof stages the actual duplicate two ribbon widths along local X because the solid gate rejects overlap.
It compares bounded volume readings and checks both bodies' bounds and centroids with that translation included.
Fusion's copy remains unmoved until its later screw move. This staging translation belongs only to the proof.


Make these required Fusion calls:

- `design.features.copyPasteBodies.add(sourceBody)`.
- `copyFeature.bodies.item(0)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CopyPasteBodies",
      "reason": null,
      "receiver": "design.features.copyPasteBodies",
      "role": "required",
      "span": "design.features.copyPasteBodies.add(sourceBody)"
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
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 12 `[GO]` Gear A doubling 1 copy

Copy the current 1-cell body and take the new body from the CopyPasteBody feature's bodies. Do not take sourceBody. [SCREW-F-COPY-BODY] [PB-EMPTY-RESULT].

The proof function is `stepEntry12GearAdoubling1copy`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry12GearAdoubling1copy, assertCopyCell) -->

The proof stages the actual duplicate two ribbon widths along local X because the solid gate rejects overlap.
It compares bounded volume readings and checks both bodies' bounds and centroids with that translation included.
Fusion's copy remains unmoved until its later screw move. This staging translation belongs only to the proof.


Make these required Fusion calls:

- `design.features.copyPasteBodies.add(sourceBody)`.
- `copyFeature.bodies.item(0)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CopyPasteBodies",
      "reason": null,
      "receiver": "design.features.copyPasteBodies",
      "role": "required",
      "span": "design.features.copyPasteBodies.add(sourceBody)"
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
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 13 `[GO]` Gear A doubling 1 move

Move only the copy by Step(4), rotating 4*P/Lambda around this gear's own origin and dir, then translating 4*P along that dir. Compose the rotation and translation matrices. Set translation on the separate mov matrix; never overwrite the pivot translation in rot. Put the copy alone in the bodies collection. [SCREW-F-SCREW-STEP] [PB-MOVE-ROTATE].

The proof function is `stepEntry13GearAdoubling1move`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry13GearAdoubling1move, assertMoveCell) -->

Make these required Fusion calls:

- `adsk.core.Matrix3D.create()`.
- `rot.setToRotation(k * pitch / lam, axisVector, axisPoint)`.
- `axisVector.copy()`.
- `shift.scaleBy(k * pitch)`.
- `rot.transformBy(mov)`.
- `adsk.core.ObjectCollection.create()`.
- `bodies.add(copyBody)`.
- `design.features.moveFeatures.createInput2(bodies)`.
- `moveInput.defineAsFreeMove(rot)`.
- `design.features.moveFeatures.add(moveInput)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

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
      "receiver": "rot",
      "role": "required",
      "span": "rot.setToRotation(k * pitch / lam, axisVector, axisPoint)"
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
      "receiver": "rot",
      "role": "required",
      "span": "rot.transformBy(mov)"
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
      "span": "bodies.add(copyBody)"
    },
    {
      "condition": null,
      "name": "createInput2",
      "owner": "adsk.fusion.MoveFeatures",
      "reason": null,
      "receiver": "design.features.moveFeatures",
      "role": "required",
      "span": "design.features.moveFeatures.createInput2(bodies)"
    },
    {
      "condition": null,
      "name": "defineAsFreeMove",
      "owner": "adsk.fusion.MoveFeatureInput",
      "reason": null,
      "receiver": "moveInput",
      "role": "required",
      "span": "moveInput.defineAsFreeMove(rot)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.MoveFeatures",
      "reason": null,
      "receiver": "design.features.moveFeatures",
      "role": "required",
      "span": "design.features.moveFeatures.add(moveInput)"
    }
  ],
  "citations": [
    {
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 14 `[GO]` Gear A doubling 1 join

Join the current 1-cell body as target to the moved 1-cell copy alone as tool. Set operation JoinFeatureOperation and isKeepToolBodies False. Require exactly one feature body and replace the current ribbon handle with it. [SCREW-F-JOIN] [PB-EMPTY-RESULT].

The proof function is `stepEntry14GearAdoubling1join`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry14GearAdoubling1join, assertJoinCell) -->

Make these required Fusion calls:

- `adsk.core.ObjectCollection.create()`.
- `tools.add(toolBody)`.
- `design.features.combineFeatures.createInput(targetBody, tools)`.
- `design.features.combineFeatures.add(combineInput)`.
- `combineFeature.bodies.item(0)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

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
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.createInput(targetBody, tools)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.add(combineInput)"
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
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 15 `[GO]` Gear A doubling 2 copy

Copy the current 2-cell body and take the new body from the CopyPasteBody feature's bodies. Do not take sourceBody. [SCREW-F-COPY-BODY] [PB-EMPTY-RESULT].

The proof function is `stepEntry15GearAdoubling2copy`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry15GearAdoubling2copy, assertCopyCell) -->

The proof stages the actual duplicate two ribbon widths along local X because the solid gate rejects overlap.
It compares bounded volume readings and checks both bodies' bounds and centroids with that translation included.
Fusion's copy remains unmoved until its later screw move. This staging translation belongs only to the proof.


Make these required Fusion calls:

- `design.features.copyPasteBodies.add(sourceBody)`.
- `copyFeature.bodies.item(0)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CopyPasteBodies",
      "reason": null,
      "receiver": "design.features.copyPasteBodies",
      "role": "required",
      "span": "design.features.copyPasteBodies.add(sourceBody)"
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
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 16 `[GO]` Gear A doubling 2 move

Move only the copy by Step(8), rotating 8*P/Lambda around this gear's own origin and dir, then translating 8*P along that dir. Compose the rotation and translation matrices. Set translation on the separate mov matrix; never overwrite the pivot translation in rot. Put the copy alone in the bodies collection. [SCREW-F-SCREW-STEP] [PB-MOVE-ROTATE].

The proof function is `stepEntry16GearAdoubling2move`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry16GearAdoubling2move, assertMoveCell) -->

Make these required Fusion calls:

- `adsk.core.Matrix3D.create()`.
- `rot.setToRotation(k * pitch / lam, axisVector, axisPoint)`.
- `axisVector.copy()`.
- `shift.scaleBy(k * pitch)`.
- `rot.transformBy(mov)`.
- `adsk.core.ObjectCollection.create()`.
- `bodies.add(copyBody)`.
- `design.features.moveFeatures.createInput2(bodies)`.
- `moveInput.defineAsFreeMove(rot)`.
- `design.features.moveFeatures.add(moveInput)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

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
      "receiver": "rot",
      "role": "required",
      "span": "rot.setToRotation(k * pitch / lam, axisVector, axisPoint)"
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
      "receiver": "rot",
      "role": "required",
      "span": "rot.transformBy(mov)"
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
      "span": "bodies.add(copyBody)"
    },
    {
      "condition": null,
      "name": "createInput2",
      "owner": "adsk.fusion.MoveFeatures",
      "reason": null,
      "receiver": "design.features.moveFeatures",
      "role": "required",
      "span": "design.features.moveFeatures.createInput2(bodies)"
    },
    {
      "condition": null,
      "name": "defineAsFreeMove",
      "owner": "adsk.fusion.MoveFeatureInput",
      "reason": null,
      "receiver": "moveInput",
      "role": "required",
      "span": "moveInput.defineAsFreeMove(rot)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.MoveFeatures",
      "reason": null,
      "receiver": "design.features.moveFeatures",
      "role": "required",
      "span": "design.features.moveFeatures.add(moveInput)"
    }
  ],
  "citations": [
    {
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 17 `[GO]` Gear A doubling 2 join

Join the current 2-cell body as target to the moved 2-cell copy alone as tool. Set operation JoinFeatureOperation and isKeepToolBodies False. Require exactly one feature body and replace the current ribbon handle with it. [SCREW-F-JOIN] [PB-EMPTY-RESULT].

The proof function is `stepEntry17GearAdoubling2join`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry17GearAdoubling2join, assertJoinCell) -->

Make these required Fusion calls:

- `adsk.core.ObjectCollection.create()`.
- `tools.add(toolBody)`.
- `design.features.combineFeatures.createInput(targetBody, tools)`.
- `design.features.combineFeatures.add(combineInput)`.
- `combineFeature.bodies.item(0)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

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
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.createInput(targetBody, tools)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.add(combineInput)"
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
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 18 `[GO]` Gear A doubling 3 copy

Copy the current 4-cell body and take the new body from the CopyPasteBody feature's bodies. Do not take sourceBody. [SCREW-F-COPY-BODY] [PB-EMPTY-RESULT].

The proof function is `stepEntry18GearAdoubling3copy`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry18GearAdoubling3copy, assertCopyCell) -->

The proof stages the actual duplicate two ribbon widths along local X because the solid gate rejects overlap.
It compares bounded volume readings and checks both bodies' bounds and centroids with that translation included.
Fusion's copy remains unmoved until its later screw move. This staging translation belongs only to the proof.


Make these required Fusion calls:

- `design.features.copyPasteBodies.add(sourceBody)`.
- `copyFeature.bodies.item(0)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CopyPasteBodies",
      "reason": null,
      "receiver": "design.features.copyPasteBodies",
      "role": "required",
      "span": "design.features.copyPasteBodies.add(sourceBody)"
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
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 19 `[GO]` Gear A doubling 3 move

Move only the copy by Step(16), rotating 16*P/Lambda around this gear's own origin and dir, then translating 16*P along that dir. Compose the rotation and translation matrices. Set translation on the separate mov matrix; never overwrite the pivot translation in rot. Put the copy alone in the bodies collection. [SCREW-F-SCREW-STEP] [PB-MOVE-ROTATE].

The proof function is `stepEntry19GearAdoubling3move`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry19GearAdoubling3move, assertMoveCell) -->

Make these required Fusion calls:

- `adsk.core.Matrix3D.create()`.
- `rot.setToRotation(k * pitch / lam, axisVector, axisPoint)`.
- `axisVector.copy()`.
- `shift.scaleBy(k * pitch)`.
- `rot.transformBy(mov)`.
- `adsk.core.ObjectCollection.create()`.
- `bodies.add(copyBody)`.
- `design.features.moveFeatures.createInput2(bodies)`.
- `moveInput.defineAsFreeMove(rot)`.
- `design.features.moveFeatures.add(moveInput)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

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
      "receiver": "rot",
      "role": "required",
      "span": "rot.setToRotation(k * pitch / lam, axisVector, axisPoint)"
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
      "receiver": "rot",
      "role": "required",
      "span": "rot.transformBy(mov)"
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
      "span": "bodies.add(copyBody)"
    },
    {
      "condition": null,
      "name": "createInput2",
      "owner": "adsk.fusion.MoveFeatures",
      "reason": null,
      "receiver": "design.features.moveFeatures",
      "role": "required",
      "span": "design.features.moveFeatures.createInput2(bodies)"
    },
    {
      "condition": null,
      "name": "defineAsFreeMove",
      "owner": "adsk.fusion.MoveFeatureInput",
      "reason": null,
      "receiver": "moveInput",
      "role": "required",
      "span": "moveInput.defineAsFreeMove(rot)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.MoveFeatures",
      "reason": null,
      "receiver": "design.features.moveFeatures",
      "role": "required",
      "span": "design.features.moveFeatures.add(moveInput)"
    }
  ],
  "citations": [
    {
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 20 `[GO]` Gear A doubling 3 join

Join the current 4-cell body as target to the moved 4-cell copy alone as tool. Set operation JoinFeatureOperation and isKeepToolBodies False. Require exactly one feature body and replace the current ribbon handle with it. [SCREW-F-JOIN] [PB-EMPTY-RESULT].

The proof function is `stepEntry20GearAdoubling3join`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry20GearAdoubling3join, assertJoinCell) -->

Make these required Fusion calls:

- `adsk.core.ObjectCollection.create()`.
- `tools.add(toolBody)`.
- `design.features.combineFeatures.createInput(targetBody, tools)`.
- `design.features.combineFeatures.add(combineInput)`.
- `combineFeature.bodies.item(0)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

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
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.createInput(targetBody, tools)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.add(combineInput)"
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
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 21 `[GO]` Gear A doubling 4 copy

Copy the current 8-cell body and take the new body from the CopyPasteBody feature's bodies. Do not take sourceBody. [SCREW-F-COPY-BODY] [PB-EMPTY-RESULT].

The proof function is `stepEntry21GearAdoubling4copy`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry21GearAdoubling4copy, assertCopyCell) -->

The proof stages the actual duplicate two ribbon widths along local X because the solid gate rejects overlap.
It compares bounded volume readings and checks both bodies' bounds and centroids with that translation included.
Fusion's copy remains unmoved until its later screw move. This staging translation belongs only to the proof.


Make these required Fusion calls:

- `design.features.copyPasteBodies.add(sourceBody)`.
- `copyFeature.bodies.item(0)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CopyPasteBodies",
      "reason": null,
      "receiver": "design.features.copyPasteBodies",
      "role": "required",
      "span": "design.features.copyPasteBodies.add(sourceBody)"
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
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 22 `[GO]` Gear A doubling 4 move

Move only the copy by Step(32), rotating 32*P/Lambda around this gear's own origin and dir, then translating 32*P along that dir. Compose the rotation and translation matrices. Set translation on the separate mov matrix; never overwrite the pivot translation in rot. Put the copy alone in the bodies collection. [SCREW-F-SCREW-STEP] [PB-MOVE-ROTATE].

The proof function is `stepEntry22GearAdoubling4move`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry22GearAdoubling4move, assertMoveCell) -->

Make these required Fusion calls:

- `adsk.core.Matrix3D.create()`.
- `rot.setToRotation(k * pitch / lam, axisVector, axisPoint)`.
- `axisVector.copy()`.
- `shift.scaleBy(k * pitch)`.
- `rot.transformBy(mov)`.
- `adsk.core.ObjectCollection.create()`.
- `bodies.add(copyBody)`.
- `design.features.moveFeatures.createInput2(bodies)`.
- `moveInput.defineAsFreeMove(rot)`.
- `design.features.moveFeatures.add(moveInput)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

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
      "receiver": "rot",
      "role": "required",
      "span": "rot.setToRotation(k * pitch / lam, axisVector, axisPoint)"
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
      "receiver": "rot",
      "role": "required",
      "span": "rot.transformBy(mov)"
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
      "span": "bodies.add(copyBody)"
    },
    {
      "condition": null,
      "name": "createInput2",
      "owner": "adsk.fusion.MoveFeatures",
      "reason": null,
      "receiver": "design.features.moveFeatures",
      "role": "required",
      "span": "design.features.moveFeatures.createInput2(bodies)"
    },
    {
      "condition": null,
      "name": "defineAsFreeMove",
      "owner": "adsk.fusion.MoveFeatureInput",
      "reason": null,
      "receiver": "moveInput",
      "role": "required",
      "span": "moveInput.defineAsFreeMove(rot)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.MoveFeatures",
      "reason": null,
      "receiver": "design.features.moveFeatures",
      "role": "required",
      "span": "design.features.moveFeatures.add(moveInput)"
    }
  ],
  "citations": [
    {
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 23 `[GO]` Gear A doubling 4 join

Join the current 8-cell body as target to the moved 8-cell copy alone as tool. Set operation JoinFeatureOperation and isKeepToolBodies False. Require exactly one feature body and replace the current ribbon handle with it. [SCREW-F-JOIN] [PB-EMPTY-RESULT].

The proof function is `stepEntry23GearAdoubling4join`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry23GearAdoubling4join, assertJoinCell) -->

Make these required Fusion calls:

- `adsk.core.ObjectCollection.create()`.
- `tools.add(toolBody)`.
- `design.features.combineFeatures.createInput(targetBody, tools)`.
- `design.features.combineFeatures.add(combineInput)`.
- `combineFeature.bodies.item(0)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

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
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.createInput(targetBody, tools)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.add(combineInput)"
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
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 24 `[GO]` Gear A aside move

Move the saved one-cell aside by Step(64), using the same pivot-preserving screw matrix recipe. The target holds sixteen cells. [SCREW-F-SCREW-STEP] [PB-MOVE-ROTATE].

The proof function is `stepEntry24GearAasidemove`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry24GearAasidemove, assertMoveCell) -->

Make these required Fusion calls:

- `adsk.core.Matrix3D.create()`.
- `rot.setToRotation(64 * pitch / lam, axisVector, axisPoint)`.
- `axisVector.copy()`.
- `shift.scaleBy(64 * pitch)`.
- `rot.transformBy(mov)`.
- `adsk.core.ObjectCollection.create()`.
- `bodies.add(asideBody)`.
- `design.features.moveFeatures.createInput2(bodies)`.
- `moveInput.defineAsFreeMove(rot)`.
- `design.features.moveFeatures.add(moveInput)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

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
      "receiver": "rot",
      "role": "required",
      "span": "rot.setToRotation(64 * pitch / lam, axisVector, axisPoint)"
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
      "span": "shift.scaleBy(64 * pitch)"
    },
    {
      "condition": null,
      "name": "transformBy",
      "owner": "adsk.core.Matrix3D",
      "reason": null,
      "receiver": "rot",
      "role": "required",
      "span": "rot.transformBy(mov)"
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
      "span": "bodies.add(asideBody)"
    },
    {
      "condition": null,
      "name": "createInput2",
      "owner": "adsk.fusion.MoveFeatures",
      "reason": null,
      "receiver": "design.features.moveFeatures",
      "role": "required",
      "span": "design.features.moveFeatures.createInput2(bodies)"
    },
    {
      "condition": null,
      "name": "defineAsFreeMove",
      "owner": "adsk.fusion.MoveFeatureInput",
      "reason": null,
      "receiver": "moveInput",
      "role": "required",
      "span": "moveInput.defineAsFreeMove(rot)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.MoveFeatures",
      "reason": null,
      "receiver": "design.features.moveFeatures",
      "role": "required",
      "span": "design.features.moveFeatures.add(moveInput)"
    }
  ],
  "citations": [
    {
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 25 `[GO]` Gear A aside join

Join the moved one-cell aside into the sixteen-cell target, leaving seventeen cells, 68 teeth. Set operation JoinFeatureOperation and isKeepToolBodies False. Require one body and name it Gear A. For other counts use the carried binary schedule; q=1 skips copy/move/join entirely. [SCREW-F-JOIN] [PB-EMPTY-RESULT].

The proof function is `stepEntry25GearAasidejoin`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry25GearAasidejoin, assertJoinCell) -->

Make these required Fusion calls:

- `adsk.core.ObjectCollection.create()`.
- `tools.add(toolBody)`.
- `design.features.combineFeatures.createInput(targetBody, tools)`.
- `design.features.combineFeatures.add(combineInput)`.
- `combineFeature.bodies.item(0)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

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
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.createInput(targetBody, tools)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.add(combineInput)"
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
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 26r1 `[GO]` Gear A remainder sections sketch

Only when N mod c is positive, draw the final r teeth at s0+q*c*P through s0+N*P, with r*n+1 sections, using the complete Cell Sections recipe above. This is a separate sketch named Gear A Cell Remainder. [PB-3D-SKETCH-SECTIONS] [PB-SKETCH-DEFER].

The proof function is `stepEntry26r1GearAremaindersectionssketch`.

<!-- proof-run: proofkit.Run(sectionCases, stepEntry26r1GearAremaindersectionssketch) -->

Make these required Fusion calls:

- `design.sketches.add(plane)`.
- `adsk.core.Point3D.create(x, y, z)`.
- `sketch.modelToSketchSpace(worldPoint)`.
- `sketch.sketchPoints.add(local)`.
- `sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)`.
- `adsk.core.ObjectCollection.create()`.
- `fitPoints.add(sectionPoint)`.
- `sketch.sketchCurves.sketchFittedSplines.add(fitPoints)`.
- `spline.fitPoints.item(i)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "design.sketches",
      "role": "required",
      "span": "design.sketches.add(plane)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "adsk.core.Point3D.create(x, y, z)"
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
      "span": "sketch.sketchPoints.add(local)"
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
      "span": "fitPoints.add(sectionPoint)"
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
      "first": 1200,
      "last": 1207,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1200–1207.

## 26r2 `[GO]` Gear A remainder loft

Loft the remainder's ordered four-curve section paths as one new body. Require one solid body. [PB-LOFT] [PB-EMPTY-RESULT].

The proof function is `stepEntry26r2GearAremainderloft`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry26r2GearAremainderloft, assertCellSegmentLoft) -->

Make these required Fusion calls:

- `design.features.loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`.
- `adsk.core.ObjectCollection.create()`.
- `curves.add(sectionCurve)`.
- `design.features.createPath(curves, False)`.
- `loftInput.loftSections.add(sectionPath)`.
- `design.features.loftFeatures.add(loftInput)`.
- `loftFeature.bodies.item(0)`.
- `cellBody.pointContainment(probePoint)`.

The supported proof substitution is explicit:

The proof builds one real two-section interval of this cell, using its actual sampled sections. The evaluator
refuses unions of adjacent intervals at their tangent triangulated caps. The sketch proof checks every sampled
section. This solid substitute checks the representative interval's solidity and prismoid volume, and establishes
no whole-cell smooth surface or volume.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.LoftFeatures",
      "reason": null,
      "receiver": "design.features.loftFeatures",
      "role": "required",
      "span": "design.features.loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)"
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
      "span": "curves.add(sectionCurve)"
    },
    {
      "condition": null,
      "name": "createPath",
      "owner": "adsk.fusion.Features",
      "reason": null,
      "receiver": "design.features",
      "role": "required",
      "span": "design.features.createPath(curves, False)"
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
      "receiver": "design.features.loftFeatures",
      "role": "required",
      "span": "design.features.loftFeatures.add(loftInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "loftFeature.bodies",
      "role": "required",
      "span": "loftFeature.bodies.item(0)"
    },
    {
      "condition": null,
      "name": "pointContainment",
      "owner": "adsk.fusion.BRepBody",
      "reason": null,
      "receiver": "cellBody",
      "role": "required",
      "span": "cellBody.pointContainment(probePoint)"
    }
  ],
  "citations": [
    {
      "first": 1200,
      "last": 1207,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1200–1207.

## 26r3 `[GO]` Gear A remainder join

Join the remainder, built in place, as sole tool to the completed whole-cell ribbon. The first remainder section equals the last whole-cell section. Require exactly one feature body. [SCREW-F-JOIN] [PB-EMPTY-RESULT].

The proof function is `stepEntry26r3GearAremainderjoin`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry26r3GearAremainderjoin, assertJoinCell) -->

Make these required Fusion calls:

- `adsk.core.ObjectCollection.create()`.
- `tools.add(toolBody)`.
- `design.features.combineFeatures.createInput(targetBody, tools)`.
- `design.features.combineFeatures.add(combineInput)`.
- `combineFeature.bodies.item(0)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

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
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.createInput(targetBody, tools)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.add(combineInput)"
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
      "first": 1200,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1200–1216.

## 27 `[GO]` Gear B paths sketch

**The Paths sketches.** 'Gear A Paths' on the Gear A Axis Plane and 'Gear B Paths' on gear B's,
each holding the two lines that gear's bores are swept along (§4), both on the gear's axis,
which lies in that plane. The '+R' bore's cut spans stations 'sIn' to 'sOut' of the gear's axis
and the '-R' bore's '-sOut' to '-sIn', with 'sIn = sqrt(Ri^2 - c^2) - 1 mm' and
'sOut = cageRadius + collarHalf + 1 mm' (§4): 7.806 and 19 mm at the defaults. The sketch holds
four reference points (Sketch Discipline), at 'origin_g + s*dir_g' for 's' = '-sOut', '-sIn',
'sIn' and 'sOut', all on the plane, and two solid lines — 'bore-' from '-sOut' to '-sIn' and
'bore+' from 'sIn' to 'sOut' — each drawn from its negative station to its positive one, sharing
both points, so its start is its negative end and it runs along '+dir_g'; then all four points
are set 'isFixed = True'. The lines carry no dimension and no constraint, and nothing else is in
the sketch. Both lie on one line with a gap between them; Fusion read a sketch of four such
lines, overlapping, fully constrained on 2026-09-28, and a path made from one of its lines with
chaining off held that one line ('[PB-PATH-FROM-SKETCH]'). A line whose two ends are fixed has a
'worldGeometry' the build can trust ('[PB-WORLDGEO-CONSTRAINED]'), and a line held by dimensions
from a projected point does not. The section plane of each bore is 'setByDistanceOnPath' on its
own line at fraction '0' ('[PB-CONSTRUCTION-PLANES]'; pass the line directly), so it stands at
the span's negative end, square to the axis; the point where the line pierces it is the span's
first station, 'origin_g + s*dir_g', and Fusion put that plane's origin on the station to four
decimals of a millimetre. The sweep's path is 'features.createPath(line, False)' on the same
line ('[SCREW-F-TWISTED-SLOT]').

[PB-PATH-FROM-SKETCH] [PB-WORLDGEO-CONSTRAINED].

The proof function is `stepEntry27GearBpathssketch`.

<!-- proof-run: proofkit.Run(pathCases, stepEntry27GearBpathssketch) -->

Make these required Fusion calls:

- `design.sketches.add(plane)`.
- `adsk.core.Point3D.create(x, y, z)`.
- `sketch.modelToSketchSpace(worldPoint)`.
- `sketch.sketchPoints.add(local)`.
- `sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "design.sketches",
      "role": "required",
      "span": "design.sketches.add(plane)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "adsk.core.Point3D.create(x, y, z)"
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
      "span": "sketch.sketchPoints.add(local)"
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
      "first": 979,
      "last": 998,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L979–998.

## 28 `[GO]` Gear B cell sections sketch

### 2: The tooth cell

One cell is 'c' tooth pitches of the finished ribbon, at the ribbon's negative end: stations 's'
from 's0 = Z0 - L/2' to 's0 + c*P', where 'Z0' is the gear's tooth phase — '0' for gear A and the
'assemblyPhase' input for gear B ("Defaults") — and 'c = min(cellTeeth, N)' is the number of
teeth the cell holds, 4 at the defaults ('cellTeeth', "Variables"). Gear B's whole ribbon is
gear A's advanced by 'Z0' under its own screw motion, ends and teeth alike, so the same cell
built 'Z0' further along its axis and repeated the same way is gear B. Build it once per gear.

**The section count is derived, not pinned, and it is not user-configurable.** The cell is lofted
through 'c*n + 1' sections, 'n' steps to the tooth, where 'n' is the larger of two counts:

- the smallest that keeps the twist between neighbouring sections under 2°,
  'ceil((P/Lambda) / 2°)', 10 at the defaults. A surface ruled straight between two sections
  cuts the corner of the true helicoid at the crest by '(W/2)*(1 - cos(dtheta/2))', 'dtheta'
  the twist per step: 1.0 µm at the defaults' 1.91°, under a hundredth of the 1.08 mm backlash,
  which is the bound the proof holds it to (the shortfall grows with the width and the backlash
  with the pitch, so it is not held to a fixed number of microns);
- eight. A straight chord of the cosine between two sections falls '(H/2)*(1 - cos(pi/n))' short
  of it at the deepest point whatever the twist: 0.064 mm at the defaults' ten steps, 6% of the
  backlash, and 0.100 mm, 3.8% of the tooth height, at eight. The proof holds it under a fifth of
  the backlash; it was 7.8% of the 0.82 mm backlash before 2026-10-02 and 14% of the 0.46 mm one
  until 2026-10-03, when the leaned tooth widened the backlash. The chord runs along 's' at every
  'v' alike, so the lean does not change it. That floor is what keeps a slow
  twist from lofting the tooth through two or three sections and losing it: at a 400 mm lead the
  twist alone asks for two sections, and the cell is built through nine.

That is 10 steps to the tooth at the defaults: 41 sections in a four-tooth cell, 11 in a
one-tooth cell. 'TestLoftSectionCountHoldsTheHelicoid' holds both bounds over leads from 20 mm
to 400 mm.

**What the loft is, and what the count guarantees for it.** Both bounds are arithmetic about a
*ruled* loft, whose surface runs straight between neighbouring sections. Fusion builds a ruled
loft only through exactly two sections; a loft through more passes through every section and is
fitted smoothly between them, and 'LoftFeatureInput' has no option to make it ruled — its
sections carry only end conditions ('[SCREW-F-CELL-LOFT]'). The cell is **one loft through all
'c*n + 1' sections**, so it is the smooth kind. What the count guarantees for it is that the
built cell carries the exact rotated section at each of its stations, 'dtheta' apart, its
toothed side the spline through the edge's points, and that between stations its surface
interpolates those sections; the chord figures above are the
departure of the ruled loft through the same sections, and they are the only figures the proof
has. What the smooth surface does between sections was measured in Fusion on 2026-09-28, at
the defaults of that day, the 10 mm ribbon ('[SCREW-F-CELL-LOFT]'): at one, four and eight
teeth, probes 0.04 mm inside and 0.04 mm outside the toothed edge and a face, at the midpoint
between every pair of sections, all fell on the right side of the built surface — 40 of 40,
160 of 160 and 320 of 320 — and the built volume was 0.06% to 0.23% under the ruled loft's. So
at that size the built surface stayed within 0.04 mm of the helicoid between sections, under a
tenth of the 0.44 mm backlash of that day, with straight-ridge rectangles for sections; the 1.5×
ribbon has the same section spacing, and neither it nor the spline sections have been
measured. The proof cannot measure that ("What the proof cannot reach"), and a Fusion
load is what checks it at other inputs. A chain of 'c*n' two-section lofts would be ruled and would carry the bounds
literally, at the cost of that many lofts and joins per cell, and this spec keeps the one loft.

**The Cell Sections sketch.** All the cell's sections go in **one sketch**, named
'{gearLabel} Cell Sections', on the gear's Axis Plane ('[SCREW-F-CELL-LOFT]',
'[PB-3D-SKETCH-SECTIONS]'), drawn with sketch computing deferred (Sketch Discipline,
'[PB-SKETCH-DEFER]'). No section plane is made. Section 'k', for 'k' from '0' to 'c*n', is the
cross-section at station 's_k = s0 + k*P/n', turned about the axis by 'theta_k'. Its back side
runs along 'u = -W/2', its faces along 'v = ±T/2', and its toothed side follows the edge of "The
part" across the thickness, through 'M' points:

'''
theta_k        = s_k/Lambda + Phi
v_j            = -T/2 + j*T/(M - 1),    j = 0 … M - 1,    M = TOOTH_SPLINE_POINTS = 11
Utooth(v, s_k) = W/2 - H/2 + (H/2)*cos(2*pi*(s_k + tan(Slant)*v - Z0)/P) - Bow*v^2
'''

'M' is a module constant, beside 'CELL_TEETH' ("Variables"): eleven points, 'T/10' apart, 0.375 mm
at the defaults, both faces included. 'TestToothSplineHoldsTheEdge' fits a natural cubic spline
through them at every station of a pitch and finds it within 0.018 mm of the edge along 'u';
through nine points it departs by 0.030 mm, through seven by 0.058 mm, because across the
thickness the edge runs through 0.69 of a pitch of phase.

Each section has 'M + 2' points: the two back corners 'B0' at '(uB, -hv)' and 'B1' at '(uB, hv)',
and the toothed points 'F_j' at '(Utooth(v_j, s_k), v_j)', of which 'F_0' and 'F_(M-1)' are the
toothed corners; 'uB = -W/2' and 'hv = T/2'. Each is the world point of §1 at those section
coordinates and station 's_k', mapped in with 'sketch.modelToSketchSpace(point)' and added with
'sketch.sketchPoints.add(point)' **with its 'z' kept**: the points lie off the sketch's plane on
purpose (Sketch Discipline, '[PB-SKETCH-ZERO-Z]' not applied). Four curves share them, in this
order ('[PB-SHARE-XOR-COINCIDENT]', '[PB-SKETCHCURVES]'):

- 'L1 = sketch.sketchCurves.sketchLines.addByTwoPoints(B0, F_0)', the face at 'v = -hv'.
- 'S = sketch.sketchCurves.sketchFittedSplines.add(fitPoints)', the toothed side, where
  'fitPoints = adsk.core.ObjectCollection.create()' holds the sketch points 'F_0' to 'F_(M-1)',
  each put in with 'fitPoints.add(F_j)' in order of 'j'. The spline runs from 'F_0', the end of
  'L1', to 'F_(M-1)', the start of 'L3'. Raise naming the section unless 'S' is not 'None' and
  'S.fitPoints.count' is 'M'.
- 'L3 = sketch.sketchCurves.sketchLines.addByTwoPoints(F_(M-1), B1)', the face at 'v = +hv'.
- 'L4 = sketch.sketchCurves.sketchLines.addByTwoPoints(B1, B0)', the back side.

The build keeps each section's four curves together, in that order, because they are what the
loft is fed. After the last curve of the last section is drawn, every point in the sketch is set
'isFixed = True': every point the build added, and every item of every spline's 'fitPoints', by
'S.fitPoints.item(i)' for 'i' below 'S.fitPoints.count'. The API reference says the spline takes
existing sketch points as fit points and does not say whether it keeps those objects or makes
its own, so both are fixed. Nothing else is in the sketch: no construction line, no constraint,
no dimension, and no tangent or curvature handle ('activateTangentHandle' and
'activateCurvatureHandle' are not called). Then set 'isComputeDeferred' back to 'False', raise
unless the sketch reports 'isFullyConstrained', and raise unless 'sketch.profiles.count' is
'c*n + 1', one profile per section. The profiles are not what the loft is fed, because nothing
says which profile is which station.

The calls and their signatures, as the Fusion API reference database ('fusion:query-api') gives
them:

| Call | Signature |
|---|---|
| 'SketchPoints.add' | '(point: core.Point3D) -> SketchPoint' |
| 'Sketch.modelToSketchSpace' | '(modelCoordinate: core.Point3D) -> core.Point3D' |
| 'SketchLines.addByTwoPoints' | '(startPoint: core.Base, endPoint: core.Base) -> SketchLine', either a 'SketchPoint' or a 'Point3D' |
| 'SketchFittedSplines.add' | '(fitPoints: core.ObjectCollection) -> SketchFittedSpline'; "any combination of existing SketchPoint or Point3D objects"; 'None' if it failed |
| 'ObjectCollection.create' | '() -> ObjectCollection', static |
| 'ObjectCollection.add' | '(item: Base) -> bool' |
| 'SketchFittedSpline.fitPoints' | 'SketchPointList', read-only, start point first and end point last; 'count: int', 'item(index: int) -> SketchPoint' |
| 'SketchPoint.isFixed' | 'bool', read/write, declared on 'SketchEntity' |
| 'Sketch.isComputeDeferred' | 'bool', read/write |
| 'Features.createPath' | '(curve: core.Base, isChain: bool = True) -> Path' |
| 'LoftFeatures.createInput' | '(operation: FeatureOperations) -> LoftFeatureInput' |
| 'LoftSections.add' | '(entity: core.Base) -> LoftSection'; a 'Path' is one of the entities it takes |
| 'LoftFeatures.add' | '(input: LoftFeatureInput) -> LoftFeature' |
| 'BRepBody.pointContainment' | '(point: core.Point3D) -> PointContainment' |

**What the loft sees.** Every section is one closed loop of the same four curves in the same
order — a line, a fitted spline, a line, a line — so every section has the same number of curves
and the same number of corners, and the loft meets the spline of one section with the spline of
the next. A section with a different count would leave the loft to pair unlike curves; the build
raises before the loft unless every section holds exactly the four curves above.

**Loft the sections in station order** ('[PB-LOFT]', '[SCREW-F-CELL-LOFT]'). Create the loft
with 'loftFeatures.createInput(NewBodyFeatureOperation)'; for each section in order of 'k', put
its four curves in an 'ObjectCollection' in the order 'L1', 'S', 'L3', 'L4', make its path with

Keep every off-plane z; never flatten this sketch. Set isComputeDeferred True before points, fix original and spline fit points after all curves, set it False before profiles or constraints are read. No tangent or curvature handle is activated. [PB-3D-SKETCH-SECTIONS] [PB-SKETCH-DEFER].

The proof function is `stepEntry28GearBcellsectionssketch`.

<!-- proof-run: proofkit.Run(sectionCases, stepEntry28GearBcellsectionssketch) -->

Make these required Fusion calls:

- `design.sketches.add(plane)`.
- `adsk.core.Point3D.create(x, y, z)`.
- `sketch.modelToSketchSpace(worldPoint)`.
- `sketch.sketchPoints.add(local)`.
- `sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)`.
- `adsk.core.ObjectCollection.create()`.
- `fitPoints.add(sectionPoint)`.
- `sketch.sketchCurves.sketchFittedSplines.add(fitPoints)`.
- `spline.fitPoints.item(i)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "design.sketches",
      "role": "required",
      "span": "design.sketches.add(plane)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "adsk.core.Point3D.create(x, y, z)"
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
      "span": "sketch.sketchPoints.add(local)"
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
      "span": "fitPoints.add(sectionPoint)"
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
      "first": 1000,
      "last": 1130,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1000–1130.

## 29 `[GO]` Gear B cell loft

'features.createPath(collection, False)' ('[PB-PATH-FROM-SKETCH]'; never 'Path.create', which
raises in this component), and 'loftSections.add(path)'; then 'loftFeatures.add(input)'. The
result is 'c' pitches of the twisted toothed ribbon in one body; raise unless the feature leaves
exactly one body and that body 'isSolid' ('[PB-EMPTY-RESULT]'). Measured on 2026-09-28 at the
10 mm ribbon, with four-line sections: the four-tooth loft took 0.23 s and held 0.164158 cm³
against the ruled loft's 0.164470. The loft through spline sections has not been timed.

**The screw step still holds.** 'Utooth(v, s + P) = Utooth(v, s)' at every 'v': the slant moves
the cosine's phase by an amount that depends on 'v' alone and the bow lowers the edge by an
amount that depends on 'v' alone, so the section 'n' steps on, one pitch along the axis, is
section 'k' carried by 'Step(1)' of §3, its toothed points included. The cell is still one
pitch-periodic piece of the ribbon, and the doubling of §3 and the remainder cell are unchanged;
'TestRibbonIsInvariantUnderItsScrewStep' carries every section's corners and toothed points
through one screw step and finds them on the next pitch's to 1e-9 mm.

**Checking the slant's sign** ('[PB-SELF-DIAGNOSING]'). Nothing in the sketch shows which way a
ridge leans, and a slant of the wrong sign builds a cell whose ridges cross the other ribbon's at
61° where the axes cross instead of lying along them ("The part"). So after the loft, before §3,
the build probes the cell with 'cellBody.pointContainment(point)' at four points. Let
'sc = Z0 + P*ceil((s0 + P/2 - Z0)/P)', the first crest station at least half a pitch into the
cell. For each face sign 'σ' of '-1' and '+1', let 'v_p = σ*(T/2 - 0.25 mm)' and
'u_p = W/2 - Bow*v_p^2 - 0.25 mm', a quarter millimetre inside the face and under the crest. The
**on-ridge** probe is the world point of §1 at '(u_p, v_p)' and station 'sc - tan(Slant)*v_p',
where the ridge through 'sc' crosses 'v_p'; the **off-ridge** probe is the same at station
'sc + tan(Slant)*v_p', where that ridge would cross under the opposite sign. For each probe the
build evaluates 'm = Utooth(v_p, s) - u_p' at the probe's station under the input slant and 'm''
under its negation. A probe is used when '|m|' and '|m'|' are both at least 0.1 mm and their signs
differ. A used probe must read 'PointInsidePointContainment' when 'm > 0' and
'PointOutsidePointContainment' when 'm < 0'; otherwise the build raises naming the gear, the
probe and what it read. When no probe is used, as with a slant near zero, the build logs with
'futil.log' ('[PB-LOGGING]') that the sign was not checked. At the defaults all four are used: on
the ridge each stands 0.25 mm inside the tooth under the right sign and 2.13 mm outside under the
wrong one, and off the ridge the reverse; gear A's stand 1.84 and 3.41 mm from the cell's start,
inside its 10.5 mm ('TestToothSlantProbesTellTheHand'). Lengths are in cm in the build, as
everywhere.


Loft all sections through one feature, in ascending station order, from four-curve paths L1, S, L3, L4. Do not feed unordered profiles. Require one solid body. [PB-LOFT] [PB-PATH-FROM-SKETCH] [PB-EMPTY-RESULT].

The proof function is `stepEntry29GearBcellloft`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry29GearBcellloft, assertCellSegmentLoft) -->

Make these required Fusion calls:

- `design.features.loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`.
- `adsk.core.ObjectCollection.create()`.
- `curves.add(sectionCurve)`.
- `design.features.createPath(curves, False)`.
- `loftInput.loftSections.add(sectionPath)`.
- `design.features.loftFeatures.add(loftInput)`.
- `loftFeature.bodies.item(0)`.
- `cellBody.pointContainment(probePoint)`.

The supported proof substitution is explicit:

The proof builds one real two-section interval of this cell, using its actual sampled sections. The evaluator
refuses unions of adjacent intervals at their tangent triangulated caps. The sketch proof checks every sampled
section. This solid substitute checks the representative interval's solidity and prismoid volume, and establishes
no whole-cell smooth surface or volume.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.LoftFeatures",
      "reason": null,
      "receiver": "design.features.loftFeatures",
      "role": "required",
      "span": "design.features.loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)"
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
      "span": "curves.add(sectionCurve)"
    },
    {
      "condition": null,
      "name": "createPath",
      "owner": "adsk.fusion.Features",
      "reason": null,
      "receiver": "design.features",
      "role": "required",
      "span": "design.features.createPath(curves, False)"
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
      "receiver": "design.features.loftFeatures",
      "role": "required",
      "span": "design.features.loftFeatures.add(loftInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "loftFeature.bodies",
      "role": "required",
      "span": "loftFeature.bodies.item(0)"
    },
    {
      "condition": null,
      "name": "pointContainment",
      "owner": "adsk.fusion.BRepBody",
      "reason": null,
      "receiver": "cellBody",
      "role": "required",
      "span": "cellBody.pointContainment(probePoint)"
    }
  ],
  "citations": [
    {
      "first": 1131,
      "last": 1166,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1131–1166.

## 30 `[GO]` Gear B aside copy

At q=17, copy the one-cell body before doubling. Keep that copy unmoved as the one-cell aside. ### 3: Repeat the cell by doubling

The finished ribbon is the cell, 'c' teeth, repeated under the **screw step**

'''
Step(k) = translate(k*P along the axis) ∘ rotate(k*P/Lambda about the axis)
'''

Write 'N = q*c + r', with 'q = floor(N/c)' whole cells and a remainder of 'r = N mod c' teeth:
'q = 17' and 'r = 0' at the defaults, 'q = 68' at a one-tooth cell. Build the 'q' cells by
doubling rather than by 'q' separate placements. A **round** is copy, move, join: copy the
current body, move the copy by 'Step(m*c)' where 'm' is the number of cells the body holds, join
the two, and the body holds '2m' cells. Doubling alone reaches only a power of two, and copying
the whole body for the rest would overlap it, so the rest is made of unmoved copies put aside on
the way up:

- Write 'q' in binary. Start with the cell, 'm = 1'.
- For each bit of 'q' below its top bit, lowest first: if that bit is set, take a copy of the
  body and keep it unmoved — an **aside** of 'm' cells; then double.
- After the last doubling 'm' is the top bit's power of two. Move each aside into place, largest
  first, by 'Step(m*c)', join it, and add its cells to 'm'. The last join brings 'm' to 'q'.

That is 'floor(log2 q) + popcount(q) - 1' rounds and as many joins: 5 at the defaults — 'q = 17',
one aside of the single cell, taken before the first doubling, then four doublings to 16 cells,
and the aside moved by 'Step(64)' — and 7 at a one-tooth cell — 'q = 68', one aside of 4 cells,
taken when the body held 4, six doublings to 64, and the aside moved by 'Step(64)' — against 67
placements. When 'q = 1' there is no round. Every placement is
exact, because the body genuinely is invariant under 'Step'
('TestRibbonIsInvariantUnderItsScrewStep'). Every move is by a whole number of teeth already
built, so each join meets its neighbour at one shared planar cross-section and leaves no sliver
and no overlap; 'TestDoublingScheduleCoversTheRibbon' runs the schedule at every count from 4 to
512 with cells of one, three and four teeth.

**The remainder.** When 'r > 0', the last 'r' teeth are a second, shorter cell, built where they
belong rather than moved there: a sketch '{gearLabel} Cell Remainder' and a loft by the recipe of
§2 with 'c' replaced by 'r', so 'r*n + 1' sections at stations from 's0 + q*c*P' to 's0 + N*P',
joined into the body after the last round. Its first section is the body's last, so the join
meets at a shared cross-section like every other. The defaults have none; 69 teeth in
four-tooth cells would have one of one tooth.

Copy with 'copyPasteBodies.add(body)' and take the copy from the feature's 'bodies'
('[SCREW-F-COPY-BODY]'); move with 'moveFeatures.createInput2(bodies)' → 'defineAsFreeMove(matrix)'
→ 'add(input)' ('[PB-MOVE-ROTATE]', '[SCREW-F-SCREW-STEP]'); join with a 'combineFeatures' join and
check that it leaves exactly one body: the combine feature's own 'bodies.count' is 1, and that
body is the ribbon from then on ('[PB-EMPTY-RESULT]', '[SCREW-F-JOIN]'). After the last
join name the body 'Gear A' or 'Gear B'.

**A zero-angle matrix is a no-op that Fusion rejects** ('[PB-MOVE-ROTATE]'). A screw step is never
zero for 'k >= 1', so no guard is needed here, but do not "optimize" a 'k = 0' case into the loop.


[SCREW-F-COPY-BODY] [PB-EMPTY-RESULT].

The proof function is `stepEntry30GearBasidecopy`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry30GearBasidecopy, assertCopyCell) -->

The proof stages the actual duplicate two ribbon widths along local X because the solid gate rejects overlap.
It compares bounded volume readings and checks both bodies' bounds and centroids with that translation included.
Fusion's copy remains unmoved until its later screw move. This staging translation belongs only to the proof.


Make these required Fusion calls:

- `design.features.copyPasteBodies.add(sourceBody)`.
- `copyFeature.bodies.item(0)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CopyPasteBodies",
      "reason": null,
      "receiver": "design.features.copyPasteBodies",
      "role": "required",
      "span": "design.features.copyPasteBodies.add(sourceBody)"
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
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 31 `[GO]` Gear B doubling 1 copy

Copy the current 1-cell body and take the new body from the CopyPasteBody feature's bodies. Do not take sourceBody. [SCREW-F-COPY-BODY] [PB-EMPTY-RESULT].

The proof function is `stepEntry31GearBdoubling1copy`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry31GearBdoubling1copy, assertCopyCell) -->

The proof stages the actual duplicate two ribbon widths along local X because the solid gate rejects overlap.
It compares bounded volume readings and checks both bodies' bounds and centroids with that translation included.
Fusion's copy remains unmoved until its later screw move. This staging translation belongs only to the proof.


Make these required Fusion calls:

- `design.features.copyPasteBodies.add(sourceBody)`.
- `copyFeature.bodies.item(0)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CopyPasteBodies",
      "reason": null,
      "receiver": "design.features.copyPasteBodies",
      "role": "required",
      "span": "design.features.copyPasteBodies.add(sourceBody)"
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
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 32 `[GO]` Gear B doubling 1 move

Move only the copy by Step(4), rotating 4*P/Lambda around this gear's own origin and dir, then translating 4*P along that dir. Compose the rotation and translation matrices. Set translation on the separate mov matrix; never overwrite the pivot translation in rot. Put the copy alone in the bodies collection. [SCREW-F-SCREW-STEP] [PB-MOVE-ROTATE].

The proof function is `stepEntry32GearBdoubling1move`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry32GearBdoubling1move, assertMoveCell) -->

Make these required Fusion calls:

- `adsk.core.Matrix3D.create()`.
- `rot.setToRotation(k * pitch / lam, axisVector, axisPoint)`.
- `axisVector.copy()`.
- `shift.scaleBy(k * pitch)`.
- `rot.transformBy(mov)`.
- `adsk.core.ObjectCollection.create()`.
- `bodies.add(copyBody)`.
- `design.features.moveFeatures.createInput2(bodies)`.
- `moveInput.defineAsFreeMove(rot)`.
- `design.features.moveFeatures.add(moveInput)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

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
      "receiver": "rot",
      "role": "required",
      "span": "rot.setToRotation(k * pitch / lam, axisVector, axisPoint)"
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
      "receiver": "rot",
      "role": "required",
      "span": "rot.transformBy(mov)"
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
      "span": "bodies.add(copyBody)"
    },
    {
      "condition": null,
      "name": "createInput2",
      "owner": "adsk.fusion.MoveFeatures",
      "reason": null,
      "receiver": "design.features.moveFeatures",
      "role": "required",
      "span": "design.features.moveFeatures.createInput2(bodies)"
    },
    {
      "condition": null,
      "name": "defineAsFreeMove",
      "owner": "adsk.fusion.MoveFeatureInput",
      "reason": null,
      "receiver": "moveInput",
      "role": "required",
      "span": "moveInput.defineAsFreeMove(rot)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.MoveFeatures",
      "reason": null,
      "receiver": "design.features.moveFeatures",
      "role": "required",
      "span": "design.features.moveFeatures.add(moveInput)"
    }
  ],
  "citations": [
    {
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 33 `[GO]` Gear B doubling 1 join

Join the current 1-cell body as target to the moved 1-cell copy alone as tool. Set operation JoinFeatureOperation and isKeepToolBodies False. Require exactly one feature body and replace the current ribbon handle with it. [SCREW-F-JOIN] [PB-EMPTY-RESULT].

The proof function is `stepEntry33GearBdoubling1join`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry33GearBdoubling1join, assertJoinCell) -->

Make these required Fusion calls:

- `adsk.core.ObjectCollection.create()`.
- `tools.add(toolBody)`.
- `design.features.combineFeatures.createInput(targetBody, tools)`.
- `design.features.combineFeatures.add(combineInput)`.
- `combineFeature.bodies.item(0)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

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
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.createInput(targetBody, tools)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.add(combineInput)"
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
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 34 `[GO]` Gear B doubling 2 copy

Copy the current 2-cell body and take the new body from the CopyPasteBody feature's bodies. Do not take sourceBody. [SCREW-F-COPY-BODY] [PB-EMPTY-RESULT].

The proof function is `stepEntry34GearBdoubling2copy`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry34GearBdoubling2copy, assertCopyCell) -->

The proof stages the actual duplicate two ribbon widths along local X because the solid gate rejects overlap.
It compares bounded volume readings and checks both bodies' bounds and centroids with that translation included.
Fusion's copy remains unmoved until its later screw move. This staging translation belongs only to the proof.


Make these required Fusion calls:

- `design.features.copyPasteBodies.add(sourceBody)`.
- `copyFeature.bodies.item(0)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CopyPasteBodies",
      "reason": null,
      "receiver": "design.features.copyPasteBodies",
      "role": "required",
      "span": "design.features.copyPasteBodies.add(sourceBody)"
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
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 35 `[GO]` Gear B doubling 2 move

Move only the copy by Step(8), rotating 8*P/Lambda around this gear's own origin and dir, then translating 8*P along that dir. Compose the rotation and translation matrices. Set translation on the separate mov matrix; never overwrite the pivot translation in rot. Put the copy alone in the bodies collection. [SCREW-F-SCREW-STEP] [PB-MOVE-ROTATE].

The proof function is `stepEntry35GearBdoubling2move`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry35GearBdoubling2move, assertMoveCell) -->

Make these required Fusion calls:

- `adsk.core.Matrix3D.create()`.
- `rot.setToRotation(k * pitch / lam, axisVector, axisPoint)`.
- `axisVector.copy()`.
- `shift.scaleBy(k * pitch)`.
- `rot.transformBy(mov)`.
- `adsk.core.ObjectCollection.create()`.
- `bodies.add(copyBody)`.
- `design.features.moveFeatures.createInput2(bodies)`.
- `moveInput.defineAsFreeMove(rot)`.
- `design.features.moveFeatures.add(moveInput)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

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
      "receiver": "rot",
      "role": "required",
      "span": "rot.setToRotation(k * pitch / lam, axisVector, axisPoint)"
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
      "receiver": "rot",
      "role": "required",
      "span": "rot.transformBy(mov)"
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
      "span": "bodies.add(copyBody)"
    },
    {
      "condition": null,
      "name": "createInput2",
      "owner": "adsk.fusion.MoveFeatures",
      "reason": null,
      "receiver": "design.features.moveFeatures",
      "role": "required",
      "span": "design.features.moveFeatures.createInput2(bodies)"
    },
    {
      "condition": null,
      "name": "defineAsFreeMove",
      "owner": "adsk.fusion.MoveFeatureInput",
      "reason": null,
      "receiver": "moveInput",
      "role": "required",
      "span": "moveInput.defineAsFreeMove(rot)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.MoveFeatures",
      "reason": null,
      "receiver": "design.features.moveFeatures",
      "role": "required",
      "span": "design.features.moveFeatures.add(moveInput)"
    }
  ],
  "citations": [
    {
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 36 `[GO]` Gear B doubling 2 join

Join the current 2-cell body as target to the moved 2-cell copy alone as tool. Set operation JoinFeatureOperation and isKeepToolBodies False. Require exactly one feature body and replace the current ribbon handle with it. [SCREW-F-JOIN] [PB-EMPTY-RESULT].

The proof function is `stepEntry36GearBdoubling2join`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry36GearBdoubling2join, assertJoinCell) -->

Make these required Fusion calls:

- `adsk.core.ObjectCollection.create()`.
- `tools.add(toolBody)`.
- `design.features.combineFeatures.createInput(targetBody, tools)`.
- `design.features.combineFeatures.add(combineInput)`.
- `combineFeature.bodies.item(0)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

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
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.createInput(targetBody, tools)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.add(combineInput)"
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
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 37 `[GO]` Gear B doubling 3 copy

Copy the current 4-cell body and take the new body from the CopyPasteBody feature's bodies. Do not take sourceBody. [SCREW-F-COPY-BODY] [PB-EMPTY-RESULT].

The proof function is `stepEntry37GearBdoubling3copy`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry37GearBdoubling3copy, assertCopyCell) -->

The proof stages the actual duplicate two ribbon widths along local X because the solid gate rejects overlap.
It compares bounded volume readings and checks both bodies' bounds and centroids with that translation included.
Fusion's copy remains unmoved until its later screw move. This staging translation belongs only to the proof.


Make these required Fusion calls:

- `design.features.copyPasteBodies.add(sourceBody)`.
- `copyFeature.bodies.item(0)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CopyPasteBodies",
      "reason": null,
      "receiver": "design.features.copyPasteBodies",
      "role": "required",
      "span": "design.features.copyPasteBodies.add(sourceBody)"
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
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 38 `[GO]` Gear B doubling 3 move

Move only the copy by Step(16), rotating 16*P/Lambda around this gear's own origin and dir, then translating 16*P along that dir. Compose the rotation and translation matrices. Set translation on the separate mov matrix; never overwrite the pivot translation in rot. Put the copy alone in the bodies collection. [SCREW-F-SCREW-STEP] [PB-MOVE-ROTATE].

The proof function is `stepEntry38GearBdoubling3move`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry38GearBdoubling3move, assertMoveCell) -->

Make these required Fusion calls:

- `adsk.core.Matrix3D.create()`.
- `rot.setToRotation(k * pitch / lam, axisVector, axisPoint)`.
- `axisVector.copy()`.
- `shift.scaleBy(k * pitch)`.
- `rot.transformBy(mov)`.
- `adsk.core.ObjectCollection.create()`.
- `bodies.add(copyBody)`.
- `design.features.moveFeatures.createInput2(bodies)`.
- `moveInput.defineAsFreeMove(rot)`.
- `design.features.moveFeatures.add(moveInput)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

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
      "receiver": "rot",
      "role": "required",
      "span": "rot.setToRotation(k * pitch / lam, axisVector, axisPoint)"
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
      "receiver": "rot",
      "role": "required",
      "span": "rot.transformBy(mov)"
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
      "span": "bodies.add(copyBody)"
    },
    {
      "condition": null,
      "name": "createInput2",
      "owner": "adsk.fusion.MoveFeatures",
      "reason": null,
      "receiver": "design.features.moveFeatures",
      "role": "required",
      "span": "design.features.moveFeatures.createInput2(bodies)"
    },
    {
      "condition": null,
      "name": "defineAsFreeMove",
      "owner": "adsk.fusion.MoveFeatureInput",
      "reason": null,
      "receiver": "moveInput",
      "role": "required",
      "span": "moveInput.defineAsFreeMove(rot)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.MoveFeatures",
      "reason": null,
      "receiver": "design.features.moveFeatures",
      "role": "required",
      "span": "design.features.moveFeatures.add(moveInput)"
    }
  ],
  "citations": [
    {
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 39 `[GO]` Gear B doubling 3 join

Join the current 4-cell body as target to the moved 4-cell copy alone as tool. Set operation JoinFeatureOperation and isKeepToolBodies False. Require exactly one feature body and replace the current ribbon handle with it. [SCREW-F-JOIN] [PB-EMPTY-RESULT].

The proof function is `stepEntry39GearBdoubling3join`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry39GearBdoubling3join, assertJoinCell) -->

Make these required Fusion calls:

- `adsk.core.ObjectCollection.create()`.
- `tools.add(toolBody)`.
- `design.features.combineFeatures.createInput(targetBody, tools)`.
- `design.features.combineFeatures.add(combineInput)`.
- `combineFeature.bodies.item(0)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

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
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.createInput(targetBody, tools)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.add(combineInput)"
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
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 40 `[GO]` Gear B doubling 4 copy

Copy the current 8-cell body and take the new body from the CopyPasteBody feature's bodies. Do not take sourceBody. [SCREW-F-COPY-BODY] [PB-EMPTY-RESULT].

The proof function is `stepEntry40GearBdoubling4copy`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry40GearBdoubling4copy, assertCopyCell) -->

The proof stages the actual duplicate two ribbon widths along local X because the solid gate rejects overlap.
It compares bounded volume readings and checks both bodies' bounds and centroids with that translation included.
Fusion's copy remains unmoved until its later screw move. This staging translation belongs only to the proof.


Make these required Fusion calls:

- `design.features.copyPasteBodies.add(sourceBody)`.
- `copyFeature.bodies.item(0)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CopyPasteBodies",
      "reason": null,
      "receiver": "design.features.copyPasteBodies",
      "role": "required",
      "span": "design.features.copyPasteBodies.add(sourceBody)"
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
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 41 `[GO]` Gear B doubling 4 move

Move only the copy by Step(32), rotating 32*P/Lambda around this gear's own origin and dir, then translating 32*P along that dir. Compose the rotation and translation matrices. Set translation on the separate mov matrix; never overwrite the pivot translation in rot. Put the copy alone in the bodies collection. [SCREW-F-SCREW-STEP] [PB-MOVE-ROTATE].

The proof function is `stepEntry41GearBdoubling4move`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry41GearBdoubling4move, assertMoveCell) -->

Make these required Fusion calls:

- `adsk.core.Matrix3D.create()`.
- `rot.setToRotation(k * pitch / lam, axisVector, axisPoint)`.
- `axisVector.copy()`.
- `shift.scaleBy(k * pitch)`.
- `rot.transformBy(mov)`.
- `adsk.core.ObjectCollection.create()`.
- `bodies.add(copyBody)`.
- `design.features.moveFeatures.createInput2(bodies)`.
- `moveInput.defineAsFreeMove(rot)`.
- `design.features.moveFeatures.add(moveInput)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

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
      "receiver": "rot",
      "role": "required",
      "span": "rot.setToRotation(k * pitch / lam, axisVector, axisPoint)"
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
      "receiver": "rot",
      "role": "required",
      "span": "rot.transformBy(mov)"
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
      "span": "bodies.add(copyBody)"
    },
    {
      "condition": null,
      "name": "createInput2",
      "owner": "adsk.fusion.MoveFeatures",
      "reason": null,
      "receiver": "design.features.moveFeatures",
      "role": "required",
      "span": "design.features.moveFeatures.createInput2(bodies)"
    },
    {
      "condition": null,
      "name": "defineAsFreeMove",
      "owner": "adsk.fusion.MoveFeatureInput",
      "reason": null,
      "receiver": "moveInput",
      "role": "required",
      "span": "moveInput.defineAsFreeMove(rot)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.MoveFeatures",
      "reason": null,
      "receiver": "design.features.moveFeatures",
      "role": "required",
      "span": "design.features.moveFeatures.add(moveInput)"
    }
  ],
  "citations": [
    {
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 42 `[GO]` Gear B doubling 4 join

Join the current 8-cell body as target to the moved 8-cell copy alone as tool. Set operation JoinFeatureOperation and isKeepToolBodies False. Require exactly one feature body and replace the current ribbon handle with it. [SCREW-F-JOIN] [PB-EMPTY-RESULT].

The proof function is `stepEntry42GearBdoubling4join`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry42GearBdoubling4join, assertJoinCell) -->

Make these required Fusion calls:

- `adsk.core.ObjectCollection.create()`.
- `tools.add(toolBody)`.
- `design.features.combineFeatures.createInput(targetBody, tools)`.
- `design.features.combineFeatures.add(combineInput)`.
- `combineFeature.bodies.item(0)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

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
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.createInput(targetBody, tools)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.add(combineInput)"
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
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 43 `[GO]` Gear B aside move

Move the saved one-cell aside by Step(64), using the same pivot-preserving screw matrix recipe. The target holds sixteen cells. [SCREW-F-SCREW-STEP] [PB-MOVE-ROTATE].

The proof function is `stepEntry43GearBasidemove`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry43GearBasidemove, assertMoveCell) -->

Make these required Fusion calls:

- `adsk.core.Matrix3D.create()`.
- `rot.setToRotation(64 * pitch / lam, axisVector, axisPoint)`.
- `axisVector.copy()`.
- `shift.scaleBy(64 * pitch)`.
- `rot.transformBy(mov)`.
- `adsk.core.ObjectCollection.create()`.
- `bodies.add(asideBody)`.
- `design.features.moveFeatures.createInput2(bodies)`.
- `moveInput.defineAsFreeMove(rot)`.
- `design.features.moveFeatures.add(moveInput)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

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
      "receiver": "rot",
      "role": "required",
      "span": "rot.setToRotation(64 * pitch / lam, axisVector, axisPoint)"
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
      "span": "shift.scaleBy(64 * pitch)"
    },
    {
      "condition": null,
      "name": "transformBy",
      "owner": "adsk.core.Matrix3D",
      "reason": null,
      "receiver": "rot",
      "role": "required",
      "span": "rot.transformBy(mov)"
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
      "span": "bodies.add(asideBody)"
    },
    {
      "condition": null,
      "name": "createInput2",
      "owner": "adsk.fusion.MoveFeatures",
      "reason": null,
      "receiver": "design.features.moveFeatures",
      "role": "required",
      "span": "design.features.moveFeatures.createInput2(bodies)"
    },
    {
      "condition": null,
      "name": "defineAsFreeMove",
      "owner": "adsk.fusion.MoveFeatureInput",
      "reason": null,
      "receiver": "moveInput",
      "role": "required",
      "span": "moveInput.defineAsFreeMove(rot)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.MoveFeatures",
      "reason": null,
      "receiver": "design.features.moveFeatures",
      "role": "required",
      "span": "design.features.moveFeatures.add(moveInput)"
    }
  ],
  "citations": [
    {
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 44 `[GO]` Gear B aside join

Join the moved one-cell aside into the sixteen-cell target, leaving seventeen cells, 68 teeth. Set operation JoinFeatureOperation and isKeepToolBodies False. Require one body and name it Gear B. For other counts use the carried binary schedule; q=1 skips copy/move/join entirely. [SCREW-F-JOIN] [PB-EMPTY-RESULT].

The proof function is `stepEntry44GearBasidejoin`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry44GearBasidejoin, assertJoinCell) -->

Make these required Fusion calls:

- `adsk.core.ObjectCollection.create()`.
- `tools.add(toolBody)`.
- `design.features.combineFeatures.createInput(targetBody, tools)`.
- `design.features.combineFeatures.add(combineInput)`.
- `combineFeature.bodies.item(0)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

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
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.createInput(targetBody, tools)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.add(combineInput)"
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
      "first": 1167,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1167–1216.

## 45r1 `[GO]` Gear B remainder sections sketch

Only when N mod c is positive, draw the final r teeth at s0+q*c*P through s0+N*P, with r*n+1 sections, using the complete Cell Sections recipe above. This is a separate sketch named Gear B Cell Remainder. [PB-3D-SKETCH-SECTIONS] [PB-SKETCH-DEFER].

The proof function is `stepEntry45r1GearBremaindersectionssketch`.

<!-- proof-run: proofkit.Run(sectionCases, stepEntry45r1GearBremaindersectionssketch) -->

Make these required Fusion calls:

- `design.sketches.add(plane)`.
- `adsk.core.Point3D.create(x, y, z)`.
- `sketch.modelToSketchSpace(worldPoint)`.
- `sketch.sketchPoints.add(local)`.
- `sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)`.
- `adsk.core.ObjectCollection.create()`.
- `fitPoints.add(sectionPoint)`.
- `sketch.sketchCurves.sketchFittedSplines.add(fitPoints)`.
- `spline.fitPoints.item(i)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "design.sketches",
      "role": "required",
      "span": "design.sketches.add(plane)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "adsk.core.Point3D.create(x, y, z)"
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
      "span": "sketch.sketchPoints.add(local)"
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
      "span": "fitPoints.add(sectionPoint)"
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
      "first": 1200,
      "last": 1207,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1200–1207.

## 45r2 `[GO]` Gear B remainder loft

Loft the remainder's ordered four-curve section paths as one new body. Require one solid body. [PB-LOFT] [PB-EMPTY-RESULT].

The proof function is `stepEntry45r2GearBremainderloft`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry45r2GearBremainderloft, assertCellSegmentLoft) -->

Make these required Fusion calls:

- `design.features.loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`.
- `adsk.core.ObjectCollection.create()`.
- `curves.add(sectionCurve)`.
- `design.features.createPath(curves, False)`.
- `loftInput.loftSections.add(sectionPath)`.
- `design.features.loftFeatures.add(loftInput)`.
- `loftFeature.bodies.item(0)`.
- `cellBody.pointContainment(probePoint)`.

The supported proof substitution is explicit:

The proof builds one real two-section interval of this cell, using its actual sampled sections. The evaluator
refuses unions of adjacent intervals at their tangent triangulated caps. The sketch proof checks every sampled
section. This solid substitute checks the representative interval's solidity and prismoid volume, and establishes
no whole-cell smooth surface or volume.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.LoftFeatures",
      "reason": null,
      "receiver": "design.features.loftFeatures",
      "role": "required",
      "span": "design.features.loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)"
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
      "span": "curves.add(sectionCurve)"
    },
    {
      "condition": null,
      "name": "createPath",
      "owner": "adsk.fusion.Features",
      "reason": null,
      "receiver": "design.features",
      "role": "required",
      "span": "design.features.createPath(curves, False)"
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
      "receiver": "design.features.loftFeatures",
      "role": "required",
      "span": "design.features.loftFeatures.add(loftInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "loftFeature.bodies",
      "role": "required",
      "span": "loftFeature.bodies.item(0)"
    },
    {
      "condition": null,
      "name": "pointContainment",
      "owner": "adsk.fusion.BRepBody",
      "reason": null,
      "receiver": "cellBody",
      "role": "required",
      "span": "cellBody.pointContainment(probePoint)"
    }
  ],
  "citations": [
    {
      "first": 1200,
      "last": 1207,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1200–1207.

## 45r3 `[GO]` Gear B remainder join

Join the remainder, built in place, as sole tool to the completed whole-cell ribbon. The first remainder section equals the last whole-cell section. Require exactly one feature body. [SCREW-F-JOIN] [PB-EMPTY-RESULT].

The proof function is `stepEntry45r3GearBremainderjoin`.

<!-- proof-run: proofkit3d.RunSolid(cellSolidCases, stepEntry45r3GearBremainderjoin, assertJoinCell) -->

Make these required Fusion calls:

- `adsk.core.ObjectCollection.create()`.
- `tools.add(toolBody)`.
- `design.features.combineFeatures.createInput(targetBody, tools)`.
- `design.features.combineFeatures.add(combineInput)`.
- `combineFeature.bodies.item(0)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

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
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.createInput(targetBody, tools)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.add(combineInput)"
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
      "first": 1200,
      "last": 1216,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1200–1216.

## 46 `[GO]` Sleeve sketch

Draw the Sleeve sketch on targetPlane with two concentric circles at mapped C, z=0. Ri=cageRadius-collarHalf, Ro=cageRadius+collarHalf. Fix each circle's own centre and give diameter 2*Ri or 2*Ro with off-centre text on that circle. Require fully constrained and exactly one profile with profileLoops.count=2. The inner disc is the other profile. Do not use find_profile_by_curve_counts for circles. [PB-USE-SELECTED-PLANE] [PB-CIRCLE-CENTER] [PB-RADIAL-DIM] [PB-SKETCH-ZERO-Z] [PB-EMPTY-RESULT].

The proof function is `stepEntry46Sleevesketch`.

<!-- proof-run: proofkit.Run(sleeveSketchCases, stepEntry46Sleevesketch) -->

Make these required Fusion calls:

- `design.sketches.add(plane)`.
- `adsk.core.Point3D.create(x, y, z)`.
- `sketch.modelToSketchSpace(worldPoint)`.
- `sketch.sketchCurves.sketchCircles.addByCenterRadius(centre, radius)`.
- `sketch.sketchDimensions.addDiameterDimension(circle, textPoint)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "design.sketches",
      "role": "required",
      "span": "design.sketches.add(plane)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "adsk.core.Point3D.create(x, y, z)"
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
      "span": "sketch.sketchCurves.sketchCircles.addByCenterRadius(centre, radius)"
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
      "first": 1243,
      "last": 1256,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1243–1256.

## 47 `[GO]` Sleeve extrude

Extrude the two-loop annulus as NewBodyFeatureOperation. Use a symmetric extent whose distance is cageRise and whose isFullLength is False, giving +/-cageRise. Require one body, retained as cageBody. [PB-THROUGH-CUT] [PB-EMPTY-RESULT].

The proof function is `stepEntry47Sleeveextrude`.

<!-- proof-run: proofkit3d.RunSolid(sleeveSolidCases, stepEntry47Sleeveextrude, assertSleeveExtrude) -->

Make these required Fusion calls:

- `design.features.extrudeFeatures.createInput(ringProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`.
- `adsk.core.ValueInput.createByReal(value)`.
- `extrudeInput.setSymmetricExtent(riseValue, False)`.
- `design.features.extrudeFeatures.add(extrudeInput)`.
- `extrudeFeature.bodies.item(0)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "design.features.extrudeFeatures",
      "role": "required",
      "span": "design.features.extrudeFeatures.createInput(ringProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(value)"
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
      "receiver": "design.features.extrudeFeatures",
      "role": "required",
      "span": "design.features.extrudeFeatures.add(extrudeInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "extrudeFeature.bodies",
      "role": "required",
      "span": "extrudeFeature.bodies.item(0)"
    }
  ],
  "citations": [
    {
      "first": 1243,
      "last": 1256,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1243–1256.

## 48 `[PROSE]` Gear A bore -R plane

Create the bore's own plane at fraction 0 on Gear A's bore- path line, passed directly. Name it Gear A Bore -R Plane. [PB-CONSTRUCTION-PLANES] [SCREW-F-TWISTED-SLOT].

Make these required Fusion calls:

- `design.constructionPlanes.createInput()`.
- `adsk.core.ValueInput.createByReal(value)`.
- `design.constructionPlanes.add(planeInput)`.
- `planeInput.setByDistanceOnPath(boreLine, startFraction)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "design.constructionPlanes",
      "role": "required",
      "span": "design.constructionPlanes.createInput()"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(value)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "design.constructionPlanes",
      "role": "required",
      "span": "design.constructionPlanes.add(planeInput)"
    },
    {
      "condition": null,
      "name": "setByDistanceOnPath",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByDistanceOnPath(boreLine, startFraction)"
    }
  ],
  "citations": [
    {
      "first": 979,
      "last": 998,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L979–998.

## 49 `[GO]` Gear A bore -R sketch

**The roof allowance** ('levelBore', 'roofSide', 'boreOpening'). With the sleeve standing on its
'−n̂' end, a bore whose long faces pass through level inside the wall has its roof bridged across
the hole by the printer. At the 14° mounting angles of the second sleeve that was each gear's
'-R' bore alone, whose long faces lay flat at station −14.30 mm, and the second sleeve printed
those two bores too tight and the two '+R' bores not ('[SCREW-F-PRINT-2]'). So each gear's
**level bore** has its roof face moved out by 'roofAllowance', and every other face of every bore
stays at the clearance. At the defaults' zero mounting angles **both** bores of each gear pass
through level, at stations ±12.37 mm, so the two tie and the '-R' bore takes the allowance; the
'+R' bores' roofs are bridged 15.4 mm across with the clearance alone, as the second sleeve's
tight roofs were ("What the print showed", "What the proof cannot reach"):

- The level bore is, of the gear's two bores, the one whose long faces come nearest level over
  the wall's span on its centre line. A long face runs along the section's 'u', which stands
  'theta' from 'û_g', and 'û_g' is along '±n̂', so a long face is level where 'cos(theta)' is zero.
  For the bore at 'σ*cageRadius', 'σ' being '-1' or '+1', take 'theta' at the two stations
  'σ*(cageRadius - collarHalf)' and 'σ*(cageRadius + collarHalf)'; when some 'pi/2 + k*pi' lies
  between them its tilt is 0, and otherwise it is the smaller '|cos theta|' of the two. The bore
  with the smaller tilt is the level bore, the '-R' bore when they tie. At the defaults both
  bores' tilt is 0, so the '-R' bore is the level bore; at the 14° of the third print the '-R'
  bores' tilt was 0 and the '+R' bores' '|cos 101.3°|', 0.195.
- The roof face is the long face that is up when the sleeve stands on its '−n̂' end: the '+v'
  face when 'v̂(sc)·n̂ = -sin(theta(sc))*(û_g·n̂)' is positive at the bore's crossing station
  'sc = σ*cageRadius', and the '-v' face otherwise. 'v̂_g·n̂' is zero, so that is the whole
  product. At the defaults it is gear A's '+v' face and gear B's '-v' face. The sign holds over
  the whole cut: 'v̂·n̂' changes sign only where a long face stands upright, 90° of twist, 12.4 mm
  of axis, from where it lies level.
- The level bore's rectangle spans 'v' from 'vLo = -ht' to 'vHi = ht + a' when its roof is the
  '+v' face, and from 'vLo = -ht - a' to 'vHi = ht' otherwise: 4.75 mm through at the defaults.
  Every other bore spans '-ht' to 'ht', and every bore spans 'u' from '-hw' to 'hw'.

The allowance adds no move of the ribbon along or across the axes and no roll: the other bore and
the level bore's floor hold the ribbon to the clearance there. It lets the ribbon tilt, its level
bore's end rising into the room while the other bore holds, which carries the crossing 0.248 mm
instead of 0.200 mm toward the other ribbon for gear A and away from it for gear B
('TestRoofAllowanceAddsOnlyATilt'). After the sleeve is built the build logs, with 'futil.log'
('[PB-LOGGING]'), 'Print the cage standing on its end below the selected plane: the roof
allowance is on the bridged roofs that way up.', so the print orientation is explicit even when
the bore marks are hard to see.

**Cut each bore with a twisted sweep** ('[SCREW-F-TWISTED-SLOT]', '[PB-SWEEP-TWIST]'): the bore's
rectangle, drawn by the rectangle scheme below on the bore's own plane,
'{gearLabel} Bore {-R|+R} Plane', 'setByDistanceOnPath' at fraction '0' of the bore's 'bore-' or
'bore+' line of §1, in the sketch '{gearLabel} Bore {-R|+R}', at that plane's station, '-sOut' or
'sIn'; then swept along that line as 'sweepFeatures.createInput(profile, path,
CutFeatureOperation)', with 'path = features.createPath(line, False)', 'twistAngle' set to
'ValueInput.createByReal(+(sOut - sIn)/Lambda)', 'participantBodies' set to a list holding the
cage body alone, and nothing else set; then 'sweepFeatures.add'. The sign is positive: the path
runs along '+dir_g', the section's angle 's/Lambda + Phi' grows with 's', and a positive
'twistAngle' turns the profile that way, as measured on 2026-09-28 on the video frame's collars
('[PB-SWEEP-TWIST]'). Each profile starts in air, in the hollow for a '+R' bore and outside the
tube for a '-R' bore, and each cut turns 80°; the cuts measured in Fusion turned 58–65° and
started on a collar's face. The add-in at c8a63b5 built such cuts at the earlier 82.31°, and both
printed ribbons screwed through them ('[SCREW-F-PRINT-MESH]'). The ribbons pass through the
channel and are not on the participant list, so the cut leaves them whole: measured on
2026-09-28, a body wholly inside the channel and off the list kept its volume to the last digit
while the cut body lost the channel's. The bore is the exact helicoid — the rectangle turning
rigidly at '1/Lambda' about the axis, the channel 'TestRibbonsStayInsideTheirBoresOverTheTravel'
and 'TestSleeveAdmitsOnlyTheScrewMotion' measure against — with no facets and no fit between
samples. The two bores of one gear stand '2*cageRadius' apart and are two sweeps on two lines, as
§1 draws them.

**The rectangle scheme.** Each of the four bore section sketches is a rectangle spanning 'v' from
'vLo' to 'vHi' and 'u' from 'uB' to 'uF', turned by 'theta = s/Lambda + Phi_g' about the axis point
'O', which lies on the line 'v = 0'; for a bore other than a level one, at its centre, and for a
level bore 'a/2' off it across the thickness. Four lines, two
dimensions, one coincidence and one angle cannot fix 'O' inside such a rectangle; a construction
spine through 'O' can, and this is the scheme:

- **References.** Two reference points (Sketch Discipline): 'O', the point where the sweep
  path pierces the sketch plane, at 'origin_g + s*dir_g' for the plane's station 's', and
  'Cp', at 'origin_g + s*dir_g + (A/2)*û_g', which is 'A/2' from 'O' along the gear's
  unrotated 'û' and lies on the plane because 'û_g' does. The construction line 'Ru' from 'O'
  to 'Cp', sharing both, is the sketch's zero of rotation; it has no freedom once its ends are
  fixed. Both points are set 'isFixed = True' after 'Ru' and the spine 'K' are drawn and before
  any dimension is added. Nothing is projected into a section sketch: the Anchor Line's
  projection would run through 'Cp' across the rectangle at 'u = A/2', which at the defaults
  is inside the rectangle and would split the profile, and '[PB-PROJECT-NOT-FIXED]' rules a
  projected point out as an anchor in any case.
- **The spine.** A construction line 'K' from 'O' to 'E', seeded at '(uF, 0)' turned by
  'theta', with a distance dimension 'O'–'E' of 'uF'.
- **The angle.** An angular dimension between 'Ru' and 'K' when '|sin theta| >= sqrt(1/2)',
  where the angle is 'theta' folded into 45°–135°, and otherwise between 'Ru' and the toothed
  side 'L2', which stands at 'theta + 90°' (Sketch Discipline). Compute the text point from the
  actual sketch-space endpoints, not from the world-frame rays. For 'Ru'–'K', take unit rays
  'O'→'Cp' and 'O'→'E', and place the text at 'O + (rayRu + rayK)*uF/3'. For 'Ru'–'L2', intersect
  the infinite lines through 'O,Cp' and 'P1,P2' in sketch space. Take the unit ray along 'Ru'
  from that intersection toward 'Cp' (toward 'O' if 'Cp' coincides with it), and the unit ray
  'P1'→'P2' along 'L2'. Place the text at 'intersection + (rayRu + rayL2)*uF/3'. Write the
  clamped 'acos' of the rays' dot product as the angular dimension's value. The text must be
  inside that angle's wedge at the lines' actual intersection ('[PB-ANGULAR-DIM]').
- **The rectangle.** Four lines sharing their corners: 'L1' from '(uB, vLo)' to '(uF, vLo)',
  'L2' on to '(uF, vHi)', 'L3' on to '(uB, vHi)', 'L4' back to the start, every seed the solved
  point ('[PB-SHARE-XOR-COINCIDENT]': shared, no coincident on a corner). Then 'L1' parallel to
  'K' with an offset dimension of '-vLo', and 'L3' on the other side with one of 'vHi'; 'E' coincident on
  'L2', and 'L2' perpendicular to 'K'; 'L4' parallel to 'L2' with an offset dimension of
  'uF - uB' ('[PB-OFFSET-DIM]', '[PB-NO-OVERCONSTRAIN]').

Create the length and angular dimensions **before** adding any rectangle parallel, coincidence,
or perpendicular constraint. Then add those five geometric constraints, followed by the three
offset dimensions. Preserve this order and the intersection-based angle text placement: Fusion
accepted it on the sleeve printed before 2026-10-05, while a regenerated sketch that changed
both details failed at 'addAngularDimension(Ru, L2, angleText)' with
'VCS_SKETCH_OVER_CONSTRAINTS' ('[SCREW-F-BORE-ANGLE-ORDER]'). The proof's solver sees only the
completed constraint system, so it cannot verify Fusion's dimension-creation order.

Ten degrees of freedom — 'E' and the four corners — against ten rows: five dimensions (the
length, the angle, three offsets) and five constraints (three parallels, a perpendicular, a
coincidence), so the sketch closes fully constrained. In every one of the four sketches
'uB = -hw', 'uF = hw', and 'vLo' and 'vHi' are the bore's own from "The roof allowance", and its
four lines are solid and are
the profile, the one loop of four lines, 'find_profile_by_curve_counts(sketch, lines=4)'
('[PB-PROFILE-MATCH]'). The construction lines bound nothing. The scheme is drawn with sketch
computing deferred (Sketch Discipline); the video frame's collar sections, drawn by the same
scheme, read fully constrained in Fusion on 2026-09-28.

After the sketch computes, compare each corner's solved 'SketchPoint.geometry' with the local
position of its expected world seed. Require each distance to be at most 0.001 mm and raise with
the sketch name, corner index and observed distance otherwise. This checks the roof's chosen
side and the solved section size; 'isFullyConstrained' alone checks neither.

**What the proof's stand-in costs.** The proof's solid engine has no twisted sweep, so the compiled step
proof builds each bore's channel as a chain of two-section lofts through rotated rectangles ("What the
proof checks"), and the count it lofts through is derived from the turn and the clearance: no two
neighbouring sections more than 5° of twist apart, nor more than the angle at which the facets between them
take 4% of the clearance, '2*acos(1 - 0.04*clearance/c)', and the count is 'ceil(turn / step) + 1' for the
smaller step. At the defaults the turn is 81.41°, the facet bound 5.1° and 5° governs: 18 sections. At a
clearance of 0.05 mm the turn is 80.15°, the facet bound 2.6° governs, and the count is 33. Those facets
are a ruled wall's: flat between sections, inside the true channel, and 0.007 mm of the 0.20 mm clearance
at the derived count, which 'TestSleeveBoreSubstituteKeepsItsClearance' holds at 95% of the clearance from
0.05 mm to 0.9 mm. 'decad' does not build that wall: it walls each cell with two flat triangles, which
depart from it by up to a quarter of the cell's twist, 0.32 mm on the long faces at the defaults, more than
the clearance (the same test logs it). So the compiled proof reads the cage's volume and the build's probes
off the stand-in, and no clearance. The build derives no section count: its channel has none.

**The bore has to twist; a straight hole binds.** Over the wall's '2*collarHalf' the ribbon turns
by '2*collarHalf/Lambda', so its corner sweeps '(W/2)*(2*collarHalf/Lambda)' across the opening.
At the defaults that is 5.7 mm against 0.20 mm of clearance.

**The bore is cut to the crest rectangle, never to a tooth.** The crests are the ribbon's outer
edge, so that rectangle holds the whole ribbon, teeth included, and its toothed side bears on the
crests ("Why the cage needs no tooth-shaped cut"); nothing in the frame is shaped like a tooth.

**The sweep's sense is checked, not assumed** ('[SCREW-F-SWEEP-CHECK]', '[PB-SELF-DIAGNOSING]').
After each bore cut the build requires the sweep feature to leave exactly one body and probes the
cage with 'pointContainment' at the two points 'origin_g + sc*dir_g ± (W/2 + clearance/2)*û(sc)'
at the crossing itself, 'sc = ±cageRadius', where 'û(s)' is the section's turned 'u' direction,
'cos(theta)*û_g + sin(theta)*v̂_g' with 'theta = s/Lambda + Phi_g'. Both must be
'PointOutsidePointContainment', the channel being open there, and the build raises naming the
bore otherwise. The probes stand 16.63 mm from the frame's axis at all four bores,
inside the wall, and clear of the ribbon and of the channel's wall by 'clearance/2'. They tell
the two senses apart on their own. The profile sits at the cut's first station 's0', 'sIn' or
'-sOut', and under the wrong sense the channel at the crossing is turned '2*(sc - s0)/Lambda'
from the right one — 103° for a '+R' bore, 58° for a '-R' bore — which puts the probes 7.39 and
6.46 mm across a channel 2.075 mm half thick, or 2.675 mm on the level bore's roof side, in the
wall
('TestSleeveBoreProbesTellTheTwistSense'). With no collar body there is no end-vertex check; the
probes are the whole of the sense check.

For the level bore when 'roofAllowance > 0', use two more containment probes at its crossing
station with 'u = 0'. Set 'roofSign = +1' when '-sin(theta(sc))*(û_g·n̂) > 0', else '-1'.
The point at 'v = roofSign*(ht + roofAllowance/2)' must be outside the cage, in the added
roof room. The point at 'v = -roofSign*(ht + roofAllowance/2)' must remain inside the cage,
beyond the floor. Raise with the bore name, roof or floor, and observed containment. The first
two probes establish the sweep's turn; these two establish which face got the allowance.

This entry draws exactly this bore's section, not the other three. It defers computing until all dimensions and constraints exist. Every planar mapped point has z=0. Require isFullyConstrained, one four-line profile and all solved corners within 0.001 mm of their expected mapped seeds. [PB-SKETCH-FIRST] [PB-SKETCH-DEFER] [PB-SKETCH-ZERO-Z].

The proof function is `stepEntry49GearAboreRsketch`.

<!-- proof-run: proofkit.Run(boreSketchCases, stepEntry49GearAboreRsketch) -->

Make these required Fusion calls:

- `design.sketches.add(plane)`.
- `adsk.core.Point3D.create(x, y, z)`.
- `sketch.modelToSketchSpace(worldPoint)`.
- `sketch.sketchPoints.add(local)`.
- `sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)`.
- `sketch.sketchDimensions.addDistanceDimension(O, E, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, lengthText)`.
- `sketch.sketchDimensions.addAngularDimension(Ru, K, angleText)`.
- `sketch.sketchDimensions.addAngularDimension(Ru, L2, angleText)`.
- `sketch.geometricConstraints.addParallel(L1, K)`.
- `sketch.geometricConstraints.addParallel(L3, K)`.
- `sketch.geometricConstraints.addParallel(L4, L2)`.
- `sketch.geometricConstraints.addCoincident(E, L2)`.
- `sketch.geometricConstraints.addPerpendicular(L2, K)`.
- `sketch.sketchDimensions.addOffsetDimension(K, L1, lowerText)`.
- `sketch.sketchDimensions.addOffsetDimension(K, L3, upperText)`.
- `sketch.sketchDimensions.addOffsetDimension(L2, L4, widthText)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "design.sketches",
      "role": "required",
      "span": "design.sketches.add(plane)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "adsk.core.Point3D.create(x, y, z)"
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
      "span": "sketch.sketchPoints.add(local)"
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
      "span": "sketch.sketchDimensions.addDistanceDimension(O, E, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, lengthText)"
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
      "span": "sketch.sketchDimensions.addOffsetDimension(K, L1, lowerText)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addOffsetDimension(K, L3, upperText)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addOffsetDimension(L2, L4, widthText)"
    }
  ],
  "citations": [
    {
      "first": 1274,
      "last": 1390,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1274–1390.

## 50 `[GO]` Gear A bore -R sweep cut

**The bores.** One per crossing: a gear's '+R' bore runs through the wall where its axis crosses
the circle of radius 'cageRadius', at station '+cageRadius' of the axis, and its '-R' bore at
'-cageRadius'. Each is the crest rectangle plus the clearance, '2*hw' by '2*ht', 15.4 by 4.15 mm,
turned at every station 's' to the ribbon's own angle 's/Lambda + Phi_g'. That is the channel the
video frame's collars were cut with, and 'TestSleeveBoresAreTheSameChannels' pins it: the
rectangle, the stations, the twist, the angles at the crossings, 109.09° and −109.09° (123.1°
and −95.1° at the 14° mounting angles before 2026-10-03), and which
bore carries the roof allowance on which face. Only the span the cut covers is the sleeve's own.
The tube's inside is a cylinder, not a plane square to the ribbon, so the channel's corners reach
into the wall before its centre line does: a corner first touches the inner face by station
'sqrt(Ri^2 - c^2)', 8.806 mm. The cut runs from a millimetre before that, 'sIn', where the whole
section is in the hollow, to a millimetre past the outer face, 'sOut', where the whole section is
outside: '[sIn, sOut]' for a '+R' bore and '[-sOut, -sIn]' for a '-R' bore, 11.194 mm of axis and
81.41° of turn. The assembly phase moves
gear B's ribbon along its axis and not its bores: any stretch of the ribbon fits a bore, and the
ribbon can go in at any phase a pitch apart, so the frame has no way to know the phase.



Sweep only this bore's rectangle on its own path. Use CutFeatureOperation, twistAngle=ValueInput of +(sOut-sIn)/Lambda and participantBodies=[cageBody]. Set no orientation, solid twist axis, guide rail, or guide surface. Require one feature body. At sc=sigma*cageRadius both u-side probes must be outside. For the level bore and positive roofAllowance, the probe at u=0, v=roofSign*(ht+roofAllowance/2) must be outside, and the opposite v probe must be inside. [PB-SWEEP-TWIST] [PB-PATH-FROM-SKETCH] [PB-SELF-DIAGNOSING].

The proof function is `stepEntry50GearAboreRsweepcut`.

<!-- proof-run: proofkit3d.RunSolid(boreSolidCases, stepEntry50GearAboreRsweepcut, assertBoreCut) -->

Make these required Fusion calls:

- `design.features.createPath(boreLine, False)`.
- `design.features.sweepFeatures.createInput(profile, borePath, adsk.fusion.FeatureOperations.CutFeatureOperation)`.
- `adsk.core.ValueInput.createByReal(value)`.
- `design.features.sweepFeatures.add(sweepInput)`.
- `sweepFeature.bodies.item(0)`.
- `cageBody.pointContainment(probePoint)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism of this bore's actual crossing rectangle, including its asymmetric roof
allowance, for the refused chain of lofts. It cuts the real uncut annular sleeve in this gear's frame and checks a
single connected solid with reduced volume. It proves no twisted-channel clearance or screw-motion admission. The
four containment checks remain required in Fusion; the current solid API provides no containment reading.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createPath",
      "owner": "adsk.fusion.Features",
      "reason": null,
      "receiver": "design.features",
      "role": "required",
      "span": "design.features.createPath(boreLine, False)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.SweepFeatures",
      "reason": null,
      "receiver": "design.features.sweepFeatures",
      "role": "required",
      "span": "design.features.sweepFeatures.createInput(profile, borePath, adsk.fusion.FeatureOperations.CutFeatureOperation)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(value)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SweepFeatures",
      "reason": null,
      "receiver": "design.features.sweepFeatures",
      "role": "required",
      "span": "design.features.sweepFeatures.add(sweepInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "sweepFeature.bodies",
      "role": "required",
      "span": "sweepFeature.bodies.item(0)"
    },
    {
      "condition": null,
      "name": "pointContainment",
      "owner": "adsk.fusion.BRepBody",
      "reason": null,
      "receiver": "cageBody",
      "role": "required",
      "span": "cageBody.pointContainment(probePoint)"
    }
  ],
  "citations": [
    {
      "first": 1257,
      "last": 1438,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1257–1438.

## 51 `[PROSE]` Gear A bore +R plane

Create the bore's own plane at fraction 0 on Gear A's bore+ path line, passed directly. Name it Gear A Bore +R Plane. [PB-CONSTRUCTION-PLANES] [SCREW-F-TWISTED-SLOT].

Make these required Fusion calls:

- `design.constructionPlanes.createInput()`.
- `adsk.core.ValueInput.createByReal(value)`.
- `design.constructionPlanes.add(planeInput)`.
- `planeInput.setByDistanceOnPath(boreLine, startFraction)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "design.constructionPlanes",
      "role": "required",
      "span": "design.constructionPlanes.createInput()"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(value)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "design.constructionPlanes",
      "role": "required",
      "span": "design.constructionPlanes.add(planeInput)"
    },
    {
      "condition": null,
      "name": "setByDistanceOnPath",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByDistanceOnPath(boreLine, startFraction)"
    }
  ],
  "citations": [
    {
      "first": 979,
      "last": 998,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L979–998.

## 52 `[GO]` Gear A bore +R sketch

**The roof allowance** ('levelBore', 'roofSide', 'boreOpening'). With the sleeve standing on its
'−n̂' end, a bore whose long faces pass through level inside the wall has its roof bridged across
the hole by the printer. At the 14° mounting angles of the second sleeve that was each gear's
'-R' bore alone, whose long faces lay flat at station −14.30 mm, and the second sleeve printed
those two bores too tight and the two '+R' bores not ('[SCREW-F-PRINT-2]'). So each gear's
**level bore** has its roof face moved out by 'roofAllowance', and every other face of every bore
stays at the clearance. At the defaults' zero mounting angles **both** bores of each gear pass
through level, at stations ±12.37 mm, so the two tie and the '-R' bore takes the allowance; the
'+R' bores' roofs are bridged 15.4 mm across with the clearance alone, as the second sleeve's
tight roofs were ("What the print showed", "What the proof cannot reach"):

- The level bore is, of the gear's two bores, the one whose long faces come nearest level over
  the wall's span on its centre line. A long face runs along the section's 'u', which stands
  'theta' from 'û_g', and 'û_g' is along '±n̂', so a long face is level where 'cos(theta)' is zero.
  For the bore at 'σ*cageRadius', 'σ' being '-1' or '+1', take 'theta' at the two stations
  'σ*(cageRadius - collarHalf)' and 'σ*(cageRadius + collarHalf)'; when some 'pi/2 + k*pi' lies
  between them its tilt is 0, and otherwise it is the smaller '|cos theta|' of the two. The bore
  with the smaller tilt is the level bore, the '-R' bore when they tie. At the defaults both
  bores' tilt is 0, so the '-R' bore is the level bore; at the 14° of the third print the '-R'
  bores' tilt was 0 and the '+R' bores' '|cos 101.3°|', 0.195.
- The roof face is the long face that is up when the sleeve stands on its '−n̂' end: the '+v'
  face when 'v̂(sc)·n̂ = -sin(theta(sc))*(û_g·n̂)' is positive at the bore's crossing station
  'sc = σ*cageRadius', and the '-v' face otherwise. 'v̂_g·n̂' is zero, so that is the whole
  product. At the defaults it is gear A's '+v' face and gear B's '-v' face. The sign holds over
  the whole cut: 'v̂·n̂' changes sign only where a long face stands upright, 90° of twist, 12.4 mm
  of axis, from where it lies level.
- The level bore's rectangle spans 'v' from 'vLo = -ht' to 'vHi = ht + a' when its roof is the
  '+v' face, and from 'vLo = -ht - a' to 'vHi = ht' otherwise: 4.75 mm through at the defaults.
  Every other bore spans '-ht' to 'ht', and every bore spans 'u' from '-hw' to 'hw'.

The allowance adds no move of the ribbon along or across the axes and no roll: the other bore and
the level bore's floor hold the ribbon to the clearance there. It lets the ribbon tilt, its level
bore's end rising into the room while the other bore holds, which carries the crossing 0.248 mm
instead of 0.200 mm toward the other ribbon for gear A and away from it for gear B
('TestRoofAllowanceAddsOnlyATilt'). After the sleeve is built the build logs, with 'futil.log'
('[PB-LOGGING]'), 'Print the cage standing on its end below the selected plane: the roof
allowance is on the bridged roofs that way up.', so the print orientation is explicit even when
the bore marks are hard to see.

**Cut each bore with a twisted sweep** ('[SCREW-F-TWISTED-SLOT]', '[PB-SWEEP-TWIST]'): the bore's
rectangle, drawn by the rectangle scheme below on the bore's own plane,
'{gearLabel} Bore {-R|+R} Plane', 'setByDistanceOnPath' at fraction '0' of the bore's 'bore-' or
'bore+' line of §1, in the sketch '{gearLabel} Bore {-R|+R}', at that plane's station, '-sOut' or
'sIn'; then swept along that line as 'sweepFeatures.createInput(profile, path,
CutFeatureOperation)', with 'path = features.createPath(line, False)', 'twistAngle' set to
'ValueInput.createByReal(+(sOut - sIn)/Lambda)', 'participantBodies' set to a list holding the
cage body alone, and nothing else set; then 'sweepFeatures.add'. The sign is positive: the path
runs along '+dir_g', the section's angle 's/Lambda + Phi' grows with 's', and a positive
'twistAngle' turns the profile that way, as measured on 2026-09-28 on the video frame's collars
('[PB-SWEEP-TWIST]'). Each profile starts in air, in the hollow for a '+R' bore and outside the
tube for a '-R' bore, and each cut turns 80°; the cuts measured in Fusion turned 58–65° and
started on a collar's face. The add-in at c8a63b5 built such cuts at the earlier 82.31°, and both
printed ribbons screwed through them ('[SCREW-F-PRINT-MESH]'). The ribbons pass through the
channel and are not on the participant list, so the cut leaves them whole: measured on
2026-09-28, a body wholly inside the channel and off the list kept its volume to the last digit
while the cut body lost the channel's. The bore is the exact helicoid — the rectangle turning
rigidly at '1/Lambda' about the axis, the channel 'TestRibbonsStayInsideTheirBoresOverTheTravel'
and 'TestSleeveAdmitsOnlyTheScrewMotion' measure against — with no facets and no fit between
samples. The two bores of one gear stand '2*cageRadius' apart and are two sweeps on two lines, as
§1 draws them.

**The rectangle scheme.** Each of the four bore section sketches is a rectangle spanning 'v' from
'vLo' to 'vHi' and 'u' from 'uB' to 'uF', turned by 'theta = s/Lambda + Phi_g' about the axis point
'O', which lies on the line 'v = 0'; for a bore other than a level one, at its centre, and for a
level bore 'a/2' off it across the thickness. Four lines, two
dimensions, one coincidence and one angle cannot fix 'O' inside such a rectangle; a construction
spine through 'O' can, and this is the scheme:

- **References.** Two reference points (Sketch Discipline): 'O', the point where the sweep
  path pierces the sketch plane, at 'origin_g + s*dir_g' for the plane's station 's', and
  'Cp', at 'origin_g + s*dir_g + (A/2)*û_g', which is 'A/2' from 'O' along the gear's
  unrotated 'û' and lies on the plane because 'û_g' does. The construction line 'Ru' from 'O'
  to 'Cp', sharing both, is the sketch's zero of rotation; it has no freedom once its ends are
  fixed. Both points are set 'isFixed = True' after 'Ru' and the spine 'K' are drawn and before
  any dimension is added. Nothing is projected into a section sketch: the Anchor Line's
  projection would run through 'Cp' across the rectangle at 'u = A/2', which at the defaults
  is inside the rectangle and would split the profile, and '[PB-PROJECT-NOT-FIXED]' rules a
  projected point out as an anchor in any case.
- **The spine.** A construction line 'K' from 'O' to 'E', seeded at '(uF, 0)' turned by
  'theta', with a distance dimension 'O'–'E' of 'uF'.
- **The angle.** An angular dimension between 'Ru' and 'K' when '|sin theta| >= sqrt(1/2)',
  where the angle is 'theta' folded into 45°–135°, and otherwise between 'Ru' and the toothed
  side 'L2', which stands at 'theta + 90°' (Sketch Discipline). Compute the text point from the
  actual sketch-space endpoints, not from the world-frame rays. For 'Ru'–'K', take unit rays
  'O'→'Cp' and 'O'→'E', and place the text at 'O + (rayRu + rayK)*uF/3'. For 'Ru'–'L2', intersect
  the infinite lines through 'O,Cp' and 'P1,P2' in sketch space. Take the unit ray along 'Ru'
  from that intersection toward 'Cp' (toward 'O' if 'Cp' coincides with it), and the unit ray
  'P1'→'P2' along 'L2'. Place the text at 'intersection + (rayRu + rayL2)*uF/3'. Write the
  clamped 'acos' of the rays' dot product as the angular dimension's value. The text must be
  inside that angle's wedge at the lines' actual intersection ('[PB-ANGULAR-DIM]').
- **The rectangle.** Four lines sharing their corners: 'L1' from '(uB, vLo)' to '(uF, vLo)',
  'L2' on to '(uF, vHi)', 'L3' on to '(uB, vHi)', 'L4' back to the start, every seed the solved
  point ('[PB-SHARE-XOR-COINCIDENT]': shared, no coincident on a corner). Then 'L1' parallel to
  'K' with an offset dimension of '-vLo', and 'L3' on the other side with one of 'vHi'; 'E' coincident on
  'L2', and 'L2' perpendicular to 'K'; 'L4' parallel to 'L2' with an offset dimension of
  'uF - uB' ('[PB-OFFSET-DIM]', '[PB-NO-OVERCONSTRAIN]').

Create the length and angular dimensions **before** adding any rectangle parallel, coincidence,
or perpendicular constraint. Then add those five geometric constraints, followed by the three
offset dimensions. Preserve this order and the intersection-based angle text placement: Fusion
accepted it on the sleeve printed before 2026-10-05, while a regenerated sketch that changed
both details failed at 'addAngularDimension(Ru, L2, angleText)' with
'VCS_SKETCH_OVER_CONSTRAINTS' ('[SCREW-F-BORE-ANGLE-ORDER]'). The proof's solver sees only the
completed constraint system, so it cannot verify Fusion's dimension-creation order.

Ten degrees of freedom — 'E' and the four corners — against ten rows: five dimensions (the
length, the angle, three offsets) and five constraints (three parallels, a perpendicular, a
coincidence), so the sketch closes fully constrained. In every one of the four sketches
'uB = -hw', 'uF = hw', and 'vLo' and 'vHi' are the bore's own from "The roof allowance", and its
four lines are solid and are
the profile, the one loop of four lines, 'find_profile_by_curve_counts(sketch, lines=4)'
('[PB-PROFILE-MATCH]'). The construction lines bound nothing. The scheme is drawn with sketch
computing deferred (Sketch Discipline); the video frame's collar sections, drawn by the same
scheme, read fully constrained in Fusion on 2026-09-28.

After the sketch computes, compare each corner's solved 'SketchPoint.geometry' with the local
position of its expected world seed. Require each distance to be at most 0.001 mm and raise with
the sketch name, corner index and observed distance otherwise. This checks the roof's chosen
side and the solved section size; 'isFullyConstrained' alone checks neither.

**What the proof's stand-in costs.** The proof's solid engine has no twisted sweep, so the compiled step
proof builds each bore's channel as a chain of two-section lofts through rotated rectangles ("What the
proof checks"), and the count it lofts through is derived from the turn and the clearance: no two
neighbouring sections more than 5° of twist apart, nor more than the angle at which the facets between them
take 4% of the clearance, '2*acos(1 - 0.04*clearance/c)', and the count is 'ceil(turn / step) + 1' for the
smaller step. At the defaults the turn is 81.41°, the facet bound 5.1° and 5° governs: 18 sections. At a
clearance of 0.05 mm the turn is 80.15°, the facet bound 2.6° governs, and the count is 33. Those facets
are a ruled wall's: flat between sections, inside the true channel, and 0.007 mm of the 0.20 mm clearance
at the derived count, which 'TestSleeveBoreSubstituteKeepsItsClearance' holds at 95% of the clearance from
0.05 mm to 0.9 mm. 'decad' does not build that wall: it walls each cell with two flat triangles, which
depart from it by up to a quarter of the cell's twist, 0.32 mm on the long faces at the defaults, more than
the clearance (the same test logs it). So the compiled proof reads the cage's volume and the build's probes
off the stand-in, and no clearance. The build derives no section count: its channel has none.

**The bore has to twist; a straight hole binds.** Over the wall's '2*collarHalf' the ribbon turns
by '2*collarHalf/Lambda', so its corner sweeps '(W/2)*(2*collarHalf/Lambda)' across the opening.
At the defaults that is 5.7 mm against 0.20 mm of clearance.

**The bore is cut to the crest rectangle, never to a tooth.** The crests are the ribbon's outer
edge, so that rectangle holds the whole ribbon, teeth included, and its toothed side bears on the
crests ("Why the cage needs no tooth-shaped cut"); nothing in the frame is shaped like a tooth.

**The sweep's sense is checked, not assumed** ('[SCREW-F-SWEEP-CHECK]', '[PB-SELF-DIAGNOSING]').
After each bore cut the build requires the sweep feature to leave exactly one body and probes the
cage with 'pointContainment' at the two points 'origin_g + sc*dir_g ± (W/2 + clearance/2)*û(sc)'
at the crossing itself, 'sc = ±cageRadius', where 'û(s)' is the section's turned 'u' direction,
'cos(theta)*û_g + sin(theta)*v̂_g' with 'theta = s/Lambda + Phi_g'. Both must be
'PointOutsidePointContainment', the channel being open there, and the build raises naming the
bore otherwise. The probes stand 16.63 mm from the frame's axis at all four bores,
inside the wall, and clear of the ribbon and of the channel's wall by 'clearance/2'. They tell
the two senses apart on their own. The profile sits at the cut's first station 's0', 'sIn' or
'-sOut', and under the wrong sense the channel at the crossing is turned '2*(sc - s0)/Lambda'
from the right one — 103° for a '+R' bore, 58° for a '-R' bore — which puts the probes 7.39 and
6.46 mm across a channel 2.075 mm half thick, or 2.675 mm on the level bore's roof side, in the
wall
('TestSleeveBoreProbesTellTheTwistSense'). With no collar body there is no end-vertex check; the
probes are the whole of the sense check.

For the level bore when 'roofAllowance > 0', use two more containment probes at its crossing
station with 'u = 0'. Set 'roofSign = +1' when '-sin(theta(sc))*(û_g·n̂) > 0', else '-1'.
The point at 'v = roofSign*(ht + roofAllowance/2)' must be outside the cage, in the added
roof room. The point at 'v = -roofSign*(ht + roofAllowance/2)' must remain inside the cage,
beyond the floor. Raise with the bore name, roof or floor, and observed containment. The first
two probes establish the sweep's turn; these two establish which face got the allowance.

This entry draws exactly this bore's section, not the other three. It defers computing until all dimensions and constraints exist. Every planar mapped point has z=0. Require isFullyConstrained, one four-line profile and all solved corners within 0.001 mm of their expected mapped seeds. [PB-SKETCH-FIRST] [PB-SKETCH-DEFER] [PB-SKETCH-ZERO-Z].

The proof function is `stepEntry52GearAboreRsketch`.

<!-- proof-run: proofkit.Run(boreSketchCases, stepEntry52GearAboreRsketch) -->

Make these required Fusion calls:

- `design.sketches.add(plane)`.
- `adsk.core.Point3D.create(x, y, z)`.
- `sketch.modelToSketchSpace(worldPoint)`.
- `sketch.sketchPoints.add(local)`.
- `sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)`.
- `sketch.sketchDimensions.addDistanceDimension(O, E, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, lengthText)`.
- `sketch.sketchDimensions.addAngularDimension(Ru, K, angleText)`.
- `sketch.sketchDimensions.addAngularDimension(Ru, L2, angleText)`.
- `sketch.geometricConstraints.addParallel(L1, K)`.
- `sketch.geometricConstraints.addParallel(L3, K)`.
- `sketch.geometricConstraints.addParallel(L4, L2)`.
- `sketch.geometricConstraints.addCoincident(E, L2)`.
- `sketch.geometricConstraints.addPerpendicular(L2, K)`.
- `sketch.sketchDimensions.addOffsetDimension(K, L1, lowerText)`.
- `sketch.sketchDimensions.addOffsetDimension(K, L3, upperText)`.
- `sketch.sketchDimensions.addOffsetDimension(L2, L4, widthText)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "design.sketches",
      "role": "required",
      "span": "design.sketches.add(plane)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "adsk.core.Point3D.create(x, y, z)"
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
      "span": "sketch.sketchPoints.add(local)"
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
      "span": "sketch.sketchDimensions.addDistanceDimension(O, E, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, lengthText)"
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
      "span": "sketch.sketchDimensions.addOffsetDimension(K, L1, lowerText)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addOffsetDimension(K, L3, upperText)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addOffsetDimension(L2, L4, widthText)"
    }
  ],
  "citations": [
    {
      "first": 1274,
      "last": 1390,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1274–1390.

## 53 `[GO]` Gear A bore +R sweep cut

**The bores.** One per crossing: a gear's '+R' bore runs through the wall where its axis crosses
the circle of radius 'cageRadius', at station '+cageRadius' of the axis, and its '-R' bore at
'-cageRadius'. Each is the crest rectangle plus the clearance, '2*hw' by '2*ht', 15.4 by 4.15 mm,
turned at every station 's' to the ribbon's own angle 's/Lambda + Phi_g'. That is the channel the
video frame's collars were cut with, and 'TestSleeveBoresAreTheSameChannels' pins it: the
rectangle, the stations, the twist, the angles at the crossings, 109.09° and −109.09° (123.1°
and −95.1° at the 14° mounting angles before 2026-10-03), and which
bore carries the roof allowance on which face. Only the span the cut covers is the sleeve's own.
The tube's inside is a cylinder, not a plane square to the ribbon, so the channel's corners reach
into the wall before its centre line does: a corner first touches the inner face by station
'sqrt(Ri^2 - c^2)', 8.806 mm. The cut runs from a millimetre before that, 'sIn', where the whole
section is in the hollow, to a millimetre past the outer face, 'sOut', where the whole section is
outside: '[sIn, sOut]' for a '+R' bore and '[-sOut, -sIn]' for a '-R' bore, 11.194 mm of axis and
81.41° of turn. The assembly phase moves
gear B's ribbon along its axis and not its bores: any stretch of the ribbon fits a bore, and the
ribbon can go in at any phase a pitch apart, so the frame has no way to know the phase.



Sweep only this bore's rectangle on its own path. Use CutFeatureOperation, twistAngle=ValueInput of +(sOut-sIn)/Lambda and participantBodies=[cageBody]. Set no orientation, solid twist axis, guide rail, or guide surface. Require one feature body. At sc=sigma*cageRadius both u-side probes must be outside. For the level bore and positive roofAllowance, the probe at u=0, v=roofSign*(ht+roofAllowance/2) must be outside, and the opposite v probe must be inside. [PB-SWEEP-TWIST] [PB-PATH-FROM-SKETCH] [PB-SELF-DIAGNOSING].

The proof function is `stepEntry53GearAboreRsweepcut`.

<!-- proof-run: proofkit3d.RunSolid(boreSolidCases, stepEntry53GearAboreRsweepcut, assertBoreCut) -->

Make these required Fusion calls:

- `design.features.createPath(boreLine, False)`.
- `design.features.sweepFeatures.createInput(profile, borePath, adsk.fusion.FeatureOperations.CutFeatureOperation)`.
- `adsk.core.ValueInput.createByReal(value)`.
- `design.features.sweepFeatures.add(sweepInput)`.
- `sweepFeature.bodies.item(0)`.
- `cageBody.pointContainment(probePoint)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism of this bore's actual crossing rectangle, including its asymmetric roof
allowance, for the refused chain of lofts. It cuts the real uncut annular sleeve in this gear's frame and checks a
single connected solid with reduced volume. It proves no twisted-channel clearance or screw-motion admission. The
four containment checks remain required in Fusion; the current solid API provides no containment reading.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createPath",
      "owner": "adsk.fusion.Features",
      "reason": null,
      "receiver": "design.features",
      "role": "required",
      "span": "design.features.createPath(boreLine, False)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.SweepFeatures",
      "reason": null,
      "receiver": "design.features.sweepFeatures",
      "role": "required",
      "span": "design.features.sweepFeatures.createInput(profile, borePath, adsk.fusion.FeatureOperations.CutFeatureOperation)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(value)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SweepFeatures",
      "reason": null,
      "receiver": "design.features.sweepFeatures",
      "role": "required",
      "span": "design.features.sweepFeatures.add(sweepInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "sweepFeature.bodies",
      "role": "required",
      "span": "sweepFeature.bodies.item(0)"
    },
    {
      "condition": null,
      "name": "pointContainment",
      "owner": "adsk.fusion.BRepBody",
      "reason": null,
      "receiver": "cageBody",
      "role": "required",
      "span": "cageBody.pointContainment(probePoint)"
    }
  ],
  "citations": [
    {
      "first": 1257,
      "last": 1438,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1257–1438.

## 54 `[PROSE]` Gear B bore -R plane

Create the bore's own plane at fraction 0 on Gear B's bore- path line, passed directly. Name it Gear B Bore -R Plane. [PB-CONSTRUCTION-PLANES] [SCREW-F-TWISTED-SLOT].

Make these required Fusion calls:

- `design.constructionPlanes.createInput()`.
- `adsk.core.ValueInput.createByReal(value)`.
- `design.constructionPlanes.add(planeInput)`.
- `planeInput.setByDistanceOnPath(boreLine, startFraction)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "design.constructionPlanes",
      "role": "required",
      "span": "design.constructionPlanes.createInput()"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(value)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "design.constructionPlanes",
      "role": "required",
      "span": "design.constructionPlanes.add(planeInput)"
    },
    {
      "condition": null,
      "name": "setByDistanceOnPath",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByDistanceOnPath(boreLine, startFraction)"
    }
  ],
  "citations": [
    {
      "first": 979,
      "last": 998,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L979–998.

## 55 `[GO]` Gear B bore -R sketch

**The roof allowance** ('levelBore', 'roofSide', 'boreOpening'). With the sleeve standing on its
'−n̂' end, a bore whose long faces pass through level inside the wall has its roof bridged across
the hole by the printer. At the 14° mounting angles of the second sleeve that was each gear's
'-R' bore alone, whose long faces lay flat at station −14.30 mm, and the second sleeve printed
those two bores too tight and the two '+R' bores not ('[SCREW-F-PRINT-2]'). So each gear's
**level bore** has its roof face moved out by 'roofAllowance', and every other face of every bore
stays at the clearance. At the defaults' zero mounting angles **both** bores of each gear pass
through level, at stations ±12.37 mm, so the two tie and the '-R' bore takes the allowance; the
'+R' bores' roofs are bridged 15.4 mm across with the clearance alone, as the second sleeve's
tight roofs were ("What the print showed", "What the proof cannot reach"):

- The level bore is, of the gear's two bores, the one whose long faces come nearest level over
  the wall's span on its centre line. A long face runs along the section's 'u', which stands
  'theta' from 'û_g', and 'û_g' is along '±n̂', so a long face is level where 'cos(theta)' is zero.
  For the bore at 'σ*cageRadius', 'σ' being '-1' or '+1', take 'theta' at the two stations
  'σ*(cageRadius - collarHalf)' and 'σ*(cageRadius + collarHalf)'; when some 'pi/2 + k*pi' lies
  between them its tilt is 0, and otherwise it is the smaller '|cos theta|' of the two. The bore
  with the smaller tilt is the level bore, the '-R' bore when they tie. At the defaults both
  bores' tilt is 0, so the '-R' bore is the level bore; at the 14° of the third print the '-R'
  bores' tilt was 0 and the '+R' bores' '|cos 101.3°|', 0.195.
- The roof face is the long face that is up when the sleeve stands on its '−n̂' end: the '+v'
  face when 'v̂(sc)·n̂ = -sin(theta(sc))*(û_g·n̂)' is positive at the bore's crossing station
  'sc = σ*cageRadius', and the '-v' face otherwise. 'v̂_g·n̂' is zero, so that is the whole
  product. At the defaults it is gear A's '+v' face and gear B's '-v' face. The sign holds over
  the whole cut: 'v̂·n̂' changes sign only where a long face stands upright, 90° of twist, 12.4 mm
  of axis, from where it lies level.
- The level bore's rectangle spans 'v' from 'vLo = -ht' to 'vHi = ht + a' when its roof is the
  '+v' face, and from 'vLo = -ht - a' to 'vHi = ht' otherwise: 4.75 mm through at the defaults.
  Every other bore spans '-ht' to 'ht', and every bore spans 'u' from '-hw' to 'hw'.

The allowance adds no move of the ribbon along or across the axes and no roll: the other bore and
the level bore's floor hold the ribbon to the clearance there. It lets the ribbon tilt, its level
bore's end rising into the room while the other bore holds, which carries the crossing 0.248 mm
instead of 0.200 mm toward the other ribbon for gear A and away from it for gear B
('TestRoofAllowanceAddsOnlyATilt'). After the sleeve is built the build logs, with 'futil.log'
('[PB-LOGGING]'), 'Print the cage standing on its end below the selected plane: the roof
allowance is on the bridged roofs that way up.', so the print orientation is explicit even when
the bore marks are hard to see.

**Cut each bore with a twisted sweep** ('[SCREW-F-TWISTED-SLOT]', '[PB-SWEEP-TWIST]'): the bore's
rectangle, drawn by the rectangle scheme below on the bore's own plane,
'{gearLabel} Bore {-R|+R} Plane', 'setByDistanceOnPath' at fraction '0' of the bore's 'bore-' or
'bore+' line of §1, in the sketch '{gearLabel} Bore {-R|+R}', at that plane's station, '-sOut' or
'sIn'; then swept along that line as 'sweepFeatures.createInput(profile, path,
CutFeatureOperation)', with 'path = features.createPath(line, False)', 'twistAngle' set to
'ValueInput.createByReal(+(sOut - sIn)/Lambda)', 'participantBodies' set to a list holding the
cage body alone, and nothing else set; then 'sweepFeatures.add'. The sign is positive: the path
runs along '+dir_g', the section's angle 's/Lambda + Phi' grows with 's', and a positive
'twistAngle' turns the profile that way, as measured on 2026-09-28 on the video frame's collars
('[PB-SWEEP-TWIST]'). Each profile starts in air, in the hollow for a '+R' bore and outside the
tube for a '-R' bore, and each cut turns 80°; the cuts measured in Fusion turned 58–65° and
started on a collar's face. The add-in at c8a63b5 built such cuts at the earlier 82.31°, and both
printed ribbons screwed through them ('[SCREW-F-PRINT-MESH]'). The ribbons pass through the
channel and are not on the participant list, so the cut leaves them whole: measured on
2026-09-28, a body wholly inside the channel and off the list kept its volume to the last digit
while the cut body lost the channel's. The bore is the exact helicoid — the rectangle turning
rigidly at '1/Lambda' about the axis, the channel 'TestRibbonsStayInsideTheirBoresOverTheTravel'
and 'TestSleeveAdmitsOnlyTheScrewMotion' measure against — with no facets and no fit between
samples. The two bores of one gear stand '2*cageRadius' apart and are two sweeps on two lines, as
§1 draws them.

**The rectangle scheme.** Each of the four bore section sketches is a rectangle spanning 'v' from
'vLo' to 'vHi' and 'u' from 'uB' to 'uF', turned by 'theta = s/Lambda + Phi_g' about the axis point
'O', which lies on the line 'v = 0'; for a bore other than a level one, at its centre, and for a
level bore 'a/2' off it across the thickness. Four lines, two
dimensions, one coincidence and one angle cannot fix 'O' inside such a rectangle; a construction
spine through 'O' can, and this is the scheme:

- **References.** Two reference points (Sketch Discipline): 'O', the point where the sweep
  path pierces the sketch plane, at 'origin_g + s*dir_g' for the plane's station 's', and
  'Cp', at 'origin_g + s*dir_g + (A/2)*û_g', which is 'A/2' from 'O' along the gear's
  unrotated 'û' and lies on the plane because 'û_g' does. The construction line 'Ru' from 'O'
  to 'Cp', sharing both, is the sketch's zero of rotation; it has no freedom once its ends are
  fixed. Both points are set 'isFixed = True' after 'Ru' and the spine 'K' are drawn and before
  any dimension is added. Nothing is projected into a section sketch: the Anchor Line's
  projection would run through 'Cp' across the rectangle at 'u = A/2', which at the defaults
  is inside the rectangle and would split the profile, and '[PB-PROJECT-NOT-FIXED]' rules a
  projected point out as an anchor in any case.
- **The spine.** A construction line 'K' from 'O' to 'E', seeded at '(uF, 0)' turned by
  'theta', with a distance dimension 'O'–'E' of 'uF'.
- **The angle.** An angular dimension between 'Ru' and 'K' when '|sin theta| >= sqrt(1/2)',
  where the angle is 'theta' folded into 45°–135°, and otherwise between 'Ru' and the toothed
  side 'L2', which stands at 'theta + 90°' (Sketch Discipline). Compute the text point from the
  actual sketch-space endpoints, not from the world-frame rays. For 'Ru'–'K', take unit rays
  'O'→'Cp' and 'O'→'E', and place the text at 'O + (rayRu + rayK)*uF/3'. For 'Ru'–'L2', intersect
  the infinite lines through 'O,Cp' and 'P1,P2' in sketch space. Take the unit ray along 'Ru'
  from that intersection toward 'Cp' (toward 'O' if 'Cp' coincides with it), and the unit ray
  'P1'→'P2' along 'L2'. Place the text at 'intersection + (rayRu + rayL2)*uF/3'. Write the
  clamped 'acos' of the rays' dot product as the angular dimension's value. The text must be
  inside that angle's wedge at the lines' actual intersection ('[PB-ANGULAR-DIM]').
- **The rectangle.** Four lines sharing their corners: 'L1' from '(uB, vLo)' to '(uF, vLo)',
  'L2' on to '(uF, vHi)', 'L3' on to '(uB, vHi)', 'L4' back to the start, every seed the solved
  point ('[PB-SHARE-XOR-COINCIDENT]': shared, no coincident on a corner). Then 'L1' parallel to
  'K' with an offset dimension of '-vLo', and 'L3' on the other side with one of 'vHi'; 'E' coincident on
  'L2', and 'L2' perpendicular to 'K'; 'L4' parallel to 'L2' with an offset dimension of
  'uF - uB' ('[PB-OFFSET-DIM]', '[PB-NO-OVERCONSTRAIN]').

Create the length and angular dimensions **before** adding any rectangle parallel, coincidence,
or perpendicular constraint. Then add those five geometric constraints, followed by the three
offset dimensions. Preserve this order and the intersection-based angle text placement: Fusion
accepted it on the sleeve printed before 2026-10-05, while a regenerated sketch that changed
both details failed at 'addAngularDimension(Ru, L2, angleText)' with
'VCS_SKETCH_OVER_CONSTRAINTS' ('[SCREW-F-BORE-ANGLE-ORDER]'). The proof's solver sees only the
completed constraint system, so it cannot verify Fusion's dimension-creation order.

Ten degrees of freedom — 'E' and the four corners — against ten rows: five dimensions (the
length, the angle, three offsets) and five constraints (three parallels, a perpendicular, a
coincidence), so the sketch closes fully constrained. In every one of the four sketches
'uB = -hw', 'uF = hw', and 'vLo' and 'vHi' are the bore's own from "The roof allowance", and its
four lines are solid and are
the profile, the one loop of four lines, 'find_profile_by_curve_counts(sketch, lines=4)'
('[PB-PROFILE-MATCH]'). The construction lines bound nothing. The scheme is drawn with sketch
computing deferred (Sketch Discipline); the video frame's collar sections, drawn by the same
scheme, read fully constrained in Fusion on 2026-09-28.

After the sketch computes, compare each corner's solved 'SketchPoint.geometry' with the local
position of its expected world seed. Require each distance to be at most 0.001 mm and raise with
the sketch name, corner index and observed distance otherwise. This checks the roof's chosen
side and the solved section size; 'isFullyConstrained' alone checks neither.

**What the proof's stand-in costs.** The proof's solid engine has no twisted sweep, so the compiled step
proof builds each bore's channel as a chain of two-section lofts through rotated rectangles ("What the
proof checks"), and the count it lofts through is derived from the turn and the clearance: no two
neighbouring sections more than 5° of twist apart, nor more than the angle at which the facets between them
take 4% of the clearance, '2*acos(1 - 0.04*clearance/c)', and the count is 'ceil(turn / step) + 1' for the
smaller step. At the defaults the turn is 81.41°, the facet bound 5.1° and 5° governs: 18 sections. At a
clearance of 0.05 mm the turn is 80.15°, the facet bound 2.6° governs, and the count is 33. Those facets
are a ruled wall's: flat between sections, inside the true channel, and 0.007 mm of the 0.20 mm clearance
at the derived count, which 'TestSleeveBoreSubstituteKeepsItsClearance' holds at 95% of the clearance from
0.05 mm to 0.9 mm. 'decad' does not build that wall: it walls each cell with two flat triangles, which
depart from it by up to a quarter of the cell's twist, 0.32 mm on the long faces at the defaults, more than
the clearance (the same test logs it). So the compiled proof reads the cage's volume and the build's probes
off the stand-in, and no clearance. The build derives no section count: its channel has none.

**The bore has to twist; a straight hole binds.** Over the wall's '2*collarHalf' the ribbon turns
by '2*collarHalf/Lambda', so its corner sweeps '(W/2)*(2*collarHalf/Lambda)' across the opening.
At the defaults that is 5.7 mm against 0.20 mm of clearance.

**The bore is cut to the crest rectangle, never to a tooth.** The crests are the ribbon's outer
edge, so that rectangle holds the whole ribbon, teeth included, and its toothed side bears on the
crests ("Why the cage needs no tooth-shaped cut"); nothing in the frame is shaped like a tooth.

**The sweep's sense is checked, not assumed** ('[SCREW-F-SWEEP-CHECK]', '[PB-SELF-DIAGNOSING]').
After each bore cut the build requires the sweep feature to leave exactly one body and probes the
cage with 'pointContainment' at the two points 'origin_g + sc*dir_g ± (W/2 + clearance/2)*û(sc)'
at the crossing itself, 'sc = ±cageRadius', where 'û(s)' is the section's turned 'u' direction,
'cos(theta)*û_g + sin(theta)*v̂_g' with 'theta = s/Lambda + Phi_g'. Both must be
'PointOutsidePointContainment', the channel being open there, and the build raises naming the
bore otherwise. The probes stand 16.63 mm from the frame's axis at all four bores,
inside the wall, and clear of the ribbon and of the channel's wall by 'clearance/2'. They tell
the two senses apart on their own. The profile sits at the cut's first station 's0', 'sIn' or
'-sOut', and under the wrong sense the channel at the crossing is turned '2*(sc - s0)/Lambda'
from the right one — 103° for a '+R' bore, 58° for a '-R' bore — which puts the probes 7.39 and
6.46 mm across a channel 2.075 mm half thick, or 2.675 mm on the level bore's roof side, in the
wall
('TestSleeveBoreProbesTellTheTwistSense'). With no collar body there is no end-vertex check; the
probes are the whole of the sense check.

For the level bore when 'roofAllowance > 0', use two more containment probes at its crossing
station with 'u = 0'. Set 'roofSign = +1' when '-sin(theta(sc))*(û_g·n̂) > 0', else '-1'.
The point at 'v = roofSign*(ht + roofAllowance/2)' must be outside the cage, in the added
roof room. The point at 'v = -roofSign*(ht + roofAllowance/2)' must remain inside the cage,
beyond the floor. Raise with the bore name, roof or floor, and observed containment. The first
two probes establish the sweep's turn; these two establish which face got the allowance.

This entry draws exactly this bore's section, not the other three. It defers computing until all dimensions and constraints exist. Every planar mapped point has z=0. Require isFullyConstrained, one four-line profile and all solved corners within 0.001 mm of their expected mapped seeds. [PB-SKETCH-FIRST] [PB-SKETCH-DEFER] [PB-SKETCH-ZERO-Z].

The proof function is `stepEntry55GearBboreRsketch`.

<!-- proof-run: proofkit.Run(boreSketchCases, stepEntry55GearBboreRsketch) -->

Make these required Fusion calls:

- `design.sketches.add(plane)`.
- `adsk.core.Point3D.create(x, y, z)`.
- `sketch.modelToSketchSpace(worldPoint)`.
- `sketch.sketchPoints.add(local)`.
- `sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)`.
- `sketch.sketchDimensions.addDistanceDimension(O, E, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, lengthText)`.
- `sketch.sketchDimensions.addAngularDimension(Ru, K, angleText)`.
- `sketch.sketchDimensions.addAngularDimension(Ru, L2, angleText)`.
- `sketch.geometricConstraints.addParallel(L1, K)`.
- `sketch.geometricConstraints.addParallel(L3, K)`.
- `sketch.geometricConstraints.addParallel(L4, L2)`.
- `sketch.geometricConstraints.addCoincident(E, L2)`.
- `sketch.geometricConstraints.addPerpendicular(L2, K)`.
- `sketch.sketchDimensions.addOffsetDimension(K, L1, lowerText)`.
- `sketch.sketchDimensions.addOffsetDimension(K, L3, upperText)`.
- `sketch.sketchDimensions.addOffsetDimension(L2, L4, widthText)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "design.sketches",
      "role": "required",
      "span": "design.sketches.add(plane)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "adsk.core.Point3D.create(x, y, z)"
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
      "span": "sketch.sketchPoints.add(local)"
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
      "span": "sketch.sketchDimensions.addDistanceDimension(O, E, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, lengthText)"
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
      "span": "sketch.sketchDimensions.addOffsetDimension(K, L1, lowerText)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addOffsetDimension(K, L3, upperText)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addOffsetDimension(L2, L4, widthText)"
    }
  ],
  "citations": [
    {
      "first": 1274,
      "last": 1390,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1274–1390.

## 56 `[GO]` Gear B bore -R sweep cut

**The bores.** One per crossing: a gear's '+R' bore runs through the wall where its axis crosses
the circle of radius 'cageRadius', at station '+cageRadius' of the axis, and its '-R' bore at
'-cageRadius'. Each is the crest rectangle plus the clearance, '2*hw' by '2*ht', 15.4 by 4.15 mm,
turned at every station 's' to the ribbon's own angle 's/Lambda + Phi_g'. That is the channel the
video frame's collars were cut with, and 'TestSleeveBoresAreTheSameChannels' pins it: the
rectangle, the stations, the twist, the angles at the crossings, 109.09° and −109.09° (123.1°
and −95.1° at the 14° mounting angles before 2026-10-03), and which
bore carries the roof allowance on which face. Only the span the cut covers is the sleeve's own.
The tube's inside is a cylinder, not a plane square to the ribbon, so the channel's corners reach
into the wall before its centre line does: a corner first touches the inner face by station
'sqrt(Ri^2 - c^2)', 8.806 mm. The cut runs from a millimetre before that, 'sIn', where the whole
section is in the hollow, to a millimetre past the outer face, 'sOut', where the whole section is
outside: '[sIn, sOut]' for a '+R' bore and '[-sOut, -sIn]' for a '-R' bore, 11.194 mm of axis and
81.41° of turn. The assembly phase moves
gear B's ribbon along its axis and not its bores: any stretch of the ribbon fits a bore, and the
ribbon can go in at any phase a pitch apart, so the frame has no way to know the phase.



Sweep only this bore's rectangle on its own path. Use CutFeatureOperation, twistAngle=ValueInput of +(sOut-sIn)/Lambda and participantBodies=[cageBody]. Set no orientation, solid twist axis, guide rail, or guide surface. Require one feature body. At sc=sigma*cageRadius both u-side probes must be outside. For the level bore and positive roofAllowance, the probe at u=0, v=roofSign*(ht+roofAllowance/2) must be outside, and the opposite v probe must be inside. [PB-SWEEP-TWIST] [PB-PATH-FROM-SKETCH] [PB-SELF-DIAGNOSING].

The proof function is `stepEntry56GearBboreRsweepcut`.

<!-- proof-run: proofkit3d.RunSolid(boreSolidCases, stepEntry56GearBboreRsweepcut, assertBoreCut) -->

Make these required Fusion calls:

- `design.features.createPath(boreLine, False)`.
- `design.features.sweepFeatures.createInput(profile, borePath, adsk.fusion.FeatureOperations.CutFeatureOperation)`.
- `adsk.core.ValueInput.createByReal(value)`.
- `design.features.sweepFeatures.add(sweepInput)`.
- `sweepFeature.bodies.item(0)`.
- `cageBody.pointContainment(probePoint)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism of this bore's actual crossing rectangle, including its asymmetric roof
allowance, for the refused chain of lofts. It cuts the real uncut annular sleeve in this gear's frame and checks a
single connected solid with reduced volume. It proves no twisted-channel clearance or screw-motion admission. The
four containment checks remain required in Fusion; the current solid API provides no containment reading.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createPath",
      "owner": "adsk.fusion.Features",
      "reason": null,
      "receiver": "design.features",
      "role": "required",
      "span": "design.features.createPath(boreLine, False)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.SweepFeatures",
      "reason": null,
      "receiver": "design.features.sweepFeatures",
      "role": "required",
      "span": "design.features.sweepFeatures.createInput(profile, borePath, adsk.fusion.FeatureOperations.CutFeatureOperation)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(value)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SweepFeatures",
      "reason": null,
      "receiver": "design.features.sweepFeatures",
      "role": "required",
      "span": "design.features.sweepFeatures.add(sweepInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "sweepFeature.bodies",
      "role": "required",
      "span": "sweepFeature.bodies.item(0)"
    },
    {
      "condition": null,
      "name": "pointContainment",
      "owner": "adsk.fusion.BRepBody",
      "reason": null,
      "receiver": "cageBody",
      "role": "required",
      "span": "cageBody.pointContainment(probePoint)"
    }
  ],
  "citations": [
    {
      "first": 1257,
      "last": 1438,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1257–1438.

## 57 `[PROSE]` Gear B bore +R plane

Create the bore's own plane at fraction 0 on Gear B's bore+ path line, passed directly. Name it Gear B Bore +R Plane. [PB-CONSTRUCTION-PLANES] [SCREW-F-TWISTED-SLOT].

Make these required Fusion calls:

- `design.constructionPlanes.createInput()`.
- `adsk.core.ValueInput.createByReal(value)`.
- `design.constructionPlanes.add(planeInput)`.
- `planeInput.setByDistanceOnPath(boreLine, startFraction)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "design.constructionPlanes",
      "role": "required",
      "span": "design.constructionPlanes.createInput()"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(value)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "design.constructionPlanes",
      "role": "required",
      "span": "design.constructionPlanes.add(planeInput)"
    },
    {
      "condition": null,
      "name": "setByDistanceOnPath",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByDistanceOnPath(boreLine, startFraction)"
    }
  ],
  "citations": [
    {
      "first": 979,
      "last": 998,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L979–998.

## 58 `[GO]` Gear B bore +R sketch

**The roof allowance** ('levelBore', 'roofSide', 'boreOpening'). With the sleeve standing on its
'−n̂' end, a bore whose long faces pass through level inside the wall has its roof bridged across
the hole by the printer. At the 14° mounting angles of the second sleeve that was each gear's
'-R' bore alone, whose long faces lay flat at station −14.30 mm, and the second sleeve printed
those two bores too tight and the two '+R' bores not ('[SCREW-F-PRINT-2]'). So each gear's
**level bore** has its roof face moved out by 'roofAllowance', and every other face of every bore
stays at the clearance. At the defaults' zero mounting angles **both** bores of each gear pass
through level, at stations ±12.37 mm, so the two tie and the '-R' bore takes the allowance; the
'+R' bores' roofs are bridged 15.4 mm across with the clearance alone, as the second sleeve's
tight roofs were ("What the print showed", "What the proof cannot reach"):

- The level bore is, of the gear's two bores, the one whose long faces come nearest level over
  the wall's span on its centre line. A long face runs along the section's 'u', which stands
  'theta' from 'û_g', and 'û_g' is along '±n̂', so a long face is level where 'cos(theta)' is zero.
  For the bore at 'σ*cageRadius', 'σ' being '-1' or '+1', take 'theta' at the two stations
  'σ*(cageRadius - collarHalf)' and 'σ*(cageRadius + collarHalf)'; when some 'pi/2 + k*pi' lies
  between them its tilt is 0, and otherwise it is the smaller '|cos theta|' of the two. The bore
  with the smaller tilt is the level bore, the '-R' bore when they tie. At the defaults both
  bores' tilt is 0, so the '-R' bore is the level bore; at the 14° of the third print the '-R'
  bores' tilt was 0 and the '+R' bores' '|cos 101.3°|', 0.195.
- The roof face is the long face that is up when the sleeve stands on its '−n̂' end: the '+v'
  face when 'v̂(sc)·n̂ = -sin(theta(sc))*(û_g·n̂)' is positive at the bore's crossing station
  'sc = σ*cageRadius', and the '-v' face otherwise. 'v̂_g·n̂' is zero, so that is the whole
  product. At the defaults it is gear A's '+v' face and gear B's '-v' face. The sign holds over
  the whole cut: 'v̂·n̂' changes sign only where a long face stands upright, 90° of twist, 12.4 mm
  of axis, from where it lies level.
- The level bore's rectangle spans 'v' from 'vLo = -ht' to 'vHi = ht + a' when its roof is the
  '+v' face, and from 'vLo = -ht - a' to 'vHi = ht' otherwise: 4.75 mm through at the defaults.
  Every other bore spans '-ht' to 'ht', and every bore spans 'u' from '-hw' to 'hw'.

The allowance adds no move of the ribbon along or across the axes and no roll: the other bore and
the level bore's floor hold the ribbon to the clearance there. It lets the ribbon tilt, its level
bore's end rising into the room while the other bore holds, which carries the crossing 0.248 mm
instead of 0.200 mm toward the other ribbon for gear A and away from it for gear B
('TestRoofAllowanceAddsOnlyATilt'). After the sleeve is built the build logs, with 'futil.log'
('[PB-LOGGING]'), 'Print the cage standing on its end below the selected plane: the roof
allowance is on the bridged roofs that way up.', so the print orientation is explicit even when
the bore marks are hard to see.

**Cut each bore with a twisted sweep** ('[SCREW-F-TWISTED-SLOT]', '[PB-SWEEP-TWIST]'): the bore's
rectangle, drawn by the rectangle scheme below on the bore's own plane,
'{gearLabel} Bore {-R|+R} Plane', 'setByDistanceOnPath' at fraction '0' of the bore's 'bore-' or
'bore+' line of §1, in the sketch '{gearLabel} Bore {-R|+R}', at that plane's station, '-sOut' or
'sIn'; then swept along that line as 'sweepFeatures.createInput(profile, path,
CutFeatureOperation)', with 'path = features.createPath(line, False)', 'twistAngle' set to
'ValueInput.createByReal(+(sOut - sIn)/Lambda)', 'participantBodies' set to a list holding the
cage body alone, and nothing else set; then 'sweepFeatures.add'. The sign is positive: the path
runs along '+dir_g', the section's angle 's/Lambda + Phi' grows with 's', and a positive
'twistAngle' turns the profile that way, as measured on 2026-09-28 on the video frame's collars
('[PB-SWEEP-TWIST]'). Each profile starts in air, in the hollow for a '+R' bore and outside the
tube for a '-R' bore, and each cut turns 80°; the cuts measured in Fusion turned 58–65° and
started on a collar's face. The add-in at c8a63b5 built such cuts at the earlier 82.31°, and both
printed ribbons screwed through them ('[SCREW-F-PRINT-MESH]'). The ribbons pass through the
channel and are not on the participant list, so the cut leaves them whole: measured on
2026-09-28, a body wholly inside the channel and off the list kept its volume to the last digit
while the cut body lost the channel's. The bore is the exact helicoid — the rectangle turning
rigidly at '1/Lambda' about the axis, the channel 'TestRibbonsStayInsideTheirBoresOverTheTravel'
and 'TestSleeveAdmitsOnlyTheScrewMotion' measure against — with no facets and no fit between
samples. The two bores of one gear stand '2*cageRadius' apart and are two sweeps on two lines, as
§1 draws them.

**The rectangle scheme.** Each of the four bore section sketches is a rectangle spanning 'v' from
'vLo' to 'vHi' and 'u' from 'uB' to 'uF', turned by 'theta = s/Lambda + Phi_g' about the axis point
'O', which lies on the line 'v = 0'; for a bore other than a level one, at its centre, and for a
level bore 'a/2' off it across the thickness. Four lines, two
dimensions, one coincidence and one angle cannot fix 'O' inside such a rectangle; a construction
spine through 'O' can, and this is the scheme:

- **References.** Two reference points (Sketch Discipline): 'O', the point where the sweep
  path pierces the sketch plane, at 'origin_g + s*dir_g' for the plane's station 's', and
  'Cp', at 'origin_g + s*dir_g + (A/2)*û_g', which is 'A/2' from 'O' along the gear's
  unrotated 'û' and lies on the plane because 'û_g' does. The construction line 'Ru' from 'O'
  to 'Cp', sharing both, is the sketch's zero of rotation; it has no freedom once its ends are
  fixed. Both points are set 'isFixed = True' after 'Ru' and the spine 'K' are drawn and before
  any dimension is added. Nothing is projected into a section sketch: the Anchor Line's
  projection would run through 'Cp' across the rectangle at 'u = A/2', which at the defaults
  is inside the rectangle and would split the profile, and '[PB-PROJECT-NOT-FIXED]' rules a
  projected point out as an anchor in any case.
- **The spine.** A construction line 'K' from 'O' to 'E', seeded at '(uF, 0)' turned by
  'theta', with a distance dimension 'O'–'E' of 'uF'.
- **The angle.** An angular dimension between 'Ru' and 'K' when '|sin theta| >= sqrt(1/2)',
  where the angle is 'theta' folded into 45°–135°, and otherwise between 'Ru' and the toothed
  side 'L2', which stands at 'theta + 90°' (Sketch Discipline). Compute the text point from the
  actual sketch-space endpoints, not from the world-frame rays. For 'Ru'–'K', take unit rays
  'O'→'Cp' and 'O'→'E', and place the text at 'O + (rayRu + rayK)*uF/3'. For 'Ru'–'L2', intersect
  the infinite lines through 'O,Cp' and 'P1,P2' in sketch space. Take the unit ray along 'Ru'
  from that intersection toward 'Cp' (toward 'O' if 'Cp' coincides with it), and the unit ray
  'P1'→'P2' along 'L2'. Place the text at 'intersection + (rayRu + rayL2)*uF/3'. Write the
  clamped 'acos' of the rays' dot product as the angular dimension's value. The text must be
  inside that angle's wedge at the lines' actual intersection ('[PB-ANGULAR-DIM]').
- **The rectangle.** Four lines sharing their corners: 'L1' from '(uB, vLo)' to '(uF, vLo)',
  'L2' on to '(uF, vHi)', 'L3' on to '(uB, vHi)', 'L4' back to the start, every seed the solved
  point ('[PB-SHARE-XOR-COINCIDENT]': shared, no coincident on a corner). Then 'L1' parallel to
  'K' with an offset dimension of '-vLo', and 'L3' on the other side with one of 'vHi'; 'E' coincident on
  'L2', and 'L2' perpendicular to 'K'; 'L4' parallel to 'L2' with an offset dimension of
  'uF - uB' ('[PB-OFFSET-DIM]', '[PB-NO-OVERCONSTRAIN]').

Create the length and angular dimensions **before** adding any rectangle parallel, coincidence,
or perpendicular constraint. Then add those five geometric constraints, followed by the three
offset dimensions. Preserve this order and the intersection-based angle text placement: Fusion
accepted it on the sleeve printed before 2026-10-05, while a regenerated sketch that changed
both details failed at 'addAngularDimension(Ru, L2, angleText)' with
'VCS_SKETCH_OVER_CONSTRAINTS' ('[SCREW-F-BORE-ANGLE-ORDER]'). The proof's solver sees only the
completed constraint system, so it cannot verify Fusion's dimension-creation order.

Ten degrees of freedom — 'E' and the four corners — against ten rows: five dimensions (the
length, the angle, three offsets) and five constraints (three parallels, a perpendicular, a
coincidence), so the sketch closes fully constrained. In every one of the four sketches
'uB = -hw', 'uF = hw', and 'vLo' and 'vHi' are the bore's own from "The roof allowance", and its
four lines are solid and are
the profile, the one loop of four lines, 'find_profile_by_curve_counts(sketch, lines=4)'
('[PB-PROFILE-MATCH]'). The construction lines bound nothing. The scheme is drawn with sketch
computing deferred (Sketch Discipline); the video frame's collar sections, drawn by the same
scheme, read fully constrained in Fusion on 2026-09-28.

After the sketch computes, compare each corner's solved 'SketchPoint.geometry' with the local
position of its expected world seed. Require each distance to be at most 0.001 mm and raise with
the sketch name, corner index and observed distance otherwise. This checks the roof's chosen
side and the solved section size; 'isFullyConstrained' alone checks neither.

**What the proof's stand-in costs.** The proof's solid engine has no twisted sweep, so the compiled step
proof builds each bore's channel as a chain of two-section lofts through rotated rectangles ("What the
proof checks"), and the count it lofts through is derived from the turn and the clearance: no two
neighbouring sections more than 5° of twist apart, nor more than the angle at which the facets between them
take 4% of the clearance, '2*acos(1 - 0.04*clearance/c)', and the count is 'ceil(turn / step) + 1' for the
smaller step. At the defaults the turn is 81.41°, the facet bound 5.1° and 5° governs: 18 sections. At a
clearance of 0.05 mm the turn is 80.15°, the facet bound 2.6° governs, and the count is 33. Those facets
are a ruled wall's: flat between sections, inside the true channel, and 0.007 mm of the 0.20 mm clearance
at the derived count, which 'TestSleeveBoreSubstituteKeepsItsClearance' holds at 95% of the clearance from
0.05 mm to 0.9 mm. 'decad' does not build that wall: it walls each cell with two flat triangles, which
depart from it by up to a quarter of the cell's twist, 0.32 mm on the long faces at the defaults, more than
the clearance (the same test logs it). So the compiled proof reads the cage's volume and the build's probes
off the stand-in, and no clearance. The build derives no section count: its channel has none.

**The bore has to twist; a straight hole binds.** Over the wall's '2*collarHalf' the ribbon turns
by '2*collarHalf/Lambda', so its corner sweeps '(W/2)*(2*collarHalf/Lambda)' across the opening.
At the defaults that is 5.7 mm against 0.20 mm of clearance.

**The bore is cut to the crest rectangle, never to a tooth.** The crests are the ribbon's outer
edge, so that rectangle holds the whole ribbon, teeth included, and its toothed side bears on the
crests ("Why the cage needs no tooth-shaped cut"); nothing in the frame is shaped like a tooth.

**The sweep's sense is checked, not assumed** ('[SCREW-F-SWEEP-CHECK]', '[PB-SELF-DIAGNOSING]').
After each bore cut the build requires the sweep feature to leave exactly one body and probes the
cage with 'pointContainment' at the two points 'origin_g + sc*dir_g ± (W/2 + clearance/2)*û(sc)'
at the crossing itself, 'sc = ±cageRadius', where 'û(s)' is the section's turned 'u' direction,
'cos(theta)*û_g + sin(theta)*v̂_g' with 'theta = s/Lambda + Phi_g'. Both must be
'PointOutsidePointContainment', the channel being open there, and the build raises naming the
bore otherwise. The probes stand 16.63 mm from the frame's axis at all four bores,
inside the wall, and clear of the ribbon and of the channel's wall by 'clearance/2'. They tell
the two senses apart on their own. The profile sits at the cut's first station 's0', 'sIn' or
'-sOut', and under the wrong sense the channel at the crossing is turned '2*(sc - s0)/Lambda'
from the right one — 103° for a '+R' bore, 58° for a '-R' bore — which puts the probes 7.39 and
6.46 mm across a channel 2.075 mm half thick, or 2.675 mm on the level bore's roof side, in the
wall
('TestSleeveBoreProbesTellTheTwistSense'). With no collar body there is no end-vertex check; the
probes are the whole of the sense check.

For the level bore when 'roofAllowance > 0', use two more containment probes at its crossing
station with 'u = 0'. Set 'roofSign = +1' when '-sin(theta(sc))*(û_g·n̂) > 0', else '-1'.
The point at 'v = roofSign*(ht + roofAllowance/2)' must be outside the cage, in the added
roof room. The point at 'v = -roofSign*(ht + roofAllowance/2)' must remain inside the cage,
beyond the floor. Raise with the bore name, roof or floor, and observed containment. The first
two probes establish the sweep's turn; these two establish which face got the allowance.

This entry draws exactly this bore's section, not the other three. It defers computing until all dimensions and constraints exist. Every planar mapped point has z=0. Require isFullyConstrained, one four-line profile and all solved corners within 0.001 mm of their expected mapped seeds. [PB-SKETCH-FIRST] [PB-SKETCH-DEFER] [PB-SKETCH-ZERO-Z].

The proof function is `stepEntry58GearBboreRsketch`.

<!-- proof-run: proofkit.Run(boreSketchCases, stepEntry58GearBboreRsketch) -->

Make these required Fusion calls:

- `design.sketches.add(plane)`.
- `adsk.core.Point3D.create(x, y, z)`.
- `sketch.modelToSketchSpace(worldPoint)`.
- `sketch.sketchPoints.add(local)`.
- `sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)`.
- `sketch.sketchDimensions.addDistanceDimension(O, E, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, lengthText)`.
- `sketch.sketchDimensions.addAngularDimension(Ru, K, angleText)`.
- `sketch.sketchDimensions.addAngularDimension(Ru, L2, angleText)`.
- `sketch.geometricConstraints.addParallel(L1, K)`.
- `sketch.geometricConstraints.addParallel(L3, K)`.
- `sketch.geometricConstraints.addParallel(L4, L2)`.
- `sketch.geometricConstraints.addCoincident(E, L2)`.
- `sketch.geometricConstraints.addPerpendicular(L2, K)`.
- `sketch.sketchDimensions.addOffsetDimension(K, L1, lowerText)`.
- `sketch.sketchDimensions.addOffsetDimension(K, L3, upperText)`.
- `sketch.sketchDimensions.addOffsetDimension(L2, L4, widthText)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "design.sketches",
      "role": "required",
      "span": "design.sketches.add(plane)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "adsk.core.Point3D.create(x, y, z)"
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
      "span": "sketch.sketchPoints.add(local)"
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
      "span": "sketch.sketchDimensions.addDistanceDimension(O, E, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, lengthText)"
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
      "span": "sketch.sketchDimensions.addOffsetDimension(K, L1, lowerText)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addOffsetDimension(K, L3, upperText)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addOffsetDimension(L2, L4, widthText)"
    }
  ],
  "citations": [
    {
      "first": 1274,
      "last": 1390,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1274–1390.

## 59 `[GO]` Gear B bore +R sweep cut

**The bores.** One per crossing: a gear's '+R' bore runs through the wall where its axis crosses
the circle of radius 'cageRadius', at station '+cageRadius' of the axis, and its '-R' bore at
'-cageRadius'. Each is the crest rectangle plus the clearance, '2*hw' by '2*ht', 15.4 by 4.15 mm,
turned at every station 's' to the ribbon's own angle 's/Lambda + Phi_g'. That is the channel the
video frame's collars were cut with, and 'TestSleeveBoresAreTheSameChannels' pins it: the
rectangle, the stations, the twist, the angles at the crossings, 109.09° and −109.09° (123.1°
and −95.1° at the 14° mounting angles before 2026-10-03), and which
bore carries the roof allowance on which face. Only the span the cut covers is the sleeve's own.
The tube's inside is a cylinder, not a plane square to the ribbon, so the channel's corners reach
into the wall before its centre line does: a corner first touches the inner face by station
'sqrt(Ri^2 - c^2)', 8.806 mm. The cut runs from a millimetre before that, 'sIn', where the whole
section is in the hollow, to a millimetre past the outer face, 'sOut', where the whole section is
outside: '[sIn, sOut]' for a '+R' bore and '[-sOut, -sIn]' for a '-R' bore, 11.194 mm of axis and
81.41° of turn. The assembly phase moves
gear B's ribbon along its axis and not its bores: any stretch of the ribbon fits a bore, and the
ribbon can go in at any phase a pitch apart, so the frame has no way to know the phase.



Sweep only this bore's rectangle on its own path. Use CutFeatureOperation, twistAngle=ValueInput of +(sOut-sIn)/Lambda and participantBodies=[cageBody]. Set no orientation, solid twist axis, guide rail, or guide surface. Require one feature body. At sc=sigma*cageRadius both u-side probes must be outside. For the level bore and positive roofAllowance, the probe at u=0, v=roofSign*(ht+roofAllowance/2) must be outside, and the opposite v probe must be inside. [PB-SWEEP-TWIST] [PB-PATH-FROM-SKETCH] [PB-SELF-DIAGNOSING].

The proof function is `stepEntry59GearBboreRsweepcut`.

<!-- proof-run: proofkit3d.RunSolid(boreSolidCases, stepEntry59GearBboreRsweepcut, assertBoreCut) -->

Make these required Fusion calls:

- `design.features.createPath(boreLine, False)`.
- `design.features.sweepFeatures.createInput(profile, borePath, adsk.fusion.FeatureOperations.CutFeatureOperation)`.
- `adsk.core.ValueInput.createByReal(value)`.
- `design.features.sweepFeatures.add(sweepInput)`.
- `sweepFeature.bodies.item(0)`.
- `cageBody.pointContainment(probePoint)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism of this bore's actual crossing rectangle, including its asymmetric roof
allowance, for the refused chain of lofts. It cuts the real uncut annular sleeve in this gear's frame and checks a
single connected solid with reduced volume. It proves no twisted-channel clearance or screw-motion admission. The
four containment checks remain required in Fusion; the current solid API provides no containment reading.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createPath",
      "owner": "adsk.fusion.Features",
      "reason": null,
      "receiver": "design.features",
      "role": "required",
      "span": "design.features.createPath(boreLine, False)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.SweepFeatures",
      "reason": null,
      "receiver": "design.features.sweepFeatures",
      "role": "required",
      "span": "design.features.sweepFeatures.createInput(profile, borePath, adsk.fusion.FeatureOperations.CutFeatureOperation)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(value)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SweepFeatures",
      "reason": null,
      "receiver": "design.features.sweepFeatures",
      "role": "required",
      "span": "design.features.sweepFeatures.add(sweepInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "sweepFeature.bodies",
      "role": "required",
      "span": "sweepFeature.bodies.item(0)"
    },
    {
      "condition": null,
      "name": "pointContainment",
      "owner": "adsk.fusion.BRepBody",
      "reason": null,
      "receiver": "cageBody",
      "role": "required",
      "span": "cageBody.pointContainment(probePoint)"
    }
  ],
  "citations": [
    {
      "first": 1257,
      "last": 1438,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1257–1438.

## 60 `[PROSE]` Window plane

Only when at least one searched window has room, create one Window Plane. For crossAngle<=90 degrees, use the Anchor Line with angle '90 deg' against targetPlane. For crossAngle>90 degrees, use its midpoint fraction 0.5. Both are direct SketchLine operands. [PB-CONSTRUCTION-PLANES] [PB-USE-SELECTED-PLANE] [SCREW-F-SLEEVE].

Make these required Fusion calls:

- `design.constructionPlanes.createInput()`.
- `adsk.core.ValueInput.createByReal(value)`.
- `design.constructionPlanes.add(planeInput)`.
- `adsk.core.ValueInput.createByString('90 deg')`.
- `planeInput.setByAngle(anchorLine, rightAngle, targetPlane)`.
- `planeInput.setByDistanceOnPath(anchorLine, midpointFraction)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "design.constructionPlanes",
      "role": "required",
      "span": "design.constructionPlanes.createInput()"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(value)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "design.constructionPlanes",
      "role": "required",
      "span": "design.constructionPlanes.add(planeInput)"
    },
    {
      "condition": null,
      "name": "createByString",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByString('90 deg')"
    },
    {
      "condition": null,
      "name": "setByAngle",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByAngle(anchorLine, rightAngle, targetPlane)"
    },
    {
      "condition": null,
      "name": "setByDistanceOnPath",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByDistanceOnPath(anchorLine, midpointFraction)"
    }
  ],
  "citations": [
    {
      "first": 1618,
      "last": 1625,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1618–1625.

## 61 `[GO]` Window d sketch

Draw only this searched window on Window Plane. Copy its corners in (t,z) from the complete precompute search carried in step 00; use C+t*across+z*nHat, modelToSketchSpace and local z=0. Share one sketch point per corner in solid lines and fix after all lines. Require fully constrained and profiles.count=1. This separate sketch avoids crossed hexagons. The window is omitted on hi<=lo, right<=left, fewer than three distinct corners, or no area. [PB-SHARE-XOR-COINCIDENT] [PB-SINGLE-PROFILE] [PB-SKETCH-ZERO-Z].

The proof function is `stepEntry61Windowdsketch`.

<!-- proof-run: proofkit.Run(windowCases, stepEntry61Windowdsketch) -->

Make these required Fusion calls:

- `design.sketches.add(plane)`.
- `adsk.core.Point3D.create(x, y, z)`.
- `sketch.modelToSketchSpace(worldPoint)`.
- `sketch.sketchPoints.add(local)`.
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
      "receiver": "design.sketches",
      "role": "required",
      "span": "design.sketches.add(plane)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "adsk.core.Point3D.create(x, y, z)"
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
      "span": "sketch.sketchPoints.add(local)"
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
      "first": 1595,
      "last": 1634,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1595–1634.

## 62 `[GO]` Window d extrude cut

Before cutting, require the probe at the average corner (tc,zc), depth (a0(tc)+a1(tc))/2, to be inside cageBody. Extrude only this window by Ro+1 mm from its axis plane toward its facing. Choose PositiveExtentDirection if the mapped C+d has positive local z, otherwise NegativeExtentDirection. Set CutFeatureOperation and participantBodies=[cageBody]. Require one body and the same probe outside after the cut. Replace cageBody with the feature body. [PB-THROUGH-CUT] [PB-SELF-DIAGNOSING] [PB-EMPTY-RESULT].

The proof function is `stepEntry62Windowdextrudecut`.

<!-- proof-run: proofkit3d.RunSolid(windowSolidCases, stepEntry62Windowdextrudecut, assertWindowCut) -->

Make these required Fusion calls:

- `design.features.extrudeFeatures.createInput(profile, operation)`.
- `adsk.core.ValueInput.createByReal(value)`.
- `adsk.fusion.DistanceExtentDefinition.create(distanceValue)`.
- `extrudeInput.setOneSideExtent(extent, direction)`.
- `design.features.extrudeFeatures.add(extrudeInput)`.
- `extrudeFeature.bodies.item(0)`.
- `sketch.modelToSketchSpace(directionPoint)`.
- `cageBody.pointContainment(probePoint)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "design.features.extrudeFeatures",
      "role": "required",
      "span": "design.features.extrudeFeatures.createInput(profile, operation)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(value)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.fusion.DistanceExtentDefinition",
      "reason": null,
      "receiver": "adsk.fusion.DistanceExtentDefinition",
      "role": "required",
      "span": "adsk.fusion.DistanceExtentDefinition.create(distanceValue)"
    },
    {
      "condition": null,
      "name": "setOneSideExtent",
      "owner": "adsk.fusion.ExtrudeFeatureInput",
      "reason": null,
      "receiver": "extrudeInput",
      "role": "required",
      "span": "extrudeInput.setOneSideExtent(extent, direction)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "design.features.extrudeFeatures",
      "role": "required",
      "span": "design.features.extrudeFeatures.add(extrudeInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "extrudeFeature.bodies",
      "role": "required",
      "span": "extrudeFeature.bodies.item(0)"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.modelToSketchSpace(directionPoint)"
    },
    {
      "condition": null,
      "name": "pointContainment",
      "owner": "adsk.fusion.BRepBody",
      "reason": null,
      "receiver": "cageBody",
      "role": "required",
      "span": "cageBody.pointContainment(probePoint)"
    }
  ],
  "citations": [
    {
      "first": 1636,
      "last": 1661,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1636–1661.

## 63 `[GO]` Window -d sketch

Draw only this searched window on Window Plane. Copy its corners in (t,z) from the complete precompute search carried in step 00; use C+t*across+z*nHat, modelToSketchSpace and local z=0. Share one sketch point per corner in solid lines and fix after all lines. Require fully constrained and profiles.count=1. This separate sketch avoids crossed hexagons. The window is omitted on hi<=lo, right<=left, fewer than three distinct corners, or no area. [PB-SHARE-XOR-COINCIDENT] [PB-SINGLE-PROFILE] [PB-SKETCH-ZERO-Z].

The proof function is `stepEntry63Windowdsketch`.

<!-- proof-run: proofkit.Run(windowCases, stepEntry63Windowdsketch) -->

Make these required Fusion calls:

- `design.sketches.add(plane)`.
- `adsk.core.Point3D.create(x, y, z)`.
- `sketch.modelToSketchSpace(worldPoint)`.
- `sketch.sketchPoints.add(local)`.
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
      "receiver": "design.sketches",
      "role": "required",
      "span": "design.sketches.add(plane)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "adsk.core.Point3D.create(x, y, z)"
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
      "span": "sketch.sketchPoints.add(local)"
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
      "first": 1595,
      "last": 1634,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1595–1634.

## 64 `[GO]` Window -d extrude cut

Before cutting, require the probe at the average corner (tc,zc), depth (a0(tc)+a1(tc))/2, to be inside cageBody. Extrude only this window by Ro+1 mm from its axis plane toward its facing. Choose PositiveExtentDirection if the mapped C+d has positive local z, otherwise NegativeExtentDirection. Set CutFeatureOperation and participantBodies=[cageBody]. Require one body and the same probe outside after the cut. Replace cageBody with the feature body. [PB-THROUGH-CUT] [PB-SELF-DIAGNOSING] [PB-EMPTY-RESULT].

The proof function is `stepEntry64Windowdextrudecut`.

<!-- proof-run: proofkit3d.RunSolid(windowSolidCases, stepEntry64Windowdextrudecut, assertWindowCut) -->

Make these required Fusion calls:

- `design.features.extrudeFeatures.createInput(profile, operation)`.
- `adsk.core.ValueInput.createByReal(value)`.
- `adsk.fusion.DistanceExtentDefinition.create(distanceValue)`.
- `extrudeInput.setOneSideExtent(extent, direction)`.
- `design.features.extrudeFeatures.add(extrudeInput)`.
- `extrudeFeature.bodies.item(0)`.
- `sketch.modelToSketchSpace(directionPoint)`.
- `cageBody.pointContainment(probePoint)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "design.features.extrudeFeatures",
      "role": "required",
      "span": "design.features.extrudeFeatures.createInput(profile, operation)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(value)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.fusion.DistanceExtentDefinition",
      "reason": null,
      "receiver": "adsk.fusion.DistanceExtentDefinition",
      "role": "required",
      "span": "adsk.fusion.DistanceExtentDefinition.create(distanceValue)"
    },
    {
      "condition": null,
      "name": "setOneSideExtent",
      "owner": "adsk.fusion.ExtrudeFeatureInput",
      "reason": null,
      "receiver": "extrudeInput",
      "role": "required",
      "span": "extrudeInput.setOneSideExtent(extent, direction)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "design.features.extrudeFeatures",
      "role": "required",
      "span": "design.features.extrudeFeatures.add(extrudeInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "extrudeFeature.bodies",
      "role": "required",
      "span": "extrudeFeature.bodies.item(0)"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.modelToSketchSpace(directionPoint)"
    },
    {
      "condition": null,
      "name": "pointContainment",
      "owner": "adsk.fusion.BRepBody",
      "reason": null,
      "receiver": "cageBody",
      "role": "required",
      "span": "cageBody.pointContainment(probePoint)"
    }
  ],
  "citations": [
    {
      "first": 1636,
      "last": 1661,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1636–1661.

## 65 `[PROSE]` Marker plane

## Bore identification marks

After the four bore cuts and any window cuts, raise one mark for each bore on the top end at
'+cageRise' along 'nHat'. The opposite end stays flat on the print bed. Follow the bore-cut
order: Gear A '-R', Gear A '+R', Gear B '-R', Gear B '+R'. Use the bore's gear index 'g' and
sign 'sigma': a **circle** identifies '+R' ('sigma = +1'),
and a **square** identifies '-R' ('sigma = -1'). Both gears get both marks. The marks join only
to the sleeve, never to either ribbon.

Use 'halfSize = min(1 mm, collarHalf/2)' and 'inset = min(0.1 mm, collarWall/2)' in the same
length units as the sleeve. At the defaults, 'halfSize = 1 mm' and 'inset = 0.1 mm'. Make one
construction plane parallel to Gear B Axis Plane. That plane is '+A/2' along 'nHat' from 'C',
so offset it by 'cageRise - inset - A/2' toward '+nHat'. Choose the signed Fusion offset from
the dot product of Gear B Axis Plane's normal with 'nHat'. Check the new plane's origin's
signed offset from 'C' along 'nHat' against 'cageRise - inset'. The end-wall range
check keeps this plane inside solid sleeve material. For each bore, create a separate sketch on
that plane, named 'Gear A Bore +R Circle Marker', 'Gear A Bore -R Square Marker', and likewise
for Gear B. Its centre in world coordinates is
'C + (cageRise - inset)*nHat + sigma*cageRadius*dirVecs[g]'. Map every point into the sketch
and set its local 'z' to zero before drawing.


[PB-CONSTRUCTION-PLANES] [PB-SKETCH-ZERO-Z] [SCREW-F-BORE-MARKS].

Make these required Fusion calls:

- `design.constructionPlanes.createInput()`.
- `adsk.core.ValueInput.createByReal(value)`.
- `design.constructionPlanes.add(planeInput)`.
- `planeInput.setByOffset(gearBAxisPlane, signedOffset)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "design.constructionPlanes",
      "role": "required",
      "span": "design.constructionPlanes.createInput()"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(value)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "design.constructionPlanes",
      "role": "required",
      "span": "design.constructionPlanes.add(planeInput)"
    },
    {
      "condition": null,
      "name": "setByOffset",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByOffset(gearBAxisPlane, signedOffset)"
    }
  ],
  "citations": [
    {
      "first": 2053,
      "last": 2073,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L2053–2073.

## 66 `[GO]` Gear A bore -R square marker sketch

For a '+R' mark, draw one circle of radius 'halfSize', fix its centre point, and dimension its
diameter to '2*halfSize'. For a '-R' mark, draw four lines joining four fixed sketch points in
counter-clockwise order. Their world coordinates are the centre plus
'(-halfSize,-halfSize)', '(halfSize,-halfSize)', '(halfSize,halfSize)', and
'(-halfSize,halfSize)' in the '(eHat,kHat)' basis. Do not draw a circle and a square in the same
sketch. Each sketch must be fully constrained and have exactly one closed profile; raise with
its name and the observed profile count or constraint status otherwise. For every accepted
'collarHalf', each profile lies strictly inside the annular top face: the square corner's radial
offset from its centre is at most 'sqrt(2)*halfSize < collarHalf', and the circle is smaller.


Draw only this bore's mark. Its name is Gear A Bore -R Square Marker. Require one closed profile, constrained geometry, and zero mapped z for every planar point. [PB-CIRCLE-CENTER] [PB-RADIAL-DIM] [PB-SHARE-XOR-COINCIDENT] [PB-SINGLE-PROFILE].

The proof function is `stepEntry66GearAboreRsquaremarkersketch`.

<!-- proof-run: proofkit.Run(markerCases, stepEntry66GearAboreRsquaremarkersketch) -->

Make these required Fusion calls:

- `design.sketches.add(plane)`.
- `adsk.core.Point3D.create(x, y, z)`.
- `sketch.modelToSketchSpace(worldPoint)`.
- `sketch.sketchPoints.add(local)`.
- `sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "design.sketches",
      "role": "required",
      "span": "design.sketches.add(plane)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "adsk.core.Point3D.create(x, y, z)"
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
      "span": "sketch.sketchPoints.add(local)"
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
      "first": 2074,
      "last": 2084,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L2074–2084.

## 67 `[GO]` Gear A bore -R marker extrude

Extrude this marker profile as NewBodyFeatureOperation by inset+0.4 mm. Choose its direction from the mapped mark-centre-plus-nHat, positive or negative local z. Require exactly one feature body; the mark starts inside the sleeve and rises 0.4 mm above the top end. [PB-THROUGH-CUT] [PB-EMPTY-RESULT] [SCREW-F-BORE-MARKS].

The proof function is `stepEntry67GearAboreRmarkerextrude`.

<!-- proof-run: proofkit3d.RunSolid(markerSolidCases, stepEntry67GearAboreRmarkerextrude, assertMarkerExtrude) -->

Make these required Fusion calls:

- `design.features.extrudeFeatures.createInput(profile, operation)`.
- `adsk.core.ValueInput.createByReal(value)`.
- `adsk.fusion.DistanceExtentDefinition.create(distanceValue)`.
- `extrudeInput.setOneSideExtent(extent, direction)`.
- `design.features.extrudeFeatures.add(extrudeInput)`.
- `extrudeFeature.bodies.item(0)`.
- `sketch.modelToSketchSpace(markCentrePlusNormal)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "design.features.extrudeFeatures",
      "role": "required",
      "span": "design.features.extrudeFeatures.createInput(profile, operation)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(value)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.fusion.DistanceExtentDefinition",
      "reason": null,
      "receiver": "adsk.fusion.DistanceExtentDefinition",
      "role": "required",
      "span": "adsk.fusion.DistanceExtentDefinition.create(distanceValue)"
    },
    {
      "condition": null,
      "name": "setOneSideExtent",
      "owner": "adsk.fusion.ExtrudeFeatureInput",
      "reason": null,
      "receiver": "extrudeInput",
      "role": "required",
      "span": "extrudeInput.setOneSideExtent(extent, direction)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "design.features.extrudeFeatures",
      "role": "required",
      "span": "design.features.extrudeFeatures.add(extrudeInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "extrudeFeature.bodies",
      "role": "required",
      "span": "extrudeFeature.bodies.item(0)"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.modelToSketchSpace(markCentrePlusNormal)"
    }
  ],
  "citations": [
    {
      "first": 2086,
      "last": 2090,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L2086–2090.

## 68 `[GO]` Gear A bore -R marker join

Join this extruded marker as the sole tool to the current cageBody. Set JoinFeatureOperation and isKeepToolBodies=False. Require one combine feature body and retain it as cageBody for the next marker. The proof targets the real uncut annular sleeve, as the spec explicitly prescribes for marker checks. [SCREW-F-JOIN] [PB-EMPTY-RESULT].

The proof function is `stepEntry68GearAboreRmarkerjoin`.

<!-- proof-run: proofkit3d.RunSolid(markerSolidCases, stepEntry68GearAboreRmarkerjoin, assertMarkerJoin) -->

Make these required Fusion calls:

- `adsk.core.ObjectCollection.create()`.
- `tools.add(markBody)`.
- `design.features.combineFeatures.createInput(cageBody, tools)`.
- `design.features.combineFeatures.add(combineInput)`.
- `combineFeature.bodies.item(0)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

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
      "span": "tools.add(markBody)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.createInput(cageBody, tools)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.add(combineInput)"
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
      "first": 2090,
      "last": 2099,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L2090–2099.

## 69 `[GO]` Gear A bore +R circle marker sketch

For a '+R' mark, draw one circle of radius 'halfSize', fix its centre point, and dimension its
diameter to '2*halfSize'. For a '-R' mark, draw four lines joining four fixed sketch points in
counter-clockwise order. Their world coordinates are the centre plus
'(-halfSize,-halfSize)', '(halfSize,-halfSize)', '(halfSize,halfSize)', and
'(-halfSize,halfSize)' in the '(eHat,kHat)' basis. Do not draw a circle and a square in the same
sketch. Each sketch must be fully constrained and have exactly one closed profile; raise with
its name and the observed profile count or constraint status otherwise. For every accepted
'collarHalf', each profile lies strictly inside the annular top face: the square corner's radial
offset from its centre is at most 'sqrt(2)*halfSize < collarHalf', and the circle is smaller.


Draw only this bore's mark. Its name is Gear A Bore +R Circle Marker. Require one closed profile, constrained geometry, and zero mapped z for every planar point. [PB-CIRCLE-CENTER] [PB-RADIAL-DIM] [PB-SHARE-XOR-COINCIDENT] [PB-SINGLE-PROFILE].

The proof function is `stepEntry69GearAboreRcirclemarkersketch`.

<!-- proof-run: proofkit.Run(markerCases, stepEntry69GearAboreRcirclemarkersketch) -->

Make these required Fusion calls:

- `design.sketches.add(plane)`.
- `adsk.core.Point3D.create(x, y, z)`.
- `sketch.modelToSketchSpace(worldPoint)`.
- `sketch.sketchCurves.sketchCircles.addByCenterRadius(centre, radius)`.
- `sketch.sketchDimensions.addDiameterDimension(circle, textPoint)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "design.sketches",
      "role": "required",
      "span": "design.sketches.add(plane)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "adsk.core.Point3D.create(x, y, z)"
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
      "span": "sketch.sketchCurves.sketchCircles.addByCenterRadius(centre, radius)"
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
      "first": 2074,
      "last": 2084,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L2074–2084.

## 70 `[GO]` Gear A bore +R marker extrude

Extrude this marker profile as NewBodyFeatureOperation by inset+0.4 mm. Choose its direction from the mapped mark-centre-plus-nHat, positive or negative local z. Require exactly one feature body; the mark starts inside the sleeve and rises 0.4 mm above the top end. [PB-THROUGH-CUT] [PB-EMPTY-RESULT] [SCREW-F-BORE-MARKS].

The proof function is `stepEntry70GearAboreRmarkerextrude`.

<!-- proof-run: proofkit3d.RunSolid(markerSolidCases, stepEntry70GearAboreRmarkerextrude, assertMarkerExtrude) -->

Make these required Fusion calls:

- `design.features.extrudeFeatures.createInput(profile, operation)`.
- `adsk.core.ValueInput.createByReal(value)`.
- `adsk.fusion.DistanceExtentDefinition.create(distanceValue)`.
- `extrudeInput.setOneSideExtent(extent, direction)`.
- `design.features.extrudeFeatures.add(extrudeInput)`.
- `extrudeFeature.bodies.item(0)`.
- `sketch.modelToSketchSpace(markCentrePlusNormal)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "design.features.extrudeFeatures",
      "role": "required",
      "span": "design.features.extrudeFeatures.createInput(profile, operation)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(value)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.fusion.DistanceExtentDefinition",
      "reason": null,
      "receiver": "adsk.fusion.DistanceExtentDefinition",
      "role": "required",
      "span": "adsk.fusion.DistanceExtentDefinition.create(distanceValue)"
    },
    {
      "condition": null,
      "name": "setOneSideExtent",
      "owner": "adsk.fusion.ExtrudeFeatureInput",
      "reason": null,
      "receiver": "extrudeInput",
      "role": "required",
      "span": "extrudeInput.setOneSideExtent(extent, direction)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "design.features.extrudeFeatures",
      "role": "required",
      "span": "design.features.extrudeFeatures.add(extrudeInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "extrudeFeature.bodies",
      "role": "required",
      "span": "extrudeFeature.bodies.item(0)"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.modelToSketchSpace(markCentrePlusNormal)"
    }
  ],
  "citations": [
    {
      "first": 2086,
      "last": 2090,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L2086–2090.

## 71 `[GO]` Gear A bore +R marker join

Join this extruded marker as the sole tool to the current cageBody. Set JoinFeatureOperation and isKeepToolBodies=False. Require one combine feature body and retain it as cageBody for the next marker. The proof targets the real uncut annular sleeve, as the spec explicitly prescribes for marker checks. [SCREW-F-JOIN] [PB-EMPTY-RESULT].

The proof function is `stepEntry71GearAboreRmarkerjoin`.

<!-- proof-run: proofkit3d.RunSolid(markerSolidCases, stepEntry71GearAboreRmarkerjoin, assertMarkerJoin) -->

Make these required Fusion calls:

- `adsk.core.ObjectCollection.create()`.
- `tools.add(markBody)`.
- `design.features.combineFeatures.createInput(cageBody, tools)`.
- `design.features.combineFeatures.add(combineInput)`.
- `combineFeature.bodies.item(0)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

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
      "span": "tools.add(markBody)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.createInput(cageBody, tools)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.add(combineInput)"
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
      "first": 2090,
      "last": 2099,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L2090–2099.

## 72 `[GO]` Gear B bore -R square marker sketch

For a '+R' mark, draw one circle of radius 'halfSize', fix its centre point, and dimension its
diameter to '2*halfSize'. For a '-R' mark, draw four lines joining four fixed sketch points in
counter-clockwise order. Their world coordinates are the centre plus
'(-halfSize,-halfSize)', '(halfSize,-halfSize)', '(halfSize,halfSize)', and
'(-halfSize,halfSize)' in the '(eHat,kHat)' basis. Do not draw a circle and a square in the same
sketch. Each sketch must be fully constrained and have exactly one closed profile; raise with
its name and the observed profile count or constraint status otherwise. For every accepted
'collarHalf', each profile lies strictly inside the annular top face: the square corner's radial
offset from its centre is at most 'sqrt(2)*halfSize < collarHalf', and the circle is smaller.


Draw only this bore's mark. Its name is Gear B Bore -R Square Marker. Require one closed profile, constrained geometry, and zero mapped z for every planar point. [PB-CIRCLE-CENTER] [PB-RADIAL-DIM] [PB-SHARE-XOR-COINCIDENT] [PB-SINGLE-PROFILE].

The proof function is `stepEntry72GearBboreRsquaremarkersketch`.

<!-- proof-run: proofkit.Run(markerCases, stepEntry72GearBboreRsquaremarkersketch) -->

Make these required Fusion calls:

- `design.sketches.add(plane)`.
- `adsk.core.Point3D.create(x, y, z)`.
- `sketch.modelToSketchSpace(worldPoint)`.
- `sketch.sketchPoints.add(local)`.
- `sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "design.sketches",
      "role": "required",
      "span": "design.sketches.add(plane)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "adsk.core.Point3D.create(x, y, z)"
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
      "span": "sketch.sketchPoints.add(local)"
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
      "first": 2074,
      "last": 2084,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L2074–2084.

## 73 `[GO]` Gear B bore -R marker extrude

Extrude this marker profile as NewBodyFeatureOperation by inset+0.4 mm. Choose its direction from the mapped mark-centre-plus-nHat, positive or negative local z. Require exactly one feature body; the mark starts inside the sleeve and rises 0.4 mm above the top end. [PB-THROUGH-CUT] [PB-EMPTY-RESULT] [SCREW-F-BORE-MARKS].

The proof function is `stepEntry73GearBboreRmarkerextrude`.

<!-- proof-run: proofkit3d.RunSolid(markerSolidCases, stepEntry73GearBboreRmarkerextrude, assertMarkerExtrude) -->

Make these required Fusion calls:

- `design.features.extrudeFeatures.createInput(profile, operation)`.
- `adsk.core.ValueInput.createByReal(value)`.
- `adsk.fusion.DistanceExtentDefinition.create(distanceValue)`.
- `extrudeInput.setOneSideExtent(extent, direction)`.
- `design.features.extrudeFeatures.add(extrudeInput)`.
- `extrudeFeature.bodies.item(0)`.
- `sketch.modelToSketchSpace(markCentrePlusNormal)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "design.features.extrudeFeatures",
      "role": "required",
      "span": "design.features.extrudeFeatures.createInput(profile, operation)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(value)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.fusion.DistanceExtentDefinition",
      "reason": null,
      "receiver": "adsk.fusion.DistanceExtentDefinition",
      "role": "required",
      "span": "adsk.fusion.DistanceExtentDefinition.create(distanceValue)"
    },
    {
      "condition": null,
      "name": "setOneSideExtent",
      "owner": "adsk.fusion.ExtrudeFeatureInput",
      "reason": null,
      "receiver": "extrudeInput",
      "role": "required",
      "span": "extrudeInput.setOneSideExtent(extent, direction)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "design.features.extrudeFeatures",
      "role": "required",
      "span": "design.features.extrudeFeatures.add(extrudeInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "extrudeFeature.bodies",
      "role": "required",
      "span": "extrudeFeature.bodies.item(0)"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.modelToSketchSpace(markCentrePlusNormal)"
    }
  ],
  "citations": [
    {
      "first": 2086,
      "last": 2090,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L2086–2090.

## 74 `[GO]` Gear B bore -R marker join

Join this extruded marker as the sole tool to the current cageBody. Set JoinFeatureOperation and isKeepToolBodies=False. Require one combine feature body and retain it as cageBody for the next marker. The proof targets the real uncut annular sleeve, as the spec explicitly prescribes for marker checks. [SCREW-F-JOIN] [PB-EMPTY-RESULT].

The proof function is `stepEntry74GearBboreRmarkerjoin`.

<!-- proof-run: proofkit3d.RunSolid(markerSolidCases, stepEntry74GearBboreRmarkerjoin, assertMarkerJoin) -->

Make these required Fusion calls:

- `adsk.core.ObjectCollection.create()`.
- `tools.add(markBody)`.
- `design.features.combineFeatures.createInput(cageBody, tools)`.
- `design.features.combineFeatures.add(combineInput)`.
- `combineFeature.bodies.item(0)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

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
      "span": "tools.add(markBody)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.createInput(cageBody, tools)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.add(combineInput)"
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
      "first": 2090,
      "last": 2099,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L2090–2099.

## 75 `[GO]` Gear B bore +R circle marker sketch

For a '+R' mark, draw one circle of radius 'halfSize', fix its centre point, and dimension its
diameter to '2*halfSize'. For a '-R' mark, draw four lines joining four fixed sketch points in
counter-clockwise order. Their world coordinates are the centre plus
'(-halfSize,-halfSize)', '(halfSize,-halfSize)', '(halfSize,halfSize)', and
'(-halfSize,halfSize)' in the '(eHat,kHat)' basis. Do not draw a circle and a square in the same
sketch. Each sketch must be fully constrained and have exactly one closed profile; raise with
its name and the observed profile count or constraint status otherwise. For every accepted
'collarHalf', each profile lies strictly inside the annular top face: the square corner's radial
offset from its centre is at most 'sqrt(2)*halfSize < collarHalf', and the circle is smaller.


Draw only this bore's mark. Its name is Gear B Bore +R Circle Marker. Require one closed profile, constrained geometry, and zero mapped z for every planar point. [PB-CIRCLE-CENTER] [PB-RADIAL-DIM] [PB-SHARE-XOR-COINCIDENT] [PB-SINGLE-PROFILE].

The proof function is `stepEntry75GearBboreRcirclemarkersketch`.

<!-- proof-run: proofkit.Run(markerCases, stepEntry75GearBboreRcirclemarkersketch) -->

Make these required Fusion calls:

- `design.sketches.add(plane)`.
- `adsk.core.Point3D.create(x, y, z)`.
- `sketch.modelToSketchSpace(worldPoint)`.
- `sketch.sketchCurves.sketchCircles.addByCenterRadius(centre, radius)`.
- `sketch.sketchDimensions.addDiameterDimension(circle, textPoint)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "design.sketches",
      "role": "required",
      "span": "design.sketches.add(plane)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "adsk.core.Point3D.create(x, y, z)"
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
      "span": "sketch.sketchCurves.sketchCircles.addByCenterRadius(centre, radius)"
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
      "first": 2074,
      "last": 2084,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L2074–2084.

## 76 `[GO]` Gear B bore +R marker extrude

Extrude this marker profile as NewBodyFeatureOperation by inset+0.4 mm. Choose its direction from the mapped mark-centre-plus-nHat, positive or negative local z. Require exactly one feature body; the mark starts inside the sleeve and rises 0.4 mm above the top end. [PB-THROUGH-CUT] [PB-EMPTY-RESULT] [SCREW-F-BORE-MARKS].

The proof function is `stepEntry76GearBboreRmarkerextrude`.

<!-- proof-run: proofkit3d.RunSolid(markerSolidCases, stepEntry76GearBboreRmarkerextrude, assertMarkerExtrude) -->

Make these required Fusion calls:

- `design.features.extrudeFeatures.createInput(profile, operation)`.
- `adsk.core.ValueInput.createByReal(value)`.
- `adsk.fusion.DistanceExtentDefinition.create(distanceValue)`.
- `extrudeInput.setOneSideExtent(extent, direction)`.
- `design.features.extrudeFeatures.add(extrudeInput)`.
- `extrudeFeature.bodies.item(0)`.
- `sketch.modelToSketchSpace(markCentrePlusNormal)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "design.features.extrudeFeatures",
      "role": "required",
      "span": "design.features.extrudeFeatures.createInput(profile, operation)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(value)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.fusion.DistanceExtentDefinition",
      "reason": null,
      "receiver": "adsk.fusion.DistanceExtentDefinition",
      "role": "required",
      "span": "adsk.fusion.DistanceExtentDefinition.create(distanceValue)"
    },
    {
      "condition": null,
      "name": "setOneSideExtent",
      "owner": "adsk.fusion.ExtrudeFeatureInput",
      "reason": null,
      "receiver": "extrudeInput",
      "role": "required",
      "span": "extrudeInput.setOneSideExtent(extent, direction)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "design.features.extrudeFeatures",
      "role": "required",
      "span": "design.features.extrudeFeatures.add(extrudeInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "extrudeFeature.bodies",
      "role": "required",
      "span": "extrudeFeature.bodies.item(0)"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.modelToSketchSpace(markCentrePlusNormal)"
    }
  ],
  "citations": [
    {
      "first": 2086,
      "last": 2090,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L2086–2090.

## 77 `[GO]` Gear B bore +R marker join

Join this extruded marker as the sole tool to the current cageBody. Set JoinFeatureOperation and isKeepToolBodies=False. Require one combine feature body and retain it as cageBody for the next marker. The proof targets the real uncut annular sleeve, as the spec explicitly prescribes for marker checks. [SCREW-F-JOIN] [PB-EMPTY-RESULT].

The proof function is `stepEntry77GearBboreRmarkerjoin`.

<!-- proof-run: proofkit3d.RunSolid(markerSolidCases, stepEntry77GearBboreRmarkerjoin, assertMarkerJoin) -->

Make these required Fusion calls:

- `adsk.core.ObjectCollection.create()`.
- `tools.add(markBody)`.
- `design.features.combineFeatures.createInput(cageBody, tools)`.
- `design.features.combineFeatures.add(combineInput)`.
- `combineFeature.bodies.item(0)`.

The supported proof substitution is explicit:

The proof substitutes a straight prism with the exact pitch-averaged section area and actual axial span. Copy and
screw placement operate on that real prism. For a refused tangent join, the proof extrudes the combined span
directly and checks its summed volume and connected solid verdict. This gives up the twisted tooth surface and
Fusion combine behavior.

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
      "span": "tools.add(markBody)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.createInput(cageBody, tools)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "design.features.combineFeatures.add(combineInput)"
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
      "first": 2090,
      "last": 2099,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L2090–2099.

## 78 `[PROSE]` Relocate Gear A

Name this body 'Gear A' and move it into its own output occurrence; preserve its world position. After the last relocation call the shared solids.hide_construction_geometry helper on self.designOcc.component. Log exactly 'Print the cage standing on its end below the selected plane: the roof allowance is on the bridged roofs that way up.' [PB-NO-CROSS-SIBLING] [PB-TREE-CLEANUP] [PB-LOGGING].

Make these required Fusion calls:

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
      "first": 1662,
      "last": 1668,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1662–1668.

## 79 `[PROSE]` Relocate Gear B

Name this body 'Gear B' and move it into its own output occurrence; preserve its world position. After the last relocation call the shared solids.hide_construction_geometry helper on self.designOcc.component. Log exactly 'Print the cage standing on its end below the selected plane: the roof allowance is on the bridged roofs that way up.' [PB-NO-CROSS-SIBLING] [PB-TREE-CLEANUP] [PB-LOGGING].

Make these required Fusion calls:

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
      "first": 1662,
      "last": 1668,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1662–1668.

## 80 `[PROSE]` Relocate Cage

Name this body 'Cage' and move it into its own output occurrence; preserve its world position. After the last relocation call the shared solids.hide_construction_geometry helper on self.designOcc.component. Log exactly 'Print the cage standing on its end below the selected plane: the roof allowance is on the bridged roofs that way up.' [PB-NO-CROSS-SIBLING] [PB-TREE-CLEANUP] [PB-LOGGING].

Make these required Fusion calls:

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
      "first": 1662,
      "last": 1668,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1662–1668.
