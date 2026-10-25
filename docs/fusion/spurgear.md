# Spur family: Fusion notes

This file covers the spur, helical and herringbone gears. `gearport` generates `lib/geargen/spurgear.py`,
`helicalgear.py` and `herringbonegear.py` from the gallery models `models/spurgear`, `models/helicalgear` and
`models/herringbonegear`. The gallery owns their geometry and its checks. This file records what Fusion needs that
the gallery does not say. Rules that hold for every gear are in `playbook.md` (`[PB-…]`).

Several entries describe `lib/geargen/decadfusion.py`, the runtime every translated gear calls. They live here
because the spur was the first gear translated, and the runtime cites them by these anchors.

## How the translated build maps to Fusion

### [SPUR-F-GALLERY-SKETCH] One gallery sketch is one Fusion sketch

`port/go2py` translates the gallery's build functions into `spurgear.py`. They call `decad_extrude`,
`decad_union` and the other operations of `decadfusion.py`. Each gallery sketch becomes one Fusion sketch
(`Tooth Profile`, `Root Disc Profile`, `Bore Profile`). Each gallery operation becomes one timeline feature
(`Tooth Extrude`, `Root Disc Extrude`, `Tooth Pattern`, `Teeth Join`, `Root Fillet`, `Bore Tool Extrude`,
`Bore Cut`, `Cap Chamfer`). `port/spur/mapping.go` holds these role names.

`decadfusion.Sketch` records the gallery's calls and draws them in `solve()`, in this order:

1. Splines first, so that every line and arc ends on a spline's own fit point. A curve drawn from a raw `Point3D` at
   the same coordinates gets a fresh point, and the earlier add-in recorded that the tooth loop was then not found
   as a closed profile.
2. Lines, then arcs.
3. Dimensions from the local origin. A point gets a horizontal and a vertical distance dimension. A point the
   gallery marks as an arc end gets one, vertical when |x| >= |y|, because the arc already fixes its radius.
   Circles get their radius or diameter dimension.
4. The coincident constraint that moves the local origin onto the anchor (`[SPUR-F-LOCAL-ORIGIN]`).

The tooth angle is part of the sampled points, so the sketch has no spine, ribs or angular dimension. That older
construction is gone.

### [SPUR-F-LOCAL-ORIGIN] Each sketch has its own movable origin

Each translated sketch adds a point at (0, 0, 0) and measures every dimension from it. It does not use
`sketch.originPoint`, which a constraint cannot move. The coincident constraint between this point and the
projected anchor comes last, so Fusion moves the finished sketch onto the anchor in one step. A sketch with a circular
pattern of splines is the exception: its anchor goes in before the pattern ([PB-SPLINE-RING]).

### [SPUR-F-ANCHOR-CHAIN] Every sketch projects the Tools sketch's anchor

The `Tools` sketch projects the user's anchor (`sketch.project(anchor).item(0)`) and must be fully constrained.
`Runtime` receives that projected point, and every later sketch, `Reference Circles` included, projects it
again. That works because the Tools sketch holds no curves: Fusion refuses to project a point from a sketch that
has gained curves and constraints ([PB-BAD-ENTITY]).

### [SPUR-F-LOCAL-FRAME] The translated code builds in the gallery's local frame

The gallery builds a gear about the local origin and +Z and moves it onto its placement at the end. The translated
spur, helical and herringbone code does the same: the frame it receives is the identity, a position it reads from an
edge (`Position.Value`) is local, and a position it hands back (a pattern axis, a plane's frame, an edge end point)
is local too. `Runtime.build_frame` records where the gear lies (the anchor, the Tools sketch's axes and their
normal), and `to_local`, `to_world` and `to_world_dir` convert at each of those places. The helical root fillet
tests a root edge by its Z component, so it only works because Z is the gear axis.

### [SPUR-F-RUNTIME-RULES] What `decadfusion.py` decides, because the gallery does not

- The extrude distances, the bore tool length and the fillet and chamfer sizes are passed as the user parameters'
  names (`ValueInput.createByString`). A radius or diameter dimension the gallery binds to a table entry gets that
  parameter as its expression.
- The tooth's point dimensions, the pattern's tooth count, the offsets of construction planes and the
  `Reference Circles` dimensions are numbers taken at generation time. Changing Module or Tooth Number therefore
  needs a new gear. No Fusion run records whether an edit to a bound parameter updates a translated gear;
  [PB-NUMERIC-SNAPSHOT] says such links did not update reliably in the old add-in.
- An axial edge starts at its end on the lower cap, as in decad.
- The pattern axis is a construction axis made with `setByCircularFace` from a cylindrical face that lies on the
  axis. For the spur family the root disc provides that face, because it is extruded before the pattern. When no
  such face exists, a construction line in a Fusion-only sketch serves as the axis. A face-less axis made with
  `setByLine` was refused ([CYCLOIDAL-F-DISK-AXIS]).
- The cap faces are the planar faces perpendicular to the axis at its lowest and highest position.
- The lower cap's frame comes from the Tools sketch: its origin is the anchor and its axes are the sketch's.
- The root, tip, base and pitch circles and their labels are not in the gallery. `drawReferenceCircles` draws them in
  a `Reference Circles` sketch of their own.

These rules had not been verified in Fusion as of 2026-10-24.

## Sketch workarounds

### [SPUR-F-ARC-CENTRE] An arc's centre gets a coincident constraint to the local origin

`addByCenterStartEnd` shares the start and end points but makes a new centre point, even when the point passed is
the local origin. [PB-SHARE-XOR-COINCIDENT] records this behaviour and the bevel arc that collapsed because of it.
`Sketch._drawArcs` therefore adds `addCoincident(arc.centerSketchPoint, localOrigin)` after each arc. The arc gets no
diameter dimension. A free centre with a diameter fixes the arc's size but not which way it bulges: an arc of the
same radius through the same two ends can curve back through the tooth, and the sketch then has two solutions at
zero degrees of freedom. Verified in Fusion on 2026-10-24.

### [SPUR-F-ALIGN-DUPLICATE] Align a point Fusion refuses to dimension with the earlier point at the same coordinate

At tooth angle 0 and 90 degrees the two flanks are mirror images. In the probes, Fusion raised
`VCS_SKETCH_OVER_CONSTRAINTS` on one horizontal (or vertical) dimension, the first point of the second flank, and
accepted all the others. A vertical (or horizontal) alignment of the two points (`addVerticalPoints`,
`addHorizontalPoints`, not `addVertical`) fixes the same coordinate and leaves the sketch fully constrained. The
cause is not understood. `decadfusion.Sketch._dimension` therefore applies the alignment to a dimension refused with
`VCS_SKETCH_OVER_CONSTRAINTS` that has an earlier dimensioned point whose coordinate matches within `SAME_COORDINATE`.
It raises any other refusal, and raises the refused dimension when Fusion refuses the alignment too.

### [SPUR-F-ROOT-ARC] A root arc on a root circle the caller drew

When a caller passes its own root circle into `decadfusion.Sketch`, an arc whose two ends lie on that circle is
not drawn. Its ends get a coincident constraint to the circle instead, and each end then needs only one dimension.
A second arc over the circle would split it and change the profile's curves. This was the old construction's rule. The
translated spur draws its root arc as an ordinary arc, and the runtime branch that applied the rule was deleted with
`legacyspur.py` on 2026-10-25.

## Helical and herringbone

The helical and herringbone add-ins run the spur's translated build with a variant. `HelicalGearGenerator.variant`
returns `_Variant(Tooth=loft_tooth, RootFillet=_fillet_root_edges)`, and the herringbone passes `_chevron` with the
same fillet. The gallery owns the tooth, the twist and the fillet radius `fillet_radius * cos(helix_angle)`.

- The twisted section lies on a construction plane, `Twisted Section Plane`, offset from the Target Plane by
  `HelixPlaneOffset` (Thickness for helical, Thickness / 2 for herringbone). The offset is a number, so editing
  Thickness does not move the plane. The runtime refuses a plane that is neither parallel nor perpendicular to the
  Target Plane.
- The loft (`Twisted Tooth Loft`) adds the lower section first, then the upper. The upper section's points are
  drawn already turned by the helix angle.
- Both sections come from `build_tooth_section`, which raises `tooth section: embedded flanks are not supported`
  when the base circle lies inside the root circle. With the root radius `(m z - 2.5 m) / 2`, that happens above
  `2.5 / (1 - cos(PressureAngle))` teeth: 41.5 at 20 degrees, 78.5 at 14.5 degrees and 26.7 at 25 degrees. The spur
  builds those gears with its embedded tooth.
- The herringbone mirrors the lofted half with a Mirror feature (`Tooth Half Mirror`). The runtime makes that
  feature's mirror plane itself (`Tooth Half Mirror Plane`), so two construction planes sit at Thickness / 2. The
  mirror does not combine. A separate join, `Tooth Halves Join`, is made when the pattern first needs the tooth
  ([PB-LAZY-JOIN]).

## Visibility after a build

After a full build, `cleanup` hides the Tools and Reference Circles sketches, and `Runtime.hide_helpers` hides
every sketch the runtime drew and turns off every construction plane it made. In sketch-only mode the build stops
at the first feature (`Tooth Extrude`, or the loft for helical and herringbone). The sketches drawn so far and the
`Twisted Section Plane` stay visible. Only a Target Plane that the add-in created is turned off.
