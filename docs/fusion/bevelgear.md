# Bevel gear: Fusion notes

This file records how the bevel add-in puts the gallery's pair into Fusion, and what Fusion has done with it. The
decad-gallery model `models/bevelgear` owns the geometry and the input rules. `port/bevel/mapping.go` maps the
gallery's parameters to dialog inputs and Fusion names, and `port/bevel/fragments/bevelgear.py.frag` holds the dialog
and the placement code. Rules that hold for every gear are in `playbook.md`. `bevelgear-spiral-tooth-trace.md` derives
the spiral tooth, and `bevelgear-design-record.md` keeps the reasons behind the input rules and the Fusion records of
the previous add-in.

## What the add-in builds

`gearport gen bevelgear` translates the gallery's `BuildPair` and everything it calls. The fragment adds the dialog,
the placement and the cleanup around one call of the translated `build_pair`.

- **Dialog.** The inputs and their defaults come from the gallery's `Params` and `DefaultParams`, in the order of
  `Inputs` in `mapping.go`. Pressure Angle (20°) and Samples (15) have no input. The Tooth Type drop-down is Fusion's
  own: Straight is the default and the gallery's default is spiral. Straight hides Mean Spiral Angle, Hand of Spiral
  and Cutter Radius and passes a spiral angle of 0, so the gallery builds straight members. Spiral shows them, and a
  spiral angle of 0 then stops the build with "Mean Spiral Angle must be above 0 for spiral teeth".
- **Units and parameters.** The translated code runs in millimetres, and the runtime converts every length to Fusion's
  centimetres (`MILLIMETRE = 0.1`). The pair makes
  no Fusion user parameters; the gallery's parameter table is evaluated in Python ([PB-PARAM-TABLE]), and a request
  for a Fusion parameter raises.
- **Placement** ([PB-PAIR-PLANE], [PB-FRAME-PLANE]). The driving gear's back face lies on the Target Plane with its
  axis through the Center Point, and the apex is above the point by the driving blank's heel station. A planar face is
  first turned into a construction plane at offset 0 (`Pair Base Plane`). The `Tools` sketch on the Target Plane holds
  the projected Center Point and a 10 mm line through it along the sketch's X axis, and the build stops if that sketch
  is not fully constrained. `Pair Plane` stands on that line at 90° to the Target Plane, or at −90° when the pinion
  would otherwise lie toward the lesser X. The `Pair Tools` sketch on it holds the apex, dimensioned horizontally and
  vertically from the projected Center Point; where one of the two distances is zero, Fusion refuses the dimension and
  the fragment aligns the points instead. The gallery's own `Placement` is not used. The user chose this placement on
  2026-10-25 after rejecting two builds that put the apex on the Center Point (`docs/gallery-translation-goal.md`).
- **Components.** Everything is built in one component, `Bevel Gear Pair (<driving>:<pinion>)`, and no occurrence is
  activated. (`Runtime.activate` only makes the runtime the module's current one.) At the end `finish_containers`
  makes the gallery's containers as components, `Pair` holding `Driving Gear` and `Pinion Gear`, and moves the two
  bodies in ([PB-CONTAINER-BODIES]). The bodies are named `Driving Gear` and `Pinion Gear`.
- **Sketches.** Every sketch the translated code makes goes through `sketchcheck_verify_fully_constrained`, which raises
  and names the free curves when Fusion reports the sketch is not fully constrained. That includes `Gear Profiles`,
  each `Tooth Profile`, `Bore Profile` and the spiral's `Slice Sheet` sketches. No sketch is exempt. The previous
  add-in exempted its labelled tooth sketches and its spiral trace sketches; the current build draws neither.
- **Straight member.** The features are `Tooth Loft` (from the apex point to the `Tooth Profile`), `Tooth Toe Trim`,
  `Tooth Heel Trim`, one circular `Tooth Pattern`, and one `Teeth Join` combine for all the teeth ([PB-LAZY-JOIN]).
  The bore is `Bore Plane`, `Bore Profile`, `Bore Extrude` and `Bore Cut`. `Driving Gear Phase Turn` turns the driving
  gear by half a tooth pitch, and `Pinion Tooth Phase Turn` turns the pinion by half a pitch when its tooth count is
  odd.
- **Spiral member.** The gallery's decad-only steps map to Fusion as follows. `Split` is a surface extrude of the slice
  line (`Slice Sheet Extrude`) and a Split Body feature, after which the sheet body is removed ([PB-SPLIT-TOOL]).
  `ScaledAt` is a Scale feature about a sketch point on a face through the anchor ([PB-SCALE-BASE-POINT]). `LoftFaces`
  is a loft through the slabs' planar faces. The rotated copies of the tooth become one circular pattern, and the
  gallery's tooth-by-tooth unions become one combine ([PB-LAZY-JOIN]; `tests/test_bevel_batching.py` counts both). On
  an equal pair the driving gear is a mirrored copy of the pinion (`Spiral Driving Gear Mirror`).
- **One try.** The gallery retries a spiral member with other overshoots and join orders, and a slice with a longer
  sheet, when decad refuses a boolean. The translation runs each loop once with its first entry: an overshoot of 0.5
  module, the first join order, and a sheet two cone distances long (`FirstOnly` in `mapping.go`). The goal document
  gives the reason.
- **Cleanup.** The runtime hides its helper planes and sketches, the fragment hides `Tools`, `Pair Tools` and
  `Pair Plane`, and `solids.hide_construction_geometry` hides what is left in the component.

## What Fusion has shown

The straight pair is Fusion-verified. The first loads of the translated build found the behaviours recorded as
[PB-PROFILE-STALE], [PB-BAD-ENTITY], [PB-POINT-POSITION], [PB-STALE-MODULE] and [PB-PROGRESS-FILE]. [PB-NO-GRID-SNAP]
records why the gallery's grid snap of the pinion's sections is not applied: with it, the pinion came out with 31 extra
cone faces.

The default spiral pair (31 and 31 teeth, 35°, right hand) also builds and looks right in Fusion, with no Scale feature
needing an active occurrence. Its first loads found [PB-SCALE-BASE-POINT] (a construction point at a bare point fails in a
parametric design), [PB-SPLIT-TOOL] (the split sheets stay in the design and show as a plate) and two bookkeeping errors in
the batched join that the straight pair also hit ([PB-LAZY-JOIN]).

## Not yet seen in Fusion

- **Other spiral inputs.** Only the default spiral pair has been loaded. A different hand, cutter radius, spiral angle,
  tooth count or shaft angle is untested, and a slice plane that misses a tooth is not handled the way the gallery
  handles it.
- **A planar face as the Target Plane.** [PB-USE-SELECTED-PLANE] records that the previous add-in lost the selected
  plane's orientation when it rebuilt a face as an offset construction plane inside an activated sub-component. The
  fragment makes `Pair Base Plane` the same way, in a component that is not activated. No record here says a planar
  face has been tried with the current build.
