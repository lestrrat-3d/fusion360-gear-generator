# Bevel Gear Creation Instructions

## Component Setup

Bevel gears are inherently created in pairs. As such, we shall have a single component containing three components

The first two components are for the driving gear and pinion gear. Because the designs are shared between the two components, the third one will be the design component.

## Variables

User inputs are listed below in the order they appear in the command dialog. Target Plane comes first so it receives keyboard/pick focus when the dialog opens (Fusion auto-focuses the first `SelectionCommandInput` and does not respect a later `hasFocus = True` override); Center Point follows so the user flows naturally from plane to point; Parent Component comes third since it is already pre-selected to the root component. Calculated values are listed after the inputs they depend on.

Target Plane: user-specified plane. This is where the bottom of the driving gear will sit flush against.

Center Point: user-specified point. This is where the driving bevel gear will be centered on. Does not need to be co-planar with target plane.

Parent Component: user-specified component. Defaults to the root component (pre-selected).

Module: user-supplied number. Specifies the module of gears.

Shaft Angle: User-supplied angle in degrees between 30° and 150°. Default 90° (perpendicular shafts — the classic bevel pair). The input is entered as a Fusion expression with a `deg` unit (e.g., `60 deg`); when read back via `UnitsManager.evaluateExpression(..., 'deg')` Fusion returns the value in its internal angle unit (radians), so validate-and-use code must convert back to degrees (`math.degrees(...)`) before comparing against the 30–150 range.

Driving Gear Teeth Number: user-specified number of teeth on the driving gear. Default is 31.

Pinion Gear Teeth Number: user-specified number of teeth on the pinion gear. Default is 31.

Driving Gear Pitch Diameter: calculated number. Module * Driving Gear Teeth Number.

Pinion Gear Pitch Diameter: calculated number. Module * Pinion Gear Teeth Number.

Driving Gear Base Height: user-specified positive number, default 0mm (i.e. unspecified).

Pinion Gear Base Height: user-specified positive number, default 0mm (i.e. unspecified).

Enable Bore: user-specified boolean, default `true`. Applies to both gears. When unchecked, no bore is cut on either gear and the per-gear bore diameter inputs below are ignored.

Driving Gear Bore Diameter: user-specified positive number, default 0mm. Only consulted when Enable Bore is checked. A value of 0 means "auto-calculate" — use `Driving Gear Pitch Diameter / 4` as the bore diameter.

Pinion Gear Bore Diameter: user-specified positive number, default 0mm. Only consulted when Enable Bore is checked. A value of 0 means "auto-calculate" — use `Pinion Gear Pitch Diameter / 4` as the bore diameter.

Face Width: User-specified positive number. If unspecified, calculate from (Cone Distance / 6).

Cone Distance: calculated number. `sqrt((Module * Driving Gear Teeth Number)**2 + (Module * Pinion Gear Teeth Number)**2)`.

## Instructions

### Create the Parent Component

Create the Bevel Gear component as a child of the Parent Component.

## Creating the Design Component

The Design Component shall contain the necessary sketches and construction planes / axis / etc that will be used by the components creating the actual gears later.

Create the Design component as a child of the Bevel Gear component.

### 1: Anchor Sketch

Start a sketch on the target plane. Mark the center point on the target plane by projecting the user-specified center point onto the sketch.

Create a line that intersects with the projected center point by applying the intersection constraint. Use the mid-point constraint to make the center point divide the line in half. This line shall only be used as a reference, and therefore its dimension does not really matter. Pick a number, say 10mm, and apply it to the line. This line shall be known as the Anchor Line.

### 2: Gear Profiles

Using ConstructionPlaneInput.setByAngle, create a plane that includes the Anchor Line, and set it at 90 degrees (as by default it would lie flush to the plane of the anchor line, but we want it perpendicular). Create a sketch on this plane.

In the sketch, project the center point from the Anchor Sketch.

From the projected center point, draw a construction line that runs along the direction of the target plane's normal (away from the target plane, on the side the normal points to). Apply Horizontal/Vertical constraint to it so it is perpendicular to the anchor line. The end of this line should be well above the Anchor line; Use x,y coordinates where x is the same as the center point, but y is shifted upwards the same amount as Driving Gear Pitch Diameter (however, do NOT use constraints). This end point shall be called the Apex.

Create a construction line from the apex representing the Driving Gear Shaft Axis. Apply a vertical constraint so it runs parallel to the previous (target-plane-normal) line, pointing downward from the apex toward the anchor line. Beginning of this line should use coincidence constraint with apex. The end of this line shall be called point B. Do **not** dimension the line's length — it is determined by the closing constraint at Apex 2 below.

Create a construction line from the apex representing the Pinion Gear Shaft Axis. Apply an angular dimension between this line and the Driving Gear Shaft Axis equal to Shaft Angle (this is the traditional "angle between the two shaft axes" — Shaft Angle = 90° gives the classic perpendicular bevel pair). The pinion shaft is drawn on the side away from the anchor sketch's leading direction, typically the +X half-plane of the Gear Profiles sketch. Beginning of this line should use coincidence constraint with apex. The end of this line shall be called point A. Do **not** dimension the line's length — it is determined by the closing constraint at Apex 2 below.

From A, create a construction line perpendicular to the Pinion Gear Shaft Axis, drawn toward the side where Apex 2 will lie (between the two shaft axes, in the direction of the anchor line). Apply a perpendicular constraint against the Pinion Gear Shaft Axis. Apply a dimensional constraint with length = Pinion Gear Pitch Diameter / 2 (this equals the pinion's pitch radius at the heel, which is the perpendicular distance from Apex 2 to the Pinion Gear Shaft Axis for any Shaft Angle). Beginning of this line should use coincidence constraint with A.

From B, create a construction line perpendicular to the Driving Gear Shaft Axis, drawn toward the side where Apex 2 will lie. Apply a perpendicular constraint against the Driving Gear Shaft Axis. Apply a dimensional constraint with length = Driving Gear Pitch Diameter / 2 (the driving pitch radius at the heel, perpendicular distance from Apex 2 to the Driving Gear Shaft Axis for any Shaft Angle). Beginning of this line should use coincidence constraint with B.

Constrain the end points of the two perpendicular lines from the previous two paragraphs with a coincident constraint. Let this point be called Apex 2. (At Shaft Angle = 90° the four points Apex, A, Apex 2, B form a rectangle. For other shaft angles the figure is a non-rectangular parallelogram-like quadrilateral; the lengths of Apex→A and Apex→B adjust so the perpendicular drops of length PPD/2 and DPD/2 coincide at Apex 2.)

Note that this quadrilateral deliberately lies well above the anchor line. The Apex's upward offset from the anchor line (= Driving Gear Pitch Diameter) is chosen to keep the whole figure above the anchor line for Shaft Angle in the supported range 30°–150°.

Draw a construction line from Apex to Apex 2. This line shall be called the Pitch Line. Each end of the Pitch Line should be constrained to the respective points using coincidence constraint.

From the Apex 2, create a construction line in either side whose length is constrained Module * 1.25, and are perpendicular to the Pitch Line. Constrain them against the Pitch Line with the Perpendicular constraint. Let the line drawn towards the anchor line be Driving Gear Dedendum, whose end point shall be point D. The one drawn away from the anchor line be Pinion Gear Dedendum; whose end point shall be point C.

Draw two construction lines, from the Apex to the point D and point C, respectively. Apply coincidence constraints on beginning and end of these lines. These lines shall be the Root Axis for driving and pinion gear, respectively.

From point A, create construction line colinear with the line from Apex to point A, extending for length equals to module (but do NOT add dimensional constraint). The line should receive a colinear constraint, and the end of Apex->A and the beginning of the new line should be constrained via coincidence constraint. The end of this new line shall be point E.

Draw a construction line from point C to point E. constrain each end to respective points from pre-existing lines. The lines A->E and C->E should be constrained with perpendicular constraint.

From point B, create a construction line colinear with Apex->B, extending for length equals to module (but do NOT add dimensional constraint). The line should receive a colinear constraint, and the end of Apex->B and the beginning of the new line should be constrained via coincidence constraint. The end of this new line shall be point F.

Draw a construction line from point D to point F. constrain each end to respective points from pre-existing lines. The lines B->F and D->F should be constrained with perpendicular constraint.

Draw a construction line from point E colinear to line A->E, with length equal to module (but do NOT add dimensional constraint). Constrain point E and the beginning of this line. Let the end be known as point G.

From point C, draw a line with length equal to module (but do NOT add dimensional constraint). Constrain point C and the beginning of this line. Let the end of the new line be point H. Line C->H should be colinear with line Apex2->C.

Connect point G and H with a line. Constrain end points of line accordingly with coincidence constraints. Constrain line E->G and H->G with a perpendicular constraint.


Draw a construction line from point F colinear to line B->F, with length equal to module (but do NOT add dimensional constraint). Constrain point F and the beginning of this line. Let the end be known as point I.

From point D, draw a line with length equal to module (but do NOT add dimensional constraint). Constrain point D and the beginning of this line. Let the end of the new line be point J. Line D->J should be colinear with line Apex2->D.

Connect point I and J with a line. Constrain end points of line accordingly with coincidence constraints. Constrain line F->I and J->I with a perpendicular constraint.

Create a dimension constraint between lines Apex2->B and J->I. The value of this constraint should be equal to Driving Gear Base Height _if_ it is specified (non-0). Otherwise, compute the value as module * Driving Gear Teeth Number / 8.

Create a dimension constraint between lines Apex2->A and G->H. The value of
this constraint should be equal to Pinion Gear Base Height _if_ it is specified (non-0). Otherwise, compute the value as Driving Gear Base Height * (Pinion Gear Teeth Number / Driving Gear Teeth Number).

Draw a line from A to G. Constrain endpoints appropriately.

Constrain Point I with center point.


Draw a construction line away from Apex, starting from point G. This new line should be colinear with Apex->A. Apply coincidence constraint on beginning of this line and point G. Let the end of this new line be point K.

Draw a construction line from point C to K. Constrain both ends appropriately. C->K should be colinear with Apex2->C.

Create line starting from Apex->C up to line A->Apex2. The new line should be parallel to C->H. The line should have a dimensional constraint against C->H, with a dimension equal to Face Width.

Let the beginning of this new line to be point M, let the end of of this line be point N. Draw a line from M to C. Draw a line from N to A.

Draw a construction line away from Apex, starting from point I. This line should be colinear with Apex->B. Apply coincidence constraint on beginning of this line and point I. Let the end of this new line be point L.

Draw a construction line from point D to L. Constrain both ends appropriately. D->L should be colinear with Apex2->D.

Create line starting from Apex->D up to line B->Apex2. The new line should be parallel to D->J. The line should have a dimensional constraint against D->J, with a dimensionequal to Face Width.

Let the beginning of this new line to be point O, let the end of of this line be point P. Draw a line from O to D. Draw a line from P to B. Draw line from B to I.

### 3: Gear Tooth Profiles

Obtain the length of line Apex2->K. This is the virtual pitch radius of the pinion tooth profile; twice this length is the virtual pitch diameter. Divide the virtual pitch diameter by module to obtain the virtual tooth number; if fractional, take the floor and convert it to an integer. This shall be the virtual tooth number of the pinion tooth profile.

Create a new plane that includes line C->K. Use setByAngle to make this plane perpendicular to the Gear Profiles sketch plane.

Using the new plane and point K as the center point, create a spur gear tooth profile with module and the virtual tooth number obtained from the previous step.

Create a construction axis through point K. This axis should be normal to the plane that the tooth profile was drawn on. Rotate the tooth profile sketch 180 degrees around this new axis.

Obtain the length of line Apex2->L. This is the virtual pitch radius of the driving gear tooth profile; twice this length is the virtual pitch diameter. Divide the virtual pitch diameter by module to obtain the virtual tooth number; if fractional, take the floor and convert it to an integer. This shall be the virtual tooth number of the driving gear tooth profile.

Create a new plane that includes line D->L. Use setByAngle to make this plane perpendicular to the Gear Profiles sketch plane.

Using the new plane and point L as the center point, create a spur gear tooth profile with module and the virtual tooth number obtained from the previous step.

Create a construction axis through point L. This axis should be normal to the plane that the tooth profile was drawn on. Rotate the tooth profile sketch 180 degrees around this new axis.

## Create the Pinion Gear

Create a new component as the child of Parent Component. The resulting bodies for this gear should end up in this component. (Implementation note: Fusion's API rejects cross-sibling sketch and project calls even when the target is activated or the entities are wrapped in `createForAssemblyContext` proxies, so the actual feature operations may run in the Design component and the finished bodies be moved here at the end. The visible end state is identical.)

Create a body by revolving the closed hexagonal profile A -> G -> H -> C -> M -> N -> A around the A->G axis (the Pinion Gear Shaft Axis). Let this be Pinion Gear Body. Because M->N is one edge of the revolved profile, the resulting body already carries the conical face produced by sweeping M->N around the axis — that face is reused as the cutting tool below.

Create a loft using the Apex point and the Pinion Gear Tooth profile. Let this be the Pinion Gear Tooth Body.

Cut the Pinion Gear Tooth Body twice, in order:

1. First, cut using the conical face on Pinion Gear Body that was generated from the M->N edge. (Implementation note: Fusion's `Path.create` on a sketch line whose owner sketch is in a sibling/parent component fails with `getObjectPath(sketchCurve, ..., nullptr, contextPath)`, so building a fresh revolved surface from the M->N sketch line does not work. Instead, locate the existing M->N face on Pinion Gear Body by walking its faces and selecting the one whose underlying conical surface contains both M and N in world coordinates; use that face as the splitting tool with `isSplittingToolExtended=True`.)
2. Then, cut the resulting bodies using the conical face on Pinion Gear Body that was generated from the C->H edge. Locate it the same way: among the cone faces of Pinion Gear Body, pick the one whose underlying surface contains both C and H in world coordinates, and use it as the splitting tool with `isSplittingToolExtended=True`.

It is an error if the two cuts together fail to split the Pinion Gear Tooth Body into three pieces.

Remove the piece that contains the Apex. Identify it by `BRepBody.pointContainment(apexWorld)` returning `PointInside` or `PointOn`; remove it via `RemoveFeatures.add()` (so the deletion appears in the timeline).

Of the two remaining pieces, remove the smaller one (compare by `BRepBody.physicalProperties.volume`); remove it via `RemoveFeatures.add()`.

Circular-pattern the remaining tooth piece around the Apex->A axis (the Pinion Gear Shaft Axis). The number of copies equals the Pinion Gear Teeth Number. (Although the pitch diameter shrinks from heel toward apex, the *angular* spacing around the shaft axis stays constant at `360° / N` for the entire face width — the radial taper is already produced by the loft from Apex to the heel-end tooth profile, so the pattern just rotates that single tapered tooth into N evenly spaced copies.)

Join all patterned tooth pieces with Pinion Gear Body in a single Combine-Join (Pinion Gear Body as the target, the patterned tooth bodies as the tools).

If Enable Bore is checked, cut a cylindrical bore through Pinion Gear Body along the Pinion Gear Shaft Axis (Apex->A). The bore diameter is the Pinion Gear Bore Diameter if specified (non-zero); otherwise use `Pinion Gear Pitch Diameter / 4`. The bore runs the full length of the body along the shaft axis (both caps of the extrude-cut extend well past the body so the hole goes all the way through). Skip this step entirely if Enable Bore is unchecked.



## Create the Driving Gear

Create a new component as the child of Parent Component. The resulting bodies for this gear should end up in this component. (Same Fusion API caveat as the Pinion Gear section above.)

Create a body by revolving the closed hexagonal profile B -> I -> J -> D -> O -> P -> B around the B->I axis (the Driving Gear Shaft Axis). Let this be Driving Gear Body. As with the pinion, the O->P edge revolves into a conical face on the body that is reused as the cutting tool below.

Create a loft using the Apex point and the Driving Gear Tooth profile. Let this be the Driving Gear Tooth Body.

Cut the Driving Gear Tooth Body twice, in order:

1. First, cut using the conical face on Driving Gear Body that was generated from the O->P edge. Locate this face the same way as for the pinion: among the cone faces of Driving Gear Body, pick the one whose underlying surface contains both O and P in world coordinates, and use it as the splitting tool with `isSplittingToolExtended=True`.
2. Then, cut the resulting bodies using the conical face on Driving Gear Body that was generated from the D->J edge (the analogue of C->H on the pinion). Locate it by finding the cone face whose underlying surface contains both D and J in world coordinates, and use it as the splitting tool with `isSplittingToolExtended=True`.

It is an error if the two cuts together fail to split the Driving Gear Tooth Body into three pieces.

Remove the piece that contains the Apex (using `pointContainment` + `RemoveFeatures.add()` as described for the pinion).

Of the two remaining pieces, remove the smaller one (compare by `BRepBody.physicalProperties.volume`); remove it via `RemoveFeatures.add()`.

Circular-pattern the remaining tooth piece around the Apex->B axis (the Driving Gear Shaft Axis). The number of copies equals the Driving Gear Teeth Number, for the same reason as the pinion: the angular spacing around the shaft axis is constant at `360° / N`, regardless of the radial taper from heel toward apex.

Join all patterned tooth pieces with Driving Gear Body in a single Combine-Join (Driving Gear Body as the target, the patterned tooth bodies as the tools).

If Enable Bore is checked, cut a cylindrical bore through Driving Gear Body along the Driving Gear Shaft Axis (Apex->B). The bore diameter is the Driving Gear Bore Diameter if specified (non-zero); otherwise use `Driving Gear Pitch Diameter / 4`. The bore runs the full length of the body along the shaft axis. Skip this step entirely if Enable Bore is unchecked.

## Cleanup

Hide all sketches, construction planes, and construction axis.

For aesthetic purposes, rotate the Driving Gear Body around the Driving Gear Shaft Axis (Apex->B) by `180° / Driving Gear Teeth Number` — i.e., half a tooth pitch. Both gears are initially patterned from a starting tooth that sits in the axial (Gear Profiles) plane, so without this offset both gears place a tooth at the axial-plane crossing and visually collide. Rotating the Driving Gear by half its own tooth pitch shifts it so that a Driving Gear valley sits where the Pinion Gear tooth crosses the axial plane (and vice versa), giving the interlocked "meshing" look.

Implementation: add a `MoveFeatures` rotation on the Driving Gear Body using a construction axis along Apex->B (added to the Driving Gear sub-component, since the body lives there after being moved), with angle `180° / drivingTeeth` in degrees.