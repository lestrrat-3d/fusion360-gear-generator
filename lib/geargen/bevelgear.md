# Bevel Gear Creation Instructions

## Component Setup

Bevel gears are inherently created in pairs. As such, we shall have a single component containing three components

The first two components are for the driving gear and pinion gear. Because the designs are shared between the two components, the third one will be the design component.

## Variables

Module: user-supplied number. Specifies the module of gears. 

Parent Component: user-specified component. Defaults to currently active component.

Target plane: user-specified plane. This is where the bottom of the driving gear will sit flush against.

Center Point: user-specified point. This is where the driving bevel gear will be centered on. Does not need to be co-planar with target plane. 

Driving Gear Teeth Number: user-specified number of teeth on the driving gear.

Driving Gear Pitch Diameter: calculated number. Module * Teeth Number of the driving gear.

Pinion Gear Teeth Number: user-specified number of teeth on the pinion gear.

Pinion Gear Pitch Diameter: calculated number. Module * Teeth Number of the pinion gear.

Driving Gear Base Height: user-specified positive number, default 0mm (i.e. unspecified) .

Pinion Gear Base Height: user-specified positive number, default 0mm (i.e. unspecified).

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

Using ConstructionPlaneInput.setByAngle, create a plane that includes the Anchor Line, and set it at 90 degrees (as by default it would lay flush to the plane of the anchor line, but we want it perpendicular). Create a sketch on this plane.

In the sketch, project the center point from the Anchor Sketch.

From the projected center point, draw a line that runs along the direction of the target plane's normal (away from the target plane, on the side the normal points to). Apply Horizontal/Vertical constraint to it so it is perpendicular to the anchor line. The end of this line should be well above the Anchor line; Use x,y coordinates where x is the same as the center point, but y is shifted upwards the same amount as Driving Gear Pitch Diameter (however, do NOT use constraints). This end point shall be called the Apex.

Create a horizontal construction line from the apex, Driving Gear Pitch Diameter / 2 length away in the x direction; this line needs a dimensional constraint using the length specified earlier. Beginning of this line should use coincidence constraint with apex. Let the axis where this line lies be called the Pinion Gear Shaft Axis. The end of this line shall be called point A. From A, create a vertical construction line, Pinion Gear Pitch Diameter / 2 length away in the y direction towards the anchor line; this line needs a dimensional constraint using the length specified earlier. Beginning of this line should use coincidence constraint with the end of the previous line.

Create a vertical construction line from the apex, Pinion Gear Pitch Diameter / 2 length away in the y direction, towards the anchor line (no constraint). Beginning of this line should use coincidence constraint with apex. Let the axis where this line lies be called the Driving Gear Shaft Axis. The end of this line shall be called point B. From B, create a horizontal construction line, Driving Gear Pitch Diameter / 2 length away in the x direction (no constraint). Beginning of this line should use coincidence constraint with the end of the previous line.

Constrain the end point of the second lines from the previous two paragraphs with a coincident constraint. Let this point be called the Apex 2.

Note that the rectangle created above deliberately lies well above the anchor line.

Draw a construction line from Apex to Apex 2. This line shall be called the Pitch Line. Each end of the Pitch Line should be constrained to the respective points using coincidence constraint.

From the Apex 2, create a construction line in either side whose length is constrained Module * 1.25, and are perpendicular to the Pitch Line. Constrain them against the Pitch Line with the Perpendicular constraint. Let the line drawn towards the anchor line be Driving Gear Dedendum, whose end point shall be point D. The one drawn away from the anchor line be Pinion Gear Dedendum; whose end point shall be point C.

Draw two lines, from the Apex to the point D and point C, respectively. Apply coincidence constraints on beginning and end of these lines. These lines shall be the Root Axis for driving and pinion gear, respectively.

From point A, create construction line colinear with the line from Apex to point A, extending for length equals to module (but do NOT add dimensional constraint). The line should receive a colinear constraint, and the end of Apex->A and the beginning of the new line should be constrained via coincidence constraint. The end of this new line shall be point E.

Draw a construction line from point C to point E. constrain each end to respective points from pre-existing lines. The lines A->E and C->E should be constrained with perpendicular constraint.

From point B, create a construction line colinear with the line from Apex to point B, extending for length equals to module (but do NOT add dimensional constraint). The line should receive a colinear constraint, and the end of Apex->B and the beginning of the new line should be constrained via coincidence constraint. The end of this new line shall be point F.

Draw a construction line from point D to point F. constrain each end to respective points from pre-existing lines. The lines B->F and D->F should be constrained with perpendicular constraint.

Draw a construction line from point E colinear to line A->E, with length equal to module (but do NOT add dimensional constraint). Constrain point E and the beginning of this line. Let the end be known as point G.

From point C, draw a line with length equal to module (but do NOT add dimensional constraint). Constrain point C and the beginning of this line.  Let the end of the new line be point H. Line C->H should be colinear with line Apex2->C.

Connect point G and H with a line. Constrain end points of line accordingly with coincidence constraints. Constrain line E->G and H->G with a perpendicular constraint.


Draw a construction line from point F colinear to line B->F, with length equal to module (but do NOT add dimensional constraint). Constrain point F and the beginning of this line. Let the end be known as point I.


From point D, draw a  line with length equal to module (but do NOT add dimensional constraint). Constrain point D and the beginning of this line. Let the end of the new line be point J. Line D->J should be colinear with line Apex2->D.

Connect point I and J with a line. Constrain end points of line accordingly with coincidence constraints. Constrain line F->I and J->I with a perpendicular constraint.

Create a dimension constraint between lines Apex2->B and J->I. The value of this constraint should be equal to Driving Gear Base Height _if_ it is specified (non-0). Otherwise, compute the value as module * teeth number / 8.

Create a dimension constraint between lines Apex2->A and G->H. The value of
this constraint should be equal to Pinion Gear Base Hiehgt _if_ it is specified (non-0). Otherwise, compute the value as Driving Gear Base Height * (Pinion Gear Teeth Number / Driving Gear Teeth Number).

Draw a line from Apex to G. Constrain endpoints appropriately.

Constrain Point I with center point.
