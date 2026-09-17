# Bevel Gear Creation Instructions

This file is the **design & geometry intent**. The bevel-specific **Fusion-API realization** (the §2
lattice constraint style, sketch-local apex positioning, the full-constraint gate and its
exemptions, the cleanup recipe) lives in the sidecar **`fusion.md`** next to this file and is cited
by anchor (`[BEVEL-F…]`). Cross-gear Fusion conventions are cited as `[PB-…]` (shared `PLAYBOOK.md`),
and the spiral-tooth geometry derivation is in `spiral-tooth-trace.md`. Read them together; the
cited rules are as binding as this body.

**Sketch-first status (`[PB-SKETCH-FIRST]`) — the waiver is retired; bevel is bench-proven.** Every
constraint-bearing sketch this gear authors is reproduced in the standalone sketch engine at
`proof/bevelgear/sketches_test.go`, which is where `[PB-SKETCH-FIRST]` puts a gear's sketch proof —
never under `spec/bevelgear/sketch/`, a path the pipeline has never used and which earlier revisions
of this paragraph named as the condition for lifting the waiver. `TestGearProfiles` runs the §2
lattice, and `TestAnchorSketch`, `TestToothProfile`, `TestProfileSketch` and `TestBoreSketch` run
the other four, each registered from `steps.md` and each through `proofkit`'s gate, which asks for
DOF 0 with nothing redundant, nothing conflicting and the conditioning above the engine's floor. So
a regen may state that the §2 scheme is bench-proven, and the earlier instruction to keep the scheme
exactly as written now rests on that proof rather than on a waiver: **change the §2 constraint
scheme only together with the proof that holds it**, and a change that fails `TestGearProfiles` is a
defect in the change.

What the bench proof does not reach is unchanged and is recorded where it bites: it seeds at the
closed form, so it proves the constraints solve from a correct seed and never that the generated
module's seed is correct (see the toe-line seeding in §2 and "Proving the §2 figure" below), and a
configuration this lattice refuses on conditioning stays in the case table as a declared refusal
rather than narrowing the range this spec states.

## Component Setup

Bevel gears are inherently created in pairs. As such, we shall have a single component containing three components

The first two components are for the driving gear and pinion gear. Because the designs are shared between the two components, the third one will be the design component.

## Variables

User inputs are listed below in the order they appear in the command dialog. Target Plane comes first so it receives keyboard/pick focus when the dialog opens (Fusion auto-focuses the first selection input — `[PB-AUTOFOCUS-FIRST]`); Center Point follows so the user flows naturally from plane to point; Parent Component comes third since it is already pre-selected to the root component. Calculated values are listed after the inputs they depend on.

Target Plane: user-specified plane. This is where the bottom of the driving gear will sit flush against.

Center Point: user-specified point. This is where the driving bevel gear will be centered on. Does not need to be co-planar with target plane.

Parent Component: user-specified component. Defaults to the root component (pre-selected).

Module: user-supplied number. Specifies the module of gears.

Shaft Angle: User-supplied angle in degrees, **at least 30°** and **at most the Maximum Shaft Angle** below. Default 90° (perpendicular shafts — the classic bevel pair). The input is a `deg` Fusion expression (e.g. `60 deg`); convert to degrees before the range check (read-back units: see "Reading the raw numbers", `[PB-EVAL-EXPRESSION]`). ⚠️ **30° is the documented floor but is not known to be reachable.** Of three independently written §2 lattices, two refuse the default pair at 30° on conditioning (`2.83e-5` and `2.94e-5` against the engine's `4e-5` floor) and first clear at 35°; the third passes. See the conditioning note under Maximum Shaft Angle — that split is a property of the construction, so this spec states the geometric floor and leaves the reachable floor to the proof.

Maximum Shaft Angle: calculated number, and the reason the upper limit is not the flat 150° earlier revisions of this spec stated. A pitch cone angle reaching 90° turns that gear's pitch cone inside out: `R * cos γ` — the along-shaft seed length this spec uses for Apex→A and Apex→B, and the denominator of the back-cone virtual pitch radius in §3 — passes through zero and changes sign, so the seed points backwards along the shaft and the virtual radius is unbounded. Both cone angles stay below 90° exactly while

    cos(Shaft Angle) > -min(Driving Gear Pitch Diameter, Pinion Gear Pitch Diameter)
                      / max(Driving Gear Pitch Diameter, Pinion Gear Pitch Diameter)

so that `acos` is a hard singularity and the range check must reject a Shaft Angle **at or above** `degrees(acos(-smaller / larger))`, naming the computed limit in the message. A 31/17 pair gives `acos(-17/31) = 123.26°`; the flat 150° this spec used to promise was never reachable for it. Equal tooth counts give `acos(-1) = 180°`, which is no constraint at all.

**The Maximum Shaft Angle is that cone-angle limit, capped at 150°**, and the cone-angle half is exclusive while the 150° half is inclusive. 150° is a practical ceiling on the figure, not a measured one.

⚠️ **A separate conditioning limit exists, and it is a property of how the §2 lattice is built rather than of this spec.** The sketch engine refuses a system whose conditioning falls below its `4e-5` trust floor, and the lattice approaches that floor at both ends of the Shaft Angle range. **Do not write a Shaft Angle bound from a conditioning measurement**, and do not treat the numbers below as a range check.

The reason is measured. Three independently written lattices, each built from this spec, each holding DOF 0 with nothing redundant, and each asserting its solved cone angles against the closed form to nine decimals, do not agree about which end of the range is reachable. Two of the three refuse the default pair at 30° (`2.83e-5` and `2.94e-5`), first clear at 35°, and pass 142° (`9.85e-5`) and 150° (`4.07e-5`). The third does the opposite: it passes 30° and refuses 142° (`3.96e-5`) and 150° (`2.49e-5`). The two that agree do so to within 4% at every angle, so the odd one out is the single net rather than the measurement. They differ in construction — the outlier holds 52 coincident constraints over 36 lines against 46 over 33 — and that difference, not the geometry, is what moves the conditioning. Ruled out as causes: the base-height bounds below (identical readings with them applied and removed), the sketch engine version, and the choice of which equation to keep for each constraint the net leaves one row dependent.

So a build that fails on conditioning has found a fact about its own constraint net. The remedy is to change how the lattice is built, never to loosen the gate and never to narrow the advertised Shaft Angle range on one net's evidence. The proof's case table is where the measurements for that net belong.

**Fusion has built a pair at a shaft angle other than 90° once — loaded 2026-09-17, from the build
at `2ad1e32` (PR #159).** It is the first load at any Shaft Angle other than 90° recorded anywhere
in this spec: every other verdict below — the bore refusal, the parallel-family spiral, the
Combine-Join stitch, the `[BEVEL-F-SEED-HELD]` gate — was taken at 90°, so until this load the whole
of the Shaft Angle range above and below the default had been exercised by the proof alone. The same
load covered five more configurations and every one built clean: the shipped default 31/31 pair at
Module 1 and Shaft Angle 90° in **both** straight and spiral form, a 16 driving / 12 pinion pair at
Module 4, a non-zero bore, and a non-zero Toe Extension with a toe radius.

**Nothing was measured on that load beyond the builds completing.** No solved point, cone angle,
volume or radius was read off any of the six, and the shaft angle the sixth was built at was not
recorded either. So what this records is that six configurations reached a finished pair of bodies
with no error raised, and it is silent about whether any of them is the right shape. In particular
it does **not** say that the lattice's conditioning holds away from 90° — the note above is about
the sketch engine's floor and Fusion's solver is a different solver — and it does not reach either
end of the admitted range, because which angle was used is unknown.

**Could the proof have caught any of this?** One configuration, yes, and the case is added: the
proof ran a positive Toe Extension and a user Toe Radius only in separate cases, never together,
which is what this load set. "The case the two toe inputs need together" below states it. The rest,
no — the proof already runs the lattice, the tooth and the solids across a Shaft Angle table that
reaches well past 90°, and already carries the 16/12 pair at Module 4 and a bore; what it does not
run is Fusion's own solver, its profile finder, its revolve and its booleans, which is the whole of
what a load adds. One gap this load leaves rather than closes: the shaft angle was not written down,
so the first non-90° load cannot be repeated from this record. **Record the configuration, not only
the verdict, on the next one.**

Driving Gear Teeth: user-specified number of teeth on the driving gear. Default is 31. (The dialog label is `Driving Gear Teeth`, per the input table; formulas below refer to this value as Driving Gear Teeth Number.)

Pinion Gear Teeth: user-specified number of teeth on the pinion gear. Default is 31. (Formulas below refer to this value as Pinion Gear Teeth Number.)

Driving Gear Pitch Diameter: calculated number. Module * Driving Gear Teeth Number.

Pinion Gear Pitch Diameter: calculated number. Module * Pinion Gear Teeth Number.

Driving Gear Base Height: user-specified positive number, default 0mm (i.e. unspecified).

Pinion Gear Base Height: user-specified positive number, default 0mm (i.e. unspecified).

Maximum Base Height: a geometric upper bound on each gear's *resolved* base height, applied per gear the same way the Maximum Face Width below is applied to Face Width. Where the Maximum Face Width stops the **toe** end of the frustum profile from crossing the shaft axis, this stops the **heel** end from doing it, and nothing in earlier revisions of this spec bounded that end at all.

For one gear, with `r` its own pitch radius (`its Pitch Diameter / 2`) and `γ` its own pitch cone angle (`γ_p` for the pinion, `γ_g` for the driving gear, both from the closed form in §2), the bound is

    Maximum Base Height = 0.95 * (r - 1.25 * Module * cos γ) * tan γ

**Read the origin carefully, because it is easy to get wrong by one dedendum.** The base height is the offset dimension between the A->Apex2 drop and G->H (resp. the B->Apex2 drop and I->J), so it is measured from **Apex 2's plane**, not from the dedendum point. Walking out along the dedendum line from Apex 2, the perpendicular distance to the shaft axis falls at `cos γ` per unit and the along-shaft coordinate rises at `sin γ`, so H (resp. J) reaches the axis when the base height reaches **`r * tan γ`**. That is the true crossing.

The bound above sits `1.25 * Module * sin γ` **below** that crossing, because it starts from the dedendum point C (resp. D) at perpendicular distance `r - 1.25 * Module * cos γ` instead of from the pitch point. It is therefore **deliberately conservative, not exact**: it refuses a band of base heights that would in fact still build. That is the trade this spec takes, since past the true crossing the hexagonal frustum profile has crossed its own axis of revolution and the revolve fails with `ASM_WIRE_X_AXIS` (`[PB-REVOLVE]`), and a heel edge that merely approaches the axis is already degenerate. The `0.95` factor is the one the Maximum Face Width uses, for the same reason. If a future revision wants the exact bound, it is `0.95 * r * tan γ`; do not adopt it without re-running the low-tooth-count cases, which the conservative form is what currently keeps buildable.

Apply it in **both** directions, per gear, mirroring the Face Width rule:
- When the base height was not specified, use `min(<the fallback below>, Maximum Base Height)`.
- When the user specified a value greater than the Maximum Base Height, reject it with a message stating the maximum, rather than proceeding.

Worked case: Module 1, Driving 31, Pinion 31, Shaft Angle 30°. Each `γ` is 15°, so this bound is `0.95 * (15.5 - 1.25*cos 15°) * tan 15° = 3.638 mm`, the true crossing is `15.5 * tan 15° = 4.153 mm`, and the driving fallback `Module * Driving Gear Teeth Number / 8` resolves to `3.875 mm`. **The default therefore sits between the two: it is capped to 3.638 mm by this bound, but it would not have folded uncapped.** An earlier revision of this paragraph claimed the shipped default folded its own profile at 30°; that was wrong, and it was wrong because the bound was read as if the base height were measured from the dedendum point. The cap is still worth having — it is what refuses an over-large *user* value — but it is not rescuing the default from a fold.

Unlike the Maximum Face Width, this bound is closed-form and needs no solved sketch geometry, because `r`, `γ` and Module are all known before §2 draws anything. Resolve it during input validation, alongside the range checks.

Minimum Base Height: the **other** end of the same heel edge, and the reason a low tooth count used to fail. The base-height offset closes H at `|C->H| = <this gear's base height> / sin γ - 1.25 * Module` beyond C, so unless the base height carries H past the dedendum's own along-shaft projection, H lands *behind* C and the edge C->H runs back inward instead of outward. That projection is `1.25 * Module * sin γ`, so

    Minimum Base Height = 1.05 * 1.25 * Module * sin γ

with a `1.05` margin for the same reason the other bounds carry `0.95`. Apply it the same way and in the same pass: raise a fallback that falls below it, and reject a user value below it with a message stating the minimum.

The fallback `Module * Driving Gear Teeth Number / 8` clears the raw projection exactly while `Driving Gear Teeth Number > 10 * sin γ` — at Shaft Angle 90° that is 7.07, so an equal 8-tooth pair was the smallest that built and a 7-tooth pair failed. Raising the fallback to the Minimum Base Height is what lets small tooth counts build at all; **do not instead raise the teeth floor**, which would refuse gears that are perfectly buildable once the base height is bounded.

Minimum Teeth: the two base-height bounds cross when the tooth count gets small enough, and below that crossing **no** base height satisfies both, so the gear cannot be built at any setting. Requiring `Minimum Base Height ≤ Maximum Base Height` and solving for the count gives

    Driving/Pinion Gear Teeth Number >= 5.27 * cos γ

(the constant is `2 * (1.05 * 1.25 / 0.95 + 1.25)` = 5.2632, rounded UP to 5.27 so the published floor stays at or above the exact crossing; do not round it down, and do not replace 5.27 with the exact value without re-running the low-tooth-count cases). Check it per gear with that gear's own `γ`, and reject below it naming the computed floor. At Shaft Angle 90° the floor is 3.72, i.e. **4 teeth** — measured: with both bounds applied an equal 4-tooth pair solves and a 3-tooth pair still fails on the heel edge, because its Maximum Base Height has fallen below its Minimum. The blanket `teeth >= 3` check admits that 3-tooth pair, so keep `teeth >= 3` as the absolute floor and apply this computed floor on top of it.

⚠️ **A configuration can satisfy every bound above and still be refused as near-singular.** See the conditioning note under Maximum Shaft Angle: that limit belongs to the particular lattice, not to this spec, and the two independent nets measured there do not even agree on whether conditioning improves or worsens as the tooth count falls. It is therefore not a validation rule and no bound here is derived from it. Treat a near-singular report as a real refusal of that construction, never as a tolerance to loosen.

Enable Bore: user-specified boolean, default `true`. Applies to both gears. When unchecked, no bore is cut on either gear and the per-gear bore diameter inputs below are ignored.

Driving Gear Bore Diameter: user-specified positive number, default 0mm. Only consulted when Enable Bore is checked. A value of 0 means "auto-calculate" — use `min(Driving Gear Pitch Diameter / 4, Maximum Bore Diameter)` as the bore diameter, that gear's own Maximum Bore Diameter below. A user-supplied value greater than that maximum is an error: reject it with a message stating the maximum, rather than proceeding. Both rules, and the step they resolve in, belong to Maximum Bore Diameter.

Pinion Gear Bore Diameter: user-specified positive number, default 0mm. Only consulted when Enable Bore is checked. A value of 0 means "auto-calculate" — use `min(Pinion Gear Pitch Diameter / 4, Maximum Bore Diameter)` as the bore diameter, that gear's own Maximum Bore Diameter below. A user-supplied value greater than that maximum is an error: reject it with a message stating the maximum, rather than proceeding. Both rules, and the step they resolve in, belong to Maximum Bore Diameter.

Maximum Bore Diameter: a geometric upper bound on each gear's *resolved* bore diameter, applied per gear the same way the Maximum Base Height above and the Maximum Face Width below are applied to their own inputs. The bore is a through cut on the shaft axis, so it removes every ring of material inside its own radius and reaches the two ends of the body first: past the heel term it takes the **entire flat back face** — the disc the back-face edge G->H (resp. I->J) sweeps — and past the toe term it takes the whole toe dish and bites into the root cone. Either way the revolved frustum comes out of the Bore step with no end face on that side.

For one gear, with `r` its own pitch radius (`its Pitch Diameter / 2`), `γ` its own pitch cone angle, `R` the Pitch Cone Distance, and `|Apex->Ded|` and the root cone angle `γ_root` as the Root Length and Toe Limit below define them, the two ends sit at

    r_heel = r - <this gear's RESOLVED Base Height> / tan γ
    r_toe  = (|Apex->Ded| - Root Length) * sin γ_root

and the bound is

    Maximum Bore Diameter = 2 * 0.95 * min(r_heel, r_toe)

`r_heel` is where H (resp. J) lands, by the same walk the Maximum Base Height above derives its crossing from: the base height is measured from Apex 2's plane, and walking out the dedendum line the perpendicular distance to the shaft axis falls at `cos γ` per unit while the along-shaft coordinate rises at `sin γ`, so H sits `<Base Height> / tan γ` inside the pitch radius. `r_toe` is where M (resp. O) lands, one Root Length back along the root element from the dedendum point; **at Toe Extension 0 it is exactly that gear's Toe Radius Ceiling below**, and it shrinks as the Toe Extension climbs, so a large Toe Extension tightens this bound.

**The flat FRONT face is deliberately NOT protected.** Its radius is the Toe Radius, which is not on the body's outer envelope — the toe dish leans inside the root cone — so a bore wider than the Toe Radius only exits through the toe cone instead of through that face, and the frustum stays whole. A bore past `r_heel` or `r_toe` does not: it cuts a corner off the profile that is being revolved. The `0.95` factor is the one the Maximum Face Width and the Maximum Base Height carry, for the same reason — a bore at exactly `r_heel` leaves a zero-width back face and a cut tangent to the body's own heel edge, and that is degenerate before it strictly crosses.

Apply it in **both** directions, per gear, mirroring the Face Width and base-height rules, and only when Enable Bore is checked:
- When the bore was auto-calculated (the input is 0), use `min(<this gear's Pitch Diameter / 4>, Maximum Bore Diameter)`.
- When the user specified a value greater than the Maximum Bore Diameter, reject it with a message stating the maximum, rather than proceeding.

Worked case: Module 1, Driving 31, Pinion 31, Shaft Angle 35°. Each `γ` is 17.5°; the driving fallback `Module * Driving Gear Teeth Number / 8 = 3.875 mm` clears its own Maximum Base Height of 4.286 mm and stands, and the pinion scales to the same value on equal tooth counts. So `r_heel = 15.5 - 3.875 / tan 17.5° = 3.210 mm`, while `r_toe` is above 12 mm and the heel term binds. The auto bore radius is `Pitch Diameter / 8 = 3.875 mm`, outside `r_heel`, so without this bound the auto bore alone deletes the back face of both gears. The shipped default pair at Shaft Angle 90° is unaffected — it is the low-`γ` end of the Shaft Angle range that is not.

**Unlike the two base-height bounds, this one cannot be resolved during input validation.** Its heel term is closed-form and is available as soon as the base heights resolve, but its toe term needs the Root Length, which needs the resolved Face Width, which needs the Maximum Face Width and therefore solved §2 sketch geometry. The bound is the minimum of the two terms, so **resolve and apply the whole bound in §2, at the step that already applies the Maximum Face Width**; do not split the check across the two passes. That step still runs before anything is revolved, so the Bore step in "Create the Gear Bodies" reads a diameter that is already bounded.

**Fusion has refused an over-maximum bore once — loaded 2026-09-16, from the build this bound was
introduced on (branch `fix-bevel-spec-defects`).** A bore diameter above the maximum was entered
and the build stopped at the rejection instead of proceeding. The message's wording was not read
back, so what this records is the refusal and not the text of it. What the load does not show is
the damage the bound exists to prevent: the bound was not lifted, so no bore past `r_heel` was cut
and the deleted back face has still never been seen in Fusion.

**Could the proof have caught this?** The arithmetic, yes, and it already does: the geometry case
computes `2 * 0.95 * min(r_heel, r_toe)` from the closed form above, and both the sketch case and
the solid case refuse a resolved bore diameter above it. What no case reaches is the generated
module raising on the user's value, because the proof never runs that module. That refusal is the
part this load covered, and there is no case to add for it.

**A bore that this bound admits has since been cut in Fusion — loaded 2026-09-17, from the build at
`2ad1e32` (PR #159; the full list of that load's six configurations is under Maximum Shaft Angle
above).** A non-zero bore diameter was entered and the pair built with no error, which is the other
half of the pair of outcomes this bound governs: the 2026-09-16 load showed a value above the
maximum being refused, and this one shows a value below it being cut. **The diameter was not read
back and nothing was measured on the result**, so the bore's size, the two faces it passes through
and whether it left the back face intact are all still unseen in Fusion — and the deleted back face
the bound exists to prevent still has never been built, because the bound was not lifted for either
load.

Face Width: User-specified positive number. If unspecified, default to (Cone Distance / 6). In **every** case (default or user-specified) the Face Width is bounded by the Maximum Face Width (defined below):
- If unspecified, use `min(Cone Distance / 6, Maximum Face Width)`.
- If the user specifies a value greater than the Maximum Face Width, this is an error: reject it with a message stating the maximum, rather than proceeding (the gear-body revolve in "Create the Gear Bodies" would otherwise fail — see the Maximum Face Width rationale).

Cone Distance: calculated number. `sqrt((Module * Driving Gear Teeth Number)**2 + (Module * Pinion Gear Teeth Number)**2)`. It depends on the two tooth counts only, never on the Shaft Angle.

**"Cone Distance" and "Pitch Cone Distance" are two different lengths in this spec, and both are used.** The Cone Distance just defined is the diagonal of the two pitch diameters. The **Pitch Cone Distance** `R` is the real apex-to-heel length along the pitch cone, `R = (Pinion Gear Pitch Diameter / 2) / sin γ_p`, defined with the closed-form cone angles in §2 and used there to seed the Apex and the along-shaft lengths, and again in §3 for the back-cone virtual radii. The two coincide as `Cone Distance = 2 * R` **exactly when Shaft Angle is 90°**, for any pair of tooth counts, and diverge everywhere else: an equal 31/31 pair at Shaft Angle 30° has `Cone Distance = 43.84 mm` against `R = 59.89 mm`, and at 140° `R = 16.49 mm`. Earlier revisions of this spec called both of them "cone distance", which is why this paragraph exists — when a step below says "Cone Distance" it means the diagonal, and `R` is always written as `R` or "Pitch Cone Distance".

The Face Width default `Cone Distance / 6` is therefore `R / 3`, the conventional face-width limit, **only at Shaft Angle 90°**. Below 90° it is conservative and below `R / 3`; above 90° it exceeds `R / 3` and the Maximum Face Width cap is what actually holds it. This is deliberate — it keeps the default independent of Shaft Angle — and it is the cap, not this default, that guarantees a buildable profile.

Maximum Face Width: a geometric upper bound that cannot be evaluated until the Gear Profiles sketch points A, B, C, D, H, J exist (see §2 — apply the bound there). It is `0.95 *` the smaller of:
- the perpendicular distance from point A to the line through C and H (the Pinion Gear Dedendum line, i.e. Apex2->C extended), and
- the perpendicular distance from point B to the line through D and J (the Driving Gear Dedendum line, i.e. Apex2->D extended).

**Compute both distances from the points' SOLVED sketch geometry — `pointA.geometry`, `pointB.geometry`, `pointC.geometry`, `pointD.geometry`, `pointH.geometry`, `pointJ.geometry` — NOT from the pre-solve seed coordinates (`[PB-SOLVED-GEOMETRY]`).** By the time §2 reaches this step the constraint network has located all six, so `.geometry` is exact; seeds diverge substantially for asymmetric tooth counts (e.g. Driving 17 / Pinion 31) or non-90° shaft angles, making a seed-based bound too loose on the binding side — the toe still crosses the axis and the cap is defeated.

Rationale (do not drop this when regenerating): the toe line M->N is C->H offset *toward the Apex*, and its mirror O->P is D->J offset toward the Apex. **Before the Toe Radius existed N was pinned to line A->Apex2**, so the offset drove it straight down that drop: when the offset reached the perpendicular distance from A to line C->H, N landed exactly on A, and any larger value drove N **past** A, across the gear's own shaft axis (Apex->A). That is the crossing this cap is measured from, and the cap is kept because Face Width still sets where the toe end starts. **N and P no longer ride that drop** — see the Toe Radius above — so the cap no longer describes where they end up; what stops the profile crossing its axis now is that the Toe Radius is strictly positive and only the front face's foot touches the axis. The frustum profile (hexagon A, G, H, C, M, N, built here in §2 and revolved later in "Create the Gear Bodies") is revolved about that shaft axis, so a profile that has crossed the axis self-intersects the axis of revolution and Fusion aborts the revolve with `ASM_WIRE_X_AXIS` (`[PB-REVOLVE]`). The pinion side is normally the binding one (its smaller pitch radius gives the smaller distance), but compute both and take the minimum so the bound holds for any Shaft Angle. The `0.95` factor keeps N clearly off A, since a near-coincident N≈A degenerates the toe edge even before it strictly crosses. At Shaft Angle 90° this limit equals `0.95 * min(Driving Gear Pitch Diameter, Pinion Gear Pitch Diameter)**2 / (2 * Cone Distance)`, the SMALLER pitch diameter and never the pinion's by name, since the pinion is only usually the smaller one. Written with the pinion's diameter it is wrong whenever the driving gear carries the smaller tooth count: on a Driving 17 / Pinion 31 pair at Module 1 the real bound is 3.883 mm and the pinion form gives 13.591, so the naive `Cone Distance / 6` default exceeds it — and the gear fails to generate — for any gear ratio above roughly √2 (e.g. Driving 31 / Pinion 17).

Toe Extension: user-specified percentage, default 0, valid range **[0, 100]**. A single value applied to **both** gears, because they share one face and must mesh. It extends the frustum's toe end past where Face Width alone puts it, toward the Apex, by moving the **root length** `Ded->Toe` — the segment `C->M` on the pinion and `D->O` on the driving gear. **0 reproduces today's toe end exactly**, which is what makes every gear built before this input existed come out unchanged; 100 reaches the far end of the window below. The percentage is read on the **DRIVING gear** and the pinion is then built to that same root length.

Root Length: calculated number, the resolved `|Ded->Toe|`. At Toe Extension 0 it is the resolved Face Width re-measured along the root element rather than perpendicular to the pitch line, which is longer by the dedendum angle's cosine: `Face Width * |Apex->Ded| / R`, with `|Apex->Ded| = sqrt(R**2 + (1.25 * Module)**2)`. At a positive Toe Extension it is that value plus the extension's share of the window. **Face Width still resolves exactly as it always did and still carries the Maximum Face Width cap** — the Toe Extension adds to what Face Width resolved, it does not replace it.

Driving Gear Toe Radius / Pinion Gear Toe Radius: user-specified non-negative numbers in mm, default 0mm each. **0 means "auto-calculate"** — use that gear's own inner toe corner radius at Toe Extension 0, `this gear's Pitch Radius - Face Width / sin γ`, which is the value that makes Toe Extension 0 today's profile exactly. The toe radius is the perpendicular distance from the shaft axis at which the **inner toe corner** N (resp. P) rides, and with it the radius of the flat front face the revolve produces. A user value must be **strictly below that gear's Toe Radius Ceiling**; reject it with a message naming the ceiling.

Toe Radius Ceiling: calculated number, per gear. It is that gear's **OUTER** toe corner radius at Toe Extension 0, `(this gear's Pitch Radius - 1.25 * Module * cos γ) * (1 - Face Width / R)`. At or above it the point X below falls behind the toe corner and the Toe Extension has nowhere to go.

Toe Limit: calculated number, per gear, `|Ded->X|` where **X is the point on the root element `Apex->Ded` at this gear's Toe Radius**. Closed form: `sqrt(R**2 + (1.25 * Module)**2) - Toe Radius / sin(γ_root)`, with the root cone angle `γ_root = γ - atan(1.25 * Module / R)`. X is where the toe end is heading: as the Toe Extension rises the toe corner climbs `Apex->Ded` toward X while N/P slides in along the toe-radius line to meet it, and at X the toe face has closed to nothing.

**Toe Extension 100 stops at 0.99 of the way from the Toe Extension 0 root length to the smaller of the two gears' Toe Limits, not at the Toe Limit itself.** The smaller limit wins because the pair shares one root length; the other gear simply stops short of its own X. The 0.99 is there because AT the limit the toe face has zero length, so the revolved gear body carries **no cone at its toe end** and the conical end-cut in "Trim the Tooth Body" — whose toe cut must split or the build fails — has no `ConeSurfaceType` face to find. The last percent is worth well under a tenth of a millimetre of root length on every case in the proof's table, so the reach given up is nil and the failure avoided is total. Do not drop this factor when regenerating.

**A defaulted Toe Radius can leave no room at all, and that is a real configuration rather than a defect.** On a driving gear with a large pitch cone angle the inner toe corner already sits at a LARGER radius than the outer one — the toe dish leans toward the heel rather than away from it — so X falls behind the toe corner and the Toe Limit comes out below the Toe Extension 0 root length. Measured over gear ratio against Shaft Angle it is a diagonal band that crosses 90° for every ratio from about 2.75 up, and Module does not move its boundary. **Reject a Toe Extension above 0 on such a pair**, with a message naming the gear and the Toe Radius Ceiling it needs to come below; Toe Extension 0 still resolves, so the gear itself stays buildable exactly as before. Do **not** silently substitute a smaller Toe Radius: that would change the toe end of a gear whose inputs asked for no change.

Tooth Spacing: user-specified non-negative number in mm, default 0mm. A single value applied to **both** gears. It is a clearance offset that shifts each virtual spur tooth profile's **center** radially outward along the dedendum line — *away from the lower corner* (the rim corner opposite the Apex: point C for the pinion, point D for the driving gear), i.e. in the C->K direction beyond K (and D->L beyond L) — by this distance, **while the tooth itself is still drawn at the original virtual pitch radius** (the exact back-cone radius §3 step 1 defines; the virtual tooth number is unchanged). At 0 (the default) the tooth center sits exactly at K / L; a positive value moves the center farther from the rim, loosening the mesh so 3D-printed teeth have more clearance. Applied in §3; see "Gear Tooth Profiles".

Mean Spiral Angle (ψ): user-specified angle in degrees, default 35°, valid range **[0, 60)**. The angle between the tooth trace and the cone element, measured at the mean cone distance (see `spiral-tooth-trace.md`). **ψ = 0 means a STRAIGHT bevel gear** — the tooth-body build takes the original straight path unchanged (apex-point loft + the two conical trims) and every spiral input below is ignored; any value **> 0 builds a curved (spiral) tooth**. The driving gear uses this hand; the meshing pinion is built with the **opposite** hand (mirror) so the pair meshes. Input is a `deg` Fusion expression; convert to degrees before the [0, 60)° range check (read-back units: see "Reading the raw numbers", `[PB-EVAL-EXPRESSION]`).

Hand of Spiral: user-specified dropdown — `Right` (default) or `Left`. The **driving** gear's hand of spiral; the pinion is built with the opposite hand. Only consulted when ψ > 0; shown only when ψ > 0 (see "Conditional visibility" under Exact input ids). The two list-item strings `Right`/`Left` are reproduced surface (module constants `_HAND_RIGHT = 'Right'`, `_HAND_LEFT = 'Left'`).

Cutter Radius: user-specified non-negative number in mm, default 0mm. The face-mill cutter radius `r_c` that sets the radius of the tooth-trace arc (see `spiral-tooth-trace.md`). **0 means auto** — use the mean cone distance `R_mean` as `r_c`. Only consulted when ψ > 0; shown only when ψ > 0 (see "Conditional visibility" under Exact input ids). Reject negative values.
### Exact input ids and parameter-name strings

These literal strings are part of the reproduced surface. Use them verbatim. The dialog **display
order** (the order `configure()` adds inputs) is fixed as the rows below — Target Plane first so it
wins Fusion's auto-focus, then Center Point, then the pre-selected Parent Component, then the
numeric/bool fields. Module-level constants name the input ids (`INPUT_ID_PLANE = 'targetPlane'`, …).

| # | Dialog input | input id | input type | unit | default | selection filters / tooltip |
|---|---|---|---|---|---|---|
| 1 | Target Plane | `targetPlane` | `addSelectionInput` | — | — | `ConstructionPlanes`, `PlanarFaces`; limit 1; tooltip `Plane the bottom of the driving gear sits flush against` |
| 2 | Center Point | `centerPoint` | `addSelectionInput` | — | — | `ConstructionPoints`, `SketchPoints`; limit 1; tooltip `Point the driving bevel gear is centered on` |
| 3 | Parent Component | `parentComponent` | `addSelectionInput` | — | root component pre-selected | `Occurrences`, `RootComponents`; limit 1; tooltip `Component the gear pair is created under` |
| 4 | Module | `module` | `addValueInput` | `''` | `createByReal(1)` | — |
| 5 | Shaft Angle | `shaftAngle` | `addValueInput` | `deg` | `createByString('90 deg')` | — |
| 6 | Driving Gear Teeth | `drivingTeeth` | `addValueInput` | `''` | `createByReal(31)` | — |
| 7 | Pinion Gear Teeth | `pinionTeeth` | `addValueInput` | `''` | `createByReal(31)` | — |
| 8 | Driving Gear Base Height | `drivingBaseHeight` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 9 | Pinion Gear Base Height | `pinionBaseHeight` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 10 | Enable Bore | `boreEnable` | `addBoolValueInput` (checkbox) | — | `True` | — |
| 11 | Driving Gear Bore Diameter | `drivingBore` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 12 | Pinion Gear Bore Diameter | `pinionBore` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 13 | Face Width | `faceWidth` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 14 | Tooth Spacing | `toothSpacing` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 15 | Mean Spiral Angle | `spiralAngle` | `addValueInput` | `deg` | `createByString('35 deg')` | — |
| 16 | Hand of Spiral | `spiralHand` | `addDropDownCommandInput` (text-list) | — | items `Right` (selected), `Left` | — |
| 17 | Cutter Radius | `cutterRadius` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 18 | Toe Extension (%) | `toeExtension` | `addValueInput` | `''` | `createByReal(0)` | — |
| 19 | Driving Gear Toe Radius | `drivingToeRadius` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 20 | Pinion Gear Toe Radius | `pinionToeRadius` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
There are now **20** dialog inputs and **20 `INPUT_ID_*`** module constants, named exactly:
`INPUT_ID_PLANE`, `INPUT_ID_CENTER_POINT`, `INPUT_ID_PARENT`, `INPUT_ID_MODULE`,
`INPUT_ID_SHAFT_ANGLE`, `INPUT_ID_DRIVING_TEETH`, `INPUT_ID_PINION_TEETH`,
`INPUT_ID_DRIVING_BASE_HEIGHT`, `INPUT_ID_PINION_BASE_HEIGHT`, `INPUT_ID_BORE_ENABLE`,
`INPUT_ID_DRIVING_BORE`, `INPUT_ID_PINION_BORE`, `INPUT_ID_FACE_WIDTH`, `INPUT_ID_TOOTH_SPACING`,
`INPUT_ID_SPIRAL_ANGLE`, `INPUT_ID_HAND`, `INPUT_ID_CUTTER_RADIUS`, `INPUT_ID_TOE_EXTENSION`,
`INPUT_ID_DRIVING_TOE_RADIUS`, `INPUT_ID_PINION_TOE_RADIUS` — holding the table's id
strings in row order. Inputs 15–17 are appended **after** Tooth Spacing in display order, and
inputs 18–20 after those. `toeExtension` is a plain unitless percentage, so it needs no `to_cm`
conversion; the two toe radii are `'mm'` inputs and read back in internal cm like every other
length in the table. The Hand dropdown is a
`DropDownStyles.TextListDropDownStyle` with `Right` added selected and `Left` added unselected; read
its `selectedItem.name` (default `Right` if none). `spiralAngle` and `cutterRadius` read back in
internal units per "Reading the raw numbers" (`[PB-EVAL-EXPRESSION]`). Still **no live
Fusion user parameters** — the spiral values are precomputed in Python like everything else.

**Conditional visibility — the spiral-only inputs show only when ψ > 0.** Hand of Spiral
(`spiralHand`) and Cutter Radius (`cutterRadius`) are relevant **only** for curved bevels, so they
are **hidden whenever Mean Spiral Angle ψ = 0 and shown when ψ > 0**. Mean Spiral Angle (`spiralAngle`)
itself is the controller and is **always visible** — it is how the user reaches ψ > 0. Realize this
with the `commandInput.isVisible` property (there is no declarative "show-if" in the Fusion API):
- Add a `@classmethod _updateSpiralInputVisibility(cls, inputs)` helper. It evaluates the
  `spiralAngle` input's **`.expression`** via
  `unitsManager.evaluateExpression(spiral.expression, 'rad')` (internal **radians** — it does NOT
  read the input's `.value`) and sets `inputs.itemById(INPUT_ID_HAND).isVisible` and
  `inputs.itemById(INPUT_ID_CUTTER_RADIUS).isVisible` to `(value > 0)`. **Guard it:** if any of the
  three inputs is `None` return early, and wrap the expression evaluation in `try/except` (a
  half-typed expression can raise mid-edit) — on failure leave both inputs **shown**. `isVisible` only hides the
  dialog row; the input still exists and `_readInputs` reads it normally (and the ψ = 0 build ignores
  Hand/Cutter anyway), so hiding is purely cosmetic and cannot affect generation.
- `configure()` calls `cls._updateSpiralInputVisibility(inputs)` **as its last step**, so the initial
  state is correct (default ψ = 35° → both shown).
- The dialog's `inputChanged` event drives the reactive update through a second classmethod
  `@classmethod def handle_input_changed(cls, args)`, which simply calls
  `cls._updateSpiralInputVisibility(args.inputs)` — recompute on **every** input change (cheap and
  robust; no need to branch on which input changed). `handle_input_changed` is bound by name from
  `commands/bevelgear/entry.py` (see Method contract — external bindings).

Selection filters (`[PB-SELECTION-FILTER-ENUM]`) and limit-1 are set per the table. Each selection
input's tooltip string (the third argument to `addSelectionInput`, shown in the table) is part of
the reproduced surface — use it verbatim. The Parent
selection pre-selects `get_design().rootComponent`. The numeric `mm`/`deg` defaults are passed in
internal units (`to_cm(...)` for lengths; `createByString('90 deg')` for the angle so the
expression engine parses it).

**No live Fusion user parameters.** Unlike the spur family, bevel registers **no** user parameters
under a prefix. Every value (pitch diameters, cone distance, base heights, bore diameters, virtual
tooth counts, face width) is **precomputed in Python in internal cm** and written into geometry
numerically — sketch dimensions via `dimension.parameter.value = <number>` and feature inputs via
`ValueInput.createByReal(<number>)`. There are therefore no `PARAM_*` name strings to reproduce;
the only module-level constants are the 20 `INPUT_ID_*` strings plus `_HAND_RIGHT`/`_HAND_LEFT`.
(`[PB-PRECOMPUTED-MODE]`.)

**Two further constants are declared on `BevelGearGenerator` rather than at module level, and this
is their declaration site.** Earlier revisions named both as tunable constants with defaults and
gave neither a home, which left the sentence above reading as if they did not exist:

| constant | declared on | default | what it does |
|---|---|---|---|
| `_CROWN_PER_RAD` | `BevelGearGenerator` | `0.5` | scales the spiral lengthwise crown — §3a step H |
| `_PINION_MESH_PHASE_TEETH` | `BevelGearGenerator` | `0.0` | the pinion's extra mesh rotation in tooth-fractions — `_pinionMeshPhase` under Method contract |

They are class attributes because both are read through `self` from inside the tooth-body build, and
neither is a reproduced API string the way the input ids are.

⚠️ **`_CROWN_PER_RAD` reaches built geometry and its value is not derived from anything.** It
multiplies the relief every crowned slab takes, so changing it changes the tooth. Traced: `0.5` was
hand-entered on 2026-06-07, in commit `c3829c0` on the hand-coded `SpiralBevelGearGenerator` that
preceded this spec, replacing a `0.0` that had the crown switched off; the comment it was written
beside says to dial the value up until a high-ratio pair runs clean. No measurement, published
source or Fusion load stands behind it, and none has been taken since. **Treat it as an unverified
tuning value**: keep it at `0.5` so a regen reproduces today's gear, and do not write a derivation
for it that the repository does not have. `_PINION_MESH_PHASE_TEETH` carries no such risk at its
default: `0.0` makes `_pinionMeshPhase` return 0, so no extra rotation is applied, and §3a step G
leaves the mid-face section unrotated precisely so that none is needed.

**Reading the raw numbers.** Read each numeric/angle input by evaluating its expression with units
`''`/`'mm'`/`'deg'` as appropriate; the values come back in Fusion internal units (cm / radians)
regardless of the unit string (`[PB-EVAL-EXPRESSION]`).

**Units — critical (Module is unitless mm; everything else is already internal):**
- The `'mm'` inputs (Driving/Pinion Base Height, Driving/Pinion Bore Diameter, Face Width, Tooth
  Spacing) and the
  `'deg'` input (Shaft Angle) come back from `evaluateExpression(expr, 'mm'|'deg')` **already in
  internal units** (cm / radians). Use them as-is; do **not** `to_cm` them again.
- **`Module` is read with unit `''`, so it comes back as a raw number that means *millimetres*** (a
  module of `1` is 1 mm). Therefore **every length derived from Module must be `to_cm`-converted
  before it touches geometry**: Pitch Diameter = `to_cm(Module * teeth)`, Cone Distance = `to_cm(...)`,
  dedendum = `to_cm(1.25 * Module)`, the construction extensions to E, F, G, H, I, J (§2 gives each
  one a closed-form seed length),
  and the default Face Width (`Cone Distance / 6`). The `VirtualSpurProxy` likewise receives Module
  in mm and applies the standard spur formulas, then `to_cm`'s the resulting circle radii/diameters
  it serves (they must be in cm, matching what the spur tooth generator expects). Mixing a raw-mm
  Module-derived length with an already-cm `'mm'` input (e.g. comparing Face Width against the
  Module-derived Maximum Face Width) without this conversion makes the gear come out ~10× off and
  the Face-Width bound meaningless. The boolean Enable-Bore input is read with
`get_boolean` (or `input.value`). Both teeth inputs are coerced to whole numbers with
`int(round(…))` before validation. Read all inputs up front in a single `_readInputs` pass that
validates ranges (module > 0, teeth ≥ 3, shaft angle above 30° and below the Maximum Shaft
Angle, non-negative heights/bores/width/tooth-spacing) and returns the primary values, stashing
the rest on `self`. The Maximum Shaft Angle depends on both tooth counts, so check it after both
are read and coerced, and put the computed limit in the rejection message.

The remaining bounds all need the two pitch cone angles, so compute `γ_p` and `γ_g` from the
closed form once the tooth counts and Shaft Angle are known, then, in this order:

1. Check each gear's tooth count against its **Minimum Teeth** floor, `5.27 * cos γ` for that
   gear's own `γ`, on top of the blanket `teeth >= 3`. Name the computed floor in the message.
2. Resolve each gear's **Minimum Base Height** and **Maximum Base Height** — both closed-form,
   see those parameters — and apply them to that gear's base height: raise a fallback that is
   below the minimum, cap one that is above the maximum, and reject a user value outside either
   end naming the bound it broke.

Order matters between those two: the Minimum Teeth check is exactly the statement that the base
height window is non-empty, so running it first means step 2 never has to describe what to do
when the minimum exceeds the maximum.

**The bore bound is not part of this pass.** Validate the two bore diameters here only as
non-negative numbers, and resolve and apply each gear's **Maximum Bore Diameter** in §2 where the
Maximum Face Width is applied (see that parameter). Its heel term would resolve here — it needs
only `r`, `γ` and the base height step 2 just resolved — but its toe term needs the Root Length,
hence the resolved Face Width, hence solved §2 geometry, and the bound is the minimum of the two.

## Architecture

Bevel uses a **standalone generator** — it does **not** subclass `base.Generator` and uses **no
`GenerationContext`**. **One** generator handles **both** straight and spiral bevels: the same class
builds a straight bevel when Mean Spiral Angle ψ = 0 and a curved (spiral) bevel when ψ > 0. There is
**no separate spiral subclass or command** — the spiral is a branch inside the tooth-body step
(`_transformToothBody`, see Method contract), gated on ψ. Two plain classes plus one framework
import (names are the reproduced surface; the entry point binds the two classes by name):

1. **`BevelGearCommandInputsConfigurator`** — `@classmethod def configure(cls, cmd)` that adds the
   **20** dialog inputs above in display order, plus `@classmethod def handle_input_changed(cls, args)`
   (with a private `_updateSpiralInputVisibility(cls, inputs)` helper) that drives the spiral-only
   inputs' conditional visibility (see "Conditional visibility" under Exact input ids). Both
   `configure` and `handle_input_changed` are bound **by name** from `commands/bevelgear/entry.py`.
2. **`BevelGearGenerator`** — `__init__(self, design)` (stores `self.design`, `self.bevelOccurrence
   = None`); `generate(inputs)`; `deleteComponent()`. Creates the occurrence tree directly with
   `parent.occurrences.addNewComponent(...)` — it does **not** use `getOccurrence` / `addParameter`
   / `parameterName` / `createSketchObject` from `base.Generator`, and registers no user parameters.
3. The virtual-spur proxy is **imported from the framework** — `from .spurproxy import
   VirtualSpurProxy` — a fake spur `parent` so the borrowed spur tooth generator can run without
   registering Fusion user parameters. Bevel defines **no local proxy or value-wrapper class**.
   See Dependencies.

From `base.py` import only the input readers (`get_selection`, `get_boolean`) — its
`Generator`/`ParamNamePrefix`/`ComponentCleaner` machinery is unused. Imports are explicit per the
PLAYBOOK's Module-layout rule (no `import *`).

## Generation Context — none

Bevel carries **no `GenerationContext` object** — no `GenerationContext` class and none of the
`base.Generator` context machinery. State is threaded three ways: (a) `_readInputs`
returns a 7-tuple `(parentComponent, targetPlane, centerPoint, module, drivingTeeth, pinionTeeth,
shaftAngle_deg)` and stashes the rest as instance attributes (`self._drivingBaseHeight_cm`,
`self._pinionBaseHeight_cm`, `self._boreEnable`, `self._drivingBore_cm`, `self._pinionBore_cm`,
`self._faceWidth_cm`, `self._toothSpacing_cm`, `self._spiralAngle_rad`, `self._hand`,
`self._cutterRadius_cm`, `self._toeExtension_pct`, `self._drivingToeRadius_cm`,
`self._pinionToeRadius_cm`); `generate()` later stashes the derived `self._coneDistance_cm`,
`self._gamma_p`, `self._gamma_g` (and `_buildGearProfiles` stashes the resolved
`self._faceWidthResolved_cm` and, once the Root Length follows from it, the resolved
`self._drivingToeRadiusResolved_cm` and `self._pinionToeRadiusResolved_cm`);
`self._toeExtension_pct` is the raw unitless percentage the dialog returns, not a length —
inputs 18 to 20 are read the way "Exact input ids" states and every one of the three is stashed
here, since §2 resolves the toe lattice from all three; (b) the per-gear geometric anchors are carried in **plain per-gear
dicts** (`pinionCtx` / `drivingCtx`), built in `_buildGearProfiles` and passed to
`_buildVirtualSpurProfile` / `_createGearBody`, which write eight further entries back into the dict
(every key, its type and its readers: the table "Exact per-gear context dictionary keys" below);
shared anchors are self-stashed
(`self._gearProfilesPlane`, `self._apexSketchPoint`, `self._gpSketch`, `self._apex2d`, the §1
`self._anchorCenterPoint`); (c) `self.bevelOccurrence` holds the top occurrence for cleanup
(`self.designOccurrence` / `self.designComponent` / `self.bevelComponent` hold the inner tree).
The "no ctx" rule means exactly that class-level shape: per-gear plain-dict carriers and self
attributes ARE the intended structure — do not introduce a `GenerationContext`-style class or the
`base.Generator` context.

### Exact per-gear context dictionary keys

**These key strings are part of the reproduced surface, exactly like the input ids above — use them
verbatim.** NEVER rename a key, split the dict, wrap it in a class, or carry one of these values
under a different shape. Both gears carry the same **18** keys and no others.

`written` is the step that puts the key in the dict; `read by` is every step that reads it back. The
first 10 are built with the dict in `_buildGearProfiles`; the last 8 are written back later and read
later still. `SketchPoint` / `SketchLine` entries are the live §2 Gear Profiles sketch entities, not
copies of their coordinates — a reader takes `.geometry` (sketch-local) or `.worldGeometry` itself.

| key | value it carries | type / unit | written | read by |
|---|---|---|---|---|
| `label` | `'Pinion'` or `'Driving'` | `str` | `_buildGearProfiles` | every `{gearLabel}` name (§3's `{gearLabel} Plane` / `{gearLabel} Tooth` / `{gearLabel} Tooth Axis`, `{gearLabel} Profile`, `{gearLabel} Bore`, the `{gearLabel} Gear` component) and the `gearLabel` argument of `_transformToothBody` / `cut_conical_ends` |
| `teeth` | this gear's Teeth Number | `int` | `_buildGearProfiles` | Pattern (`quantity`); the Meshing rotation angle; `_transformToothBody`'s `teethNumber` |
| `gamma` | this gear's pitch cone angle — `γ_p` (Pinion) / `γ_g` (Driving), matching `self._gamma_p` / `self._gamma_g` | `float`, radians | `_buildGearProfiles` | §3 step 1; `_transformToothBody`'s `gamma`, i.e. §3a step G's twist law |
| `pitchDiameter_cm` | this gear's Pitch Diameter | `float`, internal cm | `_buildGearProfiles` | §3 step 1, the virtual pitch radius |
| `toothCenterPoint` | the tooth-center point K′ (Pinion) / L′ (Driving) | `SketchPoint` | `_buildGearProfiles` | §3 step 3, as the spur drawer's `draw(anchorPoint, …)` anchor |
| `toothCenterRefLine` | the tooth-center reference line C->K′ / D->L′ | `SketchLine` | `_buildGearProfiles` | §3 step 2 (`plane_by_angle`); §3 step 4's `setByDistanceOnPath` helper plane |
| `hexVertices` | the six profile vertices in draw order — A', G, H, C, M, N / B', I, J, D, O, P | `list[SketchPoint]`, length 6 | `_buildGearProfiles` | Create the Gear Bodies → Profile sketch |
| `toeEdgePoints` | the toe edge's two endpoints — M and N / O and P, in that order | `tuple[SketchPoint, SketchPoint]` | `_buildGearProfiles` | the `toeMid` midpoint handed to `_transformToothBody` and `cut_conical_ends`; its FIRST element is `toeConeWorld`, per the §3a caller hand-off table |
| `heelEdgePoints` | the heel edge's two endpoints — C and H / D and J, in that order | `tuple[SketchPoint, SketchPoint]` | `_buildGearProfiles` | the `heelMid` midpoint handed to `_transformToothBody` and `cut_conical_ends`; its FIRST element is `heelConeWorld` — the dedendum corner C / D, **NEVER** H / J |
| `boreDiameter_cm` | this gear's Bore Diameter, already resolved AND already bounded — `generate()` resolves the raw value before §2 and §2 applies the Maximum Bore Diameter to it, so no reader re-applies the `/ 4` auto value and no reader re-derives it | `float`, internal cm | `_buildGearProfiles` | Bore |
| `toothPlane` | the `{gearLabel} Plane` construction plane | `ConstructionPlane` | §3 step 2 | `_transformToothBody`'s `parentToothPlane` — §3a step E's first cut plane |
| `toothSketch` | the `{gearLabel} Tooth` sketch | `Sketch` | §3 step 3 | tooth-profile selection (`find_profile_by_curve_counts`) |
| `toothEmbedded` | the spur drawer's `_lastToothEmbedded`, read back off the proxy | `bool` | §3 step 3 | tooth-profile selection, as `wantLines = 0 if toothEmbedded else 2` |
| `toothAxis` | the `{gearLabel} Tooth Axis` construction axis | `ConstructionAxis` | §3 step 4 | nothing — no step reads this key back, and Cleanup hides the axis by entity kind rather than through the dict. It is the one entry with no reader, and it is listed so that a regen that stashes the axis is not read as having invented a key |
| `gearOccurrence` | the `{gearLabel} Gear` occurrence | `Occurrence` | Create the Gear Bodies → Gear component | `moveToComponent`'s destination |
| `profileSketch` | the `{gearLabel} Profile` sketch | `Sketch` | Create the Gear Bodies → Profile sketch | Revolve (its single profile) |
| `shaftAxisEdge` | that sketch's first edge — A'->G / B'->I | `SketchLine` | Create the Gear Bodies → Profile sketch | Revolve axis; Pattern axis; the Bore plane's `setByDistanceOnPath`; Meshing rotation; `_transformToothBody`'s `shaftAxisEdge` |
| `gearBody` | the revolved Gear Body | `BRepBody` | Revolve | the Bore extrude's `participantBodies`. The Combine, the Meshing rotation and `moveToComponent` all run inside the same method as the Revolve and use the local body, so the key exists for the Bore alone |

**Six values are used where they are made and NEVER enter the dict**, so a regen that stashes one
has invented an entry:
- the **Root Axis** (Apex->C / Apex->D) is consumed inside §2 itself, by `addCoincident(M, Pinion
  Root Axis)` / `addCoincident(O, Driving Root Axis)`; §3a rebuilds its direction as `coneVec` from
  `apexWorld` and the heel cone point.
- the **toe and heel cone points** are the first element of `toeEdgePoints` / `heelEdgePoints`, so a
  separate `toeConePoint` / `heelConePoint` key would carry the same entity twice.
- the **shaft-edge point pair** would only duplicate the first two `hexVertices`; the shaft axis
  every body operation uses is `shaftAxisEdge`, the Profile sketch edge, never a §2 point pair.
- the **virtual tooth number** is computed in §3 step 1 and consumed in §3 step 3 by
  `VirtualSpurProxy`, inside the same step.
- the **root sink** is computed and consumed in the same two steps as the virtual tooth number, and
  travels to the drawer as `VirtualSpurProxy`'s `rootSink_mm` argument.
- the **meshing rotation angle** is computed in the Meshing rotation step itself, from `teeth`, and
  is handed straight to `rotate_body_about_edge`.

This table is pinned because nothing else can catch a drift: the dict is written and read inside one
generated module, so a regen that renames every key and every reader together still runs, and six
rebuilds of `lib/geargen/bevelgear.py` each carried these same values under a different shape. The
rebuild that introduced the exact virtual tooth count is the most recent of them: it renamed six of
these keys in one round — `toothCentrePoint`, `toothCentreLine`, `toeEdge`, `heelEdge`, `embedded`
and the bore key — and changed which values reached the dict at all.

## Method contract — call graph

No gear subclasses bevel, so there are **no override boundaries to preserve** (unlike spur). The
only hard external bindings are `commands/bevelgear/entry.py` → `BevelGearCommandInputsConfigurator.
configure(args.command)`, `BevelGearCommandInputsConfigurator.handle_input_changed(args)` (the
dialog's `inputChanged` handler delegates one line to it — drives the spiral inputs' conditional
visibility, see Exact input ids), and `BevelGearGenerator(design).generate(inputs)` +
`deleteComponent()` on failure. The internal decomposition below is the intended structure; private helper names may vary,
but keep the step boundaries (they map to the Instructions sections):

```
generate(inputs)
  → _readInputs(inputs)                      # read+validate all inputs; returns 7-tuple, stashes the rest on self
  → resolve pitch diameters & raw bore diameters (Python, cm)  # bore bound applied later, in _buildGearProfiles
  → build component tree                     # Bevel Gear → Design (Pinion/Driving components made in _createGearBody)
  → _buildAnchorSketch(design, plane, center)        # §1 → anchorLine
  → _buildGearProfiles(...)                  # §2 + §3 + per-gear body creation; internally PER GEAR,
        # pinion fully first then driving — profile→body INTERLEAVED per gear
        # (pinion profile → pinion body → driving profile → driving body),
        # NOT both profiles then both bodies:
        → _buildVirtualSpurProfile(...)      # §3 this gear: tooth plane + spur tooth + axis
        → _createGearBody(...)               # revolve → loft (uncut tooth) → _transformToothBody → pattern → combine → bore → mesh-rotate → moveToComponent
              → _transformToothBody(...)     # the tooth-body step. ψ=0 → solids.cut_conical_ends (2 conical trims, straight bevel). ψ>0 → spiral build (see §3 "Spiral tooth body")
              # mesh-rotate: see "Meshing rotation" under Create the Gear Bodies
  → _hideConstructionGeometry(bevelComponent)        # Cleanup
deleteComponent()                            # error rollback (entry point calls on exception)
```

`_transformToothBody(designComponent, toothBody, gearBody, shaftAxisEdge, apexWorld, apexSketchPoint,
toeMid, heelMid, toeConeWorld, heelConeWorld, parentToothPlane, gearLabel, teethNumber, gamma)` —
`gamma` is this gear's pitch-cone half-angle (γ_p pinion / γ_g driving, from §2), forwarded to the
spiral build's twist law (§3a step G) — is the single
tooth-body hook `_createGearBody` calls after lofting the uncut apex→heel tooth. **`_createGearBody`
builds the four `toeMid / heelMid / toeConeWorld / heelConeWorld` arguments exactly per the §3a
"Caller hand-off" table — toe edge = M→N (pinion) / O→P (driving), heel edge = C→H / D→J; `toeMid`/
`heelMid` are the toe/heel edge MIDpoints and `toeConeWorld`/`heelConeWorld` are M/O and C/D. Getting
this wrong silently inverts the spiral (see §3a).** Its first line is the
gate `if self._spiralAngle_rad <= 0: return cut_conical_ends(...)` (the framework helper from
`.solids` — see "Conical cuts") — straight bevels are byte-for-byte the prior behavior.
`_pinionMeshPhase(pinionTeeth)` returns the pinion's extra mesh rotation about its own
shaft axis (0 for straight; for spiral it is `_PINION_MESH_PHASE_TEETH` tooth-fractions, default 0).

Helpers used by the above come in two kinds.

**Framework helpers (import; do NOT re-implement — behavior pinned in the PLAYBOOK "Shared geargen
helper library"):** from `.solids`: `cut_conical_ends` / `apply_conical_cut` / `select_keeper` /
`find_cone_faces_by_midpoint` / `surface_distance` (the conical-cut machinery — see "Conical
cuts"), `slice_body_by_offset_planes` (§3a step E), `rotate_body_about_edge` (the meshing
rotations), and the spiral-only `plane_by_angle`, `combine_point`, `circle_intersect_nearest`
(§3a steps B–C), plus `hide_construction_geometry` (Cleanup); from `.utilities`:
`find_profile_by_curve_counts` (tooth-profile selection, below); from `.spurproxy`:
`VirtualSpurProxy` (see Dependencies).

**Own helpers (names may vary; behaviour is pinned by the Instructions):** `_readInputs`,
`_cutBore`, and `_pinionMeshPhase(pinionTeeth)` (returns the
pinion's extra mesh rotation in **radians**: `_PINION_MESH_PHASE_TEETH · 2π / pinionTeeth`).

**Tooth-profile selection.** Select the tooth cross-section loop with
`find_profile_by_curve_counts(toothSketch, nurbs=2, arcs=2, lines=wantLines)` — but the line count
is **DETERMINED BY the `embedded` flag, NOT guessed/accepted-either**: `wantLines = 0 if embedded
else 2`. ⚠️ Do **NOT** accept "0 **or** 2 lines" — for a given gear only ONE of those is the real
tooth; an **unrelated** loop (e.g. an inter-tooth or annular region between the drawCircles
circles) can also have 2 NURBS + 2 arcs but the *other* line count, and selecting it makes the
apex→profile loft in "Create the Gear Bodies" fail with `RuntimeError ... ASM_RBI_INTERNAL /
LOFT_NO_TOOLBODY` (the impostor loop can't form a loft tool body). The `embedded` flag is read from
the borrowed spur generator — see Dependencies (`_lastToothEmbedded`); `embedded` ⇒ tip/root/flanks
meet with no connecting lines (4 curves), non-embedded ⇒ 2 connecting lines (6 curves), mirroring
spur's own selection.

Pinion is built first; driving second. The meshing offsets both gears receive are owned by
"Meshing rotation" under Create the Gear Bodies.

## Sketch Discipline (bevel-specific)

Bevel's sketch work differs from the spur family. The bevel-specific Fusion mechanics live in
`fusion.md` (cited below); the general Fusion gotchas in `PLAYBOOK.md` still apply. In brief:

- **Every *permanent* sketch must end fully constrained** (Anchor, Gear Profiles §2, the two
  per-gear Profile sketches, Bore) — a free DOF is a generation defect; gate and raise. The two
  tooth-profile sketches (drawn by the borrowed spur generator) and the spiral build's transient
  auxiliary sketches are **exempt**. Gate, exemptions, and rationale: `[BEVEL-F-FULL-CONSTRAINT]`.
- **§2 lattice lines use the COINCIDENT style, not sharing** — build every §2 line (lattice *and*
  short reference/connector lines) from raw `Point3D` coords and `addCoincident` each endpoint to
  its existing point: `[BEVEL-F-COINCIDENT-STYLE]`. Each named §2 line is created **once** and
  reused, never redrawn: `[BEVEL-F-LINE-ONCE]`. The driven §2 lengths are **not** dimensioned:
  `[BEVEL-F-DRIVEN-DIMS]`.
- **Every §2 seed is load-bearing geometry, and the solved figure is gated against it.** 15 §2
  constraint sites admit a mirrored solution that satisfies every constraint, and no Fusion
  constraint pins any of them — the seed is the only thing that picks the figure:
  `[BEVEL-F-MIRROR-FIGURE]`. §2 therefore states a closed-form seed for every named point and ends
  by comparing the solve against those seeds: `[BEVEL-F-SEED-HELD]`.
- **The §2 figure is positioned in sketch-local 2-D coordinates** (apex = `c + perp·(R·cos γ_g +
  resolved Driving Gear Base Height)`, per §2), never via
  a world position round-trip — this is what keeps the gear off world XY: `[BEVEL-F-APEX-LOCAL]`.
  The grow side is chosen by the target-plane **normal**, not the sketch's local +Y:
  `[BEVEL-F-GROW-SIDE]`.
- **Never activate any occurrence** (`[PB-NEVER-ACTIVATE]`; bevel reason and the sole spiral-crown
  exception in `[BEVEL-F-NEVER-ACTIVATE]`). **Cleanup hides by entity kind** (`[BEVEL-F-CLEANUP]`).

## Generation Order

`generate()` reads **all** inputs first (`_readInputs`), then creates occurrences. Because bevel
registers no user parameters, nothing creates an occurrence until after every selection is already
read, so the selection-context-shift hazard the spur spec warns about does not bite here — but keep
the order (read inputs → build tree → build geometry) so it stays that way. The geometry steps run
in the order of the Instructions below (§1 → §2 → §3 → Pinion → Driving → Cleanup), pinion before
driving. **When ψ > 0**, the per-gear tooth-body step (inside `_createGearBody`, via
`_transformToothBody`) builds the spiral tooth — trace → slice → rotate → loft → crown → flush trim —
*in place of* the straight tooth's two conical trims, before pattern/combine/bore (see §3 "Spiral
tooth body"). The straight (ψ = 0) order and behavior are unchanged.

## Dependencies

Bevel **borrows the spur tooth generator** — `from .spurgear import
SpurGearInvoluteToothDesignGenerator`. It is used only inside `_buildVirtualSpurProfile`, once per
gear:

```python
proxy  = VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth, rootSink_mm=rootSink)
drawer = SpurGearInvoluteToothDesignGenerator(sketch, proxy)
drawer.draw(anchorPoint, angle=math.radians(180))   # the 180° tooth rotation IS the draw() angle
```

The borrowed generator's surface (from `spec/spurgear/instructions.md`): constructor `(sketch, parent, angle=0)`;
`draw(anchorPoint, angle=0)` runs `drawCircles()` → `drawTooth(angle)` → anchor-projection; it reads
parameters via `parent.getParameter(name).value`. So bevel must supply a `parent` exposing
`getParameter(name)` → an object with a `.value`. That is the framework's **`VirtualSpurProxy`**
(`lib/geargen/spurproxy.py` — import it; do NOT define a local copy): it precomputes, in internal
cm, exactly the keys the spur drawer reads (its defaults match bevel: pressure angle 20° — not a
bevel dialog input — and `InvoluteSteps` 15) and returns each wrapped in a `.value` carrier.
Construct it as `VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth,
rootSink_mm=rootSink)` with the raw-mm module, the §3 virtual tooth number and the §3 root sink.

**`virtualTeeth` is a REAL number here, and `rootSink_mm` is what the proxy takes to place the root
circle.** The proxy computes the pitch diameter as `virtualTeeth · module_mm`, so passing the exact
`2 · r_v / Module` is what makes the drawn pitch circle reach the back cone; `rootSink_mm` shortens
the root diameter by twice its value and leaves pitch, base and tip alone. It defaults to 0, which
is what keeps spurgear's own use of the proxy unchanged. Both values are defined in §3 step 1.

**The proxy carries `_lastToothEmbedded` — an OUTPUT the spur generator writes, and bevel MUST read it back.** During `draw()` the spur generator decides whether the tooth is *embedded* (tip/root/flanks meet with no connecting lines) and records it with `self.parent._lastToothEmbedded = <bool>` (it has no ctx of its own); the framework proxy pre-initialises the slot to absorb that write. **After `drawer.draw(...)` returns, read `proxy._lastToothEmbedded` and thread it to the tooth-profile selection (see Method contract)** — e.g. stash it alongside the tooth sketch/plane returned by `_buildVirtualSpurProfile`. This flag is **not optional bookkeeping** — it is the deterministic selector for the tooth loop's line count (`0 if embedded else 2`); skipping it and accepting either count grabs an unrelated loop and the apex→tooth loft dies with `LOFT_NO_TOOLBODY` (see Method contract).

The **180° rotation is delivered through the `draw()` angle argument** (the spur generator rotates
the whole tooth by `angle`, per `spec/spurgear/instructions.md`), *not* a post-hoc Move/sketch rotation — this relies
on spur's radial flank-to-root pinning so the connecting lines rotate with the tooth. (The
construction axis through point K / L, built normal to the tooth plane via `setByTwoPlanes`, is
still created as described in §3.)

## Instructions

### Create the Parent Component

Create the Bevel Gear component as a child of the Parent Component. Name it `Bevel Gear`.

## Creating the Design Component

The Design Component shall contain the necessary sketches and construction planes / axis / etc that will be used by the components creating the actual gears later.

Create the Design component as a child of the Bevel Gear component. Name it `Design`.

### 1: Anchor Sketch

Start the Anchor Sketch (name the sketch `Anchor`) **directly on the user-selected target plane**, whether the selection is a `ConstructionPlane` or a `PlanarFace`; don't re-derive or offset it (`[PB-USE-SELECTED-PLANE]` — re-deriving collapses the gear onto XY). Mark the center point by projecting the user-specified center point onto the sketch.

Create a line through the projected center point. **Seed its two endpoints at exactly ±0.5 cm from the projected center** along the sketch-local X (so the seeded length is 10 mm). Apply **BOTH** `addCoincident(projectedCenter, anchorLine)` (the "intersection" — pins the center onto the line) **and** `addMidPoint(projectedCenter, anchorLine)` (center bisects the line). Use both, not midpoint alone. Add an **aligned distance dimension WITHOUT assigning `.parameter.value`** — the dimension simply locks the length at the seeded 10 mm (the value is arbitrary; it's only a reference line). **Then pin its direction so the sketch ends FULLY CONSTRAINED:** add `addHorizontal(anchorLine)` (sketch-local, per `[PB-REFLINE-DIRECTION]` — works on any tilted target plane; a world-axis lock would mis-orient). The anchor line's absolute direction is arbitrary (nothing downstream depends on it — §2 derives all directions *relative* to the projected anchor line), but it must not be a free degree of freedom: with midpoint + length + Horizontal the line has zero DOF. **Stash the projected-center SketchPoint** (e.g. on `self`) so §2 re-projects *this* anchor-sketch point — not the raw user-selected center — into the Gear Profiles sketch. This line is the Anchor Line. After all constraints, the Anchor Sketch must report `isFullyConstrained` (see the full-constraint gate in Sketch Discipline).

### 2: Gear Profiles

Using setByAngle, create a plane that includes the Anchor Line, set at 90° (by default it would lie flush to the anchor line's plane, but we want it perpendicular). **Build it off the original `targetPlane`** as the reference — don't re-derive/offset it (`[PB-USE-SELECTED-PLANE]`) — this is the other place the target-plane orientation reaches the bodies; substituting a different plane here also collapses the gear onto XY. Name the plane `Gear Profiles Plane`. Create a sketch on this plane, named `Gear Profiles`.

**Every line drawn in this §2 sketch is a construction line (`isConstruction = True`)** — the lattice lines, the toe lines M->N / O->P, the front faces N->A' / P->B', and the short reference/connector lines (M->C, O->D, A'->G, B'->I, C->K/K′, D->L/L′) alike. The solid features later consume only the per-gear Profile sketches (see Create the Gear Bodies), never a §2 curve directly.

**Every length dimension in this §2 sketch is `AlignedDimensionOrientation`.** `addDistanceDimension(pointOne, pointTwo, orientation, textPoint)` takes an `adsk.fusion.DimensionOrientations` value, and this figure has no axis-aligned line in it: the shaft axes sit at the Shaft Angle to each other, the whole lattice tilts with the target plane, and the sketch is not world-aligned. `HorizontalDimensionOrientation` or `VerticalDimensionOrientation` would each dimension the line's *projection* onto a sketch axis instead of its length, so the constrained value would be the intended one only in the accidental case where the line happens to lie along that axis. Wherever a step below says "a dimensional constraint with length = X" — the PPD/2 and DPD/2 drops to Apex 2, the two `Module * 1.25` dedendum lines, the Tooth Spacing dimension on the K′ / L′ lines, the Toe Radius dimension on the two front faces N->A′ and P->B′ — it means an aligned distance dimension of that value. The offset dimensions are a different call, `addOffsetDimension`, which takes no orientation.

In the sketch, project **the Anchor Sketch's center SketchPoint** (the one you stashed in §1) — NOT the raw user-selected center point. Both happen to be coincident, but projecting the anchor-sketch point keeps the chain within the Design component and faithful to the anchor geometry; projecting the raw external point is a cross-component reference and can resolve inconsistently.

**Write the call as `sketch.project(entity)`, and do not substitute `project2`.** The compiled Fusion API reference declares `project2(entities, isLinked)` and no `project`, so every gate in this repo reports the call as unverified; that report is expected and is not a defect to fix here. `project` is what the shipped add-ins call and what the spur step list names, and this repo's settled position is to keep it and keep reporting it — it sits on `fusion_api.py`'s `UNVERIFIED_CALLS`, which is reported, not blocking, and explicitly not waived. The two are not interchangeable in any case: `project2` takes a list and returns a list, so swapping the name alone would be wrong. Only a Fusion session can settle whether `project` exists at runtime, and if it turns out not to, the fix belongs here in the spec rather than in the generated module.

From the projected center point, draw a construction line **perpendicular to the (projected) anchor line, in the gear-profiles sketch's own 2-D frame** (apply a Perpendicular constraint to the anchor line). Its far end is the **Apex**, placed in sketch-local coordinates at `c + perp·(R·cos γ_g + <resolved Driving Gear Base Height>)`, where `c` is the projected center, `perp` is the in-plane unit vector perpendicular to the projected anchor line (`(-d.y, d.x)` for anchor-line direction `d`), and `R`, `γ_g` are the Pitch Cone Distance and driving pitch cone angle from the closed form below. **Seed it at that distance and not at `Driving Gear Pitch Diameter`**, which is what earlier revisions of this spec said: the constraint net closes this line at `R·cos γ_g` above point I plus the resolved driving base height, so for the default 31/31 pair at Shaft Angle 90° the old seed sat 11.6 mm past where the solve puts it (31 mm seeded against 19.375 mm solved). Fusion converges from the far seed, so this was latent rather than broken there, but the bench solver does not, and a seed that disagrees with its own closure by that margin is a seed waiting to pick the wrong branch (`[PB-SEED-NEAR]`). The apex **position** is sketch-local — do **NOT** compute it from a world-coordinate round-trip; that is what caused the XY-collapse. The **sign of `perp`** (which side the gear grows) is chosen by the target-plane normal as a one-bit direction (see Sketch Discipline "Grow side" — toward the normal), NOT by the sketch's local +Y. Read that normal as **`targetPlane.geometry.normal`** for BOTH selection kinds — a `BRepFace`'s `geometry` and a `ConstructionPlane`'s `geometry` are each a `core.Plane` carrying `.normal`. Do NOT add a length constraint on this line.

Create a construction line from the apex representing the Driving Gear Shaft Axis, pointing from the apex back toward the anchor line, i.e. in the `-perp` direction. **Seed its far end at `apex - perp·(R·cos γ_g)`, which is `c + perp·(<resolved Driving Gear Base Height>)`** — measure from the apex, not from `c`. Earlier revisions said `c - perp·(some length)`, which puts B on the far side of the projected centre from the apex, the wrong side of the figure entirely; the closure at Apex 2 drives `|Apex→B|` to `R·cos γ_g`, so B solves to exactly one base height above `c`. It must run **parallel to the center→apex construction line** — apply `addParallel(drivingShaftAxis, centerToApex)`. **Do NOT use `addVertical`**: `addVertical` forces the line to the sketch's world-vertical, which is wrong on a tilted target plane (the gear-profiles sketch is not world-aligned) and over-constrains/mis-orients the figure. The shaft axis must be parallel to the *in-plane* apex direction (`perp`), expressed via `addParallel` to the center→apex line — never an absolute Horizontal/Vertical constraint. Beginning of this line uses a coincidence constraint with the apex. The end of this line shall be called point B. Do **not** dimension the line's length — it is determined by the closing constraint at Apex 2 below.

Create a construction line from the apex representing the Pinion Gear Shaft Axis. The pinion shaft is the Driving Gear Shaft Axis direction rotated about the apex by the Shaft Angle. Rotating by the Shaft Angle has two senses (one to each side of the driving shaft), and they place point A on opposite sides — choosing wrong mirrors the whole gear onto the wrong side of the target plane. **Select the sense (seed) this way: form both candidate point-A positions (the driving-shaft direction rotated about the apex by +Shaft Angle and by −Shaft Angle) and keep the candidate whose endpoint has the greater X coordinate in the Gear Profiles sketch.** Compare the two candidates' X and take the larger — do **not** rotate one fixed sense and only flip it when its X comes out negative; when *both* candidates have a positive X that shortcut keeps the wrong one (this is exactly the side-flip to avoid). The +X-most endpoint is the side away from the anchor sketch's leading direction and is consistent across all target planes. Call the chosen seed direction `pinionDir` (the unit Apex→A direction).

Apply an angular dimension between this line and the Driving Gear Shaft Axis equal to Shaft Angle (the traditional "angle between the two shaft axes" — 90° gives the classic perpendicular bevel pair). **Place its text point inside the Σ wedge so it measures Σ and not its supplement 180−Σ** (`[PB-ANGULAR-DIM]`) — e.g. on the interior bisector of the two shaft directions, `apex + normalize(pinionDir + drivingDir) · (PPD/4)`, where `drivingDir` is the unit Apex→B direction. (The angular dimension only fixes the angle *magnitude*; it does not by itself pin which side the pinion lies on, and the text point does not prevent a frame flip — the pinion side is held by the seed above together with the Apex 2 closure below. The real side-flip hazard lives in the perpendicular drops to Apex 2 — see the ⚠️ on the B→Apex 2 drop.) Beginning of this line should use coincidence constraint with apex. The end of this line shall be called point A. Do **not** dimension the line's length — it is determined by the closing constraint at Apex 2 below.

From A, create a construction line perpendicular to the Pinion Gear Shaft Axis, drawn toward the side where Apex 2 will lie. ⚠️ **Apex 2 sits in the interior wedge *between* the two shaft axes — so this drop must point toward the OTHER (Driving) shaft axis / point B, NOT "toward the anchor line".** Pick the perpendicular sense by the sign of its dot product with the A→B direction (toward the driving shaft), not against a generic "toward the anchor" reference. Apply a perpendicular constraint against the Pinion Gear Shaft Axis. Apply a dimensional constraint with length = Pinion Gear Pitch Diameter / 2 (this equals the pinion's pitch radius at the heel, which is the perpendicular distance from Apex 2 to the Pinion Gear Shaft Axis for any Shaft Angle). Beginning of this line should use coincidence constraint with A. **Naming convention used throughout this spec: this perpendicular drop line (A → its far end, which becomes Apex 2) is what "A->Apex2" / "line A->Apex2" always refers to — it is NOT the Apex->A shaft axis. The two share point A but are different lines (one is the PPD/2 perpendicular drop, the other the shaft axis). Whenever a later step says to pin something to or dimension against "A->Apex2", it means this drop line. The same holds for "B->Apex2" (the DPD/2 drop) vs the Apex->B shaft axis.**

From B, create a construction line perpendicular to the Driving Gear Shaft Axis, drawn toward the side where Apex 2 will lie. ⚠️ **This drop must point toward the OTHER (Pinion) shaft axis / point A** — pick the perpendicular sense by the sign of its dot product with the B→A direction. **Do NOT choose this sense by a "toward the anchor line" reference (i.e. the −perp / center→apex grow direction): the Driving Gear Shaft Axis is itself parallel to that grow direction, so the perpendicular's dot with it is ≈ 0 — a degenerate test that silently selects an arbitrary (usually wrong) side.** Both Apex 2 drops must aim at the *same* interior-wedge point: if this B→Apex 2 drop seeds Apex 2 on the wrong side of the driving shaft while the Pinion's A→Apex 2 drop seeds it on the correct side, the coincidence that closes the two drops at Apex 2 (below) makes the solver **flip the whole figure to the mirror solution** — A, C, D, G, H, K, M, N and A′ all land at negative X, and the pair comes out mirrored about the driving shaft axis. ⚠️ **An earlier revision of this paragraph said the flip is what collapses the pinion dedendum C onto the driving dedendum D. That was wrong, and it misdirected the fix.** The C-onto-D collapse is its own pair of sites — the two `Apex 2 -> dedendum` perpendiculars below — and flipping this drop does not produce it. Nothing in the build refuses the mirrored figure either; what catches it is the end-of-§2 gate (`[BEVEL-F-SEED-HELD]`), and the full site list is `[BEVEL-F-MIRROR-FIGURE]`. Apply a perpendicular constraint against the Driving Gear Shaft Axis. Apply a dimensional constraint with length = Driving Gear Pitch Diameter / 2 (the driving pitch radius at the heel, perpendicular distance from Apex 2 to the Driving Gear Shaft Axis for any Shaft Angle). Beginning of this line should use coincidence constraint with B.

Constrain the end points of the two perpendicular lines from the previous two paragraphs with a coincident constraint. Let this point be called Apex 2. (At Shaft Angle = 90° the four points Apex, A, Apex 2, B form a rectangle. For other shaft angles the figure is a non-rectangular parallelogram-like quadrilateral; the lengths of Apex→A and Apex→B adjust so the perpendicular drops of length PPD/2 and DPD/2 coincide at Apex 2.)

Note that this quadrilateral deliberately lies well above the anchor line. The Apex's upward offset from the anchor line (`R·cos γ_g` plus the resolved Driving Gear Base Height, per the seed above) keeps the whole figure above that line across the supported Shaft Angle range — from 30° up to the Maximum Shaft Angle.

**Seed the along-shaft lengths (Apex→A, Apex→B) with the closed-form cone geometry** so the solver converges on the right branch for any Shaft Angle Σ (these are seed coordinates only — the lengths stay undimensioned, fixed by the Apex2 closing constraint): `tan γ_p = sin Σ · PPD / (DPD + PPD · cos Σ)`, `γ_g = Σ − γ_p`, **Pitch Cone Distance** `R = (PPD/2) / sin γ_p` (this `R`, not the Cone Distance parameter — see the two-lengths note in the Parameters section); seed `|Apex→A| = R · cos γ_p` and `|Apex→B| = R · cos γ_g`. Both cosines are positive for every Shaft Angle the range check admits, which is what the Maximum Shaft Angle is there to guarantee. (γ_p, γ_g are also reused in §3 for the virtual tooth radii.) Seeding A/B merely by a pitch diameter is wrong for Σ≠90° and can send the solver to the wrong branch.

Draw a construction line from Apex to Apex 2. This line shall be called the Pitch Line. Each end of the Pitch Line should be constrained to the respective points using coincidence constraint.

From the Apex 2, create a construction line in either side whose length is constrained Module * 1.25, and are perpendicular to the Pitch Line. Constrain them against the Pitch Line with the Perpendicular constraint. Let the line drawn towards the anchor line be Driving Gear Dedendum, whose end point shall be point D. The one drawn away from the anchor line be Pinion Gear Dedendum; whose end point shall be point C.

**Seed the two ends by dot product against the shaft axes, not by "towards / away from the anchor line".** Let `u` be either unit perpendicular to the Pitch Line. The pinion dedendum direction is the `u` with `u · <unit Apex->A> > 0`, and the driving dedendum direction is its negation, which satisfies `(−u) · <unit Apex->B> > 0`. Those two dot products are exactly `sin γ_p` and `sin γ_g`, which are strictly positive for every configuration the range checks admit — unlike the anchor-line test the B→Apex 2 drop warns about, which reads ≈ 0 by construction. Seed **C = Apex 2 + 1.25 · Module · <pinion dedendum direction>** and **D = Apex 2 + 1.25 · Module · <driving dedendum direction>**.

⚠️ **These two sites are where the "C collapses onto D" symptom lives, and each is held by its seed alone.** The perpendicular constrains the line's direction and the dimension constrains its magnitude; neither picks a side, so each end has two solutions and the solver takes the seeded one. Flip the pinion seed and C solves exactly onto D; flip the driving seed and D solves onto C. The collapsed figure inverts that gear — the toe ends up *outside* the heel, the revolved frustum is degenerate, and the conical end-cut finds no cone face at the toe midpoint → `face dist = inf`. Seeding them from a direction such as "away from the anchor line" is not enough: it is a description of one figure rather than a value the end-of-§2 gate can check (`[BEVEL-F-SEED-HELD]`).

Draw two construction lines, from the Apex to the point D and point C, respectively. Apply coincidence constraints on beginning and end of these lines. These lines shall be the Root Axis for driving and pinion gear, respectively.

From point A, create construction line collinear with the line from Apex to point A. **Seed its far end at the closed form `E = A + <unit Apex->A> · (1.25 · Module · sin γ_p)`** (but do NOT add dimensional constraint). E is the foot of the perpendicular dropped from C onto the Pinion Gear Shaft Axis, which is what `C->E ⊥ A->E` below closes it on, so `|Apex->E| = R · cos γ_p + 1.25 · Module · sin γ_p`. Earlier revisions seeded this line one Module long; that is the correct side but not the solved position, and a seed that is not the solved position cannot be gated (`[BEVEL-F-SEED-HELD]`). The line should receive a collinear constraint, and the end of Apex->A and the beginning of the new line should be constrained via coincidence constraint. The end of this new line shall be point E.

Draw a construction line from point C to point E. constrain each end to respective points from pre-existing lines. The lines A->E and C->E should be constrained with perpendicular constraint.

From point B, create a construction line collinear with Apex->B. **Seed its far end at the closed form `F = B + <unit Apex->B> · (1.25 · Module · sin γ_g)`** (but do NOT add dimensional constraint) — the driving twin of E, closed by `D->F ⊥ B->F` below, so `|Apex->F| = R · cos γ_g + 1.25 · Module · sin γ_g`. The line should receive a collinear constraint, and the end of Apex->B and the beginning of the new line should be constrained via coincidence constraint. The end of this new line shall be point F.

Draw a construction line from point D to point F. constrain each end to respective points from pre-existing lines. The lines B->F and D->F should be constrained with perpendicular constraint.

Draw a construction line from point E collinear to **line A->E** — the collinear names A->E, **never the Apex->A shaft axis further up the chain**, even though both describe the same infinite line (`[BEVEL-F-COLLINEAR-CHAIN]`; naming the axis raises `VCS_SKETCH_OVER_CONSTRAINTS`). **Seed its far end at the closed form `G = A + <unit Apex->A> · <resolved Pinion Gear Base Height>`**, i.e. `|E->G| = <resolved Pinion Gear Base Height> − 1.25 · Module · sin γ_p` (but do NOT add dimensional constraint). That length is strictly positive because the Minimum Base Height keeps every resolved base height above `1.25 * Module * sin γ` with a 1.05 margin, which is what puts this seed on the correct side with room to spare. Constrain point E and the beginning of this line. Let the end be known as point G.

From point C, draw a line seeded at the closed form **`H = Apex 2 + <unit Apex2->C> · (<resolved Pinion Gear Base Height> / sin γ_p)`**, i.e. `|C->H| = <resolved Pinion Gear Base Height> / sin γ_p − 1.25 · Module` (but do NOT add dimensional constraint), positive by the same Minimum Base Height. Constrain point C and the beginning of this line. Let the end of the new line be point H. Line C->H should be collinear with **line Apex2->C**, the Pinion Dedendum line C is the endpoint of (`[BEVEL-F-COLLINEAR-CHAIN]`).

⚠️ **These two seeds are what pick the side of the pinion base-height offset dimension below, which is unsigned.** Flip them and G sits one base height on the Apex side of A instead of beyond it, H follows, and the pinion's heel end folds back inside the figure. Seeding at one Module — what earlier revisions did — lands on the correct side but not on the solved position, so the end-of-§2 gate cannot use it (`[BEVEL-F-SEED-HELD]`).

Connect point G and H with a line. Constrain end points of line accordingly with coincidence constraints. **Constrain line E->G and H->G with a perpendicular constraint.**

⚠️ **This perpendicular is required in Fusion and must be omitted in the proof harness, and the reason is a difference between the two engines rather than a choice.** `addOffsetDimension` in Fusion is a *distance* dimension with a precondition: its documentation requires the second entity to be "a line that is parallel to the first", and it controls only the perpendicular distance. So the parallelism has to exist before the pinion base-height offset below can be applied at all, and this perpendicular is what supplies it — E->G runs along the pinion shaft, so making H->G perpendicular to it makes H->G parallel to the A->Apex2 drop, which is perpendicular to that same shaft. Perpendicular plus offset is two equations for two freedoms, and nothing is redundant.

The proof harness's offset constraint is not the same shape. It emits **two** residual rows, holding *both* endpoints of the target line at the same signed perpendicular distance from the source, so it carries the parallelism itself. Adding this perpendicular there is a third row for the same two freedoms: measured, the lattice comes back **overconstrained at DOF 0 with 2 redundant constraints, and the engine names the two base-height offsets as the redundant pair**. A proof that models Fusion's arity here will therefore fail its own gate, and the right response is to leave the perpendicular out of the proof and say so, never to weaken the gate (`[PB-NO-OVERCONSTRAIN]`).


Draw a construction line from point F collinear to **line B->F** — the driving twin of the E->G case above, and the collinear names B->F, never the Apex->B shaft axis (`[BEVEL-F-COLLINEAR-CHAIN]`). **Seed its far end at the closed form `I = B + <unit Apex->B> · <resolved Driving Gear Base Height>`**, i.e. `|F->I| = <resolved Driving Gear Base Height> − 1.25 · Module · sin γ_g` (but do NOT add dimensional constraint), positive by the driving gear's own Minimum Base Height. Constrain point F and the beginning of this line. Let the end be known as point I.

From point D, draw a line seeded at the closed form **`J = Apex 2 + <unit Apex2->D> · (<resolved Driving Gear Base Height> / sin γ_g)`**, i.e. `|D->J| = <resolved Driving Gear Base Height> / sin γ_g − 1.25 · Module` (but do NOT add dimensional constraint). Constrain point D and the beginning of this line. Let the end of the new line be point J. Line D->J should be collinear with **line Apex2->D**, the Driving Dedendum line D is the endpoint of (`[BEVEL-F-COLLINEAR-CHAIN]`).

⚠️ **The driving pair's seeds carry the whole figure, because "Constrain Point I with center point" below hangs everything off I.** Flip the driving base-height offset's side and the entire lattice drops by twice the resolved Driving Gear Base Height, gear and pinion together, with every relative length still correct — which is why nothing downstream refuses it.

Connect point I and J with a line. Constrain end points of line accordingly with coincidence constraints. **Constrain line F->I and J->I with a perpendicular constraint** — the driving-side twin of the G->H case just above, required in Fusion and omitted in the proof for the same reason, which that paragraph gives in full.

Create an **offset dimension between the B->Apex2 perpendicular drop line (the DPD/2 drop per the naming convention above — NOT the Apex->B shaft axis) and J->I**. J->I is **already parallel** to the drop by construction (J->I ⊥ F->I, which runs along the driving shaft), so add **no** extra parallel constraint (`[PB-OFFSET-DIM]`). Set the value equal to Driving Gear Base Height _if_ specified (non-0); otherwise `module * Driving Gear Teeth Number / 8`. Either way this is the value **after** the driving gear's Maximum Base Height has been applied (a fallback capped to it, a user value already rejected if it exceeded it), because the offset set here is what drives the heel edge D->J toward the shaft axis. **`addOffsetDimension` is unsigned and does not pick which side of the drop J->I lands on** — the I and J seeds above are the only thing that does (`[BEVEL-F-MIRROR-FIGURE]`).

Create an **offset dimension between the A->Apex2 perpendicular drop line (the PPD/2 drop per the naming convention — not the Apex->A shaft axis) and G->H** — already parallel by construction (G->H ⊥ E->G), so as with the driving side add no parallel constraint (`[PB-OFFSET-DIM]`). It is unsigned in the same way, and the G and H seeds above are what pick its side. The value should be equal to Pinion Gear Base Height _if_ specified (non-0); otherwise the **RESOLVED** Driving Gear Base Height `* (Pinion Gear Teeth Number / Driving Gear Teeth Number)`. "Resolved" means the value the driving offset above actually used — i.e. after the driving side's own fallback (`module * Driving Gear Teeth Number / 8` when the driving input was 0) **and** after the driving Maximum Base Height capped it — NOT the raw driving input. Then apply the **pinion's own** Maximum Base Height to the result: the two gears have different pitch cone angles whenever the tooth counts differ, so the driving cap does not imply the pinion's, and a scaled-down driving height can still overshoot the pinion's own heel limit.

Draw a line from A' to G, the hexagon's shaft-axis edge. Constrain endpoints appropriately. It starts at the front face's foot A', not at A; the two coincide at Toe Extension 0. **This line is what CREATES A'** — nothing above it does — so draw it with its start seeded at the closed form `A' = Apex + <unit Apex->A> · <the along-shaft coordinate of N>`, the foot of the perpendicular from N onto the pinion shaft axis. The front face N->A' further below is what PINS A' to that axis; until then A' is a free endpoint sitting at its seed. Draw the line here rather than after the front face, so the hexagon's edges are created in the walk order `A' -> G -> H -> C -> M -> N` that the Profile sketch's first-edge rule depends on.

Constrain Point I with center point.


Draw a construction line away from Apex, starting from point G, extending along Apex->A, and call its end point K. Then **pin K with two point-on-line coincident constraints** — `addCoincident(K, line Apex->A)` and `addCoincident(K, the Pinion Dedendum line Apex2->C extended)` — rather than `addCollinear` on the connecting lines. By the time K is added, G and C are already fixed, so an `addCollinear` here over-constrains the sketch and Fusion errors; the two point-on-line coincidents locate K exactly (intersection of the two lines) without over-constraining. Draw a construction line from point C to K for reference.

**Tooth-center point K′ (Tooth Spacing offset).** The §3 spur tooth is centered not at K but at a tooth-center point **K′**, obtained by shifting K outward along the dedendum line by **Tooth Spacing**, *away from the lower corner C*. **When Tooth Spacing is 0 (the default), do NOT build anything here — set K′ ≡ K and reuse the C->K reference line** (a zero-length dimensioned line would be degenerate, and one segment gets ONE line — `[BEVEL-F-LINE-ONCE]`). When Tooth Spacing > 0: draw a construction line starting at K with its far end seeded at the closed form **`K′ = Apex 2 + <unit Apex2->C> · (<the pinion's virtual pitch radius> + Tooth Spacing)`**, which is K plus Tooth Spacing along `Apex2->C`, on the far side of K from C. **"Virtual pitch radius" here is the exact back-cone radius `(Pinion Gear Pitch Diameter / 2) / cos γ_p` that §3 step 1 defines, and never a radius rebuilt from a tooth count.** That is what `|Apex 2 -> K|` measures: the dedendum line Apex2->C is perpendicular to the Pitch Line, which meets the pinion shaft axis at γ_p, so walking `r_p / cos γ_p` along it from Apex 2 lands exactly on the axis, at K. Reading the term as a rounded count times half a Module puts the seed 0.4203 mm short on the shipped default geometry — 31 teeth, Module 1, Shaft Angle 90° — which is 420 times the `[BEVEL-F-SEED-HELD]` tolerance below. Pin its far end **the same way K is pinned to its line** — `addCoincident(start, K)` and `addCoincident(K′, the Pinion Dedendum line Apex2->C extended)` to keep K′ on the dedendum line — then add a **length dimension on this line = Tooth Spacing** (do **not** use `addCollinear`, for the same over-constraint reason as K). ⚠️ **That length dimension is unsigned, so the point-on-line pin plus the length admit K′ one Tooth Spacing on the C side of K just as readily — the two candidates sit `2 × Tooth Spacing` apart — and this seed is the only thing that rules the wrong one out** (`[BEVEL-F-MIRROR-FIGURE]`). A flipped K′ tightens the mesh by the clearance the input asked to add, and builds a gear that looks right, so state the seed as this formula rather than as a direction — a direction cannot be gated (`[BEVEL-F-SEED-HELD]`). The far end is K′. Build it **here, inside the Gear Profiles sketch, before that sketch's end-of-step full-constraint gate**, so the gate covers it. Finally draw the **tooth-center reference line C->K′** (from the lower corner C to K′) for §3 to use in place of C->K. Only the tooth's center moves; the virtual tooth number and drawn tooth size are unchanged (see §3).

At this point all of A, B, C, D, H, J exist **and are solved**, so resolve the **Maximum Face Width** (see the Parameters section) from their solved `.geometry` (NOT the seed coordinates — see that section) and apply it before using Face Width below: cap the auto default to it, and reject a user value that exceeds it. Skipping this — or computing it from seeds — makes the M->N / O->P line push N/P across the shaft axis for asymmetric tooth counts (either gear can be the smaller, binding side), which fails the gear-body revolve with `ASM_WIRE_X_AXIS`.

With the Face Width resolved the **Root Length** follows (see that parameter), so this is also where each gear's **Maximum Bore Diameter** resolves, and where it is applied to that gear's bore: cap an auto-calculated bore to it, and reject a user value above it with a message naming the maximum. Do this for both gears here, before either body is revolved, and skip it when Enable Bore is unchecked. This is the only step at which the whole bound can resolve — its toe term needs the Root Length — which is why the input-reading pass deliberately leaves the bore diameters unbounded (see Maximum Bore Diameter).

Create line M->N. **Seed BOTH ends at their closed-form solved positions, not near them** (`[PB-SEED-NEAR]`). Seed M on `Apex->C` at the fraction `1 - <Root Length> / |Apex->C|` from the Apex. Then seed N by sliding from that M seed along the `C->H` direction by exactly

    (<M seed's perpendicular distance from the Pinion Gear Shaft Axis> - <Pinion Gear Toe Radius>) / cos γ_p

**The slide runs in the `C->H` sense — from C toward H, the same outward sense as `Apex2->C`
continued — and along it the perpendicular distance from the Pinion Gear Shaft Axis FALLS at
`cos γ_p` per unit** (the along-shaft coordinate rises at `sin γ_p`, the same walk the Maximum Base
Height derives its crossing from). That is why the quantity above is divided by `cos γ_p`: the slide
gives back exactly `<M seed's perpendicular distance> - <Toe Radius>` of perpendicular distance, so
N lands at the Toe Radius. **Read as a rise it is the wrong sign**, and it is the natural
misreading: `C->H` runs outward from the figure, so "slide outward" reads as "move away from the
axis", while the dedendum line leans back toward the axis as it goes. Taking the sign that way puts
the N seed at `<M seed's perpendicular distance> + <Toe Radius>` rather than at the Toe Radius; the
compile round that first wrote this rule reported 20 of its 21 lattice cases failing to converge
until it corrected the sign. The pinion's `C->H` unit direction is
`sin γ_p · <unit Apex->A> - cos γ_p · <the A->Apex2 drop direction>`, and the negative second term
is the whole of the rule.

⚠️ **A seed that merely lands somewhere plausible is not enough here, and a wrong one builds the wrong gear rather than failing to converge.** N's position is fixed by the toe line together with a LENGTH dimension on the front face, and a length is unsigned: the toe line meets the Toe Radius on BOTH sides of the shaft axis, so the solver takes whichever side the seed starts on. Seeded below the axis it converges happily onto the mirror, N comes out on the far side, and the revolved hexagon crosses its own axis of revolution — Fusion then aborts the revolve with `ASM_WIRE_X_AXIS` (`[PB-REVOLVE]`) at S16, pointing at the revolve rather than at the seed that caused it.

Two earlier seeding rules are now known to do exactly that, so do not reinstate either: sliding from the M seed by the **Root Length**, and sliding by the **distance from the M seed to A**. Both were written for the scheme that pinned N to the `A->Apex2` drop, where A was N's real target. Measured on the shipped default pair, module 1 with 31/31 teeth at Shaft Angle 90° and a Toe Extension of 50%: the Root Length slide puts the N seed at a perpendicular distance of **-0.27 mm** from the shaft axis — past it — against a solved N at **+5.17 mm**, and Fusion refuses the revolve. The slide above puts it at 5.17 mm exactly. Do NOT seed M/N just `Face Width` away from C/H either — that starts N near H, far from its constraint target.

**The proof cannot catch a wrong seed here, and must say so beside its own seeding.** It seeds M and N at the closed form, which is the rule above, so it proves that the constraints solve from a correct seed and never that the module's seed is correct. A seed defect therefore reaches Fusion untested, which is how the one described above got there. Record that limit in the proof file next to the toe-line seeding, as the honest edge of what this stage checks.

**The toe lattice has been through Fusion at a non-zero Toe Extension once — loaded 2026-09-17,
from the build at `2ad1e32` (PR #159; see Maximum Shaft Angle above for that load's other five
configurations).** A non-zero Toe Extension with a toe radius built with no error, which is the
first time this seeding has run anywhere but at Toe Extension 0, where M and N sit at today's
profile by construction. The revolve is what an N seed on the far side of the shaft axis aborts, and
it did not abort, so this load says the seed landed on the correct side for the one configuration it
built. **It says nothing more**: neither the Toe Extension nor the Toe Radius was recorded, no
solved position of M, N or A′ was read back, and the front face's radius was not measured, so the
toe end's shape is still checked only where the proof checks it.

Then apply **exactly these constraints** — all three are required, and the front face below is what holds N off the shaft axis, which is what the pre-Toe-Radius scheme used the A->Apex2 pin for:
- `addCoincident(M, Pinion Root Axis)` — M lies on the Apex->C root axis;
- `addParallel(M->N, C->H)` — the toe line is parallel to C->H;
- `addOffsetDimension(C->H, M->N, textPoint).parameter.value = <the Root Length re-measured perpendicular to the pitch line, i.e. Root Length * R / |Apex->C|>` — an offset dimension controls a perpendicular distance, so it carries the root length in that form. At Toe Extension 0 the value is exactly the resolved Face Width, which is what this dimension has always been. Place the `textPoint` in the gap between C->H and M->N on the Apex side (e.g. the midpoint of the M-seed and point C, `(M_seed + C)/2`) so the dimension reads cleanly (`[PB-OFFSET-DIM]`). ⚠️ **The toe's side relative to the heel (`toe→Apex < heel→Apex`) is held by the M seed above and by nothing else.** An earlier revision of this sentence said it follows from the §2 frame being built correctly, in particular from the Apex 2 drops; that was wrong. `addOffsetDimension` is unsigned, so a correctly built frame still admits M->N one root length on the *far* side of C->H, where the toe lands outside the heel and the revolved frustum is degenerate. The text point does not control it either. The seed is the whole of the rule (`[BEVEL-F-MIRROR-FIGURE]`), and the end-of-§2 gate is what confirms the solve took it (`[BEVEL-F-SEED-HELD]`).

Let the beginning of this new line be point M, the end be point N. Draw a line from M to C.

**The front face A'->N, which is what holds N.** ⚠️ **N is NOT pinned to line A->Apex2.** Earlier revisions pinned it there, which fixed its station at A's and made the Maximum Face Width the value at which N reached A. It now rides the **Pinion Gear Toe Radius** instead, and the line that holds it there is the gear's front face:

- Draw a line from N to a new point **A'**, seeding A' at N's station on the shaft axis.
- `addCoincident(A', line Apex->A)` — A' lies on the **Apex->A shaft axis**. A' is the only toe-end point that touches that axis, and it is a *foot*, not a corner.
- `addPerpendicular(N->A', line Apex->A)` — the front face stands square to the shaft, so the revolve sweeps it into a flat annulus.
- `addDimension(N->A') = <resolved Pinion Gear Toe Radius>` — an aligned distance dimension on the whole line, per "Every length dimension in this §2 sketch is `AlignedDimensionOrientation`" above.

⚠️ **Pinning N itself to the Apex->A shaft axis remains forbidden** — that would put N *on the axis of revolution*, and the later conical split fails with `ASM_API_FAILED` for asymmetric tooth counts even though the symmetric 45° case happens to survive. A' sits on the axis; N never does, because the Toe Radius is strictly positive. Those three rows plus the offset above and `addCoincident(M, Pinion Root Axis)` fully constrain M, N and A' — six freedoms, six constraints — which is the arity the old drop pin and the old N->A connector had between them.

**A' replaces A as the hexagon's first vertex** (see the table under "Create the Gear Bodies"). At Toe Extension 0 with a defaulted Toe Radius the two coincide exactly, so nothing moves; a positive Toe Extension walks A' along the shaft axis toward the Apex and the shaft edge grows by that much.

Draw a construction line away from Apex, starting from point I, extending along Apex->B, and call its end point L. **Pin L the same way as K** — `addCoincident(L, line Apex->B)` and `addCoincident(L, the Driving Dedendum line Apex2->D extended)`; do not use `addCollinear`. Draw a construction line from point D to L for reference.

**Tooth-center point L′ (Tooth Spacing offset).** Build the driving-side tooth center **L′** exactly as K′ on the pinion side, substituting L for K, D for C, and the Driving Dedendum line Apex2->D for the pinion's; the reference line for §3 is **D->L′**. Same single Tooth Spacing value, same full-constraint gate, same reuse-the-existing-line rule at 0. The seed formula and the unsigned-length ⚠️ carry over unchanged: seed **`L′ = Apex 2 + <unit Apex2->D> · (<the driving gear's virtual pitch radius> + Tooth Spacing)`**, taking "virtual pitch radius" as the same exact back-cone radius `(Driving Gear Pitch Diameter / 2) / cos γ_g` §3 step 1 defines, which is `|Apex 2 -> L|`. The flipped twin, one Tooth Spacing on the D side of L, is ruled out by that seed alone.

Create line O->P, the mirror of M->N on the driving side. **Seed it the same way, at the closed-form solved positions**: O on `Apex->D` at the fraction `1 - <Root Length> / |Apex->D|`, then P slid from that O seed along `D->J` by `(<O seed's perpendicular distance from the Driving Gear Shaft Axis> - <Driving Gear Toe Radius>) / cos γ_g`. **The falling-distance rule carries over word for word**: along `D->J` the perpendicular distance from the Driving Gear Shaft Axis falls at `cos γ_g` per unit, so the slide lands P at the Driving Gear Toe Radius, and reading it as a rise is the same wrong sign. The ⚠️ above applies here unchanged: the length dimension on the front face is unsigned, so a P seed on the far side of the shaft axis converges onto the mirror and the revolve aborts. Then apply the same three constraints:
- `addCoincident(O, Driving Root Axis)` — O on the Apex->D root axis;
- `addParallel(O->P, D->J)`;
- `addOffsetDimension(D->J, O->P, textPoint).parameter.value = <the Root Length re-measured perpendicular to the pitch line>` — as for the pinion, place the `textPoint` in the gap on the Apex side of D->J (e.g. `(O_seed + D)/2`) so it reads cleanly (`[PB-OFFSET-DIM]`). This dimension is unsigned exactly as the pinion's is, so the O seed is what keeps the driving toe inside its heel; the ⚠️ on the pinion offset applies here word for word.

Let the beginning of this new line be point O, the end be point P. Draw a line from O to D.

Build the driving front face **B'->P** exactly as the pinion's A'->N, substituting B for A, P for N and the **Driving Gear Toe Radius** for the pinion's: the line P->B', `addCoincident(B', line Apex->B)`, `addPerpendicular(P->B', line Apex->B)` and a length dimension on P->B'. The same ⚠️ applies — P is never pinned to the Apex->B shaft axis, only B' touches it. Draw line from B' to I.

**End of §2 — gate the solved figure against its own seeds (`[BEVEL-F-SEED-HELD]`).** After the Gear Profiles sketch's full-constraint gate passes, compare every named §2 point's solved `.geometry` against the closed-form position this section seeded it at, and **raise** naming the first point that has moved, with its solved position and its seeded one. Check the points in the order §2 creates them — Apex, B, A, Apex 2, C, D, E, F, G, H, I, J, K, K′, M, N, A′, L, L′, O, P, B′ — so the message names the earliest site that flipped rather than a downstream symptom. Every one of those points has a closed-form seed stated above, so this is one comparison with no per-point exceptions. **The list is 22 points only when Tooth Spacing is above zero. At Tooth Spacing 0 — the default — K′ ≡ K and L′ ≡ L are not built at all, so drop those two and compare 20**; comparing a K′ that was never created is the one way this gate can raise on a correct figure. The tolerance, the point list's authority and the reason this is a gate rather than a constraint are all `[BEVEL-F-SEED-HELD]`.

### 3: Gear Tooth Profiles

**API note for this whole section:** pass the relevant **sketch line directly** to `setByAngle` and
`setByDistanceOnPath`; never wrap it in `Path.create` first (`[PB-CONSTRUCTION-PLANES]`).

**Throughout this section the tooth center is the §2 tooth-center point K′ (pinion) / L′ (driving) and the center reference line is C->K′ / D->L′ — which equal K / L and C->K / D->L exactly when Tooth Spacing is 0.** The virtual tooth number below is computed from the pitch diameter and is **independent of Tooth Spacing**; the spacing offset moves only the center, not the tooth size.

Do all four steps below **once per gear** — pinion first, then driving — with this gear's parameters: pinion uses tooth-center **K′**, reference line **C->K′**, and pitch-cone half-angle **γ_p** (from §2: `tan γ_p = sin Σ · PPD / (DPD + PPD · cos Σ)`); driving uses **L′**, **D->L′**, and **γ_g = Σ − γ_p**.

1. Compute this gear's virtual (back-cone / Tredgold) tooth number from the closed form, **not** by measuring Apex2->K′/L′: virtual pitch radius = `(this gear's Pitch Diameter / 2) / cos(γ)`. Virtual tooth number = `2 · virtualPitchRadius / Module`, equivalently `this gear's Teeth / cos(γ)`.

   **It is a real number and is NEVER rounded** — not floored, not ceiled, not cast to an int. The Tredgold construction puts the equivalent spur gear's pitch radius exactly at the back-cone distance `r / cos γ` with this gear's own module, and `z_v = z / cos γ` is a real number in every published form of it (NPTEL Machine Design II ch. 13 eq. 13.1–13.2; Osakue et al., *FME Transactions* 49(3), 2021, §2.2; the KHK gear technical reference eq. 11.6). Rounding it rebuilds every drawn circle from the rounded count, which draws the tooth smaller than the back cone places it and shortens the working addendum: on the shipped default — 31 teeth, Module 1, Shaft Angle 90°, γ = 45° — the exact virtual pitch radius is 21.9203 mm, a floored count of 43 draws 21.5 mm, and the addendum the tooth works over falls to 0.5797 mm against a nominal 1.0 module.

   **The real count reaches the spur drawer only as an angular half-thickness.** The drawer reads `ToothNumber` as a float and uses it in one place, `π / (2 · toothNumber)`, the angle it rotates the flank to so the pitch crossing lands there (`spec/spurgear/instructions.md`, "the pitch crossing"). With `z_v = 2 · r_v / Module` that angle gives a tooth thickness of `π · Module / 2` at the pitch circle, which is the standard tooth thickness — the same thickness the spur gear of this module carries. An INTEGER count drawn at the exact radius gives `π · r_v / round(z_v)` instead, which misses nominal by a different amount on each member of an unequal pair, so the two teeth of one pair no longer carry the same thickness.

   **Root sink.** Draw the root circle one **root sink** `0.05 · 2.25 · Module` INSIDE the dedendum corner rather than at it. At the dedendum corner exactly, the tooth's root arc touches the gear body's root cone only where the arc crosses the tooth's own centreline: the tooth is drawn on the back-cone plane, so only a point on that centreline rides the cone its own polar radius names, and the arc's two corners stand outside it — by 0.002 module on the default pair and 0.027 module on a 4/4 pair, the largest of any pair the spec admits. The sink pushes the whole arc inside, so the Combine-Join meets the gear body across the root rather than along one line.

   So the four circles the proxy is asked for are:

   | circle | radius |
   |---|---|
   | pitch | `virtualPitchRadius` |
   | base | `virtualPitchRadius · cos(20°)` |
   | tip | `virtualPitchRadius + Module` |
   | root | `virtualPitchRadius − 1.25 · Module − rootSink` |

   **Units — pin the cm→mm conversion:** the stashed pitch diameters are internal **cm** while Module is the raw **mm** value, so compute `virtualPitchRadius_mm = (pitchDia_cm · 10 / 2) / cos(γ)` — the `· 10` converts cm to mm — and then `virtualTeeth = 2 · virtualPitchRadius_mm / Module`. Skipping the ×10 makes the virtual tooth count ~10× off (see the Units note).

2. Create a new plane that includes the tooth-center reference line, named `{gearLabel} Plane`. Use setByAngle to make this plane perpendicular to the Gear Profiles sketch plane (`plane_by_angle` from `.solids`).

3. Using the new plane and the tooth-center point as the center point, create a spur gear tooth profile with module and the virtual tooth number from step 1, in a sketch named `{gearLabel} Tooth`. Draw it **already rotated 180°** by passing `angle=math.radians(180)` to the spur tooth generator's `draw(anchorPoint, angle=…)` (see Dependencies) — the generator rotates the whole tooth by that angle; do not draw it flat and rotate the sketch afterward. **After `draw()` returns do NOT hard-gate this sketch: log if `not toothSketch.isFullyConstrained`, never raise** — the tooth-profile sketches are exempt from the full-constraint gate (see Sketch Discipline: the embedded low-tooth-count tooth is legitimately under-constrained, and the profile is consumed immediately by the loft).

4. Create a construction axis (named `{gearLabel} Tooth Axis`) through the tooth-center point, normal to the plane the tooth profile was drawn on, via `setByTwoPlanes` (`[PB-CONSTRUCTION-AXES]`; `setByPerpendicularAtPoint` would need a `BRepFace`). The two planes are: the **Gear Profiles plane** and a **helper plane built `setByDistanceOnPath(<tooth-center reference line>, 1.0)`** (perpendicular to that line at its far end, the tooth-center point); their intersection is the line through the tooth center normal to the tooth plane. (Creating this axis in the never-activated Design component is proven to work — `constructionAxes.add` via `setByTwoPlanes` does not hit `[PB-CONSTRUCTION-NEEDS-ACTIVE]` here; keep the axis.)

### 3a: Spiral tooth body (ψ > 0)

This is the ψ > 0 branch of the tooth-body hook `_transformToothBody` (Method contract) — it **replaces** the straight tooth's two conical trims with a curved tooth. When **ψ = 0 the hook returns immediately** with the framework's `cut_conical_ends` (the straight tooth, trimmed to a flush band — byte-for-byte the prior behavior); everything below runs **only when ψ > 0**. It is invoked once per gear (pinion then driving), inside `_createGearBody`, on the freshly lofted uncut apex→heel `toothBody`, before pattern/combine/bore. The arc math it realizes is derived in `spiral-tooth-trace.md`; this section states **how** that construction is realized as Fusion sketches/features and the order it runs in. Use the existing `{gear}` sketch-naming (`gearLabel` is `Pinion` or `Driving`).

**Caller hand-off — the four toe/heel world points `_createGearBody` builds and passes to the hook (PIN EXACTLY; mislabeling them silently inverts the spiral — this is the single biggest spiral-regen hazard).** The §2 lattice gives each gear a **toe edge** (the inner face-width edge, nearer the apex) and a **heel edge** (the outer edge, at the back cone). Per gear they are exactly:

| gear | toe edge (inner) | heel edge (outer) | `toeConeWorld` | `heelConeWorld` |
|---|---|---|---|---|
| Pinion | **M→N** | **C→H** | **M** | **C** |
| Driving | **O→P** | **D→J** | **O** | **D** |

From those §2 sketch points' **world** geometry, `_createGearBody` computes (and passes positionally into `_transformToothBody` in the order `toeMid, heelMid, toeConeWorld, heelConeWorld`):
- `toeMid` = world **midpoint of the TOE edge** — ½(M+N) pinion / ½(O+P) driving.
- `heelMid` = world **midpoint of the HEEL edge** — ½(C+H) pinion / ½(D+J) driving.
- `toeConeWorld` = the **toe edge's inner endpoint** — **M** (pinion) / **O** (driving). This point lies on the **root cone element** (the Pinion Root Axis Apex→C / Driving Root Axis Apex→D) at the inner/toe end — M is pinned onto Apex→C in §2 (`addCoincident(M, Pinion Root Axis)`), O onto Apex→D.
- `heelConeWorld` = the **dedendum corner** (the heel edge's first endpoint) — **C** (pinion) / **D** (driving). C/D is the **outer** end of that **same root cone element**, so `coneVec = normalize(heelConeWorld − apex)` runs along Apex→C / Apex→D pointing outward and `distAlong(heelConeWorld) > distAlong(toeConeWorld)`.

⚠️ **Two scrambles to avoid** (a fresh regen has made both):
- Do **NOT** pass the two endpoints of a *single* edge as `toeMid`/`heelMid` (e.g. M as `toeMid` and N as `heelMid`). M and N both sit at the **toe**, so `span = distAlong(heelMid) − distAlong(toeMid)` collapses to ≈0 or negative and the spiral inverts. `toeMid` is the midpoint of the **toe** edge; `heelMid` is the midpoint of the **heel** edge — two different edges.
- `heelConeWorld` is the **dedendum corner C/D** (on the root axis Apex→C / Apex→D), **never H/J**. H/J lie on the `Apex2→C` / `Apex2→D` dedendum line (one Module beyond C/D), **off** the root cone element — using them skews `coneVec` away from Apex→C / Apex→D.

**A. Gate & frame.** Build a world frame from the geometry already constructed for this gear:

- `axisDir` = the **shaft axis** direction, from the two **world** endpoints of `shaftAxisEdge` (the in-sketch profile edge A'→G / B'→I), normalized.
- `coneVec` = the **dedendum (root) cone element** Apex→D (driving) / Apex→C (pinion), realized as `normalize(heelConeWorld − apex)` where `apex` = `apexWorld`. (`heelConeWorld` is the heel end of that dedendum element; `toeConeWorld` is its toe end.)
- `v` = `axisDir × coneVec`, normalized — the **circumferential** direction (the sideways sense the tooth is displaced from the radial element).
- `tpNormal` = `coneVec × v`, normalized — the **tangent-plane normal**. It completes the frame and **nothing consumes it**: step D removed the projection that once used it, so it is computed and left unread.
- `distAlong(p)` = `(p − apex) · coneVec` — the **cone distance** of a point (its distance from the apex measured along the cone element).

⚠️ **The heel MUST be the OUTER end (farther from the apex) so `coneVec` points outward and `span > 0`.** Before building `coneVec`, check the passed midpoints and **fix swapped toe/heel**: if `apex.distanceTo(heelMid) < apex.distanceTo(toeMid)`, swap `toeMid ↔ heelMid` **and** `toeConeWorld ↔ heelConeWorld`, then build `coneVec = apex → heelConeWorld`. A negative `span` (toe farther than heel) **silently inverts the entire spiral frame** — it flips the cutter-arc direction, the slice direction (the first cut misses; see step E), and the per-segment twist — and the gear comes out completely wrong with no error. (The inversion can also originate upstream in §2/§3 mislabeling the toe vs heel edges; this guard catches it at the frame.)

From `toeMid`/`heelMid` (the toe/heel root-edge midpoints, **after** the swap guard above) get `R_toe = distAlong(toeMid)`, `R_heel = distAlong(heelMid)`, `R_mean = ½(R_toe + R_heel)`, and `span = R_heel − R_toe` (the face width, now **positive**). These are the only quantities the rest of the build needs.

**B. Cutter-arc geometry.** Work in the tangent-plane 2-D frame with origin at the apex, **x = coneVec** (so a point's x is its cone distance) and **y = v** (circumferential). The cutter radius is `r_c = Cutter Radius` if non-zero, **else `R_mean`** (the auto default). The hand sign is `handSign = +1` for `Right` else `−1`, then **negated for the pinion** (the pair meshes with opposite hands). The cutter-circle centre is

```
Cx = R_mean − r_c · sin ψ
Cy = handSign · r_c · cos ψ
```

⚠️ **Gotcha — the hand sign goes on the `cos`/`Cy` term, NOT the `sin`/`Cx` term** (this was a real bug). Opposite hand mirrors the cutter centre **across the cone element (y = 0)**, which flips `Cy`. Putting `handSign` on `Cx` mirrors about `x = R_mean` instead — a *different* curve that gives the two gears **unequal twist**; for equal teeth the driving and pinion traces must come out as exact mirror images.

The trace's toe/heel arc endpoints are circle∩circle intersections taken a hair **past** the face so the kept arc reaches cleanly past the end-trims: `toe2d = circle_intersect_nearest(R_lo, …)` and `heel2d = circle_intersect_nearest(R_hi, …)` (the framework helper from `.solids`) with `R_lo = R_toe − 0.06·span` and `R_hi = R_heel + 0.06·span`. `circle_intersect_nearest` intersects the apex circle of radius R with the cutter circle (centre `(Cx,Cy)`, radius `r_c`) and keeps the solution nearest `(R_mean, 0)` — the branch the mean point sits on. (See `spiral-tooth-trace.md` §6 for why the near branch, and §5 for the centre derivation.)

**C. 2-D trace sketch (the genuine cutter arc).** Build the tangent plane with the framework's `plane_by_angle`: first draw a **cone-element construction line** Apex→(Apex + R_heel·coneVec) in a sketch on the **axial / Gear Profiles plane** (name it `{gear} Cone Element`), then make the tangent plane = that axial plane rotated **90°** about the cone-element line (`plane_by_angle(comp, coneElementLine, axialPlane, 90)`; name it `{gear} Trace Plane`). Add a sketch on it named **`{gear} 2D Tooth Trace`**. In it draw, with `tanW(px,py) = combine_point(apex, px, coneVec, py, v)` (framework) mapping 2-D coords to world:

- the **cutter circle** — centre at `tanW(Cx, Cy)`, radius `r_c` — `isConstruction`, with its centre pinned via `centerSketchPoint.isFixed = True` (`[PB-CIRCLE-CENTER]`) and a diameter dimension = `2·r_c`;
- the **trace arc** — a 3-point arc through `tanW(toe2d)`, `tanW(R_mean, 0)` (the mean point on the cone element), `tanW(heel2d)`, with its **centre coincident to the cutter circle's centre** and a **radius dimension = `r_c`**, so it is the genuine cutter circle and not a look-alike spline. ⚠️ Text points per `[PB-RADIAL-DIM]` (off-centre, on/near the curve): use the mean point `tanW(R_mean, 0)` for the trace arc's radius dimension, and a point on the cutter circle such as `tanW(Cx + r_c, Cy)` for its diameter dimension.

**Coordinates (this rule governs the `{gear} Cone Element` sketch as well as the trace sketch — carry it into both when compiling, because it is the only place either is told what frame its points are in):** the world `Point3D`s from `tanW(...)`, and the raw `apex` / cone-end points of the `{gear} Cone Element` line, are passed **directly** into the sketch calls (`addByTwoPoints`, `addByCenterRadius`, `addByThreePoints`), where they are consumed as **sketch-space** input — **no `modelToSketchSpace` conversion is applied**, even though `adsk.fusion.Sketch` offers exactly that call and the points really are model-space coordinates.

This is deliberate, and it is worth stating why it is harmless, because the reasoning is not the obvious one. The trace sketch is **construction/reference only: no downstream feature ever consumes it** — the twist is computed analytically from the 2-D endpoints in step G, and the sketch exists only so the genuine cutter arc is inspectable before cleanup hides it. The cone-element line is the one that needs the extra sentence: it *is* consumed, by `plane_by_angle(comp, coneElementLine, axialPlane, 90)`, which rotates about it to make the Trace Plane. So an unconverted cone-element line does place that plane somewhere other than the true tangent plane. That still reaches no feature, because the only thing built on the Trace Plane is the inspection-only trace sketch, and the whole chain ends there. **If a later revision ever makes any feature consume the trace sketch or the Trace Plane, this shortcut stops being safe and both sketches need `modelToSketchSpace` on every point.**

This sketch is **deliberately left with free DOF** — the arc's endpoints are pinned by the 3-point construction, not by endpoint dimensions (dimensioning them over-constrains the solve against the cone-element plane). It is therefore **exempt from the full-constraint gate**, as already declared in Sketch Discipline ("The spiral build's auxiliary sketches are EXEMPT") — do not gate it.

**D. (No 3-D projection.)** The 2-D cutter-arc sketch from step C is the only trace geometry needed — the spiral twist is computed **analytically** from it in step G, so there is **no `projectToSurface`, no root-cone-face search, and no 3-D trace sketch.** (Earlier versions projected the 2-D arc onto the root cone along `tpNormal` and measured the trace azimuth there. That projection is *fragile*: for unequal-ratio pairs the arc wraps around the cone and `projectToSurface` returns it as **multiple disjoint fragments**, so the measured azimuth collapses to a fraction of the true sweep — the pinion comes out grossly under-twisted and the pair interferes. The analytic crown-gear law in step G is exact, deterministic, and cannot wrap.)

**E. Slice the straight tooth.** Split the uncut apex→heel `toothBody` into cross-section slabs by planes **parallel to the parent transverse tooth plane** (`parentToothPlane`, the virtual-spur tooth-profile plane `{label} Plane` from §3, passed into the hook), via a **fixed** slice scheme of **exactly 8 planes** — the count is not user-configurable. ⚠️ **The slice planes are NOT perpendicular to the cone element.** The parent plane carries the tooth-center line C->K′ / D->L′, which is the back-cone line and so perpendicular to the Pitch Line, so **the parent plane's normal runs along the PITCH element** — while `coneVec` is the **ROOT** element. The two differ by the dedendum angle `δ_f = atan(1.25 · Module / R)`, equivalently `atan(2.5 · sin γ_p / N_p)` = `atan(2.5 · sin γ_g / N_g)`: Module cancels, so δ_f depends only on the tooth counts and the Shaft Angle and is the **same for both members** — `3.26°` on the default 31/31 pair at Shaft Angle 90°, growing as the tooth counts fall. The parallel family is what the build requires rather than what it happens to use: `slice_body_by_offset_planes` offsets the parent plane with `setByOffset`, which produces **parallel** planes; the sign test below reads the **parent plane's own normal**, which is meaningful only for that plane's own offsets; and the tooth is lofted from the Apex to the profile drawn in the parent plane, so **the heel-most slab's heel face IS the parent plane** and a consistent family has to contain it. ⚠️ **A build that follows "perpendicular to the cone element" instead is wrong and silent**: it tilts every cut face by δ_f and nothing in the pipeline fails — parallel planes cut a cone in similar sections whatever their orientation, so the loft still reproduces the taper; the piece count, the retry gate below and the conical trims of step J are all indifferent to slab orientation; the proof builds its own slabs and never sees the module's plane; and the runtime gate only counts pieces. What moves is the geometry: on the default pair at Module 4 a face corner lands `1.125 · Module · tan δ_f` = 0.26 mm along the cone from where the parallel cut puts it, and the step-G twist keyed across one face mismatches by up to 0.0078 rad. The first cut plane is the parent plane offset toward the apex by `span/6`; the offset **sign is chosen per gear** so it moves toward the apex (the parent plane's normal points opposite ways for the two gears — pick `sign` so `sign·normal` points apex-ward, i.e. test `(apex − planeOrigin)·normal`). The other seven step further toward the apex in `span/6` increments (`sign·(k+1)·span/6` for k = 0…7, k = 0 being the first cut plane). **Where the eight land: the first sits `span/6` inside the HEEL and none of them lies past it** — the parent plane is already the heel end, so there is no heel overshoot to give — **the sixth lands at the toe, and the last two sit `span/6` and `2·span/6` PAST the toe**; the two segments beyond the toe are what step J's toe cone trims away. (The first sits a hair more than `span/6` inside the heel, and the sixth a fraction of a millimetre inside the toe, because `R_heel`/`R_toe` are read at the two edge midpoints rather than on the root element.) Split the body with the framework's `slice_body_by_offset_planes(designComponent, toothBody, parentToothPlane, offsets)` where `offsets = [sign·(k+1)·span/6 for k in 0…7]` — it splits piece-by-piece and keeps a piece whole when a plane misses it. ⚠️ **The slice MUST actually split the tooth.** After the cut loop, if the body is still in **one piece** (no plane cut it), the offset sign was wrong or `parentToothPlane` sits outside the tooth's span — **retry the whole cut once with the opposite sign**. If it is *still* one piece, **`raise` a clear self-diagnosing error** naming the gear, the final piece count, `span`, and the sign tried. Do **NOT** return an unsliced (single-piece) result: step F then drops that one piece as the apex scrap, leaving `segments` **empty**, and the crown later crashes with `ValueError: max() iterable argument is empty` far from the cause. The result is the set of cross-section segments.

**A spiral build has been through Fusion once on the parallel family — loaded 2026-09-16, from the
build that corrected this step from "perpendicular to the cone element" (branch
`fix-bevel-spec-defects`).** A spiral gear built with no error. Each failure this step names raises,
so a clean build is the reading that the eight offset planes cut the tooth into more than one piece
and that step F left segments behind for the crown. ⚠️ **That is the whole of what it shows, and it
does not distinguish the two families** — this step already records that a build on the tilted
planes completes just as silently. No face corner and no twist angle was measured on the result, so
the 0.26 mm corner shift that separates the parallel family from the tilted one is still unmeasured
in Fusion.

**That load was taken at `7d253d8`, and the branch regenerated afterwards — the later build was
loaded too, on 2026-09-17, at `c6ccb3a`.** `fix-bevel-spec-defects` (PR #154) rebuilt
`lib/geargen/bevelgear.py` after the 2026-09-16 load, so the module that load exercised is not the
module the branch merged. The default spiral bevel built on `c6ccb3a` with no error, which carries
the reading above — eight offset planes cut the tooth into more than one piece and step F left
segments for the crown — onto the regenerated module. **That is the whole of what the second load
covered.** The 2026-09-16 load's other configurations were not rebuilt on `c6ccb3a`, nothing was
measured on the result, and it does not distinguish the two slice families either, for the reason
this step already gives.

**A third spiral build has since run on a later module — loaded 2026-09-17, at `2ad1e32` (PR
#159).** The shipped default 31/31 pair at Module 1 and Shaft Angle 90° built in spiral form as
well as straight; Maximum Shaft Angle above carries that load's full list. Again nothing was
measured, so the three loads together say that the spiral path has completed on three successive
modules and nothing about the geometry any of them produced.

**Could the proof have caught the correction?** No, for the reason this step already states: the
proof builds its own slabs from the offsets this spec fixes and never reads the plane the generated
module constructs, so the module's choice of family reaches Fusion untested. That is the same shape
of gap as the §2 seed, and the only thing that closes it is a measurement taken on a loaded spiral
gear — a face corner's position along the cone, which none of the three loads reported.

**F. Order & drop scrap.** Sort the segments by `distAlong` of their centroid (`physicalProperties.centerOfMass`). The first (apex-most) is the long **apex-side scrap** below the toe — **remove it**; keep the rest as the working `segments`. (Drop the scrap by re-slicing the list, *then* delete it — `segments = segments[1:]` before `removeFeatures.add(scrap)`.) After dropping the scrap, **`segments` must be non-empty** (≥1 cross-section); if it is empty the slice failed in step E — `raise` a clear error rather than proceeding into the twist (G) and crown (H), which assume ≥1 segment.

**G. Twist (the spiral).** Rotate each segment about the **shaft axis** (`axisDir` through `apex`) so the tooth follows the trace, **centred on R_mean so the mid-face section stays unrotated** — that section then meshes exactly like the straight tooth (critical; the pinion's zero mesh nudge depends on it). The total toe→heel shaft-axis twist comes from the **conjugate crown-gear generation law** (the standard Gleason/Litvin model — see `spiral-tooth-trace.md` and the NASA references): a spiral bevel is generated by an imaginary flat *crown gear*, and the work gear’s shaft rotation relates to the developed crown-plane azimuth by the **roll ratio `1/sin γ`** (the generating crown gear has `N/sin γ` teeth; γ = this gear’s **pitch cone angle**). Compute it **analytically — no projection, no curve sampling:**

```
phi_crown = atan2(heel2d[1], heel2d[0]) - atan2(toe2d[1], toe2d[0])   # developed azimuth of the cutter arc at the apex
total     = abs(phi_crown) / math.sin(gamma)                          # shaft-axis twist magnitude
```

`phi_crown` is the angle the cutter arc’s **toe and heel endpoints subtend at the apex** in the flat 2-D crown frame (apex at the origin, x = cone distance along `coneVec`, y = circumferential along `v` — exactly the `toe2d`/`heel2d` pairs from step B). `gamma` is this gear’s **pitch cone angle**: `self._gamma_p` (Pinion) / `self._gamma_g` (Driving), already computed in §2.

⚠️ **The two halves of this law are taken on two different cones, and that is deliberate rather than an oversight.** `phi_crown` is measured in the frame step A builds, whose x-axis is `coneVec`, the **ROOT** cone element Apex→C / Apex→D — `spiral-tooth-trace.md` §1 states the same choice and calls it a departure from the canonical reference. The divisor `sin γ` is the **PITCH** cone's roll ratio, because the crown gear the law generates against is tangent to the pitch cone. Written consistently on the root cone the divisor would be `sin γ_root` for `γ_root = γ − δ_f`, which is a real difference and not a rounding: at the default 31/31 pair at Shaft Angle 90° the dedendum angle `δ_f` is 3.26°, so `sin γ / sin γ_root` is 1.062 and the root divisor would twist the tooth about 6% further. **Keep the pitch angle.** `acos(coneVec · axisDir)` is exactly that root angle, and the build that used it inflated the twist by about 1.15× on a 17-tooth pinion meshing a 31-tooth gear, which is the defect that kept ratio pairs from meshing at all.

**What is not settled is whether the frame should move to the pitch cone to match.** Nothing in this repository measures that. The proof asserts this step against the same formula the module computes, so it confirms the arithmetic and says nothing about which cone the frame belongs on; no Fusion load has reported a trace azimuth or a face corner either. Moving the frame would change `R_toe` and `R_heel` — they are read along `coneVec` — and would change which element ψ is measured against, so it is its own derivation and its own change. **Record this beside the twist assertion in the proof file, as the honest edge of what this step checks.** `handSign` sets the direction; `total` is the magnitude. ⚠️ **Use the PITCH cone angle γ from §2 — NOT `acos(coneVec·axisDir)`** (that is the *root/dedendum* cone angle, smaller than γ by the dedendum angle δ_f of step E — 24.7° against the pitch 28.7° for a 17-tooth pinion meshing a 31-tooth gear — and yields a twist ~1.15× too large). ⚠️ The two members of a meshing pair **legitimately get different twists**: same cutter, same spiral angle ψ, but γ differs, so `1/sin γ` differs (≈2.08× for a 17-tooth pinion vs ≈1.14× for a 31-tooth gear — a ratio ~1.83). This is *why* equal-teeth pairs (31/31, equal γ) always meshed while ratio pairs failed under any method that gets `1/sin γ` wrong. **Do NOT** measure the twist off a projected 3-D cone trace (the old approach): `projectToSurface` wraps the arc around the cone for ratio pairs and the measurement collapses. The analytic law here is exact and deterministic. Each segment's rotation angle is a **linear share** keyed to the **cone distance of its HEEL FACE** (the segment's farthest-along-the-element face — the exact section the later loft samples). **Define a slab's heel face precisely: the face whose centroid has the GREATEST `distAlong(face.centroid)`, searched across ALL of the slab's faces with NO surface-type filter** (its toe/apex-side face is the LEAST-centroid one). ⚠️ Do **NOT** restrict this search to `PlaneSurfaceType` (or any surface type) — a sliced slab is bounded by a mix of the two planar cut faces and ruled side faces, and a type filter can pick the wrong face or miss the cut face, which makes the step-I loft fail with `ASM_NOT_ALL_SECTIONS_MEET / LOFT_NO_TOOLBODY`. Use this **same all-faces-by-centroid** rule (max → heel, min → toe) everywhere a slab end face is needed: the twist key here (G), the crown base (H), and the loft sections (I). The rotation:

```
ang = −handSign · total · (R_mean − R_heelFace(seg)) / span
```

⚠️ **Gotcha — key the twist on the segment's HEEL-FACE cone distance, NOT its centroid.** The loft (step I) samples each segment's heel face, so that face is what must land at the right azimuth. Centroid-keying leaves the loft's mid-face section rotated by half a segment → mid-face overlap. Apply the rotation with a free-move by a `Matrix3D.setToRotation(ang, axisDir, apex)`.

**H. Lengthwise crown (relief).** Crown the tooth by scaling each segment **except the outermost (heel) one** down by a **monotonic** factor — full at the heel, growing smoothly toward the toe — **about a sketch point on the ROOT edge of its heel face** (gotcha 3 — NOT the heel-face centroid). For each segment compute its **heel-distance fraction** `u = (R_heel − R_heelFace) / span`, where `R_heelFace` = `distAlong` of that segment's heel face — **found by the step-G all-faces-by-centroid rule but RECOMPUTED here, AFTER the step-G twist has moved the slabs** (do not reuse pre-twist values) — and `R_heel`/`span` are from step A. **`u` runs 0 at the held-full heel to 1 at the toe, and PAST 1 on the two segments that lie beyond the toe** (step E): those two segments' heel faces sit `6·span/6` and `7·span/6` in from the parent plane, so the toe-most one reads about `7/6` (≈1.18) before the twist and a little more (≈1.21) after this recompute, at the default Spiral Angle 35° on the default 31/31 pair. **Do not treat `8/6` = 1.33 as a ceiling.** That figure is the last plane's offset, and that plane is a toe face rather than any segment's heel face, so nothing ever evaluates `u` there — but the step-G twist moves the heel faces, so the recomputed `u` climbs with the Spiral Angle and is **measured at 1.351 at Spiral Angle 55°**, which the `[0, 60)` range admits. Nothing reads an upper bound on `u`; the crown factor below stays positive for every value it takes, and what the slab count actually rests on is the structural fact that the last cut plane is never a heel face. (The heel segment reads a few hundredths rather than exactly 0, because `R_heel` is read at the heel edge's midpoint rather than on the parent plane; that segment is skipped anyway.) **"Outermost (heel) segment" = the one with the GREATEST post-twist heel-face `distAlong`** — sort the segments by their recomputed heel-face `distAlong` and skip the last. Then:

```
factor = 1 − _CROWN_PER_RAD · (|total| / 2) · u
```

`total` is the full toe→heel twist from step G; `|total|/2` = the per-end peak twist magnitude (`max|ang|`), so the maximum relief — now at the **toe** — keeps the same magnitude the old per-end peak had, just relocated. This makes relief **grow monotonically from the (full) heel to the toe**, so slab heights stay **strictly ordered heel→toe** and the natural cone taper is never reversed. If a computed `factor` comes out ≤ 0 (extreme twist), `raise` a self-diagnosing error naming the gear, the segment's `u`, and the factor — never scale by a non-positive factor. **`_CROWN_PER_RAD` is a tunable class constant, default `0.5`** (0 disables the crown) — set it to 0.5, do not leave it unset/0.

⚠️ **Do NOT key the relief on `|ang|` (the twist magnitude, i.e. |distance from mid-face|).** That is **symmetric** about mid-face — maximal at BOTH ends — so, because the heel slab is held full (gotcha 2), the slab *just inside* the heel becomes the **most**-relieved one and dips below both its neighbours (a notch), reversing the heel→toe taper. This was the observed bug: the heel-adjacent slab came out at factor `0.932` while the next slab inward was `0.972` (taller). Key the relief on the monotonic heel-distance `u`, never on `|ang|`. Three gotchas:

1. **The scale base must be a sketch point** (a point added in a sketch on the heel face, or a BRep vertex), per `[PB-CONSTRUCTION-NEEDS-ACTIVE]`. (`scaleFeatures` is the ONE exception to never-activate: it needs the Design occurrence as the **active** edit target — call **`designOccurrence.activate()`** (a method on the `Occurrence`) before the crown scales, and restore afterward — in a `finally` — with **`design.activateRootComponent()`** (a method on `Design`). ⚠️ Do **NOT** write `design.rootComponent.activate()` or `someComponent.activate()` — a `Component` has **no** `.activate()` method and raises `AttributeError`. Only `Occurrence` has `.activate()`; the root is re-activated via `Design.activateRootComponent()`.)
2. **Skip the outermost (heel) segment.** Its heel face is the loft's heel end and must stay full so the heel cone (step J) trims it flush with the gear base.
3. **Anchor the scale on the heel face's ROOT edge, NOT its centroid — otherwise the crowned tooth lifts off the gear base.** `scaleFeatures` shrinks **uniformly** toward the base point, so a base point at the heel-face **centroid** (mid tooth-height) pulls the tooth's **root** edge *upward* by `(1−factor)·(½ tooth height)` → the tooth no longer seats on the gear body's root cone, floats above the base, and the Combine-Join leaves a gap (clearly visible for ratio pairs — e.g. module 2 / driving 19 / pinion 13 — the original symptom that exposed this). Put the base point on the **root** instead: of the heel face's vertices (`heelFace.vertices`, each `.geometry` a world `Point3D`), take the **two with the smallest perpendicular distance to the shaft axis** (the line through `apex` along `axisDir`; perpendicular distance = `|(p−apex) − ((p−apex)·axisDir)·axisDir|`) — those are the two **root corners** (the tip corners are the farthest from the axis) — and place the base sketch point at their **midpoint** (mapped into the heel-face sketch via `modelToSketchSpace`). The heel face is a planar cut, so that midpoint lies on it. Rationale: a uniform scale about a point keeps every line/plane through that point invariant, so anchoring on the root keeps the root edge on the seating cone (the tooth stays flush) while the tip is relieved progressively toward the toe — which is exactly the lengthwise crown intended. (Finding the heel face itself is unchanged — still the max-`distAlong`-centroid face per step G; only the point *on* it changes from centroid to root-edge midpoint.)

**I. Loft → curved tooth.** ⚠️ **Re-sort the segments by their heel-face cone distance HERE, AFTER the twist (G) and crown (H) — do NOT reuse the pre-twist slice/centroid order from step F.** The twist rotates each slab about the shaft axis, and for high-twist *unequal-ratio* pairs that rotation changes the slabs' along-cone (`distAlong`) order enough to **reorder adjacent slabs**; lofting in the stale pre-twist order then assembles the cross-sections out of sequence and the crowned tooth comes out distorted → the two gears interfere. (For equal/low-twist pairs the two orders coincide, which is why equal-teeth gears mesh even with the stale order but unequal ratios distort — this is the single thing that makes a ratio pair like 31/17 fail while 31/31 looks fine.) So: compute `order = sorted(segment indices, key = distAlong(slabHeelFace(seg).centroid))` **now**, and loft a NewBody through, in that order: first the **toe-most segment's apex-side (toe-facing) face** — the toe segment is `order[0]`, its toe face added first to push the loft past the toe cone so the toe trim bites — then the **heel-facing face of every segment, iterated in `order`** (each segment's farthest-along-the-element face by post-twist centroid; the last reaches past the heel cone). Name the resulting body **`{gear} Spiral Tooth`**. Then remove the segment scaffolding (the loft has captured their faces).

**J. Flush trim + mesh phase.** Return `cut_conical_ends(designComponent, curvedTooth, gearBody, toeMid, heelMid, apexWorld, gearLabel)` (framework) — the same toe-then-heel two-cone trim used for the straight tooth — so the curved tooth's ends sit **flush** on the gear base. The toe/heel **mesh phasing** is handled outside this hook by `_createGearBody`'s mesh-rotate step (see "Meshing rotation" under Create the Gear Bodies; the pinion's extra phase is 0 by default because the mid-face section is unrotated and already meshes).

## Create the Gear Bodies (once per gear — pinion first, then driving)

Run this whole section **once per gear** — pinion first, then driving — with these substitutions:

| | Pinion | Driving |
|---|---|---|
| hexagon vertices, in draw order | A' -> G -> H -> C -> M -> N -> A' | B' -> I -> J -> D -> O -> P -> B' |
| profile sketch name | `Pinion Profile` | `Driving Profile` |
| shaft-axis edge (the hexagon's FIRST edge) | A'->G | B'->I |
| toe cut edge | M->N | O->P |
| heel cut edge | C->H | D->J |
| teeth / bore / pitch-diameter inputs | Pinion Gear … | Driving Gear … |
| §2 shaft construction line (NOT usable as the axis) | Apex->A | Apex->B |

Create a new component as a child of the **Bevel Gear** component (the same component that owns Design — *not* the user's Parent Component; this intentionally overrides the looser "child of Parent Component" phrasing so the pair nests cleanly inside Bevel Gear), named `{gearLabel} Gear` (`Pinion Gear` / `Driving Gear`). The resulting bodies for this gear end up in this component. (Implementation note: Fusion rejects cross-sibling sketch and project calls even when the target is activated or the entities are wrapped in `createForAssemblyContext` proxies — `[PB-NO-CROSS-SIBLING]` — so the actual feature operations run in the Design component and the finished bodies are `moveToComponent`'d here at the end. The visible end state is identical.)

**Profile sketch.** Open a **fresh sketch on the axial (Gear Profiles) plane**, named per the table — **one profile sketch per gear**, so `sketch.profiles` holds exactly this one hexagon loop (do not draw both gears' hexagons in the shared Gear Profiles sketch — that would leave two identically-shaped loops to disambiguate). Build the hexagon on fixed vertices per the `[PB-PROJECT-NOT-FIXED]` recreate-share-fix recipe: recreate the six §2 vertices as new points at their exact (world-mapped) positions — valid because §2 is fully constrained by now — then draw the closed hexagon (in the table's draw order) as six `SketchLine`s **sharing** those points, then fix the lines and their endpoints **after** the lines exist (not before). The hexagon's **first edge is the gear's shaft axis** for the revolve, pattern, bore plane AND the meshing-rotation axis, so it must be fixed well enough to carry a trustworthy world position: fixed endpoints give that edge a well-defined `worldGeometry` (`[PB-WORLDGEO-CONSTRAINED]`); a free edge resolves against a default/world-XY frame and silently moves the body onto world XY (observed on the driving gear — the pinion looked fine only because it never read the edge's `worldGeometry`).

**The shaft axis used by every body operation below is this profile sketch's first edge, NOT the §2 `Apex->A`/`Apex->B` construction line.** The edge is collinear with the shaft axis but lives in the *same* sketch as the profile, which is what Fusion's revolve/pattern/path accept; reusing the §2 construction line (a different sketch) fails or misbuilds.

**Revolve.** This sketch holds exactly one hexagon loop, so take its single profile (`[PB-SINGLE-PROFILE]`) and revolve it around the shaft-axis edge; let the result be the Gear Body (the frustum). Because the toe edge is one edge of the revolved profile, the body already carries the conical face produced by sweeping it around the axis — that face is reused as the cutting tool below (likewise the heel edge's cone).

**Tooth loft.** Loft the **§2 Apex sketch point** (the `centerToApex.endSketchPoint` from the Gear Profiles sketch — the degenerate point-section) to this gear's §3 Tooth profile; let the result be the Tooth Body. Use the §2 Apex SKETCH point directly — do NOT create a construction point for it (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`; the Design component is never active).

**Conical cuts.** Trim the Tooth Body to a flush band with the framework helper — return
`cut_conical_ends(designComponent, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)`
(from `.solids`; do **NOT** re-implement the cut machinery). **Two distinct bodies are involved —
do not conflate them:** the cutting TOOLS are `ConeSurfaceType` faces of the **Gear Body (the
revolved-hexagon frustum)** — the lofted Tooth Body has no cone faces, so searching *it* for the
cone face finds none — and the TARGET being split is the **Tooth Body (the loft)**.

The helper implements the pinned cut behavior (see the PLAYBOOK "Shared geargen helper library"):
the **toe cut first**, its cone face identified by the toe edge's world **MIDPOINT** best-first
across the frustum's cone faces (`[PB-FACE-BY-MIDPOINT]` — endpoints sit near the apex
singularity), each candidate tried as the actual split tool and the first that splits (>1 piece)
kept; **keeper selection after each cut** (remove apex-containing pieces, keep the largest —
`[PB-REMOVE-PIECES]`); then the **heel cut on the keeper alone** (removing the apex tip first is
what makes it deterministically two split features for every gear ratio). A heel cone that does
not intersect the keeper at all (common on ratio pairs, e.g. module 1 / driving 31 / pinion 43,
where the heel cone never overshoots the tooth) is raised by the helper as the **typed
`solids.NonIntersectError`** and caught — the keeper is returned whole. Every failure is
self-diagnosing with the per-face distance/error history (`[PB-SELF-DIAGNOSING]`), and each cut's
outcome is logged with `force_console=True`.

**Caller obligations (these stay in the generator):** pass `toeMid` = the toe edge's world
midpoint (`(M_world + N_world)/2` pinion / `(O_world + P_world)/2` driving) and `heelMid` = the
heel edge's world midpoint (`(C_world + H_world)/2` / `(D_world + J_world)/2`) — the same
edge-midpoint pairs as the §3a hand-off; `apexWorld` = the §2 Apex sketch point's world geometry;
`gearBody` = the revolved frustum (the cone-face source). The toe cut must split (its failure
propagates and crashes the build — correct, since an uncut tooth is unusable); only the heel cut
is lenient, and only via the typed `NonIntersectError`.

**Pattern.** Circular-pattern the remaining tooth piece around the **shaft-axis edge** (the same in-sketch profile edge used for the revolve, not the §2 construction line). The number of copies equals this gear's Teeth Number. Pin the pattern inputs: `quantity = <Teeth Number>`, `totalAngle = '360 deg'` (full circle), `isSymmetric = False`. (Although the pitch diameter shrinks from heel toward apex, the *angular* spacing around the shaft axis stays constant at `360° / N` for the entire face width — the radial taper is already produced by the loft from Apex to the heel-end tooth profile, so the pattern just rotates that single tapered tooth into N evenly spaced copies.)

**Combine.** Join all patterned tooth pieces with the Gear Body in a single Combine-Join (the Gear Body as the target, the patterned tooth bodies as the tools).

**Bore.** If Enable Bore is checked, cut a cylindrical through bore through the Gear Body along the shaft axis. The bore diameter is this gear's **resolved** Bore Diameter — the user's value if specified (non-zero), otherwise `this gear's Pitch Diameter / 4`, in either case already bounded by that gear's **Maximum Bore Diameter** in §2 (an auto value capped to it, a user value above it already rejected). Take that resolved number; do not re-derive it here, or the cap is lost and the bore deletes the body's back face. Build the bore plane normal to the shaft at its start (`setByDistanceOnPath(<shaft-axis edge>, 0.0)`; pass the in-sketch edge, not the §2 construction line). In a sketch named `{gearLabel} Bore`, sketch the bore circle centered at the sketch origin (the plane is rooted at the shaft start, so the origin is on the axis): **fix the bore circle's center and add a diameter dimension** set to the bore diameter (`[PB-CIRCLE-CENTER]`). Extrude-cut it as a symmetric through-cut restricted to `[this Gear Body]` (`[PB-THROUGH-CUT]`; use `2 × Cone Distance` as the per-side half-length — generously past any face width). Skip this step entirely if Enable Bore is unchecked.

**Meshing rotation (driving gear only) — do this here, in the Design component, before the body is moved out.** Rotate the driving body by `180° / Driving Gear Teeth Number` (half a tooth pitch) about its shaft axis — `rotate_body_about_edge(designComponent, gearBody, shaftAxisEdge, angle)` (framework, from `.solids`), which takes the rotation axis/origin from the B->I profile edge's **world** endpoints (`[PB-MOVE-ROTATE]`) — so a driving valley sits where the pinion tooth crosses the axial plane, giving the interlocked meshing look. Rationale: both gears are patterned from a starting tooth in the axial plane, so without the offset a driving tooth and a pinion tooth would both sit at the axial-plane crossing and visually collide. This runs in Design before `moveToComponent` because a construction axis can't be added in the moved-out gear component (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`), so the rotation must use the edge's world geometry while still in Design. (The pinion additionally gets `_pinionMeshPhase(pinionTeeth)` — 0 unless a spiral pair needs it; see Method contract.)



## Cleanup

Call the framework's `hide_construction_geometry(bevelComponent)` (from `.solids`) — it recursively walks the Bevel Gear component tree (dedupe by `entityToken`) and hides every sketch, construction plane, and construction axis with `isLightBulbOn = False` (construction planes/axes are **not** hidden by `isVisible` — `[PB-HIDE-AFTER-USE]`). Leave only the two finished gear bodies visible.

(The driving gear's half-tooth-pitch **meshing rotation** is performed earlier, at the end of "Create the Gear Bodies", in the Design component before the body is moved out — not a cleanup step; see that section for the rationale.)

## Proving the Solid Steps (bevel-specific)

This section is about the proof rather than about Fusion. It says how every solid step of this gear
is proved, because the solid harness refuses the operations those steps are made of and a compiler
that does not know the substitutions marks the steps `[PROSE]` and drops them. It sits near the end
of the file so that adding it moves no line a compiled step list already cites.

**Every solid step is `[GO]`. None of them is `[PROSE]`.** The substitutions below are what make
that true, and each one is a real build measured against a closed form — not a weakened gate, and
not a comment standing in for a check.

### Which booleans this gear's proof performs, and how it builds their operands

**Build every boolean operand with Loft, and never with Revolve.** Measured at the pinned decad
revision over this gear's own bodies across the whole solid case table: every boolean whose operands
were both Lofts returned a body the document verified Sound, with a published volume bound between
2e-16 and 1e-8 of the value. Every boolean with an operand built by Revolve verified Suspect
instead — all 160 of them — and on 100 of those the reading beyond tolerance was the volume itself.
`proofkit3d`'s gate admits neither verdict, so a Revolve operand puts its step out of reach whatever
the geometry is. Every solid in this gear is conical, and a cone is a Loft here because Extrude
refuses a nonzero taper, so building the operands as Lofts is what the gear was going to do anyway.

**The gear body is a real `decad.Revolve`, and it is the one body no boolean consumes.** A revolved
frustum's volume agrees with Pappus on its own profile to 3.4e-16 relative and carries a published
bound near 2e-16, so it clears the gate on its own readings. That is a statement about what a
revolved body measures, not about what a boolean will take: the rule above still refuses one as an
operand.

**Two of this gear's booleans are performed and two are not.** The bore's through-cut and the toe
half of the conical end cut run on the gear's own bodies, and each step asserts the result. The heel
half of the conical end cut and the Combine-Join are not performed, each for the reason its own
bullet states, and each keeps a substitution.

**Where a substitution is still needed it is the same one at every site: build the operands, lay
them apart along the shaft axis, and assert from their own measured geometry what the operation
would have produced.** Laying them apart leaves every volume, radius and cone angle unchanged, which
is what makes the readings still mean something. State the substitution once in the proof file and
its cost at each site.

### Each step's substitution and what it costs

- **The gear-body revolve.** Substitute nothing. Revolve the §2 hexagon a full turn about the
  station axis with decad's own `Revolve`, which returns the whole frustum as one body. Assert its
  volume against Pappus on that same hexagon, with no polygon correction and no decomposition into
  bands: the two agree to 3.4e-16 relative at worst over the solid case table, inside the bound
  decad publishes for the reading. That bound is `Approximate` rather than `Exact`, so write the
  comparison with `decadtest.Measures` and a relative tolerance — `decadtest.Exactly` fails on every
  case. Then assert the body's **five faces**, matched by surface kind and by their own readings
  rather than by the order the face selector hands them back: the flat heel face at the back, a disc
  of the heel radius; the flat toe face at the front, a disc of the Toe Radius; and three cone faces,
  whose areas are those of the cone frusta the profile edges C->H, M->C and N->M sweep. **Read each
  cone's half-angle off its own face** — a `decad.Cone` publishes it — instead of deriving a tangent
  from two cap radii and a height. The heel cone and the toe-dish cone read `90° − γ` and the root
  cone reads this gear's root cone angle. **This step now costs nothing**: the frustum is one
  watertight body with the right volume, the right two flat faces and the right three cone faces,
  and there is no union left for the proof to owe.
- **The apex loft.** Substitute a shrunken section for the degenerate apex point, and nothing else.
  The tooth plane is **not** substituted: the proof builds the real back-cone plane, tilted out of
  the axis-perpendicular by γ, and takes the apex's perpendicular distance to the section as
  `sK · cos γ` rather than `sK`. **The cost is the point section**: the loft's degenerate end is
  not built, and what the volume and the two cone slopes prove is the taper it has to produce.
- **The conical end cut.** Perform the toe cut; substitute for the heel cut. Where each cut lands is
  read the same way for both: build the tooth, read each cone's apex and half-angle and each of the
  tooth's two surfaces off the bodies themselves, solve the stations where they cross from those
  readings, and check them against the flush band. Take **both half-angles off the revolved gear
  body's own cone faces** — those faces are the cutting tools Fusion's `ConeSurfaceType` search
  finds at this step, and a `decad.Cone` publishes the angle directly. The step asserts these three:
  each cut lands where the flush band requires — the toe cone meets the gear body's own root cone at
  M, the heel cone meets it at C; each cone's half-angle equals this gear's back-cone half-angle
  `90° − γ`, compared at the same slope tolerance the revolve step's cone faces use; and each cone
  crosses the tooth's tip inboard of where it crosses the tooth's root, so the
  trimmed end is shorter at the tip than at the root. **Do not assert that a cut meets the tooth's
  tip and root at different stations, and carry no message for that case.** Both cutting cones have
  their apex on the shaft axis, so a cone of wall slope `k` and apex station `a` crosses a tooth
  surface of slope `m` at `a · k / (m + k)`, which is positive for every `m > 0` and different for
  the tooth's two surfaces whenever the tooth has height. Every cut therefore crosses **both**
  surfaces in every configuration, and an assertion that the two crossings differ passes on any
  figure this spec can build. What those three readings still do not reach is in "What the conical
  end cut cannot tell apart" below.

  **Perform the toe cut on the tooth.** Build the toe cutting cone as the SOLID inside the cone —
  the whole body on the discard side, with its apex on the shaft axis at the station the toe edge's
  own lattice point M/O puts it — rather than as a band spanning that one profile edge, which is
  enough to read an angle off and is not a tool a cut can use. It is an n-gon loft, because the cut
  consumes it. Put the tooth and the cone in one frame, so that the cone meets the tooth where the
  toe end of the flush band puts it; seating the tooth on the gear body is a different placement,
  and it is the one the Combine-Join below still waits on. Then `Cut` the tooth with the cone for
  one piece and `Intersect` them for the other, in separate documents, since either operation
  retires its operands. **Tolerate one typed refusal from the `Intersect` and fail on any other
  error**: a `*decad.BooleanError` carrying `BooleanEmpty` says the cone took nothing off that
  tooth, which is the condition the generated module raises as `solids.NonIntersectError`. A probe
  over one pair of operands drew it on the two Shaft Angle 142° cases and on no other, so keep the
  branch whatever cone this step ends up building. Assert that the two pieces add back to the whole
  tooth, and that each piece decad returns is one lump and solid. **The toe split costs nothing
  now** — the evaluator divides the tooth, and both halves of the division are measured, except on
  a case the refusal above claims, where the step records that it built no split rather than
  passing silently.

  **Perform no heel cut, because its cone is tangent to the tooth plane.** The dedendum corner C/D
  and the tooth centre K′/L′ both sit on this gear's back-cone dedendum line, so the tooth plane
  contains a generator of the heel cone and the two touch along the tooth's own centreline instead
  of crossing it. decad refuses exactly that: `BooleanUnsupportedContact`, on 15 of the 20 cases for
  the `Cut` and 17 for the `Intersect`, saying the operands' facets come within the chord tolerance
  without provably interpenetrating deeper than it. Rebuilding the same cone as a Revolve replaces
  the refusal with a Suspect verdict, which the gate does not admit either, so no operand pairing at
  this revision puts the heel cut in reach. Lay the heel cone apart from the tooth and keep the
  three readings above. **The cost is the heel split**: for that end the proof does not show the
  evaluator dividing the tooth, selecting the keeper, or leaving a watertight body.
- **The Combine-Join.** Perform no join, because the two operands are not in one frame. The proof
  builds the tooth on a section perpendicular to the build axis at the Pitch Cone Distance from the
  apex, scaled about the apex — the Tredgold mapping — while the gear body is written about the
  shaft axis, and the tooth's own seating on that body is derived nowhere in this proof. The two
  therefore do not meet when they are put in one document, and neither sign of the rotation that
  relates the two planes seats them: at one sign the union returns two lumps, and at the other decad
  refuses the contact on half the case table. **The engine is not what blocks this join.** Given
  operands that do overlap it performs the union, returns one lump and publishes a volume bound of
  8e-15 of the value, Sound. What is missing is the tooth's real back-cone placement, and deriving
  it is its own change. Until then, lay the operands apart and assert the join's two
  consequences from their own measured geometry: a join leaves ONE lump when the tooth's root is
  below the body's root cone — seated, not floating — and the joined body reaches further out
  than the frustum when the tooth's tip stands proud of it. Take both readings at the toe, the
  middle and the heel of the band the join would cover. **The cost is the stitch**: the proof cannot
  show the evaluator making one boundary out of two. **The generated module draws its root circle one
  root sink inside the dedendum corner** (§3 step 1), so the root arc lies inside the gear body's
  root cone across its whole width and the join overlaps along the whole root rather than along the
  centreline alone. **The proof applies that same sink** — it is one figure, not a proof-only offset.
  ⚠️ **Read the root arc's OUTERMOST point, not the tooth's centreline.** The centreline sits inside
  both root corners, so a reading taken there passes a tooth whose corners float outside the cone,
  which is exactly the defect the sink exists to remove.

  **Fusion has made this stitch once — loaded 2026-09-16, from the build the root sink was
  introduced on (branch `fix-bevel-exact-virtual-radius`).** Two configurations built with no error:
  the shipped default of 31 teeth on both gears at Module 1 and Shaft Angle 90°, and a 16 driving /
  12 pinion pair at Module 4. The default is also the configuration where the sink drops the root
  circle below the base circle, so its tooth is drawn NON-embedded and the spur drawer adds the two
  flank-to-root lines, 0.0405 mm each; neither that profile nor a Combine-Join at a sunk root had
  been through Fusion before that load. So the stitch this substitution cannot show has been seen
  once, on those two configurations, and on nothing else in the table.

  **A second load repeated the stitch on the module that ships today — 2026-09-16, branch
  `fix-bevel-spec-defects`.** That branch regenerated `lib/geargen/bevelgear.py` after the load
  above, so the module the first load exercised is not the module in the repository now. The
  shipped default pair built **one solid per gear**, which is the join's own reading: two bodies
  would mean a tooth floating off the root cone, and one means the evaluator made a single boundary
  out of the frustum and every patterned tooth. The 16/12 pair at Module 4 was not rebuilt on that
  branch. The proof reaches no more of this than it did before — it still performs no join.

  **A third load brought the 16/12 pair back — 2026-09-17, at `2ad1e32` (PR #159).** Both
  configurations the root sink was introduced on built again on that module: the shipped default
  pair and the 16 driving / 12 pinion pair at Module 4, alongside the four further configurations
  Maximum Shaft Angle lists. So the stitch has now been seen on both, on the module the spec
  describes today. **The reading is still only that the build completed.** Nothing counted the
  bodies this time, nothing measured a tip radius, and the join is still the one boolean the proof
  performs none of — so what a second body would have meant is what a silent build leaves unsaid.

  **The heel tip radius was not measured on that load, so the tip is still checked only where the
  proof checks it.** The §3 sketch case dimensions the drawn tip circle at `virtualPitchRadius +
  Module`, and the apex loft reads the built tooth body out to the virtual tip radius laid on the
  back cone — at Module 4 through 8 only, since no solid case runs at Module 1. No case reads a tip
  radius off a joined body, because no case joins. That measurement is still outstanding.

  **Could the proof have caught any of this?** Not the join: the proof performs none, which is the
  cost recorded above. The non-embedded profile it already covers — the §3 tooth case draws both
  members of the default pair at Module 1 with the sink applied, takes the embedded flag from the
  sunk root radius, and requires the tooth loop to carry the two lines that follow from it. What
  that case cannot reach is Fusion's own profile finder selecting the loop and the loft consuming
  it, which is what the load showed.
- **The bore cut.** Build the tool as a real extrude, which a symmetric extent produces as a prism,
  and perform the cut. Assert the tool first, from its own measured geometry — its diameter, that
  its two ends sit exactly `2 * Cone Distance` either side of the shaft edge's start, and that both
  clear the frustum, which is what makes it a THROUGH cut — and compute the material the bore takes
  out of the frustum's own profile clipped to the bore radius. Then cut with it. **The target is the
  heel cone band, lofted for this step**: it is the section of the gear body the bore passes through,
  and it is a Loft, which is the form a boolean takes. The revolved gear body cannot be the target,
  for the reason the operand rule above gives. The cut verifies Sound on every case in the table.
  Assert the pierced body's volume against the band's own n-gon closed form less the prism the bore
  removes over that height, which it matches to 3.2e-16 relative at worst; assert that the result is
  one lump, which is what a through hole leaves; and assert that it is solid, which an enclosed void
  would not be. **What this still does not reach is the rest of the body**: the bore is pierced
  through the band that stands for the heel section, not through the whole frustum, because the
  frustum is a Revolve. Record that beside the assertion.
- **The spiral tooth chain.** The slab slicing, the apex-scrap drop, the twist, the crown and the
  spiral loft are each `[GO]` on the same terms: real slabs, laid apart where a boolean would
  otherwise be needed, asserted against the closed form the spiral trace fixes.

### What the conical end cut cannot tell apart

**A cone and a tilted plane read identically in everything the cut step measures.** In the axial
section a cone of half-angle `90° − γ` and a plane tilted by `γ` through the same generator are the
same line, so every station the step solves for comes out the same for either surface. The tooth's
own heel-end plane crosses the tooth's tip and root at different stations too. That is why the
tip-versus-root reading is dropped above: it is a property of any cut with a finite tilt, not a
signature of a conical one.

What makes the face conical is that it is a **surface of revolution about the shaft axis** — its
crossing with the tooth's tip surface sits at one station at every azimuth, while a tilted plane's
crossing moves with azimuth. The half-angle the step now reads comes off a `decad.Cone` face of the
revolved gear body, which is a surface of revolution by its own construction and publishes its angle
rather than having one derived. The tool the toe cut consumes is still a faceted body swept about
that axis, and a swept body cannot be anything else either, but nothing measures its azimuthal
crossing with the tooth. Record that beside the cut assertions in the proof file, as the honest edge
of what this step checks.

**The flush-band check cannot see the half-angle.** Any band through M crosses the gear body's root
ray at M whatever slope the band has, so a toe band built at the wrong angle still lands on the toe
end of the flush band and still crosses the tip inboard of the root. The half-angle assertion added
above is what pins the angle at this step, and it reads the gear body's own cone faces, which the
revolve step pins against the same closed form. Nothing about where a cut *lands* pins it.

**The heel cut is the lenient one because its cone is tangent to the tooth plane.** The dedendum
corner C/D and the tooth centre K′/L′ both sit on this gear's back-cone dedendum line, so the tooth
plane contains a generator of the heel cone and the two touch along the tooth's own centreline
instead of crossing it. That cone's apex on the shaft axis is K/L, where the same dedendum line
meets the axis. At Tooth Spacing 0 the tooth centre is that apex, so the heel cone passes exactly
through the tooth's heel-end centreline and takes only the two corners, by `py² / (2 · r · cos γ)`
for a corner lying `py` off the centreline at polar radius `r` in the tooth plane. A cut that
removes that little is a cut that can miss the keeper altogether on a ratio pair, which is the typed
`solids.NonIntersectError` the helper catches — and the reason only the toe cut must split.

### Where the tooth centre sits when Tooth Spacing is positive

**The tooth is centred at K′/L′, which is off the shaft axis, and anything that centres it on the
axis instead builds a different gear.** K/L is where the back-cone dedendum line crosses this gear's
shaft axis, so its radius is 0 and its station is `R / cos γ` from the apex, for the Pitch Cone
Distance `R` and this gear's own `γ`. K′/L′ is `Tooth Spacing` further along that same line, which
carries it *past* the axis, to

    station = R / cos γ + Tooth Spacing · sin γ
    radius  = Tooth Spacing · cos γ, on the opposite side of the axis from the dedendum corner C/D

Both halves are part of the placement. Taking the station alone and seating the tooth plane's origin
on the axis leaves every tooth point `Tooth Spacing · cos γ` too far out from the axis — 0.431 mm on
the pinion and 0.356 mm on the driving gear at Module 4, Driving 43, Pinion 31, Shaft Angle 75° and
Tooth Spacing 0.5 mm. That offset is the whole of the clearance the input asks for, so a tooth
centred on the axis carries none of it.

### The solid tables run at Module 4 to 8

**Do not put Module 1 in a solid case table.** decad's mesh bound has an absolute floor, so a figure
small enough brings every measurement inside it and the gate reports Suspect on geometry that is in
fact correct. Module is a pure scale on this figure, so a case at Module 4 through 8 proves the same
shape as one at Module 1 and clears the floor. The sketch tables are unaffected and stay at the
dialog's own default.

### Two cases the virtual tooth count needs

Both the per-gear sketch table and the solid table carry these, because the virtual tooth count is
read by the tooth profile and by every body built on it.

- **16 driving / 12 pinion at Shaft Angle 90°.** The pinion's virtual tooth count is exactly 15 and
  the driving gear's is 26.667, so one member of the pair is a case where an exact count and a
  rounded one agree and the other is a case where they do not. A pair like that rejects a rounded
  count rather than merely disagreeing with one: whatever error the rounding introduces, it is not
  the same error on both members.
- **4 / 4 at Shaft Angle 90°.** Its virtual count is 5.657, the lowest the table carries, and its
  root arc carries the largest corner float of any pair the spec admits, 0.027 module. That is the
  case the root sink has to clear, so it is what fixes whether the sink is large enough.

### The case the two toe inputs need together

**The per-gear sketch table must carry one case with a positive Toe Extension AND a user Toe Radius
set at the same time** — `toe_extension_50_toe_radius_user`: Toe Extension 50 with both gears' Toe
Radius at 3 mm, on the table's own default 31/31 pair at Module 1 and Shaft Angle 90°. The table
already runs each input alone, and alone neither reaches what they do together. At Toe Extension 0 a
user Toe Radius only moves where N and P sit on a toe line whose length is the resolved Face Width;
with a defaulted Toe Radius a positive Toe Extension walks the toe end along a Toe Limit computed
from the auto radius, which is the one radius that reproduces Toe Extension 0's profile exactly. It
is the combination that exercises the coupling: the Toe Limit is `|Ded->X|` measured to the point on
the root element at **this gear's Toe Radius**, so a user radius moves the window the Toe Extension
divides, and the Toe Radius Ceiling check is reachable at all only while the Toe Extension is above
zero.

That gap is what a Fusion load pointed at rather than the proof. The build at `2ad1e32` loaded on
2026-09-17 (see Maximum Shaft Angle) set both inputs together and built, and no case stood behind
it. **This is a case the proof could have carried and did not** — unlike the seed and the slice
family, which no case can reach — so it is added here rather than recorded as a limit.

### A refusal the case table records rather than avoids

Where this gear's §2 lattice cannot reach a configuration the spec admits — the Shaft Angle floor of
30° is the one measured today, where the net's conditioning reads below the sketch engine's floor —
**the case stays in the table and is marked as a declared refusal**, through a case-table flag the
step reads, not by narrowing the range the spec states. A configuration the spec admits and this
particular net cannot reach is a property of the net, and the table is where that is recorded.

## Proving the tooth-profile sketch (bevel-specific)

This section is about the proof rather than about Fusion, and it exists because one wrong sentence
about the tooth-top arc was carried in a proof comment and cost a full round of investigation. It
sits near the end of the file so that adding it moves no line a compiled step list already cites.

**The tooth-top arc is a CENTRE-POINT arc with a pinned centre and no dimension.** §3 hands the
sketch to the shared spur drawer, which creates it with
`sketchArcs.addByCenterStartEnd(localOrigin, rightFlankEndPoint, leftFlankEndPoint)` and then pins
the copied centre with `addCoincident(arc.centerSketchPoint, localOrigin)`, carrying no radius and
no diameter dimension (`[SPUR-F-TOOTHTOP-ARC]` in `spec/spurgear/fusion.md`, which owns the
construction). Bevel's §3 authors no arc of its own. **Never write that Fusion draws this arc as a
three-point arc and dimensions its radius**: that sentence sat in the proof file once, and what it
sent an investigation chasing was a reflected centre — the ambiguity a three-point arc with a radius
dimension really does admit, and which this construction has no room for, because the centre is
pinned.

**Why the proof cannot reproduce that coincident, which is what the comment is for.** The sketch
engine emits a coincident as two residual rows. The arc's centre has one remaining freedom left by
its own equidistance row, so two rows against it come back redundant and the engine's gate refuses
the sketch. The proof therefore pins each arc's centre with **one signed component** beside that
equidistance row, and **asserts** each radius rather than dimensioning it. **State the cost at the
site:** the pair of constraints the drawer actually makes is not the pair the proof solves.

**The §3a trace arc is the other case and it is genuinely a three-point arc with a radius
dimension** (§3a step C: three through-points, centre coincident to the cutter circle's centre,
radius dimension `r_c`). Its proof comment therefore says something different from the tooth-top
arc's, and the two must not be written from one template.

## Proof Case Scheduling (bevel-specific)

This section is about the proof rather than about Fusion, and it says which of a step's proof cases
may run at the same time as each other. It sits at the end of the file so that adding it moves no
line a compiled step list already cites.

**Register every `[GO]` step through the parallel harness entry point**, with the single exception
named below: `proofkit.RunParallel` where the step would otherwise take `proofkit.Run`, and
`proofkit3d.RunSolidParallel` where it would otherwise take `proofkit3d.RunSolid`. A bevel case
builds its own sketch, or its own document, from its own parameters, and measures only the geometry
that case constructed, so two cases of one step have nothing between them to corrupt. Running them
together is what keeps the bevel proof, the slowest package in the suite, from setting the suite's
wall time on its own.

**The Pattern step is the exception and stays serial**, on `proofkit3d.RunSolid`. The pattern
increment retires the seed tooth, so the seed cannot be measured after the step runs, and its
azimuth, radius, height and volume have to be read during the build and handed to the assertion.
That hand-off leaves the case, and two cases sharing one set of seed readings overwrite each
other. It is not a hazard that announces itself: the two gear sides differ enough in volume that
the overwrite was caught when it happened, and a pair of cases whose seeds measured alike would
have passed on each other's numbers instead. Keep the carried readings where the proof keeps them,
and record in the proof file beside them that this step is serial because of them.

No other bevel step carries a reading from its build into its assertion. A step that acquires one
moves to the serial runner in the same change.

## Proving the §2 figure (bevel-specific)

This section is about the proof rather than about Fusion, and it says how the proof must pin the
sites `[BEVEL-F-MIRROR-FIGURE]` lists. It sits at the end of the file so that adding it moves no
line a compiled step list already cites.

**Pin every one of the 15 sites with a constraint the sketch engine SIGNS.** The engine's
`NewAngle` and `NewOffset` carry a sign; `NewDistance` does not. The two Tooth Spacing sites
(K→K′ and L→L′) are the ones this bites: pinned with `NewDistance` they stay unsigned in the
proof, so the proof no longer signs every site Fusion leaves unsigned, and that is the claim the
`[BEVEL-F-MIRROR-FIGURE]` gate rests on. Use a signed constraint at both. Where no signed
constraint fits the shape, assert the sign directly instead — that K′ − K, projected on
`Apex2->C`, is `+Tooth Spacing` and not `−Tooth Spacing` — and say at the assertion which of the
two it is standing in for.

⚠️ **NEVER read a clean ambiguity probe as proof that no twin figure exists.** The engine
documents its probe as reporting a **lower bound** on the number of solutions. A
`tooth_spacing_positive` case pinned with the unsigned `NewDistance` passes today, and the twin it
misses puts K′ one Tooth Spacing on the C side of K — two candidates `2 × Tooth Spacing` apart,
0.8 mm at that case's 0.4 mm spacing, far above any tolerance. The probe's silence is not
evidence; the signed constraint is.

**The lattice assertion is the proof's copy of the `[BEVEL-F-SEED-HELD]` gate**, and it must cover
the same 22 named points against the same closed forms §2 states, including E, F, G, H, I and J.
**Record beside it what it cannot reach:** the proof seeds at the closed form, so it proves the
constraints solve from a correct seed and never that the generated module's seed is correct. That
is the same limit the toe-line seeding already records, and it is why the gate has to exist inside
the module rather than only in the proof.

**Fusion has run the `[BEVEL-F-SEED-HELD]` gate once — loaded 2026-09-16, from the build that
introduced it (branch `fix-bevel-spec-defects`).** The shipped default pair — 31 teeth on both
gears, Module 1, Shaft Angle 90° — built one solid per gear with no error. The gate runs inside
that build and raises on the first point that moved, so a clean build is the reading that all 20
points it compares at Tooth Spacing 0 solved within 0.001 mm of their closed-form seeds, and that
the gate raised on none of them. Both halves of the risk were open until this load and neither
showed: Fusion's solver did not leave the figure outside the tolerance, and the gate did not raise
on a correct figure.

**Nothing was measured on that load beyond the build completing.** No solved point, volume or
radius was read off the result, and only the one configuration was built. So this records that the
gate ran and stayed silent on the default pair, and nothing about the 1024 figures at that
configuration it exists to refuse — none of them was built.

**Could the proof have caught any of this?** No. The lattice assertion seeds at the closed form and
solves with the sketch engine this repository pins, so what it reaches is the constraint net, which
is the limit recorded just above. Whether Fusion's own solver leaves a seeded point inside 0.001 mm
is a property of Fusion's solver, and so is whether the gate false-fails on a solve that is
correct. Nothing in this repository runs that solver. Loading the gear is the only check that
reaches either reading, and it stays the only one.