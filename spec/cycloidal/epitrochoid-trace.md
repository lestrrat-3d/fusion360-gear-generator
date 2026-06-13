# Cycloidal rotor profile — geometry oracle (minimal: one lobe)

This is the geometry oracle for the cycloidal-rotor generator (cited from `instructions.md`). Reproduce
the point function and formulas **exactly** — no mechanical gate verifies geometry, so these are the
asserted facts. The generator works in Fusion-internal **cm**; convert mm inputs with `to_cm`. Profile
points computed here are **already cm** — add them to sketches as-is (never re-wrap in `to_cm`).

> Scope note: the full archived drive is at `.tmp/cycloidal-spec-full-archive/`. This spec is being grown
> back incrementally; it currently produces the **rotor disk** (lobe section → extrude → pattern → join)
> with its **center bore**, the **output holes**, the **ring pins + Housing Ring base**, the **eccentric
> input cam**, and the **output plate + `M` output pins**. The full axial stack/clocking (and a 2nd disk)
> are not built yet. **Output pins** are `M` cylinders (`D_pin = Output Pin Diameter`, resolved) on the
> output-pin circle (`Rop`) about the drive axis **`O`**, carried by a plate `1 mm` above the disk
> (opposite the housing); they pass down through the disk's output holes (which are on `Od`, `+2E` oversize).
>
> **Center bore / eccentric cam (plain bearing):** the eccentric **cam outer** diameter = `Center Bearing
> Diameter`; the disk's center bore is **enlarged** to `Center Bearing Diameter + Bearing Clearance` so the
> cam turns freely in it (concentric running gap, both on `Od`). The cam outer is on the disk centre `Od`;
> its input-shaft bore (`Input Shaft Diameter`, `0` = none) is on the drive axis `O` — the `E` offset between
> them is the eccentricity. Validity: `Input Shaft Diameter < Center Bearing Diameter`; the input bore fits
> the cam (`E + InputShaftDiameter/2 < CenterBearingDiameter/2`); the enlarged disk bore clears the output
> holes (`(CenterBearingDiameter + BearingClearance)/2 < Rop − D_hole/2`).

## Symbols

| symbol | meaning | source |
|---|---|---|
| `N` | number of ring pins | `Pin Count` |
| `L` | number of rotor lobes | `N − 1` |
| `M` | number of output pins/holes | `Output Pin Count` |
| `R` | pin-circle radius (pitch radius) | `Pin Circle Radius` = `Pin Circle Diameter`/2 |
| `Rop` | output-pin-circle radius | `Output Pin Circle Radius` = `Output Pin Circle Diameter`/2 |
| `Rr` | ring-pin (roller) **physical** radius — **derived**, see below | `Pin Radius` |
| `c` | disk profile clearance (backlash) | `Disk Clearance` |
| `Rr_eff` | effective roller radius = `Rr + c` | derived |
| `E` | eccentricity | `Eccentricity` |
| `t` | curve parameter | — |

## Pin / hole sizes — auto-derived (mean of bounds) unless overridden

Pin sizes **auto-derive to the mean of their theoretical bounds, but each is overridable**: the
corresponding input defaults to **`0`**, and `0` means "auto"; any **non-zero** value is the user's
explicit size and is used as-is (still bounds-checked).

- **Ring-pin radius** `Rr`: if `Pin Diameter > 0` → `Rr = Pin Diameter / 2`; else (auto) →
  `Rr = ½·(E + R·sin(π/N))`, the mean of its bounds `E < Rr < R·sin(π/N)` (lower `E`: the
  non-self-intersection limit; upper `R·sin(π/N)`: so the `N` pins don't overlap on the pitch circle).
- **Output-pin diameter** `D_pin`: if `Output Pin Diameter > 0` → `D_pin = Output Pin Diameter`; else
  (auto) → `D_pin = Rop·sin(π/M) − E`, the implied pin from the mean hole.
- **Output-hole diameter** `D_hole = D_pin + 2E` **always** (the `2E` orbit clearance applies to any
  pin, auto or overridden). With the auto pin this equals the mean `E + Rop·sin(π/M)`; its bounds are
  `2E < D_hole < 2·Rop·sin(π/M)` (lower `2E`: pin clears the orbit; upper `2·Rop·sin(π/M)`: the `M`
  holes don't overlap).

**Validity (always, after resolving auto vs override; reject with a clear message):**
`E < Rr < R·sin(π/N)`; `D_pin > 0`; `D_hole < 2·Rop·sin(π/M)`; `E < R/N`; `Rop < Rv`; **and the
no-undercut guard below** (`Rr_eff < ρ_min^O`). The auto sizes satisfy the first set by construction; the
checks catch bad overrides.

### No-undercut guard (the binding eccentricity limit)

⚠️ `E < R/N` is only the **base-cycloid** cusp limit. The drawn profile is the **inward equidistant** of
the base trochoid offset by `Rr_eff`, and that offset **self-intersects (undercuts) at a *lower* `E`** —
when `Rr_eff` exceeds the smallest radius of curvature of the base trochoid at the points whose centre of
curvature lies **toward `O`** (the convex flanks an inward offset can overrun). Below that limit the
profile is clean; above it the spline self-intersects and Fusion fails. **Reject when `Rr_eff ≥ ρ_min^O`**,
computed numerically (sample `t` at ~2000 points over `[0, 2π)`):

```
bx  =  R·cos t − E·cos(Nt) ;            by  = −R·sin t + E·sin(Nt)        # base trochoid point
xp  = −R·sin t + E·N·sin(Nt) ;          yp  = −R·cos t + E·N·cos(Nt)      # base'
xpp = −R·cos t + E·N²·cos(Nt) ;         ypp =  R·sin t − E·N²·sin(Nt)     # base''
k   = xp·ypp − yp·xpp                                      # skip if |k| ~ 0
rho = (xp² + yp²)^1.5 / k                                  # signed radius of curvature
s   = sqrt(xp² + yp²) ;  nx,ny = −yp/s, xp/s               # unit left normal
Cx,Cy = bx + rho·nx,  by + rho·ny                          # centre of curvature
# keep only points whose centre is toward O:
rho_min_O = min over t of |rho|  where  (Cx²+Cy²) < (bx²+by²)
```

For the defaults this gives **max safe `E ≈ 2.50 mm`** (verified to match a direct profile
self-intersection scan), vs the loose `R/N = 2.81 mm` — so this is the **binding** eccentricity ceiling.
Build the reject message with the actual limit when possible (e.g. "reduce Eccentricity below ~X mm").

## The point function (reproduce exactly)

The rotor profile is the **equidistant of a shortened epitrochoid** (the `atan2` term offsets the base
trochoid by the roller radius along the curve normal — what makes the rotor roll on round pins). The
clearance `c` is folded in by cutting with `Rr_eff = Rr + c` (so the printed rotor clears the pins).
Verified headlessly against a known-good 3D-printed drive (HowToMechatronics, `N=16, R=45, Rr=6.5,
E=1.5`): the profile is a clean, non-self-intersecting closed curve. For a rotor centred at `(cx, cy)`
with clocking `phi`:

```
def disk_point(t):                       # returns (x, y) in cm
    Rr_eff = Rr + c
    num = sin((1 - N) * t)
    den = (R / (E * N)) - cos((1 - N) * t)
    psi = atan2(num, den)                # uses R, E, N only — NOT Rr
    x0 =  R*cos(t) - Rr_eff*cos(t + psi) - E*cos(N * t)
    y0 = -R*sin(t) + Rr_eff*sin(t + psi) + E*sin(N * t)
    x = cx + (x0*cos(phi) - y0*sin(phi))     # clocking, then centre offset
    y = cy + (x0*sin(phi) + y0*cos(phi))
    return (x, y)
```

## One lobe (the section we draw)

The base curve has exactly `L = N − 1` lobes and is L-fold symmetric: `P(t + 2π/L) = Rot(−2π/L)·P(t)`.
**One lobe spans `t ∈ [0, 2π/L]`**, running **valley → tip → valley**.

**Valley radius (exact).** At `t = 0`: `ψ(0)=0`, `x0 = R − Rr_eff − E`, `y0 = 0`, so the valley radius is
**`Rv = R − Rr_eff − E = R − (Rr + c) − E`**, and both ends of the lobe (`t = 0` and `t = 2π/L`) lie on
the circle of radius `Rv` about the rotor centre (the **root/valley circle** — one of the three reference
circles we draw).

**Sampling (adaptive — required to avoid spline overshoot).** Draw the lobe as a single **open** fitted
spline (do **NOT** set `isClosed`, do **NOT** add a closing arc — closing into the full disk comes later
from the root circle + pattern). ⚠️ Do **NOT** sample `t` uniformly: an interpolating fitted spline
**overshoots** ("rabbit-ear" loops) where the lobe turns sharply. The per-step direction change is only
~6° for the gentle default but reaches **~25° near the undercut limit** at 60 uniform points — far past
where the spline overshoots. A uniform sample dense enough to be smooth there needs ~480 points; instead
sample so consecutive fit points subtend a **bounded turn angle** (≤ ~5°), which stays smooth at **any**
valid `E` with only ~30–55 points:
1. Evaluate `disk_point` at fine resolution (e.g. ~2000 `t` over `[0, 2π/L]`).
2. Greedily keep points: keep the first; accumulate the turn angle between consecutive fine points; each
   time the accumulator reaches the threshold (~5°), keep that point and reset; always keep the last.
3. Build the fitted spline through the kept points.
This concentrates points on the sharp flanks/tip and thins them on the gentle parts — fewer points than a
smooth uniform sample, and no overshoot. Valid (non-self-intersecting) for `E < min(Rr, R/N)` and the
no-undercut guard; the defaults (`N=16, R=45, Rr≈5.14, E=1.5, c=0.3`) satisfy these.
