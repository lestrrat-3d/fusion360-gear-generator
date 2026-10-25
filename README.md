# Gear Generator

This is yet another Gear Generator for Fusion 360.
I know there are a bunch of other tools to do this, I wrote this to scratch my itch, which were:

* Source code legibility - I at least wanted to have code as classes, and not a single giant function
* Fully constrained sketches - Many freely available scripts created gears whose sketches were not fully constrained. This led to funky problems when you wanted to modify or move the generated objects.
* Ability to generate gears on any surface/plane and point pair

On top of the above, there were also a few minor issues that I wanted to see happen:

* Extra information - I know you don't need to draw the pitch circle or the base circle, but I wanted to see them. Also wanted to annotate them by text objects

# Supported Gears

The add-in installs one command per generator into a **Gears** dropdown in the Fusion **SOLID > CREATE** panel:

| Gear | What it makes | Pictures and inputs |
| --- | --- | --- |
| **Spur Gear** | Straight teeth across the rim. This is the plain case: it drives a parallel shaft and pushes nothing along the axis. | [spur gear](https://github.com/lestrrat-3d/decad-gallery/tree/main/models/spurgear) |
| **Helical Gear** | The same tooth twisted to the far face, which is what *Helix Angle* sets. Each tooth comes into contact gradually rather than all at once, so it runs more quietly than a spur gear, at the price of a thrust load along the shaft. | [helical gear](https://github.com/lestrrat-3d/decad-gallery/tree/main/models/helicalgear) |
| **Herringbone Gear** | Twisted to mid-body and mirrored, so the teeth meet in a chevron. The two halves lean opposite ways and their thrust cancels, which keeps the helical gear's quiet running without the load on the shaft. | [herringbone gear](https://github.com/lestrrat-3d/decad-gallery/tree/main/models/herringbonegear) |
| **Bevel Gear** | A meshing pair with teeth on a cone, for two shafts whose axes cross. The *Tooth Type* drop-down picks straight or spiral teeth. Spiral shows *Mean Spiral Angle*, *Hand of Spiral* and *Cutter Radius*. | [bevel gear](https://github.com/lestrrat-3d/decad-gallery/tree/main/models/bevelgear) |
| **Cycloidal Drive** | A speed reducer rather than a single gear. It builds the lobed disc(s), the eccentric cam, a pinless ring housing and the output plate with its pins, each in its own sub-component. | [cycloidal drive](https://github.com/lestrrat-3d/decad-gallery/tree/main/models/cycloidal) |

Each gear's geometry is built and checked in [decad-gallery](https://github.com/lestrrat-3d/decad-gallery). Its pages
show rendered pictures of the same models and list the inputs and defaults. This add-in translates those models into
Fusion commands, so the dialog defaults come from the gallery.

# INSTALLATION

This is a standard Fusion 360 add-in. There is no Marketplace package yet, so install it manually:

1. Get the add-in into Fusion's `AddIns` directory, in a folder named exactly `Gear Generator` (it has to match `Gear Generator.py` / `Gear Generator.manifest`).

   * **Windows:** `%APPDATA%\Autodesk\Autodesk Fusion 360\API\AddIns`
   * **macOS:** `~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/AddIns`

   The simplest route is to clone straight into that folder:

   ```sh
   git clone https://github.com/lestrrat-3d/fusion360-gear-generator.git "Gear Generator"
   ```

   If you keep your checkout somewhere else, run `./deploy.sh` from it instead. That script copies only
   the files Fusion actually loads (`Gear Generator.py`, the manifest, `config.py`, `commands/`, `lib/`
   and the `diag_*.py` helpers), plus `LICENSE` and `NOTICE`, and leaves `port/`, `docs/` and `tests/` behind. It deletes
   files under `commands/` and `lib/` in the destination, and `diag_*.py` files at its top, that the checkout does not
   have; other files you keep in the destination folder are left alone. It picks the
   destination from `$FUSION_ADDIN_DIR`, then from an untracked `.deploy.conf` beside the script, then
   by searching the standard locations listed above. Re-run it after every change.

   WSL users need `deploy.sh`: Fusion's August 2026 update stopped loading add-ins from
   `\\wsl.localhost` paths, so pointing the `AddIns` folder at a WSL checkout no longer works.

2. In Fusion 360, open **Utilities > Scripts and Add-Ins** (shortcut: `Shift+S`).
3. Switch to the **Add-Ins** tab. `Gear Generator` should appear in the list.
4. Select it and click **Run** (tick *Run on Startup* if you want it loaded automatically).

The commands then live under **SOLID > CREATE > Gears**.

# USAGE

Pick a generator from the **Gears** dropdown, fill in the dialog (module, tooth count, pressure angle,
etc.), and pick the plane/point that places the result. The generated sketches are fully constrained,
and a build that leaves one free stops with an error that names the free curves. The spur, helical and herringbone gears also draw the
root, tip, base and pitch circles alongside the gear body and label each one with its radius. The
Cycloidal Drive checks its inputs as you type and shows the problem next to the offending field,
keeping OK disabled until the values work.

# DEVELOPMENT

The generators under `lib/geargen/` are build output, not hand-written source. Each one is generated by
`gearport` from a [decad-gallery](https://github.com/lestrrat-3d/decad-gallery) model, which owns the gear's contract
and its geometry proof. The Fusion construction that the gallery cannot express is stored in
`port/<gear>/fragments/`. `docs/gallery-port-pipeline.md` describes the pipeline.

So a fix goes into the fragment, the mapping or `docs/fusion/`, or into the gallery when it is a geometry rule, and then
you regenerate. Editing `lib/geargen/<gear>.py` directly is overwritten by the next regeneration. `CLAUDE.md` covers
where each kind of finding goes.

# TODO

* Diametral Pitch handling: I personally just don't need it, but I think it's just a matter of having a conversion table. Patches welcome.
* Error handling: a failed generation now rolls the new component back and reports in a dialog, and the Cycloidal Drive validates as you type. The other gears still need that same live validation.
* User-friendly UI: the dialogs are still plain value and selection inputs. I suck at UI, so if you can help me, I'd be so happy.
* Distribution: I have no idea how to package and otherwise distribute these things on AutoDesk Marketplace. If you can help me with it, I'd be so happy.

# LICENSE

Released under Creative Commons CC-BY-NC 4.0 — https://creativecommons.org/licenses/by-nc/4.0/

For commercial use, please contact the author.
