# Gear Generator

This is yet another Gear Generator for Fusion 360.
I know there are a bunch of other tools to do this, I wrote this to scratch my itch, which were:

* Source code legibility - I at least wanted to have code as classes, and not a single giant function
* Fully constrained sketches - Many freely available scripts created gears whose sketches were not fully constrained. This led to funky problems when you wanted to modify or move the generated objects.
* Ability to generate gears on any surface/plane and point pair

On top of the above, there were also a few minor issues that I wanted to see happen:

* Extra information - I know you don't need to draw the pitch circle or the base circle, but I wanted to see them. Also wanted to annotate them by text objects

# Supported Gears

The add-in installs one command per gear type into the Fusion **SOLID > CREATE** panel:

* **Spur Gear**
* **Helical Gear**
* **Herringbone Gear**
* **Bevel Gear**

# INSTALLATION

This is a standard Fusion 360 add-in. There is no Marketplace package yet, so install it manually:

1. Get the source into Fusion's `AddIns` directory. Either clone directly into it, or clone elsewhere and copy/symlink the folder there. The folder **must** be named `Gear Generator` (it has to match `Gear Generator.py` / `Gear Generator.manifest`).

   * **Windows:** `%APPDATA%\Autodesk\Autodesk Fusion 360\API\AddIns`
   * **macOS:** `~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/AddIns`

   ```sh
   git clone https://github.com/lestrrat-3d/fusion360-gear-generator.git "Gear Generator"
   ```

2. In Fusion 360, open **Utilities > Scripts and Add-Ins** (shortcut: `Shift+S`).
3. Switch to the **Add-Ins** tab. `Gear Generator` should appear in the list.
4. Select it and click **Run** (tick *Run on Startup* if you want it loaded automatically).

The gear commands then live under **SOLID > CREATE**.

# USAGE

Pick a gear command from the **CREATE** panel, fill in the dialog (module, tooth count, pressure angle, etc.), and pick the plane/point that places the gear. The pitch and base circles are drawn and annotated alongside the gear body, and every generated sketch is fully constrained.

# TODO

* Diametral Pitch handling: I personally just don't need it, but I think it's just a matter of having a conversion table. Patches welcome.
* Error handling: I have a few, but I should add more.
* User-friendly UI: Currently there's minimal UI. I suck at UI, so if you can help me, I'd be so happy.
* Distribution: I have no idea how to package and otherwise distribute these things on AutoDesk Marketplace. If you can help me with it, I'd be so happy.

# LICENSE

Released under Creative Commons CC-BY-NC 4.0 — https://creativecommons.org/licenses/by-nc/4.0/

For commercial use, please contact the author.
