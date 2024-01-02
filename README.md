# Gear Generator

This is yet another Gear Generator for Fusion 360.
I know there are a bunch of other tools to do this, I wrote this to scratch my itch, which were:

* Source code legibility - I at least wanted to have code as classes, and not a single giant function
* Fully constrained sketches - Many freely available scripts created gears whose sketches were not fully constrained. This led to funky problems when you wanted to modify or move the generated objects.
* Ability to generate gears on any surface/plane and point pair

On top of the above, there were also a few minor issues that I wanted to see happen:

* Extra information - I know you don't need to draw the pitch circle or the base circle, but I wanted to see them. Also wanted to annotate them by text objects

# TODO

* Diametral Pitch handling: I personally just don't need it, but I think it's just a matter of having a conversion table. Patches welcome.
* Error handling: I have a few, but I should add more.
* User-friendly UI: Currently there's minimal UI. I suck at UI, so if you can help me, I'd be so happy.
* Distribution: I have no idea how to package and otherwise distribute these things on AutoDesk Marketplace. If you can help me with it, I'd be so happy.

# INSTALLATION

Until we have some sort of way to