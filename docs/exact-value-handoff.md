# Exact-value handoff for gear generation

For a migrated gear, `spec/<gear>/exact_values.json` owns the dialog setup and user-parameter
values. `contract.json` owns the exported constant names and their string values. The JSON schema
uses version `1`; the `inputs` array fixes dialog order and the `parameters` array fixes
registration order. Every input and parameter refers to a constant in the contract.

An input has an `id`, `kind`, and `label`. Selection inputs also provide a prompt, filter list,
and root-component preselection flag. Numeric inputs provide a display unit and one default
conversion: `real` for an internal number, `radians` for an angle in degrees, or `millimeters`
for a length in millimeters. String inputs provide a string default. Boolean inputs provide the
Fusion `check_box` flag and the checkbox's initial boolean value.

A parameter has a `name`, registration `unit`, `comment`, and one source. `input` reads a dialog
value. `expression` is a Fusion expression with parameter references in braces, such as
`{PARAM_MODULE} * {PARAM_TOOTH_NUMBER}`. `computed` currently supports the spur gear's
`tooth_space_angle` calculation, which Fusion cannot evaluate as a live expression. The
`{fillet_helix_factor}` placeholder calls the subclass hook in the final radius expression.
Validation rejects duplicate IDs and names, missing or extra fields, invalid units, unknown
references, forward references, and dependency cycles.

After compilation, run `python3 .claude/skills/generate-gear/exact_values.py <gear> sync-steps`.
It embeds the checked values in `steps.md`. `check_compile.py` compares that section with the
source and includes the JSON in provenance, so an edit makes the steps stale.

After emission, run `python3 .claude/skills/generate-gear/exact_values.py spurgear render
.tmp/spurgear.generated.py` before the full emit gates. The renderer replaces the exported
constants, dialog calls, and parameter registration in the candidate. `check_contract.py` checks
that the candidate still matches the step handoff. The renderer leaves selection handling,
sketches, and solids to the emitted module.

The emitter may leave exact constants absent and use `pass` in `configure` and
`registerDerivedParameters`. `processInputs` still reads selections and calls
`self.registerDerivedParameters()`; the renderer fills the setup at that boundary. An existing
module with complete setup can also be rendered, which supports migration and verification.

Gears without `exact_values.json` continue through the existing prose and gate path. To migrate
one, add a versioned source, map every input and parameter constant, add a renderer for its setup,
and run a real spec-to-steps-to-module-to-Fusion pilot before using it for production.
