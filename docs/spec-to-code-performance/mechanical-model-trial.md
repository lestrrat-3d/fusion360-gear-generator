# Mechanical emitter trial, 2026-09-28

This trial compared GPT-6 Sol and GPT-6 Astra as Python drafters for the checked
spec-to-code workflow. Each pair read the same checked inputs for its gear.
The spur pair used a step view with SHA-256 `84b529033b0b599f3c50f444793e1c4f3010a05ea0746539c91faff9ac74c5b4`
and a proof bundle with SHA-256 `be6f66984887bc0a95897afa8f8bfc87c94c791497b7992968ef2d76efe4aa86`.

| Gear | Sol | Astra |
|---|---|---|
| Helical | Seven blocking emit gates passed on the first draft. | Seven blocking emit gates passed on the first draft. |
| Spur | Six blocking gates passed on the first draft. Three projection guards failed; one repair passed seven gates. | Seven blocking emit gates passed on the first draft. |

The Sol spur draft also passed `get_selection`'s selected Occurrence directly to
`Generator.parentComponent`. That object has no `occurrences` collection, so
`Generator.getOccurrence` cannot add the child gear beneath it. The compiled spur step
now names the required conversion, and its contract checks the conversion in
`processInputs`. The Sol draft fails that new guard; the Astra draft passes it.

The Astra spur module passed all seven blocking emit gates after that contract change.
In Fusion, a default spur gear and a default helical gear each generated one valid body
using the deployed spur file with SHA-256
`c21ffa3a50cd14d3653e01a6f6e0f9e87d007ed5e6c4665765fc86b6ec1f9525`.
Those Fusion runs called the generators with the real Fusion API and default values
in new unsaved documents. They did not exercise the command dialog.

The spur proof covers one default case. Its solid proof builds one final tooth
contour because repeated Decad unions exceed the engine's segment limit. It
checks the patterned tooth bodies and root fillet separately, and checks the bore
as a hole in that final contour. It does not prove Fusion's sequential union and
cut features.

Keep the current fallback to the session default for GPT-6 model names in
`pick_model.py`. Sol needed an extra gate and repair round on spur, and the trial
did not record comparable drafting time or model cost. These results do not
support a general step down from Astra to Sol.
