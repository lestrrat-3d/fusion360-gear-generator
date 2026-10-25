package helical_test

import (
	"bytes"
	"encoding/json"
	"math"
	"os/exec"
	"strconv"
	"testing"

	"github.com/lestrrat-3d/decad-gallery/models/helicalgear"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/go2py"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/helical"
	"github.com/stretchr/testify/require"
)

// helixDriver runs the translated ValidateHelixAngle on every angle, given as text so NaN and infinities travel
// through JSON, and prints its message, or "" for an accepted angle.
const helixDriver = `
import json, sys
out = []
for text in json.load(sys.stdin):
    try:
        validate_helix_angle(float(text))
        out.append('')
    except RuntimeError as error:
        out.append(str(error))
json.dump(out, sys.stdout)
`

// TestTranslatedHelixCheckMatchesTheGallery requires the translated ValidateHelixAngle to accept and refuse the
// angles the gallery's does, with its messages.
func TestTranslatedHelixCheckMatchesTheGallery(t *testing.T) {
	_, m, _, _ := generate(t)
	text, err := go2py.TranspileSpec(m, helical.FeatureSpec)
	require.NoError(t, err)
	deg := math.Pi / 180
	angles := []float64{
		helicalgear.DefaultParams().HelixAngle, 20 * deg, 1 * deg, 45 * deg, 0, 0.5 * deg, 46 * deg, -10 * deg,
		math.NaN(), math.Inf(1),
	}
	texts := make([]string, len(angles))
	for i, a := range angles {
		texts[i] = strconv.FormatFloat(a, 'g', -1, 64)
	}
	in, err := json.Marshal(texts)
	require.NoError(t, err)
	cmd := exec.CommandContext(t.Context(), "python3", "-I", "-c", "import math\n\n"+text+"\n"+helixDriver)
	cmd.Stdin = bytes.NewReader(in)
	var stderr bytes.Buffer
	cmd.Stderr = &stderr
	raw, err := cmd.Output()
	require.NoError(t, err, stderr.String())
	var got []string
	require.NoError(t, json.Unmarshal(raw, &got))
	require.Len(t, got, len(angles))
	for i, a := range angles {
		want := ""
		if err := helicalgear.ValidateHelixAngle(a); err != nil {
			want = err.Error()
		}
		require.Equal(t, want, got[i], "angle %g", a)
	}
	require.Empty(t, got[0], "the default helix angle is accepted")
}

// TestGeneratedHelicalChecksTheHelixFirst requires the add-in to check the helix angle, then run the spur's checks,
// as the gallery's Build does.
func TestGeneratedHelicalChecksTheHelixFirst(t *testing.T) {
	out, _, _, _ := generate(t)
	require.Contains(t, out, "    def validateGallery(self):\n"+
		"        \"\"\"The gallery's helical build checks the helix angle, then runs the spur's checks.\"\"\"\n"+
		"        validate_helix_angle(self.getParameter(PARAM_HELIX_ANGLE).value)\n"+
		"        super().validateGallery()\n")
	require.Contains(t, out, "\ndef validate_helix_angle(angle):\n")
}
