package spurgear_test

import (
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/spurgear"
	"github.com/stretchr/testify/require"
)

func TestChordedOutlineBuildsOneValidProfile(t *testing.T) {
	out, err := spurgear.NewOutline(1, 20, 20, 15, 8)
	require.NoError(t, err)
	require.Len(t, out.Points, 20*(2*15+8+1))
	require.Greater(t, out.Area, 0.0)
	require.Greater(t, out.MinSegment, 0.0)

	_, profile, err := out.Sketch()
	require.NoError(t, err)
	require.True(t, profile.Valid)
}

func TestChordedOutlineRejectsEmbeddedFlanks(t *testing.T) {
	_, err := spurgear.NewOutline(1, 45, 20, 15, 8)
	require.ErrorIs(t, err, spurgear.ErrEmbedded)
}
