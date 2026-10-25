package main

import (
	"os"
	"path/filepath"
	"testing"

	"github.com/stretchr/testify/require"
)

// TestIDReadsOnlyTheFile requires gearport id to work where the gallery cannot be located: it reads the id from
// the generated file alone.
func TestIDReadsOnlyTheFile(t *testing.T) {
	dir := t.TempDir()
	require.NoError(t, os.MkdirAll(filepath.Join(dir, "lib", "geargen"), 0o755))
	require.NoError(t, os.WriteFile(filepath.Join(dir, "lib", "geargen", "spurgear.py"), []byte("# header\nx = 1\n"), 0o600))
	t.Chdir(dir)
	require.NoError(t, run(t.Context(), []string{"id", "spurgear"}))
}
