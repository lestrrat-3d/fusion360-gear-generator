package gallery

import (
	"context"
	"fmt"
	"os/exec"
	"path/filepath"
	"strings"
)

// Module is the module of the decad-gallery repository.
const Module = "github.com/lestrrat-3d/decad-gallery"

// Source locates a gallery model package at the revision go.mod pins.
type Source struct {
	// Dir is the model package directory in the module cache.
	Dir string
	// Version is the pseudo-version go.mod pins for the gallery module.
	Version string
}

// Locate finds the model package modelName (for example "spurgear") in the gallery
// module that the Go module in workDir requires. It reads the module cache and
// never touches a sibling checkout, so the revision is the one go.mod records.
func Locate(ctx context.Context, workDir, modelName string) (Source, error) {
	cmd := exec.CommandContext(ctx, "go", "list", "-m", "-f", "{{.Dir}}\n{{.Version}}", Module)
	cmd.Dir = workDir
	out, err := cmd.Output()
	if err != nil {
		return Source{}, fmt.Errorf("locate gallery module in %s: %w", workDir, err)
	}
	dir, version, ok := strings.Cut(strings.TrimSpace(string(out)), "\n")
	if !ok || dir == "" || version == "" {
		return Source{}, fmt.Errorf("locate gallery module: unexpected go list output %q", out)
	}
	return Source{Dir: filepath.Join(dir, "models", modelName), Version: version}, nil
}
