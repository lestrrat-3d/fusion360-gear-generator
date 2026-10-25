package gallery

import (
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"os"
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

// goModule is the part of the JSON that go list -m -json and go mod download -json print that Locate reads.
type goModule struct {
	Path    string
	Version string
	Dir     string
	Error   any
	Replace *struct {
		Path    string
		Version string
		Dir     string
	}
}

// Locate finds the model package modelName (for example "spurgear") in the gallery module that the Go module in
// workDir requires, and downloads the module into the module cache when it is not there yet. It reads the module
// cache and never a sibling checkout, so the revision is the one go.mod records. A replace directive for the
// gallery is refused: the code would come from the replacement while the generated header named the pinned version.
func Locate(ctx context.Context, workDir, modelName string) (Source, error) {
	var mod goModule
	if err := runGoJSON(ctx, workDir, &mod, "list", "-m", "-json", Module); err != nil {
		return Source{}, fmt.Errorf("locate gallery module in %s: %w", workDir, err)
	}
	if mod.Replace != nil {
		target := mod.Replace.Path
		if mod.Replace.Version != "" {
			target += "@" + mod.Replace.Version
		}
		return Source{}, fmt.Errorf("locate gallery module in %s: go.mod replaces %s with %s; remove the replace "+
			"directive, because a generated file names the pinned version %s", workDir, Module, target, mod.Version)
	}
	if mod.Version == "" {
		return Source{}, fmt.Errorf("locate gallery module in %s: go.mod pins no version of %s", workDir, Module)
	}
	if mod.Dir == "" {
		var dl goModule
		err := runGoJSON(ctx, workDir, &dl, "mod", "download", "-json", Module+"@"+mod.Version)
		if dl.Error != nil {
			return Source{}, fmt.Errorf("download gallery module %s@%s: %v", Module, mod.Version, dl.Error)
		}
		if err != nil {
			return Source{}, fmt.Errorf("download gallery module %s@%s: %w", Module, mod.Version, err)
		}
		if dl.Dir == "" {
			return Source{}, fmt.Errorf("download gallery module %s@%s: go mod download reported no directory",
				Module, mod.Version)
		}
		mod.Dir = dl.Dir
	}
	dir := filepath.Join(mod.Dir, "models", modelName)
	if info, err := os.Stat(dir); err != nil || !info.IsDir() {
		return Source{}, fmt.Errorf("gallery %s@%s has no model %s (looked in %s)", Module, mod.Version, modelName, dir)
	}
	return Source{Dir: dir, Version: mod.Version}, nil
}

// runGoJSON runs the go command with args in dir and decodes the JSON object it prints into out. go mod download
// prints a JSON object with an Error field and exits non-zero when it fails, so out is decoded before the exit
// status is reported.
func runGoJSON(ctx context.Context, dir string, out any, args ...string) error {
	cmd := exec.CommandContext(ctx, "go", args...)
	cmd.Dir = dir
	var stdout, stderr bytes.Buffer
	cmd.Stdout, cmd.Stderr = &stdout, &stderr
	runErr := cmd.Run()
	if stdout.Len() > 0 {
		if err := json.Unmarshal(stdout.Bytes(), out); err != nil {
			return fmt.Errorf("go %s: parse output: %w", strings.Join(args, " "), err)
		}
	}
	if runErr != nil {
		return fmt.Errorf("go %s: %w: %s", strings.Join(args, " "), runErr, strings.TrimSpace(stderr.String()))
	}
	return nil
}
