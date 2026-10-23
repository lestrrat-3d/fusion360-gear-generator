// Command gearport writes a Fusion add-in gear from the decad-gallery model that
// port/go.mod pins.
//
//	gearport gen spurgear      write lib/geargen/spurgear.py
//	gearport check spurgear    fail when that file differs from a fresh generation
//	gearport accept spurgear   record the gallery code the fragment was reviewed against
//
// Run it from the repository root.
package main

import (
	"context"
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"

	"github.com/lestrrat-3d/fusion360-gear-generator/port/gallery"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/spur"
)

const usage = "usage: gearport gen|check|accept spurgear"

func main() {
	if err := run(context.Background(), os.Args[1:]); err != nil {
		fmt.Fprintln(os.Stderr, "gearport:", err)
		os.Exit(1)
	}
}

func run(ctx context.Context, args []string) error {
	if len(args) != 2 || args[1] != "spurgear" {
		return fmt.Errorf("%s", usage)
	}
	src, err := gallery.Locate(ctx, "port", "spurgear")
	if err != nil {
		return err
	}
	m, err := gallery.Load(src.Dir)
	if err != nil {
		return err
	}
	switch args[0] {
	case "gen":
		return gen(m, src.Version, false)
	case "check":
		return gen(m, src.Version, true)
	case "accept":
		return accept(m)
	}
	return fmt.Errorf("%s", usage)
}

var (
	fragmentPath = filepath.Join("port", "spur", "fragments", "spurgear.py.frag")
	reviewedPath = filepath.Join("port", "spur", "reviewed.json")
	outputPath   = filepath.Join("lib", "geargen", "spurgear.py")
)

func gen(m *gallery.Model, version string, checkOnly bool) error {
	fragment, err := os.ReadFile(fragmentPath)
	if err != nil {
		return err
	}
	raw, err := os.ReadFile(reviewedPath)
	if err != nil {
		return fmt.Errorf("%w: run gearport accept spurgear", err)
	}
	var reviewed spur.Reviewed
	if err := json.Unmarshal(raw, &reviewed); err != nil {
		return fmt.Errorf("parse %s: %w", reviewedPath, err)
	}
	out, err := spur.Generate(m, version, string(fragment), reviewed)
	if err != nil {
		return err
	}
	if !checkOnly {
		return os.WriteFile(outputPath, []byte(out), 0o644)
	}
	have, err := os.ReadFile(outputPath)
	if err != nil {
		return err
	}
	if string(have) != out {
		return fmt.Errorf("%s differs from a fresh generation: run gearport gen spurgear", outputPath)
	}
	return nil
}

func accept(m *gallery.Model) error {
	reviewed, err := spur.ComputeReviewed(m)
	if err != nil {
		return err
	}
	raw, err := json.MarshalIndent(reviewed, "", "  ")
	if err != nil {
		return err
	}
	return os.WriteFile(reviewedPath, append(raw, '\n'), 0o644)
}
