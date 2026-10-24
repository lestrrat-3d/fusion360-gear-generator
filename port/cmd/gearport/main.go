// Command gearport writes a Fusion add-in gear from the decad-gallery model that
// port/go.mod pins.
//
//	gearport gen <gear>      write lib/geargen/<gear>.py
//	gearport check <gear>    fail when that file differs from a fresh generation
//	gearport accept <gear>   record the gallery code the fragment was reviewed against
//	gearport id <gear>       print the build id the file's contents carry
//
// The gears are spurgear, helicalgear, herringbonegear and bevelgear.
//
// Run it from the repository root.
package main

import (
	"context"
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"

	"github.com/lestrrat-3d/fusion360-gear-generator/port/bevel"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/gallery"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/helical"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/herringbone"
	"github.com/lestrrat-3d/fusion360-gear-generator/port/spur"
)

const usage = "usage: gearport gen|check|accept|id spurgear|helicalgear|herringbonegear|bevelgear"

// gear describes one ported gear: where its stored files live and how to generate it.
type gear struct {
	fragment, reviewed, output string
	generate                   func(m *gallery.Model, version, fragment string, reviewed spur.Reviewed) (string, error)
	computeReviewed            func(m *gallery.Model) (spur.Reviewed, error)
}

var gears = map[string]gear{
	"spurgear": {
		fragment:        filepath.Join("port", "spur", "fragments", "spurgear.py.frag"),
		reviewed:        filepath.Join("port", "spur", "reviewed.json"),
		output:          filepath.Join("lib", "geargen", "spurgear.py"),
		generate:        spur.Generate,
		computeReviewed: spur.ComputeReviewed,
	},
	"helicalgear": {
		fragment:        filepath.Join("port", "helical", "fragments", "helicalgear.py.frag"),
		reviewed:        filepath.Join("port", "helical", "reviewed.json"),
		output:          filepath.Join("lib", "geargen", "helicalgear.py"),
		generate:        helical.Generate,
		computeReviewed: helical.ComputeReviewed,
	},
	"bevelgear": {
		fragment:        filepath.Join("port", "bevel", "fragments", "bevelgear.py.frag"),
		reviewed:        filepath.Join("port", "bevel", "reviewed.json"),
		output:          filepath.Join("lib", "geargen", "bevelgear.py"),
		generate:        bevel.Generate,
		computeReviewed: bevel.ComputeReviewed,
	},
	"herringbonegear": {
		fragment:        filepath.Join("port", "herringbone", "fragments", "herringbonegear.py.frag"),
		reviewed:        filepath.Join("port", "herringbone", "reviewed.json"),
		output:          filepath.Join("lib", "geargen", "herringbonegear.py"),
		generate:        herringbone.Generate,
		computeReviewed: herringbone.ComputeReviewed,
	},
}

func main() {
	if err := run(context.Background(), os.Args[1:]); err != nil {
		fmt.Fprintln(os.Stderr, "gearport:", err)
		os.Exit(1)
	}
}

func run(ctx context.Context, args []string) error {
	if len(args) != 2 {
		return fmt.Errorf("%s", usage)
	}
	g, ok := gears[args[1]]
	if !ok {
		return fmt.Errorf("%s", usage)
	}
	src, err := gallery.Locate(ctx, "port", args[1])
	if err != nil {
		return err
	}
	m, err := gallery.Load(src.Dir)
	if err != nil {
		return err
	}
	switch args[0] {
	case "id":
		have, err := os.ReadFile(g.output)
		if err != nil {
			return err
		}
		fmt.Println(spur.FileID(string(have)))
		return nil
	case "gen":
		return g.gen(m, src.Version, args[1], false)
	case "check":
		return g.gen(m, src.Version, args[1], true)
	case "accept":
		return g.accept(m)
	}
	return fmt.Errorf("%s", usage)
}

func (g gear) gen(m *gallery.Model, version, name string, checkOnly bool) error {
	fragment, err := os.ReadFile(g.fragment)
	if err != nil {
		return err
	}
	raw, err := os.ReadFile(g.reviewed)
	if err != nil {
		return fmt.Errorf("%w: run gearport accept %s", err, name)
	}
	var reviewed spur.Reviewed
	if err := json.Unmarshal(raw, &reviewed); err != nil {
		return fmt.Errorf("parse %s: %w", g.reviewed, err)
	}
	out, err := g.generate(m, version, string(fragment), reviewed)
	if err != nil {
		return err
	}
	if !checkOnly {
		return os.WriteFile(g.output, []byte(out), 0o644)
	}
	have, err := os.ReadFile(g.output)
	if err != nil {
		return err
	}
	if string(have) != out {
		return fmt.Errorf("%s differs from a fresh generation: run gearport gen %s", g.output, name)
	}
	return nil
}

func (g gear) accept(m *gallery.Model) error {
	reviewed, err := g.computeReviewed(m)
	if err != nil {
		return err
	}
	raw, err := json.MarshalIndent(reviewed, "", "  ")
	if err != nil {
		return err
	}
	return os.WriteFile(g.reviewed, append(raw, '\n'), 0o644)
}
