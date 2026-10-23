package gallery

import (
	"crypto/sha256"
	"encoding/hex"
	"fmt"
	"go/ast"
	"go/printer"
	"strings"
)

// featureOps are the decad functions that add or change a body. Selectors such as
// Edges and ParallelTo, and constructors such as New, are not feature operations.
var featureOps = map[string]struct{}{
	"Extrude": {}, "Revolve": {}, "Loft": {}, "Sweep": {},
	"Union": {}, "Cut": {}, "Intersect": {},
	"PatternCopies": {}, "Patterned": {}, "PatternJoin": {},
	"Fillet": {}, "Chamfer": {}, "Shell": {},
}

// DecadCalls lists, in source order, the decad feature operations the build
// function calls, following calls to package-local functions depth first. A helper
// that the build calls from two places is listed at each call. Both branches of an
// if are listed, so the result is every operation the build may perform in the order
// the source mentions it.
func (m *Model) DecadCalls(build string) ([]string, error) {
	var out []string
	if err := m.pkg.collectCalls(build, map[string]bool{}, &out); err != nil {
		return nil, err
	}
	return out, nil
}

func (p *parsedPackage) collectCalls(name string, active map[string]bool, out *[]string) error {
	if active[name] {
		return fmt.Errorf("function %s calls itself", name)
	}
	fd, err := p.funcDecl(name)
	if err != nil {
		return err
	}
	active[name] = true
	defer delete(active, name)
	var walkErr error
	ast.Inspect(fd.Body, func(n ast.Node) bool {
		call, ok := n.(*ast.CallExpr)
		if !ok || walkErr != nil {
			return walkErr == nil
		}
		switch fn := call.Fun.(type) {
		case *ast.SelectorExpr:
			if id, ok := fn.X.(*ast.Ident); ok && id.Name == "decad" {
				if _, op := featureOps[fn.Sel.Name]; op {
					*out = append(*out, fn.Sel.Name)
				}
			}
		case *ast.Ident:
			if _, err := p.funcDecl(fn.Name); err == nil {
				walkErr = p.collectCalls(fn.Name, active, out)
			}
		}
		return walkErr == nil
	})
	return walkErr
}

// SourceDigest returns a digest of the named functions' code. Comments and
// formatting do not change it; any change to a statement does. A fragment records
// the digest of the Go it was written against, so a changed gallery function
// forces a review of the fragment.
func (m *Model) SourceDigest(funcs ...string) (string, error) {
	h := sha256.New()
	for _, name := range funcs {
		fd, err := m.pkg.funcDecl(name)
		if err != nil {
			return "", err
		}
		var sb strings.Builder
		cfg := printer.Config{Mode: printer.RawFormat}
		if err := cfg.Fprint(&sb, m.pkg.fset, fd); err != nil {
			return "", fmt.Errorf("print %s: %w", name, err)
		}
		h.Write([]byte(name + "\n" + sb.String() + "\n"))
	}
	return hex.EncodeToString(h.Sum(nil)), nil
}
