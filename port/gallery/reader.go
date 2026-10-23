package gallery

import (
	"fmt"
	"go/ast"
	"go/parser"
	"go/token"
	"os"
	"path/filepath"
	"strings"
)

// Load parses the non-test Go files in dir, which must hold one gallery model
// package, and returns the facts extracted from them.
func Load(dir string) (*Model, error) {
	fset := token.NewFileSet()
	entries, err := os.ReadDir(dir)
	if err != nil {
		return nil, fmt.Errorf("read model directory: %w", err)
	}
	var files []*ast.File
	for _, e := range entries {
		name := e.Name()
		if e.IsDir() || !strings.HasSuffix(name, ".go") || strings.HasSuffix(name, "_test.go") {
			continue
		}
		f, err := parser.ParseFile(fset, filepath.Join(dir, name), nil, parser.SkipObjectResolution)
		if err != nil {
			return nil, fmt.Errorf("parse %s: %w", name, err)
		}
		files = append(files, f)
	}
	if len(files) == 0 {
		return nil, fmt.Errorf("no Go source files in %s", dir)
	}
	pkg := &parsedPackage{fset: fset, files: files}
	m := &Model{Package: files[0].Name.Name, pkg: pkg}
	if m.ParamsAlias, err = pkg.paramsAlias(); err != nil {
		return nil, err
	}
	if m.Inputs, err = pkg.inputs(); err != nil {
		return nil, err
	}
	m.Table, m.TableErr = pkg.table()
	return m, nil
}

// parsedPackage holds the parsed files of one model package.
type parsedPackage struct {
	fset  *token.FileSet
	files []*ast.File
}

// funcDecl returns the package-level function or method-less declaration called name.
func (p *parsedPackage) funcDecl(name string) (*ast.FuncDecl, error) {
	for _, f := range p.files {
		for _, d := range f.Decls {
			fd, ok := d.(*ast.FuncDecl)
			if ok && fd.Recv == nil && fd.Name.Name == name {
				return fd, nil
			}
		}
	}
	return nil, fmt.Errorf("function %s not found", name)
}

// callable returns the package-level function called name or, when there is none, the
// first method of that name on any receiver.
func (p *parsedPackage) callable(name string) (*ast.FuncDecl, error) {
	if fd, err := p.funcDecl(name); err == nil {
		return fd, nil
	}
	for _, f := range p.files {
		for _, d := range f.Decls {
			if fd, ok := d.(*ast.FuncDecl); ok && fd.Recv != nil && fd.Name.Name == name {
				return fd, nil
			}
		}
	}
	return nil, fmt.Errorf("function %s not found", name)
}

// method returns the first method called name on any receiver.
func (p *parsedPackage) method(name string) (*ast.FuncDecl, error) {
	for _, f := range p.files {
		for _, d := range f.Decls {
			if fd, ok := d.(*ast.FuncDecl); ok && fd.Recv != nil && fd.Name.Name == name {
				return fd, nil
			}
		}
	}
	return nil, fmt.Errorf("method %s not found", name)
}

// typeSpec returns the type declaration called name.
func (p *parsedPackage) typeSpec(name string) (*ast.TypeSpec, error) {
	for _, f := range p.files {
		for _, d := range f.Decls {
			gd, ok := d.(*ast.GenDecl)
			if !ok || gd.Tok != token.TYPE {
				continue
			}
			for _, s := range gd.Specs {
				ts := s.(*ast.TypeSpec)
				if ts.Name.Name == name {
					return ts, nil
				}
			}
		}
	}
	return nil, fmt.Errorf("type %s not found", name)
}

// pos formats the source position of n for error messages.
func (p *parsedPackage) pos(n ast.Node) string {
	return p.fset.Position(n.Pos()).String()
}

// paramsAlias returns the aliased type of Params, or "" when Params is not an alias.
func (p *parsedPackage) paramsAlias() (string, error) {
	ts, err := p.typeSpec("Params")
	if err != nil || !ts.Assign.IsValid() {
		return "", err
	}
	return p.exprText(ts.Type)
}
