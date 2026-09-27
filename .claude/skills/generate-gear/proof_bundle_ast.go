// Command proof_bundle_ast lists top-level Go declarations and their identifiers.
// The Python exporter uses this syntax tree rather than guessing declaration
// boundaries from text. It does not type-check the proof or replace its tests.
package main

import (
	"encoding/json"
	"fmt"
	"go/ast"
	"go/parser"
	"go/token"
	"go/types"
	"os"
	pathpkg "path"
	"path/filepath"
	"sort"
	"strconv"
)

type declaration struct {
	File     string   `json:"file"`
	Start    int      `json:"start"`
	End      int      `json:"end"`
	Names    []string `json:"names"`
	Refs     []string `json:"refs"`
	Receiver string   `json:"receiver,omitempty"`
}

type source struct {
	File       string        `json:"file"`
	Package    string        `json:"package"`
	Imports    []string      `json:"imports"`
	Unresolved []string      `json:"unresolved"`
	Decls      []declaration `json:"declarations"`
}

func position(fset *token.FileSet, pos token.Pos) int {
	return fset.PositionFor(pos, false).Offset
}

func docStart(pos token.Pos, doc *ast.CommentGroup) token.Pos {
	if doc != nil {
		return doc.Pos()
	}
	return pos
}

func receiverName(fn *ast.FuncDecl) string {
	if fn.Recv == nil || len(fn.Recv.List) == 0 {
		return ""
	}
	var expr ast.Expr = fn.Recv.List[0].Type
	if ptr, ok := expr.(*ast.StarExpr); ok {
		expr = ptr.X
	}
	if name, ok := expr.(*ast.Ident); ok {
		return name.Name
	}
	return ""
}

func identifiers(node ast.Node) []string {
	seen := map[string]struct{}{}
	ast.Inspect(node, func(n ast.Node) bool {
		if id, ok := n.(*ast.Ident); ok {
			seen[id.Name] = struct{}{}
		}
		return true
	})
	result := make([]string, 0, len(seen))
	for name := range seen {
		result = append(result, name)
	}
	sort.Strings(result)
	return result
}

func readFile(path string) (source, error) {
	fset := token.NewFileSet()
	file, err := parser.ParseFile(fset, path, nil, parser.ParseComments)
	if err != nil {
		return source{}, err
	}
	result := source{File: filepath.ToSlash(path), Package: file.Name.Name,
		Imports: []string{}, Unresolved: []string{}, Decls: []declaration{}}
	importNames := map[string]struct{}{}
	for _, decl := range file.Decls {
		switch d := decl.(type) {
		case *ast.FuncDecl:
			name := d.Name.Name
			receiver := receiverName(d)
			if receiver != "" {
				name = receiver + "." + name
			}
			result.Decls = append(result.Decls, declaration{
				File: result.File, Start: position(fset, docStart(d.Pos(), d.Doc)),
				End: position(fset, d.End()), Names: []string{name},
				Refs: identifiers(d), Receiver: receiver,
			})
		case *ast.GenDecl:
			if d.Tok == token.IMPORT {
				for _, spec := range d.Specs {
					imp := spec.(*ast.ImportSpec)
					result.Imports = append(result.Imports, imp.Path.Value)
					if imp.Name != nil {
						importNames[imp.Name.Name] = struct{}{}
					} else if value, err := strconv.Unquote(imp.Path.Value); err == nil {
						importNames[pathpkg.Base(value)] = struct{}{}
					}
				}
				continue
			}
			names := []string{}
			for _, spec := range d.Specs {
				switch s := spec.(type) {
				case *ast.TypeSpec:
					names = append(names, s.Name.Name)
				case *ast.ValueSpec:
					for _, name := range s.Names {
						names = append(names, name.Name)
					}
				}
			}
			result.Decls = append(result.Decls, declaration{
				File: result.File, Start: position(fset, docStart(d.Pos(), d.Doc)),
				End: position(fset, d.End()), Names: names, Refs: identifiers(d),
			})
		}
	}
	for _, id := range file.Unresolved {
		if types.Universe.Lookup(id.Name) != nil {
			continue
		}
		if _, imported := importNames[id.Name]; imported {
			continue
		}
		result.Unresolved = append(result.Unresolved, id.Name)
	}
	sort.Strings(result.Unresolved)
	return result, nil
}

func main() {
	if len(os.Args) < 2 {
		fmt.Fprintln(os.Stderr, "usage: go run proof_bundle_ast.go <proof.go> ...")
		os.Exit(2)
	}
	sources := make([]source, 0, len(os.Args)-1)
	for _, path := range os.Args[1:] {
		parsed, err := readFile(path)
		if err != nil {
			fmt.Fprintln(os.Stderr, err)
			os.Exit(2)
		}
		sources = append(sources, parsed)
	}
	if err := json.NewEncoder(os.Stdout).Encode(sources); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(2)
	}
}
