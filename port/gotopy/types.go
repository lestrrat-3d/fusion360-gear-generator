// Package gotopy translates a small subset of Go into Python. The subset is the
// pure numeric helpers of a gallery model: float and int arithmetic, math calls,
// small structs, slices, if, and counted for loops. Anything outside it is an error,
// so a helper that grows past the subset fails the port instead of mistranslating.
package gotopy

import (
	"fmt"
	"go/ast"
)

type kind int

const (
	kindFloat kind = iota
	kindInt
	// kindUntyped is an integer constant that takes the type of its other operand.
	kindUntyped
	kindBool
	kindStruct
	kindSlice
	// kindOpaque is a handle the Python runtime owns, such as a sketch point.
	kindOpaque
)

// typ is the Go type of a translated expression.
type typ struct {
	kind kind
	// name is the struct type name when kind is kindStruct, and the Go type when kind is kindOpaque.
	name string
	// elem is the element type when kind is kindSlice.
	elem *typ
}

var (
	floatType   = typ{kind: kindFloat}
	intType     = typ{kind: kindInt}
	untypedType = typ{kind: kindUntyped}
	boolType    = typ{kind: kindBool}
)

// numeric reports whether t is a float, int or untyped integer.
func (t typ) numeric() bool { return t.kind == kindFloat || t.kind == kindInt || t.kind == kindUntyped }

// zero returns the Python literal for the zero value of t.
func (t typ) zero() (string, error) {
	switch t.kind {
	case kindFloat:
		return "0.0", nil
	case kindInt, kindUntyped:
		return "0", nil
	case kindBool:
		return "False", nil
	case kindOpaque:
		return "None", nil
	}
	return "", fmt.Errorf("no zero value for a %v type", t.kind)
}

// fromAST converts a Go type expression. structs lists the struct types that may be used.
func fromAST(e ast.Expr, structs map[string]*structInfo) (typ, error) {
	switch e := e.(type) {
	case *ast.Ident:
		switch e.Name {
		case "float64":
			return floatType, nil
		case "int":
			return intType, nil
		case "bool":
			return boolType, nil
		}
		if _, ok := structs[e.Name]; ok {
			return typ{kind: kindStruct, name: e.Name}, nil
		}
	case *ast.StarExpr:
		if name, ok := sketchType(e.X); ok {
			return typ{kind: kindOpaque, name: "sketch." + name}, nil
		}
	case *ast.ArrayType:
		if e.Len != nil {
			break
		}
		elem, err := fromAST(e.Elt, structs)
		if err != nil {
			return typ{}, err
		}
		return typ{kind: kindSlice, elem: &elem}, nil
	}
	return typ{}, fmt.Errorf("unsupported type %T", e)
}

// sketchType returns the type name of a sketch.Name selector that is an opaque handle.
func sketchType(e ast.Expr) (string, bool) {
	sel, ok := e.(*ast.SelectorExpr)
	if !ok {
		return "", false
	}
	pkg, ok := sel.X.(*ast.Ident)
	if !ok || pkg.Name != "sketch" || !opaqueTypes[sel.Sel.Name] {
		return "", false
	}
	return sel.Sel.Name, true
}

// opaqueTypes are the sketch package types a translated function may hold without looking inside.
var opaqueTypes = map[string]bool{"Point": true, "Sketch": true}

// structInfo is a translated struct type: a Python namedtuple.
type structInfo struct {
	name   string
	fields []string
	types  map[string]typ
}

// mutable reports whether the struct holds slices or handles, which a function fills in
// field by field. It becomes a plain class instead of an immutable namedtuple.
func (s *structInfo) mutable() bool {
	for _, t := range s.types {
		if t.kind == kindSlice || t.kind == kindOpaque {
			return true
		}
	}
	return false
}
