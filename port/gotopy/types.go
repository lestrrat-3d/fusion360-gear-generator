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
)

// typ is the Go type of a translated expression.
type typ struct {
	kind kind
	// name is the struct type name when kind is kindStruct.
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

// structInfo is a translated struct type: a Python namedtuple.
type structInfo struct {
	name   string
	fields []string
	types  map[string]typ
}
