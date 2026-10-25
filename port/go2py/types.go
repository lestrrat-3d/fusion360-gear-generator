// Package go2py translates a small subset of Go into Python. The subset is the
// pure numeric helpers of a gallery model: float and int arithmetic, math calls,
// small structs, slices, if, and counted for loops. Anything outside it is an error,
// so a helper that grows past the subset fails the port instead of mistranslating.
package go2py

import (
	"fmt"
	"go/ast"
	"strings"
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
	// kindCtx is a context.Context. A translated function takes none and passes none.
	kindCtx
	kindString
	// kindNil is the nil literal, and the value of any field of a zero-valued struct.
	kindNil
	kindMap
)

// typ is the Go type of a translated expression.
type typ struct {
	kind kind
	// name is the struct type name when kind is kindStruct, and the Go type when kind is kindOpaque.
	name string
	// elem is the element type when kind is kindSlice, and the value type when kind is kindMap.
	elem *typ
	// key is the key type when kind is kindMap.
	key *typ
	// variadic marks the type of a parameter written ...T, which Python takes as *name.
	variadic bool
}

var (
	floatType   = typ{kind: kindFloat}
	intType     = typ{kind: kindInt}
	untypedType = typ{kind: kindUntyped}
	boolType    = typ{kind: kindBool}
	stringType  = typ{kind: kindString}
	nilType     = typ{kind: kindNil}
	ctxType     = typ{kind: kindCtx}
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
	case kindOpaque, kindNil:
		return "None", nil
	case kindMap:
		return "{}", nil
	case kindString:
		return `""`, nil
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
		case "string":
			return stringType, nil
		case "any":
			return typ{kind: kindOpaque, name: "any"}, nil
		}
		if _, ok := structs[e.Name]; ok {
			return typ{kind: kindStruct, name: e.Name}, nil
		}
	case *ast.StarExpr:
		return fromAST(e.X, structs)
	case *ast.SelectorExpr:
		if name, ok := opaqueType(e); ok {
			return typ{kind: kindOpaque, name: name}, nil
		}
		if pkg, ok := e.X.(*ast.Ident); ok {
			if info, ok := structs[pkg.Name+"."+e.Sel.Name]; ok && info.external {
				return typ{kind: kindOpaque, name: info.name}, nil
			}
		}
		if pkg, ok := e.X.(*ast.Ident); ok && pkg.Name == "context" && e.Sel.Name == "Context" {
			return ctxType, nil
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
	case *ast.MapType:
		key, err := fromAST(e.Key, structs)
		if err != nil {
			return typ{}, err
		}
		val, err := fromAST(e.Value, structs)
		if err != nil {
			return typ{}, err
		}
		return typ{kind: kindMap, key: &key, elem: &val}, nil
	case *ast.Ellipsis:
		elem, err := fromAST(e.Elt, structs)
		if err != nil {
			return typ{}, err
		}
		return typ{kind: kindSlice, elem: &elem, variadic: true}, nil
	case *ast.FuncType:
		ft := typ{kind: kindOpaque, name: "func"}
		if e.Results != nil {
			for _, r := range e.Results.List {
				if id, ok := r.Type.(*ast.Ident); ok && id.Name == "error" {
					continue
				}
				rt, err := fromAST(r.Type, structs)
				if err != nil {
					return typ{}, err
				}
				ft.elem = &rt
			}
		}
		return ft, nil
	case *ast.StructType:
		if len(e.Fields.List) == 0 {
			return typ{kind: kindOpaque, name: "struct{}"}, nil
		}
		return anonStruct(e, structs)
	}
	return typ{}, fmt.Errorf("unsupported type %T", e)
}

// opaquePackages are the packages whose types a translated function may hold without
// looking inside. The Python runtime implements the methods the API table lists.
var opaquePackages = map[string]bool{"sketch": true, "decad": true, "r3": true, "units": true}

// opaqueType returns "pkg.Name" for a selector naming a type of an opaque package.
func opaqueType(e *ast.SelectorExpr) (string, bool) {
	pkg, ok := e.X.(*ast.Ident)
	if !ok || !opaquePackages[pkg.Name] {
		return "", false
	}
	return pkg.Name + "." + e.Sel.Name, true
}

// structInfo is a translated struct type: a Python namedtuple.
type structInfo struct {
	name   string
	fields []string
	types  map[string]typ
	// zero marks a struct the translation treats as its zero value, such as an options
	// struct with every field unset. Every field reads as nil.
	zero bool
	// external marks a struct of another gallery package, which the translation holds as an
	// opaque handle and reads only the listed fields of.
	external bool
	// anonymous marks a struct type written inline; its class is rendered with the others.
	anonymous bool
	// assigned marks a struct the translated code fills in field by field, which becomes a class.
	assigned bool
}

// mutable reports whether the struct holds slices or handles, which a function fills in
// field by field. It becomes a plain class instead of an immutable namedtuple.
func (s *structInfo) mutable() bool {
	if s.assigned {
		return true
	}
	for _, t := range s.types {
		if t.kind == kindSlice || t.kind == kindOpaque {
			return true
		}
	}
	return false
}

// anonStruct registers an anonymous struct type, such as the row type of a table written inline, under a
// name made from its fields, and returns it. The same fields give the same name.
func anonStruct(e *ast.StructType, structs map[string]*structInfo) (typ, error) {
	info := &structInfo{types: map[string]typ{}, anonymous: true}
	for _, f := range e.Fields.List {
		ft, err := fromAST(f.Type, structs)
		if err != nil {
			return typ{}, err
		}
		for _, n := range f.Names {
			info.fields = append(info.fields, n.Name)
			info.types[n.Name] = ft
		}
	}
	info.name = "Anon" + strings.Join(info.fields, "")
	if existing, ok := structs[info.name]; ok {
		info = existing
	} else {
		structs[info.name] = info
	}
	return typ{kind: kindStruct, name: info.name}, nil
}
