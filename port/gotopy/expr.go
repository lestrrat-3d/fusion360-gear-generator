package gotopy

import (
	"fmt"
	"go/ast"
	"go/token"
	"strings"
)

// mathFuncs maps the math package functions the subset allows to Python.
var mathFuncs = map[string]string{
	"Sin": "math.sin", "Cos": "math.cos", "Tan": "math.tan",
	"Asin": "math.asin", "Acos": "math.acos", "Atan": "math.atan", "Atan2": "math.atan2",
	"Sqrt": "math.sqrt", "Hypot": "math.hypot", "Abs": "abs", "Pow": "math.pow",
	"Floor": "math.floor", "Ceil": "math.ceil",
}

// expr renders an expression and reports its Go type.
func (f *fnState) expr(e ast.Expr) (string, typ, error) {
	switch e := e.(type) {
	case *ast.BasicLit:
		return f.literal(e)
	case *ast.Ident:
		return f.ident(e)
	case *ast.ParenExpr:
		text, t, err := f.expr(e.X)
		return "(" + text + ")", t, err
	case *ast.UnaryExpr:
		return f.unary(e)
	case *ast.BinaryExpr:
		return f.binary(e)
	case *ast.CallExpr:
		return f.call(e)
	case *ast.SelectorExpr:
		return f.selector(e)
	case *ast.IndexExpr:
		return f.index(e)
	case *ast.CompositeLit:
		return f.composite(e)
	}
	return "", typ{}, fmt.Errorf("%s: unsupported expression %T", f.pos(e), e)
}

func (f *fnState) literal(e *ast.BasicLit) (string, typ, error) {
	switch e.Kind {
	case token.INT:
		return e.Value, untypedType, nil
	case token.FLOAT:
		return e.Value, floatType, nil
	}
	return "", typ{}, fmt.Errorf("%s: unsupported literal %s", f.pos(e), e.Value)
}

func (f *fnState) ident(e *ast.Ident) (string, typ, error) {
	switch e.Name {
	case "true":
		return "True", boolType, nil
	case "false":
		return "False", boolType, nil
	}
	t, ok := f.env[e.Name]
	if !ok {
		return "", typ{}, fmt.Errorf("%s: unknown name %s", f.pos(e), e.Name)
	}
	return pyName(e.Name), t, nil
}

func (f *fnState) unary(e *ast.UnaryExpr) (string, typ, error) {
	text, t, err := f.expr(e.X)
	if err != nil {
		return "", typ{}, err
	}
	switch e.Op {
	case token.SUB:
		if !t.numeric() {
			return "", typ{}, fmt.Errorf("%s: negation of a non-number", f.pos(e))
		}
		return "-" + text, t, nil
	case token.NOT:
		return "not " + text, boolType, nil
	}
	return "", typ{}, fmt.Errorf("%s: unsupported unary operator %s", f.pos(e), e.Op)
}

var pyOps = map[token.Token]string{
	token.ADD: "+", token.SUB: "-", token.MUL: "*", token.QUO: "/",
	token.LSS: "<", token.LEQ: "<=", token.GTR: ">", token.GEQ: ">=", token.EQL: "==", token.NEQ: "!=",
	token.LAND: "and", token.LOR: "or",
}

func (f *fnState) binary(e *ast.BinaryExpr) (string, typ, error) {
	op, ok := pyOps[e.Op]
	if !ok {
		return "", typ{}, fmt.Errorf("%s: unsupported operator %s", f.pos(e), e.Op)
	}
	l, lt, err := f.operand(e.X)
	if err != nil {
		return "", typ{}, err
	}
	r, rt, err := f.operand(e.Y)
	if err != nil {
		return "", typ{}, err
	}
	text := l + " " + op + " " + r
	switch e.Op {
	case token.LSS, token.LEQ, token.GTR, token.GEQ, token.EQL, token.NEQ:
		return text, boolType, nil
	case token.LAND, token.LOR:
		return text, boolType, nil
	}
	rtype, err := arithmeticType(e.Op, lt, rt)
	if err != nil {
		return "", typ{}, fmt.Errorf("%s: %w", f.pos(e), err)
	}
	return text, rtype, nil
}

// operand renders a binary operand, wrapping a nested binary expression in parentheses.
func (f *fnState) operand(e ast.Expr) (string, typ, error) {
	text, t, err := f.expr(e)
	if err != nil {
		return "", typ{}, err
	}
	if _, nested := e.(*ast.BinaryExpr); nested {
		text = "(" + text + ")"
	}
	return text, t, nil
}

// arithmeticType returns the type of l op r. Integer division is refused because
// Go truncates and Python's / does not.
func arithmeticType(op token.Token, l, r typ) (typ, error) {
	if !l.numeric() || !r.numeric() {
		return typ{}, fmt.Errorf("operator %s on a non-number", op)
	}
	result := l
	switch {
	case l.kind == kindFloat || r.kind == kindFloat:
		result = floatType
	case l.kind == kindUntyped:
		result = r
	}
	if op == token.QUO && result.kind != kindFloat {
		return typ{}, fmt.Errorf("integer division is unsupported; convert an operand with float64")
	}
	return result, nil
}

func (f *fnState) call(e *ast.CallExpr) (string, typ, error) {
	switch fn := e.Fun.(type) {
	case *ast.Ident:
		return f.identCall(e, fn)
	case *ast.SelectorExpr:
		pkg, ok := fn.X.(*ast.Ident)
		if ok && pkg.Name == "math" {
			return f.mathCall(e, fn.Sel.Name)
		}
	}
	return "", typ{}, fmt.Errorf("%s: unsupported call", f.pos(e))
}

func (f *fnState) mathCall(e *ast.CallExpr, name string) (string, typ, error) {
	py, ok := mathFuncs[name]
	if !ok {
		return "", typ{}, fmt.Errorf("%s: math.%s is unsupported", f.pos(e), name)
	}
	args, err := f.args(e.Args)
	if err != nil {
		return "", typ{}, err
	}
	return py + "(" + strings.Join(args, ", ") + ")", floatType, nil
}

func (f *fnState) args(list []ast.Expr) ([]string, error) {
	out := make([]string, len(list))
	for i, a := range list {
		text, _, err := f.expr(a)
		if err != nil {
			return nil, err
		}
		out[i] = text
	}
	return out, nil
}

func (f *fnState) identCall(e *ast.CallExpr, fn *ast.Ident) (string, typ, error) {
	switch fn.Name {
	case "float64":
		return f.conversion(e, "float", floatType)
	case "int":
		return f.conversion(e, "int", intType)
	case "len":
		args, err := f.args(e.Args)
		if err != nil || len(args) != 1 {
			return "", typ{}, fmt.Errorf("%s: len takes one argument", f.pos(e))
		}
		return "len(" + args[0] + ")", intType, nil
	case "make":
		return f.make(e)
	}
	callee, ok := f.t.funcs[fn.Name]
	if !ok {
		return "", typ{}, fmt.Errorf("%s: call of %s, which is not translated", f.pos(e), fn.Name)
	}
	if len(callee.results) != 1 {
		return "", typ{}, fmt.Errorf("%s: %s returns %d values; the subset calls single-result functions", f.pos(e), fn.Name, len(callee.results))
	}
	args, err := f.args(e.Args)
	if err != nil {
		return "", typ{}, err
	}
	return f.t.prefix + pySnake(fn.Name) + "(" + strings.Join(args, ", ") + ")", callee.results[0].typ, nil
}

func (f *fnState) conversion(e *ast.CallExpr, py string, t typ) (string, typ, error) {
	if len(e.Args) != 1 {
		return "", typ{}, fmt.Errorf("%s: conversion takes one argument", f.pos(e))
	}
	text, at, err := f.expr(e.Args[0])
	if err != nil {
		return "", typ{}, err
	}
	if !at.numeric() {
		return "", typ{}, fmt.Errorf("%s: conversion of a non-number", f.pos(e))
	}
	return py + "(" + text + ")", t, nil
}

// make renders make([]T, n) as a list of n zero values.
func (f *fnState) make(e *ast.CallExpr) (string, typ, error) {
	if len(e.Args) != 2 {
		return "", typ{}, fmt.Errorf("%s: make needs a slice type and a length", f.pos(e))
	}
	st, err := fromAST(e.Args[0], f.t.structs)
	if err != nil || st.kind != kindSlice {
		return "", typ{}, fmt.Errorf("%s: make of a non-slice", f.pos(e))
	}
	n, _, err := f.expr(e.Args[1])
	if err != nil {
		return "", typ{}, err
	}
	fill := "None"
	if zero, err := st.elem.zero(); err == nil {
		fill = zero
	}
	return "[" + fill + "] * (" + n + ")", st, nil
}

func (f *fnState) selector(e *ast.SelectorExpr) (string, typ, error) {
	if pkg, ok := e.X.(*ast.Ident); ok && pkg.Name == "math" {
		if e.Sel.Name == "Pi" {
			return "math.pi", floatType, nil
		}
		return "", typ{}, fmt.Errorf("%s: math.%s is unsupported", f.pos(e), e.Sel.Name)
	}
	x, xt, err := f.expr(e.X)
	if err != nil {
		return "", typ{}, err
	}
	if xt.kind != kindStruct {
		return "", typ{}, fmt.Errorf("%s: field access on a non-struct", f.pos(e))
	}
	ft, ok := f.t.structs[xt.name].types[e.Sel.Name]
	if !ok {
		return "", typ{}, fmt.Errorf("%s: struct %s has no field %s", f.pos(e), xt.name, e.Sel.Name)
	}
	return x + "." + e.Sel.Name, ft, nil
}

func (f *fnState) index(e *ast.IndexExpr) (string, typ, error) {
	x, xt, err := f.expr(e.X)
	if err != nil {
		return "", typ{}, err
	}
	if xt.kind != kindSlice {
		return "", typ{}, fmt.Errorf("%s: index of a non-slice", f.pos(e))
	}
	i, _, err := f.expr(e.Index)
	if err != nil {
		return "", typ{}, err
	}
	return x + "[" + i + "]", *xt.elem, nil
}

// composite renders an unkeyed struct literal as a namedtuple construction.
func (f *fnState) composite(e *ast.CompositeLit) (string, typ, error) {
	id, ok := e.Type.(*ast.Ident)
	if !ok {
		return "", typ{}, fmt.Errorf("%s: unsupported composite literal", f.pos(e))
	}
	info, ok := f.t.structs[id.Name]
	if !ok {
		return "", typ{}, fmt.Errorf("%s: literal of untranslated type %s", f.pos(e), id.Name)
	}
	if len(e.Elts) != len(info.fields) {
		return "", typ{}, fmt.Errorf("%s: literal of %s needs %d unkeyed fields", f.pos(e), id.Name, len(info.fields))
	}
	args, err := f.args(e.Elts)
	if err != nil {
		return "", typ{}, err
	}
	return f.t.prefix + pyClass(id.Name) + "(" + strings.Join(args, ", ") + ")", typ{kind: kindStruct, name: id.Name}, nil
}
