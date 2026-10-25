package go2py

import (
	"fmt"
	"go/ast"
	"go/token"
	"strconv"
	"strings"
)

// mathFuncs maps the math package functions the subset allows to Python.
var mathFuncs = map[string]string{
	"Sin": "math.sin", "Cos": "math.cos", "Tan": "math.tan",
	"Asin": "math.asin", "Acos": "math.acos", "Atan": "math.atan", "Atan2": "math.atan2",
	"Sqrt": "math.sqrt", "Hypot": "math.hypot", "Abs": "abs", "Pow": "math.pow",
	"Floor": "math.floor", "Ceil": "math.ceil", "Min": "min", "Max": "max",
	"Ldexp": "math.ldexp", "Round": "_fx.math_round", "Ilogb": "_fx.math_ilogb",
}

// expr renders an expression and reports its Go type.
func (f *fnState) expr(e ast.Expr) (string, typ, error) {
	switch e := e.(type) {
	case *ast.BasicLit:
		return f.literal(e)
	case *ast.Ident:
		return f.ident(e)
	case *ast.StarExpr:
		text, t, err := f.expr(e.X)
		if err == nil && t.ptr {
			t.ptr = false
			return text + ".value", t, nil
		}
		return text, t, err
	case *ast.SliceExpr:
		return f.slice(e)
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
		return e.Value, untypedFloatType, nil
	case token.STRING:
		if f.t.sketch {
			if strings.HasPrefix(e.Value, "`") {
				// A raw string has no escapes in Go; Python reads backslashes and quotes in it.
				text, err := strconv.Unquote(e.Value)
				if err != nil {
					return "", typ{}, fmt.Errorf("%s: %w", f.pos(e), err)
				}
				return quote(text), stringType, nil
			}
			return e.Value, stringType, nil
		}
	}
	return "", typ{}, fmt.Errorf("%s: unsupported literal %s", f.pos(e), e.Value)
}

func (f *fnState) ident(e *ast.Ident) (string, typ, error) {
	switch e.Name {
	case "true":
		return "True", boolType, nil
	case "false":
		return "False", boolType, nil
	case "nil":
		return "None", nilType, nil
	}
	t, ok := f.env[e.Name]
	if !ok {
		if g, ok := f.t.globals[e.Name]; ok {
			return g.py, g.t, nil
		}
		if f.t.sketch && isErrIdent(e) {
			// An error variable: a translated call raises, so none is ever set.
			return "None", nilType, nil
		}
		return "", typ{}, fmt.Errorf("%s: unknown name %s", f.pos(e), e.Name)
	}
	return pyName(e.Name), t, nil
}

func (f *fnState) unary(e *ast.UnaryExpr) (string, typ, error) {
	if e.Op == token.AND {
		if sel, ok := e.X.(*ast.SelectorExpr); ok {
			// The address of a number field is a reference to the field.
			obj, _, err := f.expr(sel.X)
			if err != nil {
				return "", typ{}, err
			}
			_, ft, err := f.expr(sel)
			if err == nil && (ft.numeric() || ft.kind == kindBool) {
				ft.ptr = true
				return f.t.runtime + ".field_ref(" + obj + ", " + quote(sel.Sel.Name) + ")", ft, nil
			}
		}
		// A pointer to a literal is the literal: translated structs are Python objects.
		return f.expr(e.X)
	}
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
	token.ADD: "+", token.SUB: "-", token.MUL: "*", token.QUO: "/", token.REM: "%",
	token.LSS: "<", token.LEQ: "<=", token.GTR: ">", token.GEQ: ">=", token.EQL: "==", token.NEQ: "!=",
	token.LAND: "and", token.LOR: "or",
}

func (f *fnState) binary(e *ast.BinaryExpr) (string, typ, error) {
	op, ok := pyOps[e.Op]
	if !ok {
		return "", typ{}, fmt.Errorf("%s: unsupported operator %s", f.pos(e), e.Op)
	}
	if e.Op == token.EQL || e.Op == token.NEQ {
		// x % y == 0 holds in Go exactly when it holds in Python, whatever the signs.
		for _, side := range [][2]ast.Expr{{e.X, e.Y}, {e.Y, e.X}} {
			if rem, ok := unparen(side[0]).(*ast.BinaryExpr); ok && rem.Op == token.REM && isZero(side[1]) {
				f.remZero = append(f.remZero, rem)
			}
		}
	}
	l, lt, err := f.operand(e.X)
	if err != nil {
		return "", typ{}, err
	}
	r, rt, err := f.operand(e.Y)
	if err != nil {
		return "", typ{}, err
	}
	if (lt.kind == kindCtx || rt.kind == kindCtx) && (e.Op == token.EQL || e.Op == token.NEQ) {
		// A translated function takes no context, so one is never nil.
		return map[token.Token]string{token.EQL: "False", token.NEQ: "True"}[e.Op], boolType, nil
	}
	if (lt.kind == kindNil || rt.kind == kindNil) && (e.Op == token.EQL || e.Op == token.NEQ) {
		if lt.kind == kindSlice || rt.kind == kindSlice || lt.kind == kindMap || rt.kind == kindMap {
			return "", typ{}, fmt.Errorf("%s: comparing a slice or map with nil is unsupported: a nil slice is an "+
				"empty list in Python; test len instead", f.pos(e))
		}
		op = map[token.Token]string{token.EQL: "is", token.NEQ: "is not"}[e.Op]
		return l + " " + op + " " + r, boolType, nil
	}
	if (e.Op == token.EQL || e.Op == token.NEQ) && (lt.kind == kindStruct || rt.kind == kindStruct) {
		for _, t := range []typ{lt, rt} {
			if info := f.t.structs[t.name]; t.kind == kindStruct && info != nil && info.mutable() && !info.assigned {
				return "", typ{}, fmt.Errorf("%s: comparing %s values needs equality by field; list %s in Spec.Mutable",
					f.pos(e), t.name, t.name)
			}
		}
	}
	if e.Op == token.REM && !f.remainderAgrees(e) {
		// Go's remainder takes the sign of the dividend and Python's the sign of the divisor; fmod is Go's.
		if !lt.numeric() || !rt.numeric() {
			return "", typ{}, fmt.Errorf("%s: operator %% on a non-number", f.pos(e))
		}
		return "int(math.fmod(" + l + ", " + r + "))", intType, nil
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

// remainderAgrees reports whether Python's x % y gives Go's result: when the remainder is only compared with zero,
// or when neither operand can be negative.
func (f *fnState) remainderAgrees(e *ast.BinaryExpr) bool {
	for _, z := range f.remZero {
		if z == e {
			return true
		}
	}
	return f.neverNegative(e.X) && f.neverNegative(e.Y)
}

// neverNegative reports whether an int expression cannot be below zero: a constant, len, a loop counter that never
// goes below zero, or a sum, product or remainder of those.
func (f *fnState) neverNegative(e ast.Expr) bool {
	switch e := unparen(e).(type) {
	case *ast.BasicLit:
		return e.Kind == token.INT
	case *ast.Ident:
		_, ok := f.counters[e.Name]
		return ok
	case *ast.CallExpr:
		return isIdent(e.Fun, "len")
	case *ast.BinaryExpr:
		return (e.Op == token.ADD || e.Op == token.MUL || e.Op == token.REM) && f.neverNegative(e.X) && f.neverNegative(e.Y)
	}
	return false
}

// nonNegativeConstant reports whether e is an integer literal, which is never negative.
func nonNegativeConstant(e ast.Expr) bool {
	lit, ok := unparen(e).(*ast.BasicLit)
	return ok && lit.Kind == token.INT
}

func unparen(e ast.Expr) ast.Expr {
	for {
		p, ok := e.(*ast.ParenExpr)
		if !ok {
			return e
		}
		e = p.X
	}
}

func isZero(e ast.Expr) bool {
	lit, ok := unparen(e).(*ast.BasicLit)
	return ok && lit.Kind == token.INT && lit.Value == "0"
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
	if (l.kind == kindUntypedFloat && r.kind == kindInt) || (l.kind == kindInt && r.kind == kindUntypedFloat) {
		return typ{}, fmt.Errorf("operator %s on an int and a floating-point constant: Go converts the constant to "+
			"int and Python does not; write the constant as an integer or convert the int with float64", op)
	}
	result := l
	switch {
	case l.kind == kindFloat || r.kind == kindFloat:
		result = floatType
	case l.kind == kindUntypedFloat || r.kind == kindUntypedFloat:
		result = untypedFloatType
	case l.kind == kindUntyped:
		result = r
	}
	if op == token.QUO && result.kind != kindFloat && result.kind != kindUntypedFloat {
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
		if ok && f.t.sketch && apiConversions[pkg.Name+"."+fn.Sel.Name] && len(e.Args) == 1 {
			// A conversion between handles the runtime holds as one object.
			text, _, err := f.expr(e.Args[0])
			return text, typ{kind: kindOpaque, name: pkg.Name + "." + fn.Sel.Name}, err
		}
		if ok && f.t.sketch {
			switch pkg.Name + "." + fn.Sel.Name {
			case "errors.Is":
				// A translated call raises instead of returning an error: see "Retry loops".
				return "False", boolType, nil
			case "fmt.Sprintf":
				return f.sprintf(e)
			case "sort.Slice":
				return f.sortSlice(e)
			}
		}
		if f.t.sketch {
			if text, rt, handled, err := f.externalCall(e, fn); handled {
				return text, rt, err
			}
			if text, rt, handled, err := f.apiCall(e, fn); handled {
				return text, rt, err
			}
			return f.structCall(e, fn)
		}
	}
	return "", typ{}, fmt.Errorf("%s: unsupported call", f.pos(e))
}

func (f *fnState) mathCall(e *ast.CallExpr, name string) (string, typ, error) {
	if name == "Inf" && len(e.Args) == 1 {
		sign, ok := constantSign(e.Args[0])
		if !ok {
			return "", typ{}, fmt.Errorf("%s: math.Inf needs a constant sign", f.pos(e))
		}
		if sign < 0 {
			return "-math.inf", floatType, nil
		}
		return "math.inf", floatType, nil
	}
	switch name {
	case "IsNaN", "IsInf":
		if (name == "IsNaN" && len(e.Args) != 1) || (name == "IsInf" && len(e.Args) != 2) {
			return "", typ{}, fmt.Errorf("%s: math.%s has the wrong number of arguments", f.pos(e), name)
		}
		x, _, err := f.expr(e.Args[0])
		if err != nil {
			return "", typ{}, err
		}
		if name == "IsNaN" {
			return "math.isnan(" + x + ")", boolType, nil
		}
		sign, ok := constantSign(e.Args[1])
		switch {
		case !ok:
			return "", typ{}, fmt.Errorf("%s: math.IsInf needs a constant sign", f.pos(e))
		case sign > 0:
			return "(" + x + " == math.inf)", boolType, nil
		case sign < 0:
			return "(" + x + " == -math.inf)", boolType, nil
		}
		// IsInf(x, 0) is true for an infinity of either sign, as math.isinf is.
		return "math.isinf(" + x + ")", boolType, nil
	}
	py, ok := mathFuncs[name]
	if !ok {
		return "", typ{}, fmt.Errorf("%s: math.%s is unsupported", f.pos(e), name)
	}
	args, err := f.args(e.Args)
	if err != nil {
		return "", typ{}, err
	}
	if name == "Ilogb" {
		return py + "(" + strings.Join(args, ", ") + ")", intType, nil
	}
	return py + "(" + strings.Join(args, ", ") + ")", floatType, nil
}

// constantSign returns the sign of an integer literal, possibly negated: -1, 0 or 1.
func constantSign(e ast.Expr) (int, bool) {
	e = unparen(e)
	negative := false
	if u, ok := e.(*ast.UnaryExpr); ok && (u.Op == token.SUB || u.Op == token.ADD) {
		negative = u.Op == token.SUB
		e = unparen(u.X)
	}
	lit, ok := e.(*ast.BasicLit)
	if !ok || lit.Kind != token.INT {
		return 0, false
	}
	v, err := strconv.ParseInt(lit.Value, 0, 64)
	if err != nil {
		return 0, false
	}
	switch {
	case v == 0:
		return 0, true
	case negative:
		return -1, true
	}
	return 1, true
}

func (f *fnState) args(list []ast.Expr) ([]string, error) {
	out := make([]string, 0, len(list))
	for _, a := range list {
		text, at, err := f.expr(a)
		if err != nil {
			return nil, err
		}
		if at.kind == kindCtx {
			continue
		}
		out = append(out, text)
	}
	return out, nil
}

// structCall renders a call on a value of a translated struct: a field that holds a function,
// such as v.Tooth(ctx, in), or a method, such as in.Shifted(d), which becomes a call of the
// translated function with the receiver first.
func (f *fnState) structCall(e *ast.CallExpr, sel *ast.SelectorExpr) (string, typ, error) {
	x, xt, err := f.expr(sel.X)
	if err != nil || xt.kind != kindStruct || f.t.structs[xt.name] == nil {
		return "", typ{}, fmt.Errorf("%s: unsupported call", f.pos(e))
	}
	info := f.t.structs[xt.name]
	if ft, ok := info.types[sel.Sel.Name]; ok && ft.kind == kindOpaque && ft.name == "func" {
		args, err := f.args(e.Args)
		if err != nil {
			return "", typ{}, err
		}
		rt := typ{kind: kindOpaque, name: "void"}
		if ft.elem != nil {
			rt = *ft.elem
		}
		return x + "." + sel.Sel.Name + "(" + strings.Join(args, ", ") + ")", rt, nil
	}
	if b, ok := f.t.bound[sel.Sel.Name]; ok {
		// A method the runtime implements: the receiver is its first argument.
		f.t.use("Bound", sel.Sel.Name)
		args, err := f.args(e.Args)
		if err != nil {
			return "", typ{}, err
		}
		return b.Python + "(" + strings.Join(append([]string{x}, args...), ", ") + ")", boundType(b.Result), nil
	}
	callee, ok := f.t.funcs[sel.Sel.Name]
	if !ok || callee.recvType == nil || callee.recvType.name != xt.name {
		return "", typ{}, fmt.Errorf("%s: %s.%s is not translated", f.pos(e), xt.name, sel.Sel.Name)
	}
	if info.zero {
		return f.translatedCall(e, sel.Sel.Name)
	}
	args, err := f.args(e.Args)
	if err != nil {
		return "", typ{}, err
	}
	rt := typ{kind: kindOpaque, name: "void"}
	if len(callee.results) == 1 {
		rt = callee.results[0].typ
	} else if len(callee.results) > 1 {
		return "", typ{}, fmt.Errorf("%s: %s returns %d values; use it in a multi-value assignment", f.pos(e), sel.Sel.Name, len(callee.results))
	}
	return f.t.pyFunc(callee) + "(" + strings.Join(append([]string{x}, args...), ", ") + ")", rt, nil
}

// externalCall renders a call of a function or method of another gallery package.
func (f *fnState) externalCall(e *ast.CallExpr, sel *ast.SelectorExpr) (string, typ, bool, error) {
	key, recv, ok := f.externalKey(sel)
	if !ok {
		return "", typ{}, false, nil
	}
	ext, ok := f.t.external[key]
	if !ok {
		return "", typ{}, false, nil
	}
	if len(ext.Results) > 1 {
		return "", typ{}, true, fmt.Errorf("%s: %s returns %d values; use it in a multi-value assignment", f.pos(e), key, len(ext.Results))
	}
	f.t.use("External", key)
	args, err := f.args(e.Args)
	if err != nil {
		return "", typ{}, true, err
	}
	if ext.Method {
		args = append([]string{recv}, args...)
	}
	rt := typ{kind: kindOpaque, name: "void"}
	if len(ext.Results) == 1 {
		rt = parseAPIType(ext.Results[0])
	}
	return ext.Python + "(" + strings.Join(args, ", ") + ")", rt, true, nil
}

// externalKey names the callee of a selector call for the External table: "pkg.Func" for a
// package-level function and "pkg.Type.Method" for a method of an external struct. recv is the
// rendered receiver of a method.
func (f *fnState) externalKey(sel *ast.SelectorExpr) (key, recv string, ok bool) {
	if pkg, isPkg := sel.X.(*ast.Ident); isPkg {
		if _, local := f.env[pkg.Name]; !local {
			return pkg.Name + "." + sel.Sel.Name, "", true
		}
	}
	text, xt, err := f.expr(sel.X)
	if err != nil || xt.kind != kindOpaque || f.t.structs[xt.name] == nil || !f.t.structs[xt.name].external {
		return "", "", false
	}
	return xt.name + "." + sel.Sel.Name, text, true
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
	case "copy":
		args, err := f.args(e.Args)
		if err != nil || len(args) != 2 {
			return "", typ{}, fmt.Errorf("%s: copy takes two arguments", f.pos(e))
		}
		n := "min(len(" + args[0] + "), len(" + args[1] + "))"
		return args[0] + "[:" + n + "] = " + args[1] + "[:" + n + "]", intType, nil
	case "append":
		return f.appendCall(e)
	case "min", "max":
		if _, local := f.env[fn.Name]; !local {
			args, err := f.args(e.Args)
			if err != nil {
				return "", typ{}, err
			}
			_, t, err := f.expr(e.Args[0])
			if err != nil {
				return "", typ{}, err
			}
			return fn.Name + "(" + strings.Join(args, ", ") + ")", t, nil
		}
	}
	if t, ok := f.env[fn.Name]; ok && t.kind == kindOpaque && t.name == "func" {
		args, err := f.args(e.Args)
		if err != nil {
			return "", typ{}, err
		}
		rt := typ{kind: kindOpaque, name: "void"}
		if t.elem != nil {
			rt = *t.elem
		}
		return pyName(fn.Name) + "(" + strings.Join(args, ", ") + ")", rt, nil
	}
	return f.translatedCall(e, fn.Name)
}

// translatedCall renders a call of a function in the translation list. A function whose only
// result is an error has no value, and a function with a trailing error returns its other results.
func (f *fnState) translatedCall(e *ast.CallExpr, name string) (string, typ, error) {
	if f.t.provided[name] {
		f.t.use("Provided", name)
		return "None", typ{kind: kindOpaque, name: "void"}, nil
	}
	if b, ok := f.t.bound[name]; ok {
		f.t.use("Bound", name)
		args, err := f.args(e.Args)
		if err != nil {
			return "", typ{}, err
		}
		return b.Python + "(" + strings.Join(args, ", ") + ")", boundType(b.Result), nil
	}
	callee, ok := f.t.funcs[name]
	if !ok {
		return "", typ{}, fmt.Errorf("%s: call of %s, which is not translated", f.pos(e), name)
	}
	if len(callee.results) > 1 && !f.multi {
		return "", typ{}, fmt.Errorf("%s: %s returns %d values; use it in a multi-value assignment", f.pos(e), name, len(callee.results))
	}
	args, err := f.args(e.Args)
	if err != nil {
		return "", typ{}, err
	}
	rt := typ{kind: kindOpaque, name: "void"}
	if len(callee.results) == 1 {
		rt = callee.results[0].typ
	}
	return f.t.pyFunc(callee) + "(" + strings.Join(args, ", ") + ")", rt, nil
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

// make renders make([]T, n) as a list of n zero values. The capacity of make([]T, n, c) only reserves room, so
// it is dropped.
func (f *fnState) make(e *ast.CallExpr) (string, typ, error) {
	if len(e.Args) == 1 || len(e.Args) == 2 {
		if mt, err := fromAST(e.Args[0], f.t.structs); err == nil && mt.kind == kindMap {
			return "{}", mt, nil
		}
	}
	if len(e.Args) != 2 && len(e.Args) != 3 {
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
	if pkg, ok := e.X.(*ast.Ident); ok && f.t.sketch && opaquePackages[pkg.Name] {
		name := pkg.Name + "." + e.Sel.Name
		ct, ok := apiConsts[name]
		if !ok {
			return "", typ{}, fmt.Errorf("%s: %s is not in the API table", f.pos(e), name)
		}
		return f.t.runtime + "." + strings.ToUpper(pkg.Name) + "_" + strings.ToUpper(pySnake(e.Sel.Name)), parseAPIType(ct), nil
	}
	x, xt, err := f.expr(e.X)
	if err != nil {
		return "", typ{}, err
	}
	if xt.kind == kindOpaque && f.t.structs[xt.name] != nil && f.t.structs[xt.name].external {
		ft, ok := f.t.structs[xt.name].types[e.Sel.Name]
		if !ok {
			return "", typ{}, fmt.Errorf("%s: %s.%s is not listed for the external type", f.pos(e), xt.name, e.Sel.Name)
		}
		return x + "." + e.Sel.Name, ft, nil
	}
	if xt.kind == kindOpaque {
		ft, ok := apiFields[xt.name+"."+e.Sel.Name]
		if !ok {
			return "", typ{}, fmt.Errorf("%s: %s.%s is not in the API table", f.pos(e), xt.name, e.Sel.Name)
		}
		return x + "." + e.Sel.Name, parseAPIType(ft), nil
	}
	if xt.kind == kindStruct && f.t.structs[xt.name] != nil && f.t.structs[xt.name].zero {
		return "None", nilType, nil
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
	if xt.kind != kindSlice && xt.kind != kindMap {
		return "", typ{}, fmt.Errorf("%s: index of a non-slice", f.pos(e))
	}
	i, _, err := f.expr(e.Index)
	if err != nil {
		return "", typ{}, err
	}
	if xt.kind == kindMap {
		// A missing key reads as the element's zero value.
		if zero, zerr := xt.elem.zero(); zerr == nil {
			return x + ".get(" + i + ", " + zero + ")", *xt.elem, nil
		}
	}
	return x + "[" + i + "]", *xt.elem, nil
}

// composite renders a struct literal. An unkeyed literal of a translated struct is a namedtuple
// construction. A keyed literal of an API struct is a runtime call with keyword arguments, and
// a slice literal is a list.
func (f *fnState) composite(e *ast.CompositeLit) (string, typ, error) {
	switch t := e.Type.(type) {
	case *ast.MapType:
		if len(e.Elts) != 0 {
			return "", typ{}, fmt.Errorf("%s: a map literal with entries is unsupported", f.pos(e))
		}
		mt, err := fromAST(t, f.t.structs)
		if err != nil {
			return "", typ{}, fmt.Errorf("%s: %w", f.pos(e), err)
		}
		return "{}", mt, nil
	case *ast.ArrayType:
		elem, err := fromAST(t.Elt, f.t.structs)
		if err != nil {
			return "", typ{}, fmt.Errorf("%s: %w", f.pos(e), err)
		}
		// An element written {..} takes the element type of the slice.
		elts := make([]ast.Expr, len(e.Elts))
		for i, elt := range e.Elts {
			if lit, ok := elt.(*ast.CompositeLit); ok && lit.Type == nil {
				typed := *lit
				typed.Type = t.Elt
				elt = &typed
			}
			elts[i] = elt
		}
		args, err := f.valueArgs(elts)
		if err != nil {
			return "", typ{}, err
		}
		return "[" + strings.Join(args, ", ") + "]", typ{kind: kindSlice, elem: &elem}, nil
	case *ast.SelectorExpr:
		name, ok := opaqueType(t)
		if !ok || apiLiterals[name] == "" || !f.t.sketch {
			return "", typ{}, fmt.Errorf("%s: unsupported composite literal", f.pos(e))
		}
		var kw []string
		for _, elt := range e.Elts {
			kv, ok := elt.(*ast.KeyValueExpr)
			key, isIdent := ast.Expr(nil), false
			if ok {
				key, isIdent = kv.Key, true
			}
			id, idOK := key.(*ast.Ident)
			if !isIdent || !idOK {
				return "", typ{}, fmt.Errorf("%s: a %s literal needs keyed fields", f.pos(e), name)
			}
			text, _, err := f.expr(kv.Value)
			if err != nil {
				return "", typ{}, err
			}
			kw = append(kw, id.Name+"="+text)
		}
		return f.t.runtime + "." + runtimeName(name) + "(" + strings.Join(kw, ", ") + ")", typ{kind: kindOpaque, name: name}, nil
	}
	if st, ok := e.Type.(*ast.StructType); ok && len(st.Fields.List) == 0 && len(e.Elts) == 0 {
		return "True", typ{kind: kindOpaque, name: "struct{}"}, nil
	}
	if st, ok := e.Type.(*ast.StructType); ok {
		at, err := anonStruct(st, f.t.structs)
		if err != nil {
			return "", typ{}, err
		}
		args, err := f.args(e.Elts)
		if err != nil {
			return "", typ{}, err
		}
		return f.t.prefix + pyClass(at.name) + "(" + strings.Join(args, ", ") + ")", at, nil
	}
	id, ok := e.Type.(*ast.Ident)
	if !ok {
		return "", typ{}, fmt.Errorf("%s: unsupported composite literal", f.pos(e))
	}
	info, ok := f.t.structs[id.Name]
	if !ok {
		return "", typ{}, fmt.Errorf("%s: literal of untranslated type %s", f.pos(e), id.Name)
	}
	if len(e.Elts) > 0 {
		if _, keyed := e.Elts[0].(*ast.KeyValueExpr); keyed {
			var kw []string
			given := map[string]bool{}
			for _, elt := range e.Elts {
				kv, ok := elt.(*ast.KeyValueExpr)
				if !ok {
					return "", typ{}, fmt.Errorf("%s: a literal of %s mixes keyed and unkeyed fields", f.pos(e), id.Name)
				}
				key, ok := kv.Key.(*ast.Ident)
				if !ok {
					return "", typ{}, fmt.Errorf("%s: a literal of %s needs field names", f.pos(e), id.Name)
				}
				text, vt, err := f.expr(kv.Value)
				if err != nil {
					return "", typ{}, err
				}
				if text, err = f.copyValue(kv.Value, text, vt, ""); err != nil {
					return "", typ{}, err
				}
				kw = append(kw, key.Name+"="+text)
				given[key.Name] = true
			}
			// A field the literal leaves out holds the zero value of its type.
			for _, field := range info.fields {
				if !given[field] {
					kw = append(kw, field+"="+info.defaultOf(field))
				}
			}
			return f.t.prefix + pyClass(id.Name) + "(" + strings.Join(kw, ", ") + ")", typ{kind: kindStruct, name: id.Name}, nil
		}
	}
	if len(e.Elts) != len(info.fields) {
		return "", typ{}, fmt.Errorf("%s: literal of %s needs %d unkeyed fields", f.pos(e), id.Name, len(info.fields))
	}
	args, err := f.valueArgs(e.Elts)
	if err != nil {
		return "", typ{}, err
	}
	return f.t.prefix + pyClass(id.Name) + "(" + strings.Join(args, ", ") + ")", typ{kind: kindStruct, name: id.Name}, nil
}

// appendCall renders append(xs, ys...) and append(xs, y, z) as list concatenation.
func (f *fnState) appendCall(e *ast.CallExpr) (string, typ, error) {
	if len(e.Args) < 2 {
		return "", typ{}, fmt.Errorf("%s: append needs a slice and values", f.pos(e))
	}
	xs, xt, err := f.expr(e.Args[0])
	if err != nil {
		return "", typ{}, err
	}
	if xt.kind != kindSlice {
		return "", typ{}, fmt.Errorf("%s: append to a non-slice", f.pos(e))
	}
	rest := e.Args[1:]
	if e.Ellipsis.IsValid() {
		if len(rest) != 1 {
			return "", typ{}, fmt.Errorf("%s: append with a spread takes one slice", f.pos(e))
		}
		ys, _, err := f.expr(rest[0])
		if err != nil {
			return "", typ{}, err
		}
		return xs + " + list(" + ys + ")", xt, nil
	}
	vals, err := f.valueArgs(rest)
	if err != nil {
		return "", typ{}, err
	}
	return xs + " + [" + strings.Join(vals, ", ") + "]", xt, nil
}

// slice renders xs[lo:hi].
func (f *fnState) slice(e *ast.SliceExpr) (string, typ, error) {
	if e.Slice3 {
		return "", typ{}, fmt.Errorf("%s: three-index slices are unsupported", f.pos(e))
	}
	xs, xt, err := f.expr(e.X)
	if err != nil {
		return "", typ{}, err
	}
	if xt.kind != kindSlice {
		return "", typ{}, fmt.Errorf("%s: slice of a non-slice", f.pos(e))
	}
	var lo, hi string
	if e.Low != nil {
		if lo, _, err = f.expr(e.Low); err != nil {
			return "", typ{}, err
		}
	}
	if e.High != nil {
		if hi, _, err = f.expr(e.High); err != nil {
			return "", typ{}, err
		}
	}
	return xs + "[" + lo + ":" + hi + "]", xt, nil
}

// sprintf renders fmt.Sprintf as Python's % operator. A %v prints with %s, and a spread argument
// list becomes a tuple.
func (f *fnState) sprintf(e *ast.CallExpr) (string, typ, error) {
	if len(e.Args) == 0 {
		return "", typ{}, fmt.Errorf("%s: Sprintf needs a format", f.pos(e))
	}
	format, _, err := f.expr(e.Args[0])
	if err != nil {
		return "", typ{}, err
	}
	if lit, ok := e.Args[0].(*ast.BasicLit); ok {
		if err := f.checkVerbs(lit, e.Args[1:], e.Ellipsis.IsValid()); err != nil {
			return "", typ{}, err
		}
		format = strings.ReplaceAll(format, "%v", "%s")
	}
	if len(e.Args) == 1 {
		return format, stringType, nil
	}
	if e.Ellipsis.IsValid() {
		spread, _, err := f.expr(e.Args[len(e.Args)-1])
		if err != nil {
			return "", typ{}, err
		}
		return format + " % tuple(" + spread + ")", stringType, nil
	}
	args, err := f.args(e.Args[1:])
	if err != nil {
		return "", typ{}, err
	}
	return format + " % (" + strings.Join(args, ", ") + ",)", stringType, nil
}

// checkVerbs refuses a format whose verbs Python's % operator would print differently from Go's fmt. The verbs
// allowed are %d and %x of an int, %s of a string or a handle, %g and %f of a number with an optional
// precision (%.2f), %v of anything but a float (it prints as %s), and %%. One difference remains: without a precision,
// Python's %g prints six significant digits where Go's prints the fewest that identify the value, so 0.1234567 reads
// 0.123457 in Python. A spread argument list is not checked against the verbs.
func (f *fnState) checkVerbs(lit *ast.BasicLit, args []ast.Expr, spread bool) error {
	format, err := strconv.Unquote(lit.Value)
	if err != nil {
		return fmt.Errorf("%s: %w", f.pos(lit), err)
	}
	arg := 0
	for i := 0; i < len(format); i++ {
		if format[i] != '%' {
			continue
		}
		j := i + 1
		if j < len(format) && format[j] == '%' {
			i = j
			continue
		}
		precision := false
		if j < len(format) && format[j] == '.' {
			precision = true
			j++
			for j < len(format) && format[j] >= '0' && format[j] <= '9' {
				j++
			}
		}
		if j >= len(format) {
			return fmt.Errorf("%s: format %s ends inside a verb", f.pos(lit), lit.Value)
		}
		verb := format[j]
		allowed := map[byte]bool{'d': true, 'x': true, 's': true, 'v': true, 'g': true, 'f': true}[verb]
		if !allowed || (precision && verb != 'g' && verb != 'f') {
			return fmt.Errorf("%s: the verb %s in %s is unsupported: Python's %% prints it differently", f.pos(lit),
				format[i:j+1], lit.Value)
		}
		if !spread {
			if arg >= len(args) {
				return fmt.Errorf("%s: format %s has more verbs than arguments", f.pos(lit), lit.Value)
			}
			_, at, err := f.expr(args[arg])
			if err != nil {
				return err
			}
			float := at.kind == kindFloat || at.kind == kindUntypedFloat
			integer := at.kind == kindInt || at.kind == kindUntyped
			switch {
			case verb == 'v' && float:
				return fmt.Errorf("%s: %%v of a float prints 3 in Go and 3.0 in Python; use %%g", f.pos(args[arg]))
			case (verb == 'd' || verb == 'x') && !integer:
				return fmt.Errorf("%s: %%%c needs an int", f.pos(args[arg]), verb)
			case (verb == 'g' || verb == 'f') && !at.numeric():
				return fmt.Errorf("%s: %%%c needs a number", f.pos(args[arg]), verb)
			case verb == 's' && (float || integer || at.kind == kindBool):
				return fmt.Errorf("%s: %%s of a number or bool prints %%!s(...) in Go", f.pos(args[arg]))
			}
		}
		arg++
		i = j
	}
	if !spread && arg != len(args) {
		return fmt.Errorf("%s: format %s has %d verbs for %d arguments", f.pos(lit), lit.Value, arg, len(args))
	}
	return nil
}

// sortSlice renders sort.Slice(xs, func(i, j int) bool { return key(xs[i]) < key(xs[j]) }) as a sort
// by key. The comparison must be the same expression of xs[i] and of xs[j], or the port stops.
func (f *fnState) sortSlice(e *ast.CallExpr) (string, typ, error) {
	bad := fmt.Errorf("%s: sort.Slice must compare key(xs[i]) < key(xs[j]) for one key", f.pos(e))
	if len(e.Args) != 2 {
		return "", typ{}, bad
	}
	xs, ok := e.Args[0].(*ast.Ident)
	lit, ok2 := e.Args[1].(*ast.FuncLit)
	if !ok || !ok2 || len(lit.Body.List) != 1 || len(lit.Type.Params.List) != 1 || len(lit.Type.Params.List[0].Names) != 2 {
		return "", typ{}, bad
	}
	ret, ok := lit.Body.List[0].(*ast.ReturnStmt)
	if !ok || len(ret.Results) != 1 {
		return "", typ{}, bad
	}
	cmp, ok := ret.Results[0].(*ast.BinaryExpr)
	if !ok || (cmp.Op != token.LSS && cmp.Op != token.GTR) {
		return "", typ{}, bad
	}
	i, j := lit.Type.Params.List[0].Names[0].Name, lit.Type.Params.List[0].Names[1].Name
	inner := &fnState{t: f.t, info: f.info, env: map[string]typ{}, calls: map[string]int{}}
	for k, v := range f.env {
		inner.env[k] = v
	}
	inner.env[i], inner.env[j] = intType, intType
	left, _, err := inner.expr(cmp.X)
	if err != nil {
		return "", typ{}, err
	}
	right, _, err := inner.expr(cmp.Y)
	if err != nil {
		return "", typ{}, err
	}
	name := pyName(xs.Name)
	at := func(index string) string { return name + "[" + index + "]" }
	key := strings.ReplaceAll(left, at(pyName(i)), "_v")
	if strings.ReplaceAll(right, at(pyName(j)), "_v") != key || key == left {
		return "", typ{}, bad
	}
	reverse := ""
	if cmp.Op == token.GTR {
		reverse = ", reverse=True"
	}
	return name + ".sort(key=lambda _v: " + key + reverse + ")", typ{kind: kindOpaque, name: "void"}, nil
}

// boundType is the Go type of a bound function's result: nothing, a slice, a handle of an opaque package, or a
// translated struct.
func boundType(result string) typ {
	switch {
	case result == "":
		return typ{kind: kindOpaque, name: "void"}
	case strings.HasPrefix(result, "[]"):
		elem := boundType(result[2:])
		return typ{kind: kindSlice, elem: &elem}
	case strings.Contains(result, "."):
		return typ{kind: kindOpaque, name: result}
	}
	return typ{kind: kindStruct, name: result}
}
