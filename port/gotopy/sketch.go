package gotopy

import (
	"fmt"
	"go/ast"
	"go/parser"
	"go/token"
	"strings"
)

// sketchMethods are the sketch.Sketch methods a drawing function may call and the
// result each has. A call renders as a method call with the snake_case name on the
// sketch value, so the Python runtime provides one method per entry.
var sketchMethods = map[string]typ{
	"CreateLine":      {kind: kindOpaque, name: "void"},
	"CreateArc":       {kind: kindOpaque, name: "void"},
	"CreateFitSpline": {kind: kindOpaque, name: "void"},
	"Origin":          {kind: kindOpaque, name: "sketch.Point"},
}

// checkHelpers are the sketchcheck functions a drawing function may call. Each renders as
// a method call on its first argument, the sketch.
var checkHelpers = map[string]typ{
	"DimensionPoint":  {kind: kindOpaque, name: "sketch.Point"},
	"DimensionArcEnd": {kind: kindOpaque, name: "sketch.Point"},
}

// steps renders the creation sequence of one drawing function.
func (t *transpiler) steps(st Steps) (string, error) {
	decl, err := t.model.Func(st.Name)
	if err != nil {
		return "", err
	}
	fn := &fnState{t: t, info: &funcInfo{name: st.Name, decl: decl}, env: map[string]typ{}}
	names := make([]string, len(st.Args))
	for i, a := range st.Args {
		expr, err := parser.ParseExpr(a.Type)
		if err != nil {
			return "", fmt.Errorf("argument %s: %w", a.Name, err)
		}
		at, err := fromAST(expr, t.structs)
		if err != nil {
			return "", fmt.Errorf("argument %s: %w", a.Name, err)
		}
		fn.env[a.Name] = at
		names[i] = pyName(a.Name)
	}
	var sb strings.Builder
	fmt.Fprintf(&sb, "def %s%s(%s):\n", t.prefix, st.PyName, strings.Join(names, ", "))
	fn.indent = 1
	stopped := false
	for _, s := range decl.Body.List {
		if callsFunc(s, st.StopAt) {
			stopped = true
			break
		}
		if err := fn.stmt(&sb, s); err != nil {
			return "", err
		}
	}
	if !stopped {
		return "", fmt.Errorf("no call of %s to stop at", st.StopAt)
	}
	return strings.TrimRight(sb.String(), "\n"), nil
}

// callsFunc reports whether the statement calls the function name.
func callsFunc(s ast.Stmt, name string) bool {
	found := false
	ast.Inspect(s, func(n ast.Node) bool {
		if c, ok := n.(*ast.CallExpr); ok {
			if id, ok := c.Fun.(*ast.Ident); ok && id.Name == name {
				found = true
			}
		}
		return !found
	})
	return found
}

// isErrCheck reports whether e is err != nil.
func isErrCheck(e ast.Expr) bool {
	b, ok := e.(*ast.BinaryExpr)
	return ok && b.Op == token.NEQ && isIdent(b.X, "err") && isNil(b.Y)
}

// sketchAssign renders the assignments only a drawing function makes. It reports false
// when the assignment is an ordinary one.
func (f *fnState) sketchAssign(sb *strings.Builder, s *ast.AssignStmt) (bool, error) {
	if len(s.Rhs) != 1 {
		return false, nil
	}
	call, ok := s.Rhs[0].(*ast.CallExpr)
	if !ok {
		return false, nil
	}
	if len(s.Lhs) == 2 && isIdent(s.Lhs[1], "err") {
		sel, ok := call.Fun.(*ast.SelectorExpr)
		if !ok {
			return false, nil
		}
		switch {
		case sel.Sel.Name == "CreateSketch":
			return true, nil
		case isIdent(s.Lhs[0], "_"):
			text, _, err := f.expr(call)
			if err != nil {
				return true, err
			}
			f.line(sb, "%s", text)
			return true, nil
		}
		return false, fmt.Errorf("%s: unsupported two-value assignment", f.pos(s))
	}
	if len(s.Lhs) > 1 {
		return f.multiResult(sb, s, call)
	}
	return false, nil
}

// multiResult renders "a, b := f(...)" for a translated function with that many results.
func (f *fnState) multiResult(sb *strings.Builder, s *ast.AssignStmt, call *ast.CallExpr) (bool, error) {
	id, ok := call.Fun.(*ast.Ident)
	if !ok {
		return false, nil
	}
	callee, ok := f.t.funcs[id.Name]
	if !ok || len(callee.results) != len(s.Lhs) {
		return false, nil
	}
	args, err := f.args(call.Args)
	if err != nil {
		return true, err
	}
	targets := make([]string, len(s.Lhs))
	for i, l := range s.Lhs {
		if targets[i], err = f.target(l, s.Tok == token.DEFINE, callee.results[i].typ); err != nil {
			return true, err
		}
	}
	f.line(sb, "%s = %s%s(%s)", strings.Join(targets, ", "), f.t.prefix, pySnake(id.Name), strings.Join(args, ", "))
	return true, nil
}

// rangeStmt renders "for i, v := range xs" as an enumerate loop.
func (f *fnState) rangeStmt(sb *strings.Builder, s *ast.RangeStmt) error {
	key, kok := s.Key.(*ast.Ident)
	val, vok := s.Value.(*ast.Ident)
	if !kok || !vok || s.Tok != token.DEFINE {
		return fmt.Errorf("%s: range needs \"i, v :=\"", f.pos(s))
	}
	xs, xt, err := f.expr(s.X)
	if err != nil {
		return err
	}
	if xt.kind != kindSlice {
		return fmt.Errorf("%s: range over a non-slice", f.pos(s))
	}
	f.env[key.Name], f.env[val.Name] = intType, *xt.elem
	switch {
	case key.Name == "_":
		f.line(sb, "for %s in %s:", pyName(val.Name), xs)
	default:
		f.line(sb, "for %s, %s in enumerate(%s):", pyName(key.Name), pyName(val.Name), xs)
	}
	f.indent++
	defer func() { f.indent-- }()
	return f.block(sb, s.Body.List)
}

// sketchCall renders a call to a sketchcheck helper or a sketch method. It reports false
// when the call is neither.
func (f *fnState) sketchCall(e *ast.CallExpr, sel *ast.SelectorExpr) (string, typ, bool, error) {
	if !f.t.sketch {
		return "", typ{}, false, nil
	}
	var (
		recv string
		args []ast.Expr
		res  typ
	)
	switch pkg, _ := sel.X.(*ast.Ident); {
	case pkg != nil && pkg.Name == "sketchcheck":
		r, ok := checkHelpers[sel.Sel.Name]
		if !ok || len(e.Args) == 0 {
			return "", typ{}, true, fmt.Errorf("%s: sketchcheck.%s is not translated", f.pos(e), sel.Sel.Name)
		}
		res, args = r, e.Args
		text, _, err := f.expr(e.Args[0])
		if err != nil {
			return "", typ{}, true, err
		}
		recv, args = text, e.Args[1:]
	default:
		r, ok := sketchMethods[sel.Sel.Name]
		if !ok {
			return "", typ{}, false, nil
		}
		text, xt, err := f.expr(sel.X)
		if err != nil {
			return "", typ{}, true, err
		}
		if xt.kind != kindOpaque || xt.name != "sketch.Sketch" {
			return "", typ{}, true, fmt.Errorf("%s: %s on a value that is not a sketch", f.pos(e), sel.Sel.Name)
		}
		res, recv, args = r, text, e.Args
	}
	parts := make([]string, len(args))
	for i, a := range args {
		text, _, err := f.expr(a)
		if err != nil {
			return "", typ{}, true, err
		}
		if e.Ellipsis.IsValid() && i == len(args)-1 {
			text = "*" + text
		}
		parts[i] = text
	}
	return fmt.Sprintf("%s.%s(%s)", recv, pySnake(sel.Sel.Name), strings.Join(parts, ", ")), res, true, nil
}
