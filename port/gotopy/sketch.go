package gotopy

import (
	"fmt"
	"go/ast"
	"go/parser"
	"go/token"
	"strings"
)

// steps renders the creation sequence of one drawing function.
func (t *transpiler) steps(st Steps) (string, error) {
	decl, err := t.model.Func(st.Name)
	if err != nil {
		return "", err
	}
	fn := &fnState{t: t, info: &funcInfo{name: st.Name, decl: decl}, env: map[string]typ{}, calls: map[string]int{}}
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
		if createsSketch(s) {
			continue
		}
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

// apiAssign renders the assignments a drawing function makes with a trailing error: the
// call is made and the error result dropped, because the Python runtime raises instead. It
// reports false when the assignment is an ordinary one.
func (f *fnState) apiAssign(sb *strings.Builder, s *ast.AssignStmt) (bool, error) {
	if len(s.Rhs) != 1 {
		return false, nil
	}
	call, ok := s.Rhs[0].(*ast.CallExpr)
	if !ok {
		return false, nil
	}
	last := len(s.Lhs) - 1
	if !isIdent(s.Lhs[last], "err") {
		if len(s.Lhs) > 1 {
			return f.multiResult(sb, s, call, 0)
		}
		return false, nil
	}
	if last >= 2 {
		return f.multiResult(sb, s, call, 1)
	}
	text, rt, err := f.expr(call)
	if err != nil {
		return true, err
	}
	switch last {
	case 0:
		if text != "None" {
			f.line(sb, "%s", text)
		}
	case 1:
		if isIdent(s.Lhs[0], "_") {
			if text != "None" {
				f.line(sb, "%s", text)
			}
			return true, nil
		}
		target, err := f.target(s.Lhs[0], s.Tok == token.DEFINE, rt)
		if err != nil {
			return true, err
		}
		f.line(sb, "%s = %s", target, text)
	}
	return true, nil
}

// multiResult renders "a, b := f(...)" for a translated function with that many results,
// not counting a trailing error.
func (f *fnState) multiResult(sb *strings.Builder, s *ast.AssignStmt, call *ast.CallExpr, errs int) (bool, error) {
	id, ok := call.Fun.(*ast.Ident)
	if !ok {
		return false, nil
	}
	callee, ok := f.t.funcs[id.Name]
	if !ok || len(callee.results) != len(s.Lhs)-errs {
		return false, nil
	}
	args, err := f.args(call.Args)
	if err != nil {
		return true, err
	}
	targets := make([]string, len(callee.results))
	for i := range callee.results {
		if targets[i], err = f.target(s.Lhs[i], s.Tok == token.DEFINE, callee.results[i].typ); err != nil {
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

// createsSketch reports whether the statement is "s, err := world.CreateSketch(plane)". A drawing
// function that is translated up to a stop call draws in a sketch its caller made.
func createsSketch(s ast.Stmt) bool {
	a, ok := s.(*ast.AssignStmt)
	if !ok || len(a.Rhs) != 1 {
		return false
	}
	call, ok := a.Rhs[0].(*ast.CallExpr)
	if !ok {
		return false
	}
	sel, ok := call.Fun.(*ast.SelectorExpr)
	return ok && sel.Sel.Name == "CreateSketch"
}
