package go2py

import (
	"fmt"
	"go/ast"
	"go/token"
	"strings"
)

// rootIdent returns the variable an lvalue such as x, x.f, x[i].f or *x starts from, or nil for an expression
// that makes a new value, such as a call or a literal.
func rootIdent(e ast.Expr) *ast.Ident {
	for {
		switch x := e.(type) {
		case *ast.Ident:
			return x
		case *ast.SelectorExpr:
			e = x.X
		case *ast.IndexExpr:
			e = x.X
		case *ast.StarExpr:
			e = x.X
		case *ast.ParenExpr:
			e = x.X
		default:
			return nil
		}
	}
}

// changedThrough reports whether the function changes a value through the variable name after the position at: it
// assigns a field or an element under it (name.f = v, name[i].f++), or takes the address of one, later in the source
// or anywhere in a loop around at. Rebinding the name itself does not count.
func (f *fnState) changedThrough(name string, at token.Pos) bool {
	if f.root == nil {
		return true
	}
	var loops [][2]token.Pos
	ast.Inspect(f.root, func(n ast.Node) bool {
		switch n.(type) {
		case *ast.ForStmt, *ast.RangeStmt:
			if n.Pos() <= at && at <= n.End() {
				loops = append(loops, [2]token.Pos{n.Pos(), n.End()})
			}
		}
		return true
	})
	later := func(p token.Pos) bool {
		if p > at {
			return true
		}
		for _, l := range loops {
			if l[0] <= p && p <= l[1] {
				return true
			}
		}
		return false
	}
	through := func(l ast.Expr) bool {
		if _, plain := l.(*ast.Ident); plain {
			return false
		}
		id := rootIdent(l)
		return id != nil && id.Name == name && later(l.Pos())
	}
	found := false
	ast.Inspect(f.root, func(n ast.Node) bool {
		switch n := n.(type) {
		case *ast.AssignStmt:
			for _, l := range n.Lhs {
				found = found || through(l)
			}
		case *ast.IncDecStmt:
			found = found || through(n.X)
		case *ast.UnaryExpr:
			found = found || (n.Op == token.AND && through(n.X))
		}
		return !found
	})
	return found
}

// copyValue renders the Go copy of a struct value read from src, whose text is text, into a variable that dest
// names (empty when the copy has no name). A struct the translation fills field by field is an object in Python, so
// a copy of one read from a variable is a real copy when either side is changed through later: the destination,
// or the variable the source starts from. A value that src makes anew, such as a call result, needs no copy.
func (f *fnState) copyValue(src ast.Expr, text string, t typ, dest string) (string, error) {
	if t.kind != kindStruct {
		return text, nil
	}
	info := f.t.structs[t.name]
	from := rootIdent(src)
	if info == nil || from == nil || isNil(from) || (!info.assigned && !info.filled) {
		return text, nil
	}
	if !f.changedThrough(from.Name, src.End()) && (dest == "" || !f.changedThrough(dest, src.End())) {
		return text, nil
	}
	if info.filled {
		return "", fmt.Errorf("%s: %s copies a %s, which Spec.Filled says is never copied; list it in Spec.Mutable",
			f.pos(src), f.info.name, t.name)
	}
	return text + "._copy()", nil
}

// valueArgs renders values a composite literal or an append stores, each copied as copyValue requires. A context is
// left out, as f.args does.
func (f *fnState) valueArgs(list []ast.Expr) ([]string, error) {
	out := make([]string, 0, len(list))
	for _, a := range list {
		text, at, err := f.expr(a)
		if err != nil {
			return nil, err
		}
		if at.kind == kindCtx {
			continue
		}
		if text, err = f.copyValue(a, text, at, ""); err != nil {
			return nil, err
		}
		out = append(out, text)
	}
	return out, nil
}

// copyParams renders, at the top of a function, a copy of each struct parameter the function changes through: the
// caller's value is a copy in Go, so the change must not reach the caller's object in Python.
func (f *fnState) copyParams(sb *strings.Builder, params []namedType) error {
	for _, p := range params {
		if p.typ.kind != kindStruct || p.typ.variadic {
			continue
		}
		info := f.t.structs[p.typ.name]
		if info == nil || (!info.assigned && !info.filled) || !f.changedThrough(p.name, token.NoPos) {
			continue
		}
		if info.filled {
			return fmt.Errorf("%s changes its parameter %s, a %s, which Spec.Filled says is never copied; list it in "+
				"Spec.Mutable", f.info.name, p.name, p.typ.name)
		}
		f.line(sb, "%s = %s._copy()", pyName(p.name), pyName(p.name))
	}
	return nil
}
