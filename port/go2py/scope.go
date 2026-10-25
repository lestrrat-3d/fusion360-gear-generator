package go2py

import (
	"fmt"
	"go/ast"
	"go/token"
)

// scope is a Go block open at the statement being translated: its source span and the names it declares, each
// with the position of its declaration.
type scope struct {
	names    map[string]token.Pos
	pos, end token.Pos
}

// push opens a Go block that spans pos to end.
func (f *fnState) push(pos, end token.Pos) {
	f.scopes = append(f.scopes, scope{names: map[string]token.Pos{}, pos: pos, end: end})
}

// pop closes the innermost Go block.
func (f *fnState) pop() { f.scopes = f.scopes[:len(f.scopes)-1] }

// define declares name with type t in the innermost Go block. An untyped constant takes its default type.
//
// Python has one scope per function, so a Go declaration that hides a name of an enclosing block assigns the outer
// variable in Python. That is harmless while the outer variable is not read again, and wrong when it is: after the
// inner block ends, or in a later turn of a loop around the inner block. define refuses the second case. A name
// the innermost block already declares is the same variable, as in a multi-value := that declares another name.
func (f *fnState) define(at ast.Node, name string, t typ) error {
	switch t.kind {
	case kindUntyped:
		t = intType
	case kindUntypedFloat:
		t = floatType
	}
	if name == "_" {
		return nil
	}
	// A new variable of the name is not known to stay at zero or above.
	delete(f.counters, name)
	if n := len(f.scopes); n > 0 {
		inner := f.scopes[n-1]
		if _, same := inner.names[name]; !same {
			for _, outer := range f.scopes[:n-1] {
				declared, ok := outer.names[name]
				if ok && f.readsHidden(name, declared, outer, inner) {
					return fmt.Errorf("%s: %s hides %s of an enclosing block, which is read again where Python, with "+
						"one scope per function, would see the inner value; rename one of them", f.pos(at), name, name)
				}
			}
			inner.names[name] = at.Pos()
		}
	}
	f.env[name] = t
	return nil
}

// readsHidden reports whether the variable name, declared at declared in block outer, is read where a declaration
// of the same name in block inner would have overwritten it in Python: after inner ends but inside outer, or inside
// a loop that lies in outer and holds inner, outside inner. A read that another, later declaration of the name
// covers reads that one instead and does not count.
func (f *fnState) readsHidden(name string, declared token.Pos, outer, inner scope) bool {
	if f.root == nil {
		return true
	}
	spans := declarationSpans(f.root, name)
	covered := func(p token.Pos) bool {
		for _, s := range spans {
			if s[0] > declared && s[0] <= p && p <= s[1] {
				return true
			}
		}
		return false
	}
	var loops [][2]token.Pos
	ast.Inspect(f.root, func(n ast.Node) bool {
		var body *ast.BlockStmt
		switch n := n.(type) {
		case *ast.ForStmt:
			body = n.Body
		case *ast.RangeStmt:
			body = n.Body
		}
		if body != nil && n.Pos() > outer.pos && n.End() <= outer.end && n.Pos() <= inner.pos && inner.end <= n.End() {
			loops = append(loops, [2]token.Pos{body.Pos(), body.End()})
		}
		return true
	})
	found := false
	inspectReads(f.root, func(id *ast.Ident) {
		p := id.Pos()
		if found || id.Name != name || covered(p) || (inner.pos <= p && p <= inner.end) {
			return
		}
		if inner.end < p && p <= outer.end {
			found = true
			return
		}
		for _, l := range loops {
			if l[0] <= p && p <= l[1] {
				found = true
				return
			}
		}
	})
	return found
}

// inspectReads calls fn for every identifier under root that can name a variable: not a field or method name after
// a dot, and not a key of a composite literal.
func inspectReads(root ast.Node, fn func(*ast.Ident)) {
	ast.Inspect(root, func(n ast.Node) bool {
		switch n := n.(type) {
		case *ast.SelectorExpr:
			ast.Inspect(n.X, func(m ast.Node) bool {
				if id, ok := m.(*ast.Ident); ok {
					fn(id)
				}
				return true
			})
			return false
		case *ast.KeyValueExpr:
			if _, isIdent := n.Key.(*ast.Ident); !isIdent {
				inspectReads(n.Key, fn)
			}
			inspectReads(n.Value, fn)
			return false
		case *ast.Ident:
			fn(n)
		}
		return true
	})
}

// declarationSpans lists the spans of source under root in which a declaration of name is the one the name refers
// to: from the declaration to the end of the block that holds it.
func declarationSpans(root ast.Node, name string) [][2]token.Pos {
	var out [][2]token.Pos
	declares := func(s ast.Stmt) bool {
		switch s := s.(type) {
		case *ast.AssignStmt:
			if s.Tok != token.DEFINE {
				return false
			}
			for _, l := range s.Lhs {
				if isIdent(l, name) {
					return true
				}
			}
		case *ast.DeclStmt:
			gd, ok := s.Decl.(*ast.GenDecl)
			if !ok {
				return false
			}
			for _, spec := range gd.Specs {
				if vs, ok := spec.(*ast.ValueSpec); ok {
					for _, n := range vs.Names {
						if n.Name == name {
							return true
						}
					}
				}
			}
		}
		return false
	}
	list := func(stmts []ast.Stmt, end token.Pos) {
		for _, s := range stmts {
			if declares(s) {
				out = append(out, [2]token.Pos{s.Pos(), end})
			}
		}
	}
	ast.Inspect(root, func(n ast.Node) bool {
		switch n := n.(type) {
		case *ast.BlockStmt:
			list(n.List, n.End())
		case *ast.CaseClause:
			list(n.Body, n.End())
		case *ast.IfStmt:
			if n.Init != nil && declares(n.Init) {
				out = append(out, [2]token.Pos{n.Pos(), n.End()})
			}
		case *ast.ForStmt:
			if n.Init != nil && declares(n.Init) {
				out = append(out, [2]token.Pos{n.Pos(), n.End()})
			}
		case *ast.RangeStmt:
			if n.Tok == token.DEFINE && (isIdent(n.Key, name) || isIdent(n.Value, name)) {
				out = append(out, [2]token.Pos{n.Pos(), n.End()})
			}
		case *ast.FuncLit:
			for _, fld := range n.Type.Params.List {
				for _, id := range fld.Names {
					if id.Name == name {
						out = append(out, [2]token.Pos{n.Pos(), n.End()})
					}
				}
			}
		}
		return true
	})
	return out
}
