package go2py

import (
	"fmt"
	"go/ast"
	"go/token"
	"slices"
	"strings"
)

// caughtVar is an error variable a test of errors.Is examines: its name and the position its declaration's scope
// starts at, which tells it from another variable of the same name.
type caughtVar struct {
	name string
	decl token.Pos
}

// variableAt returns the variable the name refers to at position p: the innermost declaration of it whose scope
// holds p. A name with no declaration in the function, such as a parameter, has the declaration position 0.
func (f *fnState) variableAt(name string, p token.Pos) caughtVar {
	v := caughtVar{name: name}
	if f.root == nil {
		return v
	}
	for _, s := range declarationSpans(f.root, name) {
		if s[0] <= p && p <= s[1] && s[0] > v.decl {
			v.decl = s[0]
		}
	}
	return v
}

// catchSite is a call whose error the gallery tests with errors.Is against an error value Spec.Catch lists. The
// translation wraps it in a try statement that stores the exception in the error variable.
type catchSite struct {
	errVar  string
	classes []string
}

// planCatches finds, in a function body, every errors.Is(err, X) with X in Spec.Catch, and the call it examines:
// the last assignment to err before it. It records the call as a catch site and err as a caught variable. A check
// of err != nil between the call and the errors.Is is refused, because the translation drops such a check while
// err may hold a caught error.
func (f *fnState) planCatches(body *ast.BlockStmt) error {
	if len(f.t.catch) == 0 || body == nil {
		return nil
	}
	type test struct {
		errVar, class string
		pos           token.Pos
	}
	var tests []test
	var planErr error
	ast.Inspect(body, func(n ast.Node) bool {
		if _, closure := n.(*ast.FuncLit); closure {
			// A closure plans its own catches.
			return false
		}
		call, ok := n.(*ast.CallExpr)
		if !ok || !isPkgCall(call, "errors", "Is") || len(call.Args) != 2 {
			return true
		}
		key := selectorKey(call.Args[1])
		class, listed := f.t.catch[key]
		if !listed {
			return true
		}
		id, ok := call.Args[0].(*ast.Ident)
		if !ok {
			planErr = fmt.Errorf("%s: errors.Is must test an error variable", f.pos(call))
			return false
		}
		f.t.use("Catch", key)
		tests = append(tests, test{id.Name, class, call.Pos()})
		return true
	})
	if planErr != nil {
		return planErr
	}
	for _, tt := range tests {
		var site *ast.AssignStmt
		ast.Inspect(body, func(n ast.Node) bool {
			if _, closure := n.(*ast.FuncLit); closure {
				return false
			}
			as, ok := n.(*ast.AssignStmt)
			if !ok || as.End() > tt.pos || len(as.Rhs) != 1 {
				return true
			}
			if _, isCall := as.Rhs[0].(*ast.CallExpr); !isCall {
				return true
			}
			for _, l := range as.Lhs {
				if isIdent(l, tt.errVar) && (site == nil || as.End() > site.End()) {
					site = as
				}
			}
			return true
		})
		if site == nil {
			return fmt.Errorf("%s: errors.Is(%s, ...) follows no call that sets %s", f.pos(body), tt.errVar, tt.errVar)
		}
		var dropped ast.Node
		ast.Inspect(body, func(n ast.Node) bool {
			if is, ok := n.(*ast.IfStmt); ok && is.Pos() > site.End() && is.Pos() < tt.pos && isErrCheck(is.Cond) &&
				isIdent(is.Cond.(*ast.BinaryExpr).X, tt.errVar) {
				dropped = is
			}
			return dropped == nil
		})
		if dropped != nil {
			return fmt.Errorf("%s: a check of %s != nil lies between the call errors.Is examines and the test; the "+
				"translation drops it, so a caught error would pass it", f.pos(dropped), tt.errVar)
		}
		if f.catchSites == nil {
			f.catchSites, f.caught = map[*ast.AssignStmt]*catchSite{}, map[caughtVar]struct{}{}
		}
		cs := f.catchSites[site]
		if cs == nil {
			cs = &catchSite{errVar: tt.errVar}
			f.catchSites[site] = cs
		}
		if !slices.Contains(cs.classes, tt.class) {
			cs.classes = append(cs.classes, tt.class)
		}
		f.caught[f.variableAt(tt.errVar, tt.pos)] = struct{}{}
	}
	return nil
}

// selectorKey renders pkg.Name for a selector of a package, or "" for any other expression.
func selectorKey(e ast.Expr) string {
	sel, ok := e.(*ast.SelectorExpr)
	if !ok {
		return ""
	}
	pkg, ok := sel.X.(*ast.Ident)
	if !ok {
		return ""
	}
	return pkg.Name + "." + sel.Sel.Name
}

// errorsIs renders errors.Is(err, X): an isinstance test when X is in Spec.Catch, and False otherwise, because a
// translated call raises instead of returning an error (docs/gallery-translation-goal.md, "Retry loops").
func (f *fnState) errorsIs(e *ast.CallExpr) (string, typ, error) {
	if len(e.Args) == 2 {
		if class, listed := f.t.catch[selectorKey(e.Args[1])]; listed {
			id, ok := e.Args[0].(*ast.Ident)
			if !ok {
				return "", typ{}, fmt.Errorf("%s: errors.Is must test an error variable", f.pos(e))
			}
			if _, caught := f.caught[f.variableAt(id.Name, id.Pos())]; !caught {
				return "", typ{}, fmt.Errorf("%s: errors.Is of an error the translation does not catch", f.pos(e))
			}
			return "isinstance(" + pyName(id.Name) + ", " + class + ")", boolType, nil
		}
	}
	return "False", boolType, nil
}

// catchingAssign renders an assignment from a call. At a catch site the call goes in a try statement whose
// handler stores the exception in the error variable; elsewhere a call that sets a caught variable clears it, as
// Go's call returns a nil error when it succeeds and the runtime raises when it fails. It reports false for an
// assignment that sets no caught variable.
func (f *fnState) catchingAssign(sb *strings.Builder, s *ast.AssignStmt) (bool, error) {
	errVar := ""
	for _, l := range s.Lhs {
		if id, ok := l.(*ast.Ident); ok {
			if _, caught := f.caught[f.variableAt(id.Name, id.Pos())]; caught {
				errVar = id.Name
			}
		}
	}
	if errVar == "" {
		return false, nil
	}
	if _, known := f.env[errVar]; !known {
		if err := f.define(s, errVar, nilType); err != nil {
			return true, err
		}
	}
	emit := func() error {
		if handled, err := f.apiAssign(sb, s); handled || err != nil {
			return err
		}
		return f.assign(sb, s)
	}
	site := f.catchSites[s]
	if site == nil {
		if err := emit(); err != nil {
			return true, err
		}
		f.line(sb, "%s = None", pyName(errVar))
		return true, nil
	}
	f.line(sb, "try:")
	f.indent++
	if err := emit(); err != nil {
		return true, err
	}
	f.line(sb, "%s = None", pyName(errVar))
	f.indent--
	classes := strings.Join(site.classes, ", ")
	if len(site.classes) > 1 {
		classes = "(" + classes + ")"
	}
	f.line(sb, "except %s as _caught:", classes)
	f.indent++
	f.line(sb, "%s = _caught", pyName(errVar))
	f.indent--
	return true, nil
}
