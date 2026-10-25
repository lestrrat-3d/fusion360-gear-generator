package go2py

import (
	"fmt"
	"go/ast"
	"go/token"
	"slices"
	"strings"
)

// isErrIdent reports whether e names an error variable: err, or a name ending in Err such as phaseErr.
func isErrIdent(e ast.Expr) bool {
	id, ok := e.(*ast.Ident)
	return ok && (id.Name == "err" || strings.HasSuffix(id.Name, "Err"))
}

// isErrCheck reports whether e is err != nil.
func isErrCheck(e ast.Expr) bool {
	b, ok := e.(*ast.BinaryExpr)
	return ok && b.Op == token.NEQ && isErrIdent(b.X) && isNil(b.Y)
}

// apiAssign renders the assignments a drawing function makes with a trailing error: the
// call is made and the error result dropped, because the Python runtime raises instead. It
// reports false when the assignment is an ordinary one.
func (f *fnState) apiAssign(sb *strings.Builder, s *ast.AssignStmt) (bool, error) {
	if len(s.Rhs) != 1 {
		return false, nil
	}
	if len(s.Lhs) == 2 {
		if handled, err := f.commaOK(sb, s); handled || err != nil {
			return handled, err
		}
	}
	call, ok := s.Rhs[0].(*ast.CallExpr)
	if !ok {
		return false, nil
	}
	last := len(s.Lhs) - 1
	if !isErrIdent(s.Lhs[last]) {
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
	var (
		callee string
		recv   string
		types  []typ
	)
	switch fn := call.Fun.(type) {
	case *ast.Ident:
		info, ok := f.t.funcs[fn.Name]
		if !ok || len(info.results) != len(s.Lhs)-errs {
			return false, nil
		}
		callee = f.t.pyFunc(info)
		for _, r := range info.results {
			types = append(types, r.typ)
		}
	case *ast.SelectorExpr:
		if x, xt, err := f.expr(fn.X); err == nil && xt.kind == kindStruct && f.t.structs[xt.name] != nil {
			info, ok := f.t.funcs[fn.Sel.Name]
			if ok && info.recvType != nil && info.recvType.name == xt.name && len(info.results) == len(s.Lhs)-errs {
				callee, recv = f.t.pyFunc(info), x
				for _, r := range info.results {
					types = append(types, r.typ)
				}
				break
			}
		}
		if x, xt, err := f.expr(fn.X); err == nil && xt.kind == kindOpaque {
			if sig, found := apiMethods[xt.name+"."+fn.Sel.Name]; found && len(sig.results) == len(s.Lhs)-errs {
				for _, r := range sig.results {
					types = append(types, parseAPIType(r))
				}
				callee, recv = x+"."+pyKeyword(pySnake(fn.Sel.Name)), ""
				break
			}
		}
		key, r, ok := f.externalKey(fn)
		if !ok {
			return false, nil
		}
		ext, ok := f.t.external[key]
		if !ok || len(ext.Results) != len(s.Lhs)-errs {
			return false, nil
		}
		f.t.use("External", key)
		callee, recv = ext.Python, r
		for _, r := range ext.Results {
			types = append(types, parseAPIType(r))
		}
		if !ext.Method {
			recv = ""
		}
	default:
		return false, nil
	}
	args, err := f.args(call.Args)
	if err != nil {
		return true, err
	}
	if recv != "" {
		args = append([]string{recv}, args...)
	}
	targets := make([]string, len(types))
	for i := range types {
		if targets[i], err = f.target(s.Lhs[i], s.Tok == token.DEFINE, types[i]); err != nil {
			return true, err
		}
	}
	f.line(sb, "%s = %s(%s)", strings.Join(targets, ", "), callee, strings.Join(args, ", "))
	return true, nil
}

// rangeStmt renders the range loops a gallery model writes: "i, v := range xs" as an enumerate
// loop, "i := range xs" as a loop over the indexes, "i := range n" and "range n" as a counted loop.
// A slice listed in Spec.FirstOnly is walked once, with its first entry.
func (f *fnState) rangeStmt(sb *strings.Builder, s *ast.RangeStmt) error {
	key, _ := s.Key.(*ast.Ident)
	val, _ := s.Value.(*ast.Ident)
	if (s.Key != nil && key == nil) || (s.Value != nil && val == nil) || (s.Key != nil && s.Tok != token.DEFINE) {
		return fmt.Errorf("%s: range needs \"i, v :=\"", f.pos(s))
	}
	xs, xt, err := f.expr(s.X)
	if err != nil {
		return err
	}
	// The key and the value are declared in a block around the loop.
	f.push(s.Pos(), s.End())
	defer f.pop()
	keyName := "_"
	if key != nil && key.Name != "_" {
		keyName = pyName(key.Name)
		if err := f.define(key, key.Name, intType); err != nil {
			return err
		}
		f.nonNegative(key.Name)
	}
	switch {
	case xt.numeric():
		if val != nil {
			return fmt.Errorf("%s: range over a number has no value", f.pos(s))
		}
		f.line(sb, "for %s in range(%s):", keyName, xs)
	case xt.kind == kindSlice:
		if id, ok := s.X.(*ast.Ident); ok && f.t.firstOnly[id.Name] {
			f.t.use("FirstOnly", id.Name)
			xs += "[:1]"
		} else if val != nil && f.t.firstOnly[f.info.name+":"+val.Name] {
			// A list written in the loop itself is named by the function and the loop variable.
			f.t.use("FirstOnly", f.info.name+":"+val.Name)
			xs += "[:1]"
		}
		if val != nil && val.Name != "_" {
			if err := f.define(val, val.Name, *xt.elem); err != nil {
				return err
			}
			if keyName == "_" {
				f.line(sb, "for %s in %s:", pyName(val.Name), xs)
			} else {
				f.line(sb, "for %s, %s in enumerate(%s):", keyName, pyName(val.Name), xs)
			}
			// The value is a copy of the element in Go.
			elem := &ast.IndexExpr{X: s.X, Lbrack: s.X.End(), Index: &ast.BasicLit{Kind: token.INT, Value: "0"},
				Rbrack: s.X.End()}
			copied, err := f.copyValue(elem, pyName(val.Name), *xt.elem, val.Name)
			if err != nil {
				return err
			}
			if copied != pyName(val.Name) {
				f.indent++
				f.line(sb, "%s = %s", pyName(val.Name), copied)
				f.indent--
			}
		} else {
			f.line(sb, "for %s in range(len(%s)):", keyName, xs)
		}
	default:
		return fmt.Errorf("%s: range over a value that is neither a slice nor a number", f.pos(s))
	}
	f.indent++
	defer func() { f.indent-- }()
	return f.loopBody(sb, s.Body.List)
}

// switchStmt renders a switch as if, elif and else. A switch with no tag tests each case expression,
// and a tagged switch tests the tag against each.
func (f *fnState) switchStmt(sb *strings.Builder, s *ast.SwitchStmt) error {
	f.push(s.Pos(), s.End())
	defer f.pop()
	if s.Init != nil {
		return fmt.Errorf("%s: switch with an init statement is unsupported", f.pos(s))
	}
	tag := ""
	if s.Tag != nil {
		var err error
		if tag, _, err = f.expr(s.Tag); err != nil {
			return err
		}
	}
	word := "if"
	var def *ast.CaseClause
	for _, c := range s.Body.List {
		clause := c.(*ast.CaseClause)
		if clause.List == nil {
			def = clause
			continue
		}
		conds := make([]string, len(clause.List))
		for i, e := range clause.List {
			text, _, err := f.expr(e)
			if err != nil {
				return err
			}
			if tag != "" {
				text = tag + " == " + text
			}
			conds[i] = text
		}
		f.line(sb, "%s %s:", word, strings.Join(conds, " or "))
		word = "elif"
		f.indent++
		if err := f.caseBody(sb, clause.Body); err != nil {
			return err
		}
		f.indent--
	}
	if def != nil {
		if word == "if" {
			return f.caseBody(sb, def.Body)
		}
		f.line(sb, "else:")
		f.indent++
		defer func() { f.indent-- }()
		return f.caseBody(sb, def.Body)
	}
	return nil
}

// caseBody renders the body of a switch case, where a break would leave the switch.
func (f *fnState) caseBody(sb *strings.Builder, list []ast.Stmt) error {
	f.breaks = append(f.breaks, breakSwitch)
	defer func() { f.breaks = f.breaks[:len(f.breaks)-1] }()
	return f.block(sb, list)
}

// closure renders "name := func(params) result { ... }" as a nested def. The body sees the names of
// the enclosing function, and an assignment to one of them declares it nonlocal.
func (f *fnState) closure(sb *strings.Builder, name string, lit *ast.FuncLit) error {
	inner := &fnState{t: f.t, env: map[string]typ{}, calls: map[string]int{}, indent: f.indent + 1,
		info: &funcInfo{name: name}}
	for k, v := range f.env {
		inner.env[k] = v
	}
	// The closure is declared where it is written, and its parameters and body are a block inside the blocks open
	// there.
	if err := f.define(lit, name, typ{kind: kindOpaque, name: "func"}); err != nil {
		return err
	}
	inner.root = f.root
	inner.scopes = slices.Clone(f.scopes)
	inner.push(lit.Pos(), lit.End())
	if err := inner.planCatches(lit.Body); err != nil {
		return err
	}
	var params []string
	local := map[string]bool{}
	for _, fld := range lit.Type.Params.List {
		ft, err := fromAST(fld.Type, f.t.structs)
		if err != nil {
			return fmt.Errorf("%s: %w", f.pos(lit), err)
		}
		for _, n := range fld.Names {
			if err := inner.define(n, n.Name, ft); err != nil {
				return err
			}
			local[n.Name] = true
			if ft.variadic {
				params = append(params, "*"+pyName(n.Name))
			} else {
				params = append(params, pyName(n.Name))
			}
		}
	}
	result := typ{kind: kindOpaque, name: "void"}
	if lit.Type.Results != nil {
		for _, r := range lit.Type.Results.List {
			if isIdent(r.Type, "error") {
				// The runtime raises instead of returning an error.
				inner.info.errorResult = true
				continue
			}
			rt, err := fromAST(r.Type, f.t.structs)
			if err != nil {
				return fmt.Errorf("%s: %w", f.pos(lit), err)
			}
			inner.info.results = append(inner.info.results, namedType{"", rt})
			result = rt
		}
	}
	var outer []string
	declared := map[string]bool{}
	ast.Inspect(lit.Body, func(n ast.Node) bool {
		if inc, ok := n.(*ast.IncDecStmt); ok {
			// n++ assigns n, as n = n + 1 does.
			n = &ast.AssignStmt{Lhs: []ast.Expr{inc.X}, Tok: token.ADD_ASSIGN}
		}
		a, ok := n.(*ast.AssignStmt)
		if !ok {
			return true
		}
		for _, l := range a.Lhs {
			id, ok := l.(*ast.Ident)
			if !ok || id.Name == "_" {
				continue
			}
			if a.Tok == token.DEFINE {
				declared[id.Name] = true
				continue
			}
			if _, isOuter := f.env[id.Name]; isOuter && !local[id.Name] && !declared[id.Name] && !slices.Contains(outer, id.Name) {
				outer = append(outer, id.Name)
			}
		}
		return true
	})
	f.env[name] = typ{kind: kindOpaque, name: "func", elem: &result}
	inner.env[name] = f.env[name]
	f.line(sb, "def %s(%s):", pyName(name), strings.Join(params, ", "))
	if len(outer) > 0 {
		sb.WriteString(strings.Repeat("    ", inner.indent) + "nonlocal " + strings.Join(outer, ", ") + "\n")
	}
	return inner.stmts(sb, lit.Body.List)
}

// commaOK renders "v, ok := x.(T)" and "v, ok := m[k]". A type assertion is a test of the
// value by a runtime function named after the type, and a map lookup is a membership test.
func (f *fnState) commaOK(sb *strings.Builder, s *ast.AssignStmt) (bool, error) {
	okName, isIdent := s.Lhs[1].(*ast.Ident)
	if !isIdent {
		return false, nil
	}
	switch r := s.Rhs[0].(type) {
	case *ast.TypeAssertExpr:
		sel, ok := r.Type.(*ast.SelectorExpr)
		if !ok {
			return false, nil
		}
		name, ok := opaqueType(sel)
		if !ok || !apiTypeTests[name] {
			return true, fmt.Errorf("%s: a type assertion to %s is not in the API table", f.pos(s), name)
		}
		x, _, err := f.expr(r.X)
		if err != nil {
			return true, err
		}
		test := f.t.runtime + "." + typeTestName(name)
		value := x
		if !isIdentNamed(s.Lhs[0], "_") {
			target, err := f.target(s.Lhs[0], s.Tok == token.DEFINE, typ{kind: kindOpaque, name: name})
			if err != nil {
				return true, err
			}
			f.line(sb, "%s = %s", target, x)
			value = target
		}
		okTarget, err := f.target(okName, s.Tok == token.DEFINE, boolType)
		if err != nil {
			return true, err
		}
		f.line(sb, "%s = %s(%s)", okTarget, test, value)
		return true, nil
	case *ast.IndexExpr:
		m, mt, err := f.expr(r.X)
		if err != nil || mt.kind != kindMap {
			return false, nil
		}
		k, _, err := f.expr(r.Index)
		if err != nil {
			return true, err
		}
		if !isIdentNamed(s.Lhs[0], "_") {
			target, err := f.target(s.Lhs[0], s.Tok == token.DEFINE, *mt.elem)
			if err != nil {
				return true, err
			}
			f.line(sb, "%s = %s.get(%s)", target, m, k)
		}
		okTarget, err := f.target(okName, s.Tok == token.DEFINE, boolType)
		if err != nil {
			return true, err
		}
		f.line(sb, "%s = %s in %s", okTarget, k, m)
		return true, nil
	}
	return false, nil
}

func isIdentNamed(e ast.Expr, name string) bool { return isIdent(e, name) }
