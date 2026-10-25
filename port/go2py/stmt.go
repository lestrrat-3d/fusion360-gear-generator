package go2py

import (
	"errors"
	"fmt"
	"go/ast"
	"go/token"
	"strings"
)

// block renders the statements of a nested Go block at the current indent. An empty block renders "pass".
func (f *fnState) block(sb *strings.Builder, list []ast.Stmt) error {
	var pos, end token.Pos
	if len(list) > 0 {
		pos, end = list[0].Pos(), list[len(list)-1].End()
	}
	f.push(pos, end)
	defer f.pop()
	return f.stmts(sb, list)
}

// breakKind is what an unlabelled break inside a statement leaves.
type breakKind int

const (
	breakLoop breakKind = iota
	breakSwitch
)

// loopBody renders the body of a loop, where a break leaves the loop.
func (f *fnState) loopBody(sb *strings.Builder, list []ast.Stmt) error {
	f.breaks = append(f.breaks, breakLoop)
	defer func() { f.breaks = f.breaks[:len(f.breaks)-1] }()
	return f.block(sb, list)
}

// stmts renders statements in the current Go block at the current indent. An empty list renders "pass".
func (f *fnState) stmts(sb *strings.Builder, list []ast.Stmt) error {
	before := sb.Len()
	for _, s := range list {
		if err := f.stmt(sb, s); err != nil {
			return err
		}
	}
	if sb.Len() == before {
		f.line(sb, "pass")
	}
	return nil
}

func (f *fnState) stmt(sb *strings.Builder, s ast.Stmt) error {
	switch s := s.(type) {
	case *ast.AssignStmt:
		if handled, err := f.catchingAssign(sb, s); handled || err != nil {
			return err
		}
		if f.t.sketch {
			if handled, err := f.apiAssign(sb, s); handled || err != nil {
				return err
			}
		}
		return f.assign(sb, s)
	case *ast.ExprStmt:
		text, _, err := f.expr(s.X)
		if err != nil {
			return err
		}
		if text != "None" {
			f.line(sb, "%s", text)
		}
		return nil
	case *ast.RangeStmt:
		return f.rangeStmt(sb, s)
	case *ast.SwitchStmt:
		return f.switchStmt(sb, s)
	case *ast.ReturnStmt:
		return f.returnStmt(sb, s)
	case *ast.IfStmt:
		return f.ifStmt(sb, s, "if")
	case *ast.ForStmt:
		return f.forStmt(sb, s)
	case *ast.DeclStmt:
		return f.declStmt(sb, s)
	case *ast.IncDecStmt:
		text, _, err := f.expr(s.X)
		if err != nil {
			return err
		}
		op := "+="
		if s.Tok == token.DEC {
			op = "-="
		}
		if ix, ok := s.X.(*ast.IndexExpr); ok {
			if _, xt, err := f.expr(ix.X); err == nil && xt.kind == kindMap {
				// A missing key reads as zero and the store writes the key: m[k] = m.get(k, 0) + 1.
				target, err := f.target(s.X, false, typ{})
				if err != nil {
					return err
				}
				f.line(sb, "%s = %s %s 1", target, text, op[:1])
				return nil
			}
		}
		f.line(sb, "%s %s 1", text, op)
		return nil
	case *ast.BranchStmt:
		if s.Label != nil {
			return fmt.Errorf("%s: a labelled %s is unsupported", f.pos(s), s.Tok)
		}
		switch s.Tok {
		case token.BREAK:
			if n := len(f.breaks); n > 0 && f.breaks[n-1] == breakSwitch {
				return fmt.Errorf("%s: break in a switch leaves the switch in Go and the loop around it in Python; "+
					"restructure the case", f.pos(s))
			}
			f.line(sb, "break")
			return nil
		case token.CONTINUE:
			f.line(sb, "continue")
			return nil
		}
	}
	return fmt.Errorf("%s: unsupported statement %T", f.pos(s), s)
}

func (f *fnState) declStmt(sb *strings.Builder, s *ast.DeclStmt) error {
	gd, ok := s.Decl.(*ast.GenDecl)
	if ok && gd.Tok == token.CONST {
		for _, spec := range gd.Specs {
			vs := spec.(*ast.ValueSpec)
			for i, n := range vs.Names {
				if i >= len(vs.Values) {
					return fmt.Errorf("%s: a constant without a value is unsupported", f.pos(s))
				}
				text, ct, err := f.expr(vs.Values[i])
				if err != nil {
					return err
				}
				if err := f.define(n, n.Name, ct); err != nil {
					return err
				}
				f.line(sb, "%s = %s", pyName(n.Name), text)
			}
		}
		return nil
	}
	if ok && gd.Tok == token.TYPE {
		// A struct type declared inside a function is rendered with the other classes.
		for _, spec := range gd.Specs {
			ts := spec.(*ast.TypeSpec)
			st, isStruct := ts.Type.(*ast.StructType)
			if !isStruct {
				return fmt.Errorf("%s: only a struct type may be declared in a function", f.pos(s))
			}
			info := &structInfo{name: ts.Name.Name, types: map[string]typ{}, anonymous: true}
			for _, fld := range st.Fields.List {
				ft, err := fromAST(fld.Type, f.t.structs)
				if err != nil {
					return fmt.Errorf("%s: %w", f.pos(s), err)
				}
				for _, n := range fld.Names {
					info.fields = append(info.fields, n.Name)
					info.types[n.Name] = ft
				}
			}
			f.t.structs[info.name] = info
		}
		return nil
	}
	if !ok || gd.Tok != token.VAR {
		return fmt.Errorf("%s: unsupported declaration", f.pos(s))
	}
	for _, spec := range gd.Specs {
		vs := spec.(*ast.ValueSpec)
		if len(vs.Values) != 0 {
			return fmt.Errorf("%s: var with a value is unsupported", f.pos(s))
		}
		if isIdent(vs.Type, "error") {
			// The runtime raises instead of returning an error. A variable that holds the last one stays None.
			for _, n := range vs.Names {
				if err := f.define(n, n.Name, nilType); err != nil {
					return err
				}
				f.line(sb, "%s = None", pyName(n.Name))
			}
			continue
		}
		vt, err := fromAST(vs.Type, f.t.structs)
		if err != nil {
			return fmt.Errorf("%s: %w", f.pos(s), err)
		}
		zero, err := vt.zero()
		if vt.kind == kindSlice {
			zero, err = vt.emptyList()
		}
		if vt.kind == kindStruct && f.t.structs[vt.name].mutable() {
			zero, err = f.t.prefix+pyClass(vt.name)+"("+f.t.structs[vt.name].arrayFields()+")", nil
		}
		if err != nil {
			return fmt.Errorf("%s: %w", f.pos(s), err)
		}
		for _, n := range vs.Names {
			if err := f.define(n, n.Name, vt); err != nil {
				return err
			}
			f.line(sb, "%s = %s", pyName(n.Name), zero)
		}
	}
	return nil
}

func (f *fnState) assign(sb *strings.Builder, s *ast.AssignStmt) error {
	if len(s.Lhs) == 1 && len(s.Rhs) == 1 && s.Tok == token.DEFINE {
		if lit, ok := s.Rhs[0].(*ast.FuncLit); ok {
			if id, ok := s.Lhs[0].(*ast.Ident); ok {
				return f.closure(sb, id.Name, lit)
			}
		}
	}
	switch s.Tok {
	case token.DEFINE, token.ASSIGN:
	case token.ADD_ASSIGN, token.SUB_ASSIGN, token.MUL_ASSIGN, token.QUO_ASSIGN:
		return f.opAssign(sb, s)
	default:
		return fmt.Errorf("%s: unsupported assignment %s", f.pos(s), s.Tok)
	}
	if len(s.Lhs) == 2 && len(s.Rhs) == 1 {
		return f.sincos(sb, s)
	}
	if len(s.Lhs) != len(s.Rhs) {
		return fmt.Errorf("%s: unsupported multi-value assignment", f.pos(s))
	}
	// Evaluate every right-hand side before any assignment, as Go does.
	lhs := make([]string, len(s.Lhs))
	rhs := make([]string, len(s.Rhs))
	types := make([]typ, len(s.Rhs))
	for i, r := range s.Rhs {
		text, rt, err := f.expr(r)
		if err != nil {
			return err
		}
		// A struct is a value in Go: copying it from a name must not share the object the copy is changed in.
		if _, named := r.(*ast.Ident); named && s.Tok == token.DEFINE && rt.kind == kindStruct &&
			f.t.structs[rt.name] != nil && f.t.structs[rt.name].assigned {
			text += "._copy()"
		} else {
			dest := ""
			if id := rootIdent(s.Lhs[i]); id != nil {
				dest = id.Name
			}
			if text, err = f.copyValue(r, text, rt, dest); err != nil {
				return err
			}
		}
		rhs[i], types[i] = text, rt
	}
	for i, l := range s.Lhs {
		text, err := f.target(l, s.Tok == token.DEFINE, types[i])
		if err != nil {
			return err
		}
		lhs[i] = text
	}
	f.line(sb, "%s = %s", strings.Join(lhs, ", "), strings.Join(rhs, ", "))
	return nil
}

// target renders an assignment target, recording the type of a newly defined name.
func (f *fnState) target(l ast.Expr, define bool, rt typ) (string, error) {
	switch l := l.(type) {
	case *ast.Ident:
		if l.Name == "_" {
			return "_", nil
		}
		if define {
			if err := f.define(l, l.Name, rt); err != nil {
				return "", err
			}
		} else if _, ok := f.env[l.Name]; !ok {
			return "", fmt.Errorf("%s: assignment to unknown name %s", f.pos(l), l.Name)
		}
		return pyName(l.Name), nil
	case *ast.IndexExpr:
		// A store writes the key itself, where a read of a map takes the zero value for a missing one.
		text, _, err := f.expr(l)
		if i := strings.LastIndex(text, ".get("); err == nil && i >= 0 && strings.HasSuffix(text, ")") {
			x, _, xerr := f.expr(l.X)
			key, _, kerr := f.expr(l.Index)
			if xerr != nil || kerr != nil {
				return "", errors.Join(xerr, kerr)
			}
			return x + "[" + key + "]", nil
		}
		return text, err
	case *ast.SelectorExpr:
		if err := f.checkFieldWrite(l); err != nil {
			return "", err
		}
		text, _, err := f.expr(l)
		return text, err
	case *ast.StarExpr:
		text, _, err := f.expr(l)
		return text, err
	}
	return "", fmt.Errorf("%s: unsupported assignment target %T", f.pos(l), l)
}

// checkFieldWrite refuses a write to a field of a translated struct that the Spec does not let the code fill field
// by field. A namedtuple's fields cannot be written in Python at all, and a struct listed in neither Spec.Mutable nor
// Spec.Filled would share one object between the copies Go makes.
func (f *fnState) checkFieldWrite(l *ast.SelectorExpr) error {
	_, xt, err := f.expr(l.X)
	if err != nil || xt.kind != kindStruct {
		return nil
	}
	info := f.t.structs[xt.name]
	if info == nil || info.external || info.assigned || info.filled {
		return nil
	}
	if !info.mutable() {
		return fmt.Errorf("%s: %s writes a field of %s, which is a namedtuple in Python; list %s in Spec.Mutable",
			f.pos(l), f.info.name, xt.name, xt.name)
	}
	return fmt.Errorf("%s: %s writes a field of %s; list %s in Spec.Mutable, or in Spec.Filled when it is never copied",
		f.pos(l), f.info.name, xt.name, xt.name)
}

func (f *fnState) opAssign(sb *strings.Builder, s *ast.AssignStmt) error {
	if len(s.Lhs) != 1 || len(s.Rhs) != 1 {
		return fmt.Errorf("%s: unsupported operator assignment", f.pos(s))
	}
	op := strings.TrimSuffix(s.Tok.String(), "=")
	bin := &ast.BinaryExpr{X: s.Lhs[0], Op: map[string]token.Token{
		"+": token.ADD, "-": token.SUB, "*": token.MUL, "/": token.QUO,
	}[op], Y: s.Rhs[0], OpPos: s.TokPos}
	text, _, err := f.expr(bin)
	if err != nil {
		return err
	}
	target, err := f.target(s.Lhs[0], false, typ{})
	if err != nil {
		return err
	}
	f.line(sb, "%s = %s", target, text)
	return nil
}

// sincos renders "s, c := math.Sincos(x)", the one two-value call the subset allows.
func (f *fnState) sincos(sb *strings.Builder, s *ast.AssignStmt) error {
	call, ok := s.Rhs[0].(*ast.CallExpr)
	if ok && isMathCall(call, "Frexp") && len(call.Args) == 1 {
		arg, _, err := f.expr(call.Args[0])
		if err != nil {
			return err
		}
		mantissa, err := f.target(s.Lhs[0], s.Tok == token.DEFINE, floatType)
		if err != nil {
			return err
		}
		exponent, err := f.target(s.Lhs[1], s.Tok == token.DEFINE, intType)
		if err != nil {
			return err
		}
		f.line(sb, "%s, %s = math.frexp(%s)", mantissa, exponent, arg)
		return nil
	}
	if !ok || !isMathCall(call, "Sincos") || len(call.Args) != 1 {
		return fmt.Errorf("%s: only math.Sincos and math.Frexp may return two values", f.pos(s))
	}
	arg, _, err := f.expr(call.Args[0])
	if err != nil {
		return err
	}
	sin, err := f.target(s.Lhs[0], s.Tok == token.DEFINE, floatType)
	if err != nil {
		return err
	}
	cos, err := f.target(s.Lhs[1], s.Tok == token.DEFINE, floatType)
	if err != nil {
		return err
	}
	f.line(sb, "%s, %s = math.sin(%s), math.cos(%s)", sin, cos, arg, arg)
	return nil
}

func (f *fnState) returnStmt(sb *strings.Builder, s *ast.ReturnStmt) error {
	info := f.info
	if len(s.Results) == 0 {
		if !info.named {
			return fmt.Errorf("%s: bare return without named results", f.pos(s))
		}
		names := make([]string, len(info.results))
		for i, r := range info.results {
			names[i] = pyName(r.name)
		}
		f.line(sb, "return %s", strings.Join(names, ", "))
		return nil
	}
	results := s.Results
	if len(results) == 1 && info.errorResult && len(info.results) >= 1 {
		if call, ok := results[0].(*ast.CallExpr); ok {
			f.multi = true
			text, _, err := f.expr(call)
			f.multi = false
			if err != nil {
				return err
			}
			f.line(sb, "return %s", text)
			return nil
		}
	}
	if info.errorResult {
		last := results[len(results)-1]
		results = results[:len(results)-1]
		// A stored error value (return nil, err) is never set in a translated function, which raises
		// instead, so only the other results are returned.
		if !isNil(last) && (!f.t.sketch || !isErrIdent(last)) {
			return f.raise(sb, last)
		}
	}
	if len(results) != len(info.results) {
		return fmt.Errorf("%s: return has %d values, want %d", f.pos(s), len(results), len(info.results))
	}
	if len(results) == 0 {
		f.line(sb, "return")
		return nil
	}
	parts := make([]string, len(results))
	for i, r := range results {
		text, _, err := f.expr(r)
		if err != nil {
			return err
		}
		parts[i] = text
	}
	f.line(sb, "return %s", strings.Join(parts, ", "))
	return nil
}

// raise renders "return ..., fmt.Errorf(format, args...)" as a RuntimeError.
func (f *fnState) raise(sb *strings.Builder, e ast.Expr) error {
	if id, isName := e.(*ast.Ident); isName {
		if t, local := f.env[id.Name]; local && t.kind == kindNil {
			// A local variable holding an earlier error: nothing sets it in a translated function.
			f.line(sb, "raise RuntimeError(str(%s))", pyName(id.Name))
			return nil
		}
		// A package-level error value, var errX = fmt.Errorf("..."), raises its own message.
		if value, err := f.t.model.PackageValue(id.Name); err == nil {
			e = value
		}
	}
	call, ok := e.(*ast.CallExpr)
	if !ok || !isPkgCall(call, "fmt", "Errorf") || len(call.Args) == 0 {
		return fmt.Errorf("%s: an error result must be nil or fmt.Errorf", f.pos(e))
	}
	lit, ok := call.Args[0].(*ast.BasicLit)
	if !ok || lit.Kind != token.STRING {
		return fmt.Errorf("%s: fmt.Errorf needs a literal format", f.pos(e))
	}
	if err := f.checkVerbs(lit, call.Args[1:], call.Ellipsis.IsValid()); err != nil {
		return err
	}
	format := strings.ReplaceAll(lit.Value, "%v", "%s")
	args := make([]string, 0, len(call.Args)-1)
	for _, a := range call.Args[1:] {
		text, _, err := f.expr(a)
		if err != nil {
			return err
		}
		args = append(args, text)
	}
	if len(args) == 0 {
		f.line(sb, "raise RuntimeError(%s)", format)
		return nil
	}
	f.line(sb, "raise RuntimeError(%s %% (%s,))", format, strings.Join(args, ", "))
	return nil
}

func (f *fnState) ifStmt(sb *strings.Builder, s *ast.IfStmt, word string) error {
	// The init statement declares names in a block around the whole if.
	f.push(s.Pos(), s.End())
	defer f.pop()
	if f.t.sketch && s.Else == nil && isErrCheck(s.Cond) {
		// The check guards a call whose error the drawing runtime raises itself.
		if s.Init != nil {
			if err := f.stmt(sb, s.Init); err != nil {
				return err
			}
		}
		return nil
	}
	if s.Init != nil {
		if !f.t.sketch {
			return fmt.Errorf("%s: if with an init statement is unsupported", f.pos(s))
		}
		if err := f.stmt(sb, s.Init); err != nil {
			return err
		}
	}
	cond, ct, err := f.expr(s.Cond)
	if err != nil {
		return err
	}
	if ct.kind != kindBool {
		return fmt.Errorf("%s: if condition is not a bool", f.pos(s.Cond))
	}
	if cond == "None is not None" || cond == "False" {
		// A field of a zero-valued struct is never set, so this branch never runs.
		switch e := s.Else.(type) {
		case nil:
			return nil
		case *ast.IfStmt:
			return f.ifStmt(sb, e, word)
		case *ast.BlockStmt:
			return f.block(sb, e.List)
		}
		return fmt.Errorf("%s: unsupported else", f.pos(s))
	}
	f.line(sb, "%s %s:", word, cond)
	f.indent++
	if err := f.block(sb, s.Body.List); err != nil {
		return err
	}
	f.indent--
	switch e := s.Else.(type) {
	case nil:
		return nil
	case *ast.IfStmt:
		return f.ifStmt(sb, e, "elif")
	case *ast.BlockStmt:
		f.line(sb, "else:")
		f.indent++
		defer func() { f.indent-- }()
		return f.block(sb, e.List)
	}
	return fmt.Errorf("%s: unsupported else", f.pos(s))
}

// forStmt renders "for i := a; i < b; i++" as a range loop.
func (f *fnState) forStmt(sb *strings.Builder, s *ast.ForStmt) error {
	// The loop variable is declared in a block around the loop.
	f.push(s.Pos(), s.End())
	defer f.pop()
	if s.Init == nil && s.Post == nil && s.Cond != nil {
		cond, ct, err := f.expr(s.Cond)
		if err != nil {
			return err
		}
		if ct.kind != kindBool {
			return fmt.Errorf("%s: loop condition is not a bool", f.pos(s.Cond))
		}
		f.line(sb, "while %s:", cond)
		f.indent++
		defer func() { f.indent-- }()
		return f.loopBody(sb, s.Body.List)
	}
	init, ok := s.Init.(*ast.AssignStmt)
	if !ok || init.Tok != token.DEFINE || len(init.Lhs) != 1 || len(init.Rhs) != 1 {
		return fmt.Errorf("%s: for loop needs a single i := start init", f.pos(s))
	}
	id, ok := init.Lhs[0].(*ast.Ident)
	if !ok {
		return fmt.Errorf("%s: for loop variable is not a name", f.pos(s))
	}
	if s.Cond == nil {
		return f.openLoop(sb, s, init, id)
	}
	cond, ok := s.Cond.(*ast.BinaryExpr)
	if !ok || (cond.Op != token.LSS && cond.Op != token.LEQ) || !isIdent(cond.X, id.Name) {
		return fmt.Errorf("%s: for loop condition must be i < n or i <= n", f.pos(s))
	}
	step := ""
	var stepExpr ast.Expr
	switch post := s.Post.(type) {
	case *ast.IncDecStmt:
		if post.Tok != token.INC || !isIdent(post.X, id.Name) {
			return fmt.Errorf("%s: for loop post statement must be i++ or i += n", f.pos(s))
		}
	case *ast.AssignStmt:
		if post.Tok != token.ADD_ASSIGN || len(post.Lhs) != 1 || !isIdent(post.Lhs[0], id.Name) {
			return fmt.Errorf("%s: for loop post statement must be i++ or i += n", f.pos(s))
		}
		var err error
		var stepType typ
		if step, stepType, err = f.expr(post.Rhs[0]); err != nil {
			return err
		}
		if stepType.kind == kindFloat || stepType.kind == kindUntypedFloat {
			return fmt.Errorf("%s: a for loop with a floating-point step is unsupported; count with an int", f.pos(s))
		}
		stepExpr = post.Rhs[0]
	default:
		return fmt.Errorf("%s: for loop post statement must be i++ or i += n", f.pos(s))
	}
	start, startType, err := f.expr(init.Rhs[0])
	if err != nil {
		return err
	}
	if startType.kind == kindFloat || startType.kind == kindUntypedFloat {
		return fmt.Errorf("%s: a for loop over the floating-point variable %s is unsupported; count with an int",
			f.pos(s), id.Name)
	}
	// Python's range takes its bounds once; Go reads the condition and the step on every turn.
	if err := f.fixedBounds(s, id.Name, cond.Y, stepExpr); err != nil {
		return err
	}
	if err := f.define(id, id.Name, intType); err != nil {
		return err
	}
	if nonNegativeConstant(init.Rhs[0]) && (stepExpr == nil || nonNegativeConstant(stepExpr)) {
		f.nonNegative(id.Name)
	}
	limit, _, err := f.expr(cond.Y)
	if err != nil {
		return err
	}
	if cond.Op == token.LEQ {
		limit += " + 1"
	}
	if step != "" {
		f.line(sb, "for %s in range(%s, %s, %s):", pyName(id.Name), start, limit, step)
	} else {
		f.line(sb, "for %s in range(%s, %s):", pyName(id.Name), start, limit)
	}
	f.indent++
	defer func() { f.indent-- }()
	return f.loopBody(sb, s.Body.List)
}

// fixedBounds refuses a counted loop whose bound or step can change while it runs, which a Python range would not
// see: the body assigns the loop variable or a name the bound or the step reads, or they call something other than
// len, a conversion or a math function.
func (f *fnState) fixedBounds(s *ast.ForStmt, loopVar string, bound, step ast.Expr) error {
	assigned := map[string]struct{}{}
	ast.Inspect(s.Body, func(n ast.Node) bool {
		switch n := n.(type) {
		case *ast.AssignStmt:
			for _, l := range n.Lhs {
				if id, ok := l.(*ast.Ident); ok {
					assigned[id.Name] = struct{}{}
				}
			}
		case *ast.IncDecStmt:
			if id, ok := n.X.(*ast.Ident); ok {
				assigned[id.Name] = struct{}{}
			}
		}
		return true
	})
	if _, ok := assigned[loopVar]; ok {
		return fmt.Errorf("%s: the body of a counted loop assigns its variable %s; Python's range would not see it",
			f.pos(s), loopVar)
	}
	for _, e := range []ast.Expr{bound, step} {
		if e == nil {
			continue
		}
		var bad error
		ast.Inspect(e, func(n ast.Node) bool {
			switch n := n.(type) {
			case *ast.Ident:
				if _, ok := assigned[n.Name]; ok && bad == nil {
					bad = fmt.Errorf("%s: the body of a counted loop changes %s, which its bound or step reads; "+
						"Python's range reads them once", f.pos(s), n.Name)
				}
			case *ast.CallExpr:
				switch fn := n.Fun.(type) {
				case *ast.Ident:
					if fn.Name == "len" || fn.Name == "int" || fn.Name == "float64" || fn.Name == "min" || fn.Name == "max" {
						return true
					}
				case *ast.SelectorExpr:
					if isIdent(fn.X, "math") {
						return true
					}
				}
				if bad == nil {
					bad = fmt.Errorf("%s: the bound or step of a counted loop calls a function, which Go calls on every "+
						"turn and Python's range once; compute it before the loop", f.pos(s))
				}
			}
			return true
		})
		if bad != nil {
			return bad
		}
	}
	return nil
}

func isIdent(e ast.Expr, name string) bool {
	id, ok := e.(*ast.Ident)
	return ok && id.Name == name
}

func isNil(e ast.Expr) bool { return isIdent(e, "nil") }

func isMathCall(c *ast.CallExpr, fn string) bool { return isPkgCall(c, "math", fn) }

func isPkgCall(c *ast.CallExpr, pkg, fn string) bool {
	sel, ok := c.Fun.(*ast.SelectorExpr)
	return ok && isIdent(sel.X, pkg) && sel.Sel.Name == fn
}

// openLoop renders "for i := start; ; i++": a loop with no bound, which the body leaves with break.
func (f *fnState) openLoop(sb *strings.Builder, s *ast.ForStmt, init *ast.AssignStmt, id *ast.Ident) error {
	post, ok := s.Post.(*ast.IncDecStmt)
	if !ok || post.Tok != token.INC || !isIdent(post.X, id.Name) {
		return fmt.Errorf("%s: an unbounded for loop post statement must be i++", f.pos(s))
	}
	continues := false
	ast.Inspect(s.Body, func(n ast.Node) bool {
		if b, ok := n.(*ast.BranchStmt); ok && b.Tok == token.CONTINUE {
			continues = true
		}
		return !continues
	})
	if continues {
		return fmt.Errorf("%s: an unbounded for loop cannot continue", f.pos(s))
	}
	start, st, err := f.expr(init.Rhs[0])
	if err != nil {
		return err
	}
	if err := f.define(id, id.Name, st); err != nil {
		return err
	}
	f.line(sb, "%s = %s", pyName(id.Name), start)
	f.line(sb, "while True:")
	f.indent++
	defer func() { f.indent-- }()
	if err := f.loopBody(sb, s.Body.List); err != nil {
		return err
	}
	f.line(sb, "%s += 1", pyName(id.Name))
	return nil
}
