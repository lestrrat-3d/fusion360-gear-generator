package gotopy

import (
	"fmt"
	"go/ast"
	"go/token"
	"strings"
)

// block renders statements at the current indent. An empty block renders "pass".
func (f *fnState) block(sb *strings.Builder, list []ast.Stmt) error {
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
		return f.assign(sb, s)
	case *ast.ReturnStmt:
		return f.returnStmt(sb, s)
	case *ast.IfStmt:
		return f.ifStmt(sb, s, "if")
	case *ast.ForStmt:
		return f.forStmt(sb, s)
	case *ast.DeclStmt:
		return f.declStmt(sb, s)
	}
	return fmt.Errorf("%s: unsupported statement %T", f.pos(s), s)
}

func (f *fnState) declStmt(sb *strings.Builder, s *ast.DeclStmt) error {
	gd, ok := s.Decl.(*ast.GenDecl)
	if !ok || gd.Tok != token.VAR {
		return fmt.Errorf("%s: unsupported declaration", f.pos(s))
	}
	for _, spec := range gd.Specs {
		vs := spec.(*ast.ValueSpec)
		if len(vs.Values) != 0 {
			return fmt.Errorf("%s: var with a value is unsupported", f.pos(s))
		}
		vt, err := fromAST(vs.Type, f.t.structs)
		if err != nil {
			return fmt.Errorf("%s: %w", f.pos(s), err)
		}
		zero, err := vt.zero()
		if err != nil {
			return fmt.Errorf("%s: %w", f.pos(s), err)
		}
		for _, n := range vs.Names {
			f.env[n.Name] = vt
			f.line(sb, "%s = %s", pyName(n.Name), zero)
		}
	}
	return nil
}

func (f *fnState) assign(sb *strings.Builder, s *ast.AssignStmt) error {
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
			if rt.kind == kindUntyped {
				rt = intType
			}
			f.env[l.Name] = rt
		} else if _, ok := f.env[l.Name]; !ok {
			return "", fmt.Errorf("%s: assignment to unknown name %s", f.pos(l), l.Name)
		}
		return pyName(l.Name), nil
	case *ast.IndexExpr:
		text, _, err := f.expr(l)
		return text, err
	}
	return "", fmt.Errorf("%s: unsupported assignment target %T", f.pos(l), l)
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
	if !ok || !isMathCall(call, "Sincos") || len(call.Args) != 1 {
		return fmt.Errorf("%s: only math.Sincos may return two values", f.pos(s))
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
	if info.errorResult {
		last := results[len(results)-1]
		results = results[:len(results)-1]
		if !isNil(last) {
			return f.raise(sb, last)
		}
	}
	if len(results) != len(info.results) {
		return fmt.Errorf("%s: return has %d values, want %d", f.pos(s), len(results), len(info.results))
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
	call, ok := e.(*ast.CallExpr)
	if !ok || !isPkgCall(call, "fmt", "Errorf") || len(call.Args) == 0 {
		return fmt.Errorf("%s: an error result must be nil or fmt.Errorf", f.pos(e))
	}
	lit, ok := call.Args[0].(*ast.BasicLit)
	if !ok || lit.Kind != token.STRING {
		return fmt.Errorf("%s: fmt.Errorf needs a literal format", f.pos(e))
	}
	format := strings.ReplaceAll(lit.Value, "%v", "%s")
	if strings.Contains(format, "%w") {
		return fmt.Errorf("%s: %%w is unsupported", f.pos(e))
	}
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
	if s.Init != nil {
		return fmt.Errorf("%s: if with an init statement is unsupported", f.pos(s))
	}
	cond, ct, err := f.expr(s.Cond)
	if err != nil {
		return err
	}
	if ct.kind != kindBool {
		return fmt.Errorf("%s: if condition is not a bool", f.pos(s.Cond))
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
	init, ok := s.Init.(*ast.AssignStmt)
	if !ok || init.Tok != token.DEFINE || len(init.Lhs) != 1 || len(init.Rhs) != 1 {
		return fmt.Errorf("%s: for loop needs a single i := start init", f.pos(s))
	}
	id, ok := init.Lhs[0].(*ast.Ident)
	if !ok {
		return fmt.Errorf("%s: for loop variable is not a name", f.pos(s))
	}
	cond, ok := s.Cond.(*ast.BinaryExpr)
	if !ok || (cond.Op != token.LSS && cond.Op != token.LEQ) || !isIdent(cond.X, id.Name) {
		return fmt.Errorf("%s: for loop condition must be i < n or i <= n", f.pos(s))
	}
	post, ok := s.Post.(*ast.IncDecStmt)
	if !ok || post.Tok != token.INC || !isIdent(post.X, id.Name) {
		return fmt.Errorf("%s: for loop post statement must be i++", f.pos(s))
	}
	start, _, err := f.expr(init.Rhs[0])
	if err != nil {
		return err
	}
	f.env[id.Name] = intType
	limit, _, err := f.expr(cond.Y)
	if err != nil {
		return err
	}
	if cond.Op == token.LEQ {
		limit += " + 1"
	}
	f.line(sb, "for %s in range(%s, %s):", pyName(id.Name), start, limit)
	f.indent++
	defer func() { f.indent-- }()
	return f.block(sb, s.Body.List)
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
