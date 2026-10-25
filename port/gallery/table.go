package gallery

import (
	"fmt"
	"go/ast"
	"go/token"
	"strconv"
)

// table reads the parameter table entries defineParameters writes. The function
// must make its Set calls with string-literal names and expressions and no loops,
// because the reader takes the names and expressions from the source text.
func (p *parsedPackage) table() ([]Parameter, error) {
	fd, err := p.funcDecl("defineParameters")
	if err != nil {
		// A model that derives its values from another model has no table of its own.
		return nil, nil
	}
	recv, err := p.tableReceiver(fd)
	if err != nil {
		return nil, err
	}
	p.unitAliases = unitAliasesIn(fd)
	p.paramsName, p.tableLocals = paramsAndLocalsOf(fd)
	var out []Parameter
	var walkErr error
	ast.Inspect(fd.Body, func(n ast.Node) bool {
		if walkErr != nil {
			return false
		}
		switch n := n.(type) {
		case *ast.ForStmt, *ast.RangeStmt:
			walkErr = fmt.Errorf("%s: defineParameters must not loop", p.pos(n))
		case *ast.CallExpr:
			param, ok, err := p.tableCall(n, recv)
			if err != nil {
				walkErr = err
				return false
			}
			if ok {
				out = append(out, param)
			}
		}
		return walkErr == nil
	})
	if walkErr != nil {
		return nil, walkErr
	}
	if len(out) == 0 {
		return nil, fmt.Errorf("%s: defineParameters writes no parameters", p.pos(fd))
	}
	return out, nil
}

// tableReceiver returns the name of defineParameters' parameter table argument.
func (p *parsedPackage) tableReceiver(fd *ast.FuncDecl) (string, error) {
	params := fd.Type.Params.List
	if len(params) == 0 || len(params[0].Names) == 0 {
		return "", fmt.Errorf("%s: defineParameters has no table parameter", p.pos(fd))
	}
	return params[0].Names[0].Name, nil
}

// tableCall decodes one recv.SetValue, recv.SetNumber or recv.SetExpr call. It
// reports false for any other call.
func (p *parsedPackage) tableCall(call *ast.CallExpr, recv string) (Parameter, bool, error) {
	sel, ok := call.Fun.(*ast.SelectorExpr)
	if !ok {
		return Parameter{}, false, nil
	}
	if id, ok := sel.X.(*ast.Ident); !ok || id.Name != recv {
		return Parameter{}, false, nil
	}
	switch sel.Sel.Name {
	case "SetValue":
		param, err := p.tableValue(call, KindInput, "units")
		return param, true, err
	case "SetNumber":
		param, err := p.tableValue(call, KindCount, "float64")
		return param, true, err
	case "SetExpr":
		param, err := p.tableExpr(call)
		return param, true, err
	}
	return Parameter{}, false, nil
}

// tableValue decodes recv.SetValue("name", units.Unit(p.Field)) and
// recv.SetNumber("name", float64(p.Field)). wrap names the package or builtin that
// qualifies the conversion call.
func (p *parsedPackage) tableValue(call *ast.CallExpr, kind ParameterKind, wrap string) (Parameter, error) {
	if len(call.Args) != 2 {
		return Parameter{}, fmt.Errorf("%s: want 2 arguments, got %d", p.pos(call), len(call.Args))
	}
	name, err := p.stringLiteral(call.Args[0])
	if err != nil {
		return Parameter{}, err
	}
	conv, ok := call.Args[1].(*ast.CallExpr)
	if !ok || len(conv.Args) != 1 {
		return Parameter{}, fmt.Errorf("%s: parameter %s value is not a conversion of one Params field", p.pos(call), name)
	}
	param := Parameter{Name: name, Kind: kind}
	switch f := conv.Fun.(type) {
	case *ast.SelectorExpr:
		if id, ok := f.X.(*ast.Ident); !ok || id.Name != wrap {
			return Parameter{}, fmt.Errorf("%s: parameter %s conversion is not a %s call", p.pos(call), name, wrap)
		}
		param.Unit = f.Sel.Name
	case *ast.Ident:
		if unit, aliased := p.unitAliases[f.Name]; aliased && wrap == "units" {
			param.Unit = unit
		} else if f.Name != wrap {
			return Parameter{}, fmt.Errorf("%s: parameter %s conversion is not a %s call", p.pos(call), name, wrap)
		}
	default:
		return Parameter{}, fmt.Errorf("%s: parameter %s conversion is not a call by name", p.pos(call), name)
	}
	if id, isIdent := conv.Args[0].(*ast.Ident); isIdent {
		value, err := p.packageConstant(id.Name)
		if err != nil {
			return Parameter{}, fmt.Errorf("%s: parameter %s: %w", p.pos(call), name, err)
		}
		param.Kind = KindConstant
		param.Field = id.Name
		param.Value = value
		return param, nil
	}
	field, ok := conv.Args[0].(*ast.SelectorExpr)
	if !ok {
		return Parameter{}, fmt.Errorf("%s: parameter %s does not read a Params field", p.pos(call), name)
	}
	base, ok := field.X.(*ast.Ident)
	switch {
	case !ok:
		return Parameter{}, fmt.Errorf("%s: parameter %s reads a field of a field, which the reader does not follow",
			p.pos(call), name)
	case base.Name == p.paramsName:
		param.Field = field.Sel.Name
		return param, nil
	}
	if _, local := p.tableLocals[base.Name]; !local {
		return Parameter{}, fmt.Errorf("%s: parameter %s reads %s.%s, which is neither the Params argument %q nor a "+
			"value defineParameters computes", p.pos(call), name, base.Name, field.Sel.Name, p.paramsName)
	}
	// A value computed before the table is written, not a field of Params.
	param.Kind = KindSnapshot
	param.Field = base.Name + "." + field.Sel.Name
	return param, nil
}

// paramsAndLocalsOf returns the name of the Params argument of defineParameters and the names its body declares
// with :=, which hold values computed before the table is written.
func paramsAndLocalsOf(fd *ast.FuncDecl) (string, map[string]struct{}) {
	params := ""
	for _, f := range fd.Type.Params.List {
		if id, ok := f.Type.(*ast.Ident); ok && id.Name == "Params" && len(f.Names) == 1 {
			params = f.Names[0].Name
		}
	}
	locals := map[string]struct{}{}
	ast.Inspect(fd.Body, func(n ast.Node) bool {
		if as, ok := n.(*ast.AssignStmt); ok && as.Tok == token.DEFINE {
			for _, l := range as.Lhs {
				if id, ok := l.(*ast.Ident); ok {
					locals[id.Name] = struct{}{}
				}
			}
		}
		return true
	})
	return params, locals
}

// unitAliasesIn finds assignments such as mm := units.Millimeters in a function body.
func unitAliasesIn(fd *ast.FuncDecl) map[string]string {
	out := map[string]string{}
	ast.Inspect(fd.Body, func(n ast.Node) bool {
		as, ok := n.(*ast.AssignStmt)
		if !ok || as.Tok != token.DEFINE || len(as.Lhs) != 1 || len(as.Rhs) != 1 {
			return true
		}
		name, ok := as.Lhs[0].(*ast.Ident)
		sel, ok2 := as.Rhs[0].(*ast.SelectorExpr)
		if !ok || !ok2 {
			return true
		}
		if pkg, ok := sel.X.(*ast.Ident); ok && pkg.Name == "units" {
			out[name.Name] = sel.Sel.Name
		}
		return true
	})
	return out
}

// tableExpr decodes recv.SetExpr("name", "expression", units.Unit).
func (p *parsedPackage) tableExpr(call *ast.CallExpr) (Parameter, error) {
	if len(call.Args) != 3 {
		return Parameter{}, fmt.Errorf("%s: want 3 arguments, got %d", p.pos(call), len(call.Args))
	}
	name, err := p.stringLiteral(call.Args[0])
	if err != nil {
		return Parameter{}, err
	}
	expr, err := p.stringLiteral(call.Args[1])
	if err != nil {
		return Parameter{}, err
	}
	unit, ok := call.Args[2].(*ast.SelectorExpr)
	if !ok {
		return Parameter{}, fmt.Errorf("%s: parameter %s unit is not a units constant", p.pos(call), name)
	}
	return Parameter{Name: name, Kind: KindExpr, Unit: unit.Sel.Name, Expr: expr}, nil
}

// stringLiteral returns the value of a string literal and rejects any other expression.
func (p *parsedPackage) stringLiteral(e ast.Expr) (string, error) {
	lit, ok := e.(*ast.BasicLit)
	if !ok || lit.Kind != token.STRING {
		text, _ := p.exprText(e)
		return "", fmt.Errorf("%s: %q is not a string literal", p.pos(e), text)
	}
	return strconv.Unquote(lit.Value)
}

// packageConstant evaluates the numeric constant the package declares as name.
func (p *parsedPackage) packageConstant(name string) (float64, error) {
	for _, f := range p.files {
		for _, d := range f.Decls {
			gd, ok := d.(*ast.GenDecl)
			if !ok || gd.Tok != token.CONST {
				continue
			}
			for _, spec := range gd.Specs {
				vs := spec.(*ast.ValueSpec)
				for i, id := range vs.Names {
					if id.Name == name && i < len(vs.Values) {
						return p.constant(vs.Values[i])
					}
				}
			}
		}
	}
	return 0, fmt.Errorf("%s is not a constant of the package", name)
}
