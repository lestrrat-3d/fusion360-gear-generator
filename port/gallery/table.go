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
		return nil, err
	}
	recv, err := p.tableReceiver(fd)
	if err != nil {
		return nil, err
	}
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
		if f.Name != wrap {
			return Parameter{}, fmt.Errorf("%s: parameter %s conversion is not a %s call", p.pos(call), name, wrap)
		}
	default:
		return Parameter{}, fmt.Errorf("%s: parameter %s conversion is not a call by name", p.pos(call), name)
	}
	field, ok := conv.Args[0].(*ast.SelectorExpr)
	if !ok {
		return Parameter{}, fmt.Errorf("%s: parameter %s does not read a Params field", p.pos(call), name)
	}
	param.Field = field.Sel.Name
	return param, nil
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
