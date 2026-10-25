package gallery

import (
	"fmt"
	"go/ast"
	"go/printer"
	"go/token"
	"math"
	"strconv"
	"strings"
)

// inputs reads the Params struct fields and the defaults DefaultParams assigns.
func (p *parsedPackage) inputs() ([]Input, error) {
	ts, err := p.typeSpec("Params")
	if err != nil {
		return nil, err
	}
	if ts.Assign.IsValid() {
		// A model that aliases another model's Params adds no inputs of its own.
		return nil, nil
	}
	st, ok := ts.Type.(*ast.StructType)
	if !ok {
		return nil, fmt.Errorf("%s: Params is not a struct", p.pos(ts))
	}
	var in []Input
	for _, field := range st.Fields.List {
		typ, err := p.exprText(field.Type)
		if err != nil {
			return nil, err
		}
		for _, name := range field.Names {
			in = append(in, Input{Name: name.Name, Type: typ})
		}
	}
	defaults, texts, err := p.defaults()
	if err != nil {
		return nil, err
	}
	for i := range in {
		if v, ok := defaults[in[i].Name]; ok {
			in[i].Default = v
			in[i].HasDefault = true
		}
		in[i].DefaultText = texts[in[i].Name]
	}
	return in, nil
}

// defaults evaluates the keyed fields of the Params literal DefaultParams returns.
func (p *parsedPackage) defaults() (map[string]float64, map[string]string, error) {
	fd, err := p.funcDecl("DefaultParams")
	if err != nil {
		return nil, nil, err
	}
	var lit *ast.CompositeLit
	ast.Inspect(fd.Body, func(n ast.Node) bool {
		if r, ok := n.(*ast.ReturnStmt); ok && len(r.Results) == 1 && lit == nil {
			lit, _ = r.Results[0].(*ast.CompositeLit)
		}
		return lit == nil
	})
	if lit == nil {
		return nil, nil, fmt.Errorf("%s: DefaultParams does not return a composite literal", p.pos(fd))
	}
	out := make(map[string]float64, len(lit.Elts))
	texts := map[string]string{}
	for _, e := range lit.Elts {
		kv, ok := e.(*ast.KeyValueExpr)
		if !ok {
			return nil, nil, fmt.Errorf("%s: DefaultParams element is not keyed", p.pos(e))
		}
		key, ok := kv.Key.(*ast.Ident)
		if !ok {
			return nil, nil, fmt.Errorf("%s: DefaultParams key is not a field name", p.pos(kv.Key))
		}
		if _, embedded := kv.Value.(*ast.CallExpr); embedded && key.Name == "Params" {
			// A model that embeds another model's Params starts from that model's
			// defaults; only its own fields are read here.
			continue
		}
		if text, ok := p.textConstant(kv.Value); ok {
			texts[key.Name] = text
			continue
		}
		v, err := p.constant(kv.Value)
		if err != nil {
			return nil, nil, fmt.Errorf("default of %s: %w", key.Name, err)
		}
		out[key.Name] = v
	}
	return out, texts, nil
}

// textConstant reads a string literal or a true or false identifier.
func (p *parsedPackage) textConstant(e ast.Expr) (string, bool) {
	switch e := e.(type) {
	case *ast.BasicLit:
		if e.Kind == token.STRING {
			if text, err := strconv.Unquote(e.Value); err == nil {
				return text, true
			}
		}
	case *ast.Ident:
		if e.Name == "true" || e.Name == "false" {
			return e.Name, true
		}
	}
	return "", false
}

// constant evaluates a numeric constant expression made of literals, math.Pi,
// parentheses, unary minus and the four arithmetic operators.
func (p *parsedPackage) constant(e ast.Expr) (float64, error) {
	switch e := e.(type) {
	case *ast.BasicLit:
		if e.Kind != token.INT && e.Kind != token.FLOAT {
			return 0, fmt.Errorf("%s: %s is not a number", p.pos(e), e.Value)
		}
		return strconv.ParseFloat(e.Value, 64)
	case *ast.ParenExpr:
		return p.constant(e.X)
	case *ast.UnaryExpr:
		if e.Op != token.SUB {
			return 0, fmt.Errorf("%s: unsupported unary operator %s", p.pos(e), e.Op)
		}
		v, err := p.constant(e.X)
		return -v, err
	case *ast.SelectorExpr:
		if id, ok := e.X.(*ast.Ident); ok && id.Name == "math" && e.Sel.Name == "Pi" {
			return math.Pi, nil
		}
	case *ast.BinaryExpr:
		return p.constantBinary(e)
	}
	text, _ := p.exprText(e)
	return 0, fmt.Errorf("%s: %q is not a supported constant", p.pos(e), text)
}

func (p *parsedPackage) constantBinary(e *ast.BinaryExpr) (float64, error) {
	l, err := p.constant(e.X)
	if err != nil {
		return 0, err
	}
	r, err := p.constant(e.Y)
	if err != nil {
		return 0, err
	}
	switch e.Op {
	case token.ADD:
		return l + r, nil
	case token.SUB:
		return l - r, nil
	case token.MUL:
		return l * r, nil
	case token.QUO:
		return l / r, nil
	}
	return 0, fmt.Errorf("%s: unsupported operator %s", p.pos(e), e.Op)
}

// exprText renders an expression as Go source.
func (p *parsedPackage) exprText(e ast.Expr) (string, error) {
	var sb strings.Builder
	if err := printer.Fprint(&sb, p.fset, e); err != nil {
		return "", fmt.Errorf("%s: print expression: %w", p.pos(e), err)
	}
	return sb.String(), nil
}
