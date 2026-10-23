// Package fusionexpr rewrites a gallery parameter expression into the expression
// language of a Fusion user parameter.
//
// The gallery evaluates expressions with sketch/param, where angles are plain
// radian numbers and deg() converts radians to degrees. A Fusion parameter carries
// a unit, and a unit-carrying angle cannot be added to a plain number. The rules
// here are the translations that keep the value the gallery computes.
//
// A rule marked "unverified" below is a hypothesis about Fusion that only a Fusion
// load can confirm. When Fusion disagrees, change the rule here and record the finding
// beside it with the Fusion version, as the pi rule does.
package fusionexpr

import (
	"fmt"
	"go/ast"
	"go/parser"
	"go/token"
	"strings"
)

// Kind is the quantity an expression evaluates to.
type Kind int

const (
	// KindNone is a plain number.
	KindNone Kind = iota
	// KindLength is a length.
	KindLength
	// KindAngle is an angle.
	KindAngle
	// KindOther is any other combination of units.
	KindOther
)

// Result is a translated expression. Format is a Python str.format template with one
// "{}" for each name in Names, in order of appearance. A name that appears twice is
// listed twice.
type Result struct {
	Format string
	Names  []string
	Kind   Kind
}

// Translate rewrites expr. kinds gives the kind of every parameter name the
// expression may reference. It returns an error for syntax, names, operators and
// functions it has no rule for.
func Translate(expr string, kinds map[string]Kind) (Result, error) {
	tree, err := parser.ParseExpr(expr)
	if err != nil {
		return Result{}, fmt.Errorf("parse %q: %w", expr, err)
	}
	tr := &translator{kinds: kinds}
	text, kind, err := tr.walk(tree)
	if err != nil {
		return Result{}, fmt.Errorf("translate %q: %w", expr, err)
	}
	return Result{Format: text, Names: tr.names, Kind: kind}, nil
}

type translator struct {
	kinds map[string]Kind
	names []string
}

func (t *translator) walk(e ast.Expr) (string, Kind, error) {
	switch e := e.(type) {
	case *ast.BasicLit:
		if e.Kind != token.INT && e.Kind != token.FLOAT {
			return "", 0, fmt.Errorf("literal %s is not a number", e.Value)
		}
		return e.Value, KindNone, nil
	case *ast.Ident:
		return t.ident(e)
	case *ast.ParenExpr:
		text, kind, err := t.walk(e.X)
		return "(" + text + ")", kind, err
	case *ast.UnaryExpr:
		return t.unary(e)
	case *ast.BinaryExpr:
		return t.binary(e)
	case *ast.CallExpr:
		return t.call(e)
	}
	return "", 0, fmt.Errorf("unsupported syntax %T", e)
}

func (t *translator) ident(e *ast.Ident) (string, Kind, error) {
	if e.Name == "pi" {
		// Through the API, Fusion parses lowercase "pi" as a unit, so a parameter built on it
		// evaluates to 0 or is rejected next to a length or an angle. Verified in Fusion 2705.1.25
		// on 2026-10-23 with diag_params.py. Capital "PI" is the constant.
		return "PI", KindNone, nil
	}
	kind, ok := t.kinds[e.Name]
	if !ok {
		return "", 0, fmt.Errorf("unknown parameter %q", e.Name)
	}
	t.names = append(t.names, e.Name)
	return "{}", kind, nil
}

func (t *translator) unary(e *ast.UnaryExpr) (string, Kind, error) {
	if e.Op != token.SUB && e.Op != token.ADD {
		return "", 0, fmt.Errorf("unsupported unary operator %s", e.Op)
	}
	text, kind, err := t.walk(e.X)
	return e.Op.String() + text, kind, err
}

func (t *translator) binary(e *ast.BinaryExpr) (string, Kind, error) {
	switch e.Op {
	case token.ADD, token.SUB, token.MUL, token.QUO:
	default:
		return "", 0, fmt.Errorf("unsupported operator %s", e.Op)
	}
	l, lk, err := t.walk(e.X)
	if err != nil {
		return "", 0, err
	}
	r, rk, err := t.walk(e.Y)
	if err != nil {
		return "", 0, err
	}
	kind, err := combine(e.Op, lk, rk)
	if err != nil {
		return "", 0, err
	}
	return l + " " + e.Op.String() + " " + r, kind, nil
}

// combine returns the kind of a binary operation on operands of kinds l and r.
func combine(op token.Token, l, r Kind) (Kind, error) {
	switch op {
	case token.ADD, token.SUB:
		if l != r {
			return 0, fmt.Errorf("operator %s combines kinds %d and %d", op, l, r)
		}
		return l, nil
	case token.MUL:
		switch {
		case l == KindNone:
			return r, nil
		case r == KindNone:
			return l, nil
		}
		return KindOther, nil
	}
	switch {
	case r == KindNone:
		return l, nil
	case l == r:
		return KindNone, nil
	}
	return KindOther, nil
}

// call translates a function call. Only the functions with a rule below are allowed.
func (t *translator) call(e *ast.CallExpr) (string, Kind, error) {
	fn, ok := e.Fun.(*ast.Ident)
	if !ok {
		return "", 0, fmt.Errorf("call of a non-name")
	}
	if len(e.Args) != 1 {
		return "", 0, fmt.Errorf("function %s takes %d arguments, rule expects 1", fn.Name, len(e.Args))
	}
	arg, kind, err := t.walk(e.Args[0])
	if err != nil {
		return "", 0, err
	}
	switch fn.Name {
	case "cos", "sin", "tan":
		// The argument may be an angle parameter or a plain radian number.
		if kind != KindAngle && kind != KindNone {
			return "", 0, fmt.Errorf("%s of a kind %d value", fn.Name, kind)
		}
		return fn.Name + "(" + strings.TrimSpace(arg) + ")", KindNone, nil
	case "deg":
		// deg(angle) is the angle in degrees as a plain number. Verified in Fusion
		// 2705.1.25: dividing an angle by "1 deg" gives a unitless ratio.
		if kind != KindAngle {
			return "", 0, fmt.Errorf("deg of a kind %d value, want an angle", kind)
		}
		return "(" + arg + " / 1 deg)", KindNone, nil
	}
	return "", 0, fmt.Errorf("no Fusion rule for function %s", fn.Name)
}
