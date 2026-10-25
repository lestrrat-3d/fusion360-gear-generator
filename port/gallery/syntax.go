package gallery

import (
	"fmt"
	"go/ast"
	"go/token"
)

// FileSet returns the file set that positions the nodes the model returns.
func (m *Model) FileSet() *token.FileSet { return m.pkg.fset }

// Func returns the declaration of the package-level function called name.
func (m *Model) Func(name string) (*ast.FuncDecl, error) { return m.pkg.funcDecl(name) }

// Callable returns the package-level function called name or, when there is none, the first
// method of that name on any receiver.
func (m *Model) Callable(name string) (*ast.FuncDecl, error) { return m.pkg.callable(name) }

// Struct returns the field list of the struct type called name.
func (m *Model) Struct(name string) (*ast.StructType, error) {
	ts, err := m.pkg.typeSpec(name)
	if err != nil {
		return nil, err
	}
	st, ok := ts.Type.(*ast.StructType)
	if !ok {
		return nil, fmt.Errorf("%s: type %s is not a struct", m.pkg.pos(ts), name)
	}
	return st, nil
}
