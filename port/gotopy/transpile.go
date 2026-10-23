package gotopy

import (
	"fmt"
	"go/ast"
	"strings"
	"unicode"

	"github.com/lestrrat-3d/fusion360-gear-generator/port/gallery"
)

// Transpile renders the named struct types as Python namedtuples and the named
// functions as Python functions, in that order. A function may call another one in
// funcs. Python function names are the Go names in snake_case and class names are
// the struct names capitalized, each with prefix in front. A prefix of "_" keeps the
// names out of a star import.
func Transpile(m *gallery.Model, prefix string, structNames, funcNames []string) (string, error) {
	tr := &transpiler{
		model: m, prefix: prefix, structs: map[string]*structInfo{}, funcs: map[string]*funcInfo{},
	}
	var out strings.Builder
	for _, name := range structNames {
		info, err := tr.loadStruct(name)
		if err != nil {
			return "", err
		}
		tr.structs[name] = info
	}
	for _, name := range funcNames {
		info, err := tr.loadSignature(name)
		if err != nil {
			return "", err
		}
		tr.funcs[name] = info
	}
	for _, name := range structNames {
		out.WriteString(tr.structs[name].python(prefix))
		out.WriteString("\n\n")
	}
	for _, name := range funcNames {
		src, err := tr.function(tr.funcs[name])
		if err != nil {
			return "", fmt.Errorf("function %s: %w", name, err)
		}
		out.WriteString(src)
		out.WriteString("\n\n")
	}
	return strings.TrimRight(out.String(), "\n") + "\n", nil
}

type transpiler struct {
	model   *gallery.Model
	prefix  string
	structs map[string]*structInfo
	funcs   map[string]*funcInfo
}

// funcInfo is the parsed signature of a function to translate.
type funcInfo struct {
	name    string
	decl    *ast.FuncDecl
	params  []namedType
	results []namedType
	// errorResult is true when the last result is an error. The Python function
	// returns the other results and raises RuntimeError for a non-nil error.
	errorResult bool
	named       bool
}

type namedType struct {
	name string
	typ  typ
}

func (s *structInfo) python(prefix string) string {
	quoted := make([]string, len(s.fields))
	for i, f := range s.fields {
		quoted[i] = "'" + f + "'"
	}
	return fmt.Sprintf("%s%s = collections.namedtuple('%s', [%s])",
		prefix, pyClass(s.name), pyClass(s.name), strings.Join(quoted, ", "))
}

func (t *transpiler) loadStruct(name string) (*structInfo, error) {
	st, err := t.model.Struct(name)
	if err != nil {
		return nil, err
	}
	info := &structInfo{name: name, types: map[string]typ{}}
	for _, f := range st.Fields.List {
		ft, err := fromAST(f.Type, map[string]*structInfo{})
		if err != nil {
			return nil, fmt.Errorf("struct %s: %w", name, err)
		}
		for _, n := range f.Names {
			info.fields = append(info.fields, n.Name)
			info.types[n.Name] = ft
		}
	}
	return info, nil
}

func (t *transpiler) loadSignature(name string) (*funcInfo, error) {
	decl, err := t.model.Func(name)
	if err != nil {
		return nil, err
	}
	info := &funcInfo{name: name, decl: decl}
	for _, f := range decl.Type.Params.List {
		ft, err := fromAST(f.Type, t.structs)
		if err != nil {
			return nil, fmt.Errorf("function %s: %w", name, err)
		}
		for _, n := range f.Names {
			info.params = append(info.params, namedType{n.Name, ft})
		}
	}
	if decl.Type.Results == nil {
		return nil, fmt.Errorf("function %s returns nothing", name)
	}
	for i, f := range decl.Type.Results.List {
		isLast := i == len(decl.Type.Results.List)-1
		if id, ok := f.Type.(*ast.Ident); ok && id.Name == "error" && isLast && len(f.Names) == 0 {
			info.errorResult = true
			continue
		}
		rt, err := fromAST(f.Type, t.structs)
		if err != nil {
			return nil, fmt.Errorf("function %s: %w", name, err)
		}
		if len(f.Names) == 0 {
			info.results = append(info.results, namedType{"", rt})
			continue
		}
		info.named = true
		for _, n := range f.Names {
			info.results = append(info.results, namedType{n.Name, rt})
		}
	}
	return info, nil
}

// function renders one function.
func (t *transpiler) function(fi *funcInfo) (string, error) {
	fn := &fnState{t: t, info: fi, env: map[string]typ{}}
	args := make([]string, len(fi.params))
	for i, p := range fi.params {
		fn.env[p.name] = p.typ
		args[i] = pyName(p.name)
	}
	var sb strings.Builder
	fmt.Fprintf(&sb, "def %s%s(%s):\n", t.prefix, pySnake(fi.name), strings.Join(args, ", "))
	fn.indent = 1
	if fi.named {
		for _, r := range fi.results {
			fn.env[r.name] = r.typ
			if r.typ.kind == kindSlice {
				fn.line(&sb, "%s = []", pyName(r.name))
				continue
			}
			zero, err := r.typ.zero()
			if err != nil {
				return "", err
			}
			fn.line(&sb, "%s = %s", pyName(r.name), zero)
		}
	}
	if err := fn.block(&sb, fi.decl.Body.List); err != nil {
		return "", err
	}
	return strings.TrimRight(sb.String(), "\n"), nil
}

// fnState is the state of the function being translated.
type fnState struct {
	t      *transpiler
	info   *funcInfo
	env    map[string]typ
	indent int
}

func (f *fnState) line(sb *strings.Builder, format string, args ...any) {
	sb.WriteString(strings.Repeat("    ", f.indent))
	fmt.Fprintf(sb, format, args...)
	sb.WriteString("\n")
}

func (f *fnState) pos(n ast.Node) string { return f.t.model.FileSet().Position(n.Pos()).String() }

// pyName avoids Python keywords and builtins the translated code uses.
func pyName(s string) string {
	switch s {
	case "and", "as", "class", "def", "del", "elif", "except", "from", "global", "in", "is",
		"lambda", "not", "or", "pass", "raise", "try", "with", "yield", "None", "True", "False",
		"list", "float", "int", "len", "range", "abs", "math":
		return s + "_"
	}
	return s
}

// pySnake converts a Go camelCase name to snake_case.
func pySnake(s string) string {
	var sb strings.Builder
	for i, r := range s {
		if unicode.IsUpper(r) {
			if i > 0 {
				sb.WriteByte('_')
			}
			sb.WriteRune(unicode.ToLower(r))
			continue
		}
		sb.WriteRune(r)
	}
	return sb.String()
}

// pyClass converts a Go struct name to a Python class name.
func pyClass(s string) string {
	if s == "" {
		return s
	}
	return strings.ToUpper(s[:1]) + s[1:]
}
