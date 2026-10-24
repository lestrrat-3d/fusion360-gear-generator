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
	return TranspileSpec(m, Spec{Prefix: prefix, Structs: structNames, Funcs: funcNames})
}

// Spec lists what TranspileSpec renders.
type Spec struct {
	// Prefix goes in front of every Python name. "_" keeps names out of a star import.
	Prefix string
	// Structs are rendered, in order. A struct may use the ones before it.
	Structs []string
	// TypeOnly structs are known to the translator but not rendered. Fields of a type
	// the translator does not know are left out.
	TypeOnly []string
	// Funcs are translated whole.
	Funcs []string
	// Steps are the creation sequences of drawing functions, translated up to a stop call.
	Steps []Steps
	// Runtime is the name of the Python module that implements the API table. Default "_fx".
	Runtime string
	// Roles maps a creating call, keyed "<Go function>:<Type.Method or pkg.Func>#<n>", to the
	// Fusion name of the object it creates.
	Roles map[string]Role
	// Provided lists Go functions the Fusion wiring already does, such as registering the
	// parameters. A call of one renders nothing.
	Provided []string
	// Zero lists the struct types the translation treats as their zero value.
	Zero []string
}

// Steps names a gallery function that draws on a sketch and the part of it to translate.
// The translation keeps the calls that create or constrain sketch entities and drops
// the error checks, the sketch creation, and everything from the StopAt call on.
type Steps struct {
	Name   string
	PyName string
	// Args are the Python function's parameters in order.
	Args []StepArg
	// StopAt is the name of the first call the translation leaves out.
	StopAt string
}

// StepArg is a parameter of a Steps function and its Go type, such as "*sketch.Sketch".
type StepArg struct{ Name, Type string }

// TranspileSpec renders the structs, functions and drawing sequences of spec, in that order.
func TranspileSpec(m *gallery.Model, spec Spec) (string, error) {
	tr := &transpiler{
		model: m, prefix: spec.Prefix, structs: map[string]*structInfo{}, funcs: map[string]*funcInfo{},
		sketch:  len(spec.Steps) > 0 || len(spec.TypeOnly) > 0,
		runtime: spec.Runtime, roles: spec.Roles, provided: map[string]bool{},
	}
	for _, name := range spec.Provided {
		tr.provided[name] = true
	}
	if tr.runtime == "" {
		tr.runtime = "_fx"
	}
	var out strings.Builder
	for _, name := range spec.Structs {
		info, err := tr.loadStruct(name, false)
		if err != nil {
			return "", err
		}
		tr.structs[name] = info
	}
	for _, name := range spec.TypeOnly {
		info, err := tr.loadStruct(name, true)
		if err != nil {
			return "", err
		}
		tr.structs[name] = info
	}
	for _, name := range spec.Zero {
		tr.structs[name] = &structInfo{name: name, types: map[string]typ{}, zero: true}
	}
	for _, name := range spec.Funcs {
		info, err := tr.loadSignature(name)
		if err != nil {
			return "", err
		}
		tr.funcs[name] = info
	}
	for _, name := range spec.Structs {
		out.WriteString(tr.structs[name].python(spec.Prefix))
		out.WriteString("\n\n")
	}
	for _, name := range spec.Funcs {
		src, err := tr.function(tr.funcs[name])
		if err != nil {
			return "", fmt.Errorf("function %s: %w", name, err)
		}
		out.WriteString(src)
		out.WriteString("\n\n")
	}
	for _, st := range spec.Steps {
		src, err := tr.steps(st)
		if err != nil {
			return "", fmt.Errorf("function %s: %w", st.Name, err)
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
	// sketch enables the sketch calls and error-check handling drawing functions need.
	sketch   bool
	runtime  string
	roles    map[string]Role
	provided map[string]bool
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
	// recvName and recvType describe the receiver of a method. The receiver is a zero-valued
	// struct, so the Python function takes no argument for it.
	recvName string
	recvType *typ
}

type namedType struct {
	name string
	typ  typ
}

func (s *structInfo) python(prefix string) string {
	if s.mutable() {
		var sb strings.Builder
		fmt.Fprintf(&sb, "class %s%s:\n    def __init__(self):\n", prefix, pyClass(s.name))
		for _, f := range s.fields {
			fmt.Fprintf(&sb, "        self.%s = None\n", f)
		}
		return strings.TrimRight(sb.String(), "\n")
	}
	quoted := make([]string, len(s.fields))
	for i, f := range s.fields {
		quoted[i] = "'" + f + "'"
	}
	return fmt.Sprintf("%s%s = collections.namedtuple('%s', [%s])",
		prefix, pyClass(s.name), pyClass(s.name), strings.Join(quoted, ", "))
}

func (t *transpiler) loadStruct(name string, lenient bool) (*structInfo, error) {
	st, err := t.model.Struct(name)
	if err != nil {
		return nil, err
	}
	info := &structInfo{name: name, types: map[string]typ{}}
	for _, f := range st.Fields.List {
		ft, err := fromAST(f.Type, t.structs)
		if err != nil {
			if lenient {
				continue
			}
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
	decl, err := t.model.Callable(name)
	if err != nil {
		return nil, err
	}
	info := &funcInfo{name: name, decl: decl}
	if decl.Recv != nil && len(decl.Recv.List) == 1 {
		rt, err := fromAST(decl.Recv.List[0].Type, t.structs)
		if err != nil {
			return nil, fmt.Errorf("function %s: receiver: %w", name, err)
		}
		info.recvType = &rt
		if names := decl.Recv.List[0].Names; len(names) == 1 {
			info.recvName = names[0].Name
		}
	}
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
	fn := &fnState{t: t, info: fi, env: map[string]typ{}, calls: map[string]int{}}
	if fi.recvType != nil && fi.recvName != "" {
		fn.env[fi.recvName] = *fi.recvType
	}
	var args []string
	for _, p := range fi.params {
		fn.env[p.name] = p.typ
		if p.typ.kind == kindCtx {
			continue
		}
		args = append(args, pyName(p.name))
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
	// calls counts the creating calls made so far, per operation, to key the roles table.
	calls map[string]int
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
	rs := []rune(s)
	var sb strings.Builder
	for i, r := range rs {
		if unicode.IsUpper(r) {
			// A run of capitals is one word (XY → xy); a new word starts at a capital that follows a
			// lower-case letter or digit, or that begins a word after a run of capitals.
			if i > 0 && (!unicode.IsUpper(rs[i-1]) || (i+1 < len(rs) && unicode.IsLower(rs[i+1]))) {
				sb.WriteByte('_')
			}
			sb.WriteRune(unicode.ToLower(r))
			continue
		}
		sb.WriteRune(r)
	}
	return sb.String()
}

// pyKeyword avoids Python keywords for a method name.
func pyKeyword(s string) string {
	switch s {
	case "and", "as", "class", "def", "del", "elif", "except", "from", "global", "in", "is",
		"lambda", "not", "or", "pass", "raise", "try", "with", "yield":
		return s + "_"
	}
	return s
}

// pyClass converts a Go struct name to a Python class name.
func pyClass(s string) string {
	if s == "" {
		return s
	}
	return strings.ToUpper(s[:1]) + s[1:]
}
