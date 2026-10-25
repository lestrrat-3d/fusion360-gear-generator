package go2py

import (
	"errors"
	"fmt"
	"go/ast"
	"go/parser"
	"maps"
	"slices"
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
	// Mutable lists the structs the translated code assigns field by field. They become classes
	// with equality by field and a copy method, not namedtuples, and a Go copy of one is a copy in Python.
	Mutable []string
	// Filled lists the structs with slice or handle fields, which are classes anyway, that the translated code
	// fills field by field and never copies. A copy of one is refused, because Python would share the object.
	Filled []string
	// Values are package-level constants and variables, rendered before the functions in the
	// order listed. A value may use the ones before it.
	Values []string
	// FirstOnly lists package-level slices that a retry loop walks. A loop over one runs once,
	// with the first entry, and errors.Is is false: see docs/gallery-translation-goal.md,
	// "Retry loops".
	FirstOnly []string
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
	// Bound maps Go functions the runtime implements to the Python callee, with the struct
	// type each returns, such as derive.
	Bound map[string]Bound
	// External lists functions and methods of another gallery package that the translated
	// code calls, keyed "pkg.Func" or "pkg.Type.Method". The Python module that holds the
	// translation of that package provides them.
	External map[string]External
	// Aliases lists defined types whose underlying type is a number, bool or string, keyed by name:
	// "joinOrder": "int". The translation treats a value of one as its underlying type.
	Aliases map[string]string
	// IotaConsts lists the constants of a const block written with iota, in order: the first is 0.
	IotaConsts []string
	// ExternalTypes lists the structs of another gallery package, keyed "pkg.Type", and the
	// type of each field the translated code reads.
	ExternalTypes map[string]map[string]string
}

// Bound is a Go function that the Python runtime implements instead of a translation.
type Bound struct {
	// Python is the callee, such as "_fx.derive".
	Python string
	// Result is the type the function returns: a translated struct, a handle such as "decad.Body", a slice such as
	// "[]axialPoint", or empty for nothing.
	Result string
}

// External is a function of another gallery package that the translation calls.
type External struct {
	// Python is the callee, such as "_spur.build_tooth_section".
	Python string
	// Results are the types of the results, not counting a trailing error, in the notation of
	// the API table.
	Results []string
	// Method is true when the Go call has a receiver, which becomes the first argument.
	Method bool
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
	tr, err := Translate(m, spec)
	return tr.Python, err
}

// Translation is the result of Translate.
type Translation struct {
	// Python is the rendered code.
	Python string
	// Creations lists, for each translated function that creates Fusion objects, its creating calls in the order
	// the translation makes them. An entry is the role key's operation and number, then the Go call:
	// "decad.Extrude#1 decad.Extrude(ctx, doc, s, prof, extent)". It is the sequence the Roles keys count, so a
	// gallery change to it, such as two swapped calls of one operation, moves a role name to another call.
	Creations map[string][]string
}

// Translate renders spec as TranspileSpec does and also reports the creating calls of each function.
func Translate(m *gallery.Model, spec Spec) (Translation, error) {
	text, creations, err := translate(m, spec)
	if err != nil {
		return Translation{}, err
	}
	return Translation{Python: text, Creations: creations}, nil
}

func translate(m *gallery.Model, spec Spec) (string, map[string][]string, error) {
	tr := &transpiler{
		model: m, prefix: spec.Prefix, structs: map[string]*structInfo{}, funcs: map[string]*funcInfo{},
		sketch:  len(spec.Steps) > 0 || len(spec.TypeOnly) > 0 || len(spec.Roles) > 0 || len(spec.External) > 0,
		runtime: spec.Runtime, roles: spec.Roles, provided: map[string]bool{}, external: spec.External, bound: spec.Bound,
		globals: map[string]global{}, firstOnly: map[string]bool{}, used: map[string]struct{}{},
		creations: map[string][]string{},
	}
	for _, name := range spec.FirstOnly {
		tr.firstOnly[name] = true
	}
	for name, fields := range spec.ExternalTypes {
		info := &structInfo{name: name, types: map[string]typ{}, external: true}
		for field, ft := range fields {
			info.types[field] = parseAPIType(ft)
		}
		tr.structs[name] = info
	}
	for _, name := range spec.Provided {
		tr.provided[name] = true
	}
	if tr.runtime == "" {
		tr.runtime = "_fx"
	}
	var out strings.Builder
	// A struct may use another that is listed after it, so load until none is left.
	type pending struct {
		name    string
		lenient bool
	}
	var todo []pending
	for _, name := range spec.Structs {
		todo = append(todo, pending{name, false})
	}
	for _, name := range spec.TypeOnly {
		todo = append(todo, pending{name, true})
	}
	for len(todo) > 0 {
		var failed []pending
		var lastErr error
		for _, p := range todo {
			info, err := tr.loadStruct(p.name, p.lenient)
			if err != nil {
				failed, lastErr = append(failed, p), err
				continue
			}
			tr.structs[p.name] = info
		}
		if len(failed) == len(todo) {
			return "", nil, lastErr
		}
		todo = failed
	}
	for _, name := range spec.Mutable {
		if info, ok := tr.structs[name]; ok {
			info.assigned = true
		}
	}
	for _, name := range spec.Filled {
		info, ok := tr.structs[name]
		if !ok || !info.mutable() {
			return "", nil, fmt.Errorf("Spec.Filled lists %s, which is not a translated struct with a slice or handle field", name)
		}
		info.filled = true
	}
	for name, underlying := range spec.Aliases {
		expr, err := parser.ParseExpr(underlying)
		if err != nil {
			return "", nil, fmt.Errorf("alias %s: %w", name, err)
		}
		at, err := fromAST(expr, tr.structs)
		if err != nil {
			return "", nil, fmt.Errorf("alias %s: %w", name, err)
		}
		tr.structs[name] = &structInfo{name: name, types: map[string]typ{}, alias: &at}
	}
	for _, name := range spec.Zero {
		tr.structs[name] = &structInfo{name: name, types: map[string]typ{}, zero: true}
	}
	var values strings.Builder
	for i, name := range spec.IotaConsts {
		py := tr.prefix + pySnake(name)
		tr.globals[name] = global{py: py, t: intType}
		fmt.Fprintf(&out, "%s = %d\n\n", py, i)
	}
	for _, name := range spec.Values {
		expr, err := m.PackageValue(name)
		if err != nil {
			return "", nil, err
		}
		fn := &fnState{t: tr, info: &funcInfo{name: name}, env: map[string]typ{}, calls: map[string]int{}}
		text, vt, err := fn.expr(expr)
		if err != nil {
			return "", nil, fmt.Errorf("value %s: %w", name, err)
		}
		switch vt.kind {
		case kindUntyped:
			vt = intType
		case kindUntypedFloat:
			vt = floatType
		}
		py := tr.prefix + pySnake(name)
		tr.globals[name] = global{py: py, t: vt}
		if vt.kind == kindStruct {
			// A value that builds a struct follows the struct classes.
			fmt.Fprintf(&values, "%s = %s\n\n", py, text)
		} else {
			fmt.Fprintf(&out, "%s = %s\n\n", py, text)
		}
	}
	var signatureFailures []error
	for _, name := range spec.Funcs {
		info, err := tr.loadSignature(name)
		if err != nil {
			signatureFailures = append(signatureFailures, fmt.Errorf("signature of %s: %w", name, err))
			continue
		}
		tr.funcs[name] = info
	}
	if len(signatureFailures) > 0 {
		return "", nil, errors.Join(signatureFailures...)
	}
	for _, name := range spec.Structs {
		out.WriteString(tr.structs[name].python(spec.Prefix))
		out.WriteString("\n\n")
	}
	out.WriteString(values.String())
	var body strings.Builder
	// Every function is translated on its own, so report all that fail at once.
	var failures []error
	for _, name := range spec.Funcs {
		src, err := tr.function(tr.funcs[name])
		if err != nil {
			failures = append(failures, fmt.Errorf("function %s: %w", name, err))
			continue
		}
		body.WriteString(src)
		body.WriteString("\n\n")
	}
	if len(failures) > 0 {
		return "", nil, errors.Join(failures...)
	}
	for _, st := range spec.Steps {
		src, err := tr.steps(st)
		if err != nil {
			return "", nil, fmt.Errorf("function %s: %w", st.Name, err)
		}
		body.WriteString(src)
		body.WriteString("\n\n")
	}
	// Struct types written inline are known only after the functions that use them are translated.
	var anon []string
	for name, info := range tr.structs {
		if info.anonymous {
			anon = append(anon, name)
		}
	}
	slices.Sort(anon)
	for _, name := range anon {
		out.WriteString(tr.structs[name].python(spec.Prefix))
		out.WriteString("\n\n")
	}
	out.WriteString(body.String())
	if err := tr.checkUnused(spec); err != nil {
		return "", nil, err
	}
	return strings.TrimRight(out.String(), "\n") + "\n", tr.creations, nil
}

// use records that the translation used the entry key of the Spec table named table.
func (t *transpiler) use(table, key string) { t.used[table+" "+key] = struct{}{} }

// checkUnused fails for an entry of Roles, FirstOnly, Bound, External or Provided that the translation never
// used. A stale key does nothing, or names the wrong call when the calls of a function are renumbered.
func (t *transpiler) checkUnused(spec Spec) error {
	var stale []string
	check := func(table string, keys []string) {
		slices.Sort(keys)
		for _, k := range keys {
			if _, ok := t.used[table+" "+k]; !ok {
				stale = append(stale, fmt.Sprintf("%s[%q]", table, k))
			}
		}
	}
	check("Roles", slices.Collect(maps.Keys(spec.Roles)))
	check("FirstOnly", slices.Clone(spec.FirstOnly))
	check("Bound", slices.Collect(maps.Keys(spec.Bound)))
	check("External", slices.Collect(maps.Keys(spec.External)))
	check("Provided", slices.Clone(spec.Provided))
	if len(stale) > 0 {
		return fmt.Errorf("the translation never uses these Spec entries; remove or correct them: %s",
			strings.Join(stale, ", "))
	}
	return nil
}

type transpiler struct {
	model   *gallery.Model
	prefix  string
	structs map[string]*structInfo
	funcs   map[string]*funcInfo
	// sketch enables the sketch calls and error-check handling drawing functions need.
	sketch   bool
	runtime  string
	external map[string]External
	bound    map[string]Bound
	roles    map[string]Role
	provided map[string]bool
	// globals are the package-level values, by Go name, with their Python names and types.
	globals   map[string]global
	firstOnly map[string]bool
	// used records the Spec table entries the translation used, as "<table> <key>".
	used map[string]struct{}
	// creations lists the creating calls of each function: see Translation.Creations.
	creations map[string][]string
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
		params := make([]string, len(s.fields))
		for i, f := range s.fields {
			params[i] = f + "=None"
		}
		fmt.Fprintf(&sb, "class %s%s:\n    def __init__(self, %s):\n", prefix, pyClass(s.name), strings.Join(params, ", "))
		for _, f := range s.fields {
			if s.types[f].kind == kindSlice {
				// A Go slice field starts as nil, which appends to; Python's None does not. The translation passes
				// an array field's zero values itself (defaultOf, arrayFields).
				fmt.Fprintf(&sb, "        self.%s = [] if %s is None else %s\n", f, f, f)
				continue
			}
			fmt.Fprintf(&sb, "        self.%s = %s\n", f, f)
		}
		if s.assigned {
			fmt.Fprintf(&sb, "\n    def __eq__(self, other):\n        return vars(self) == vars(other)\n")
			fmt.Fprintf(&sb, "\n    def _copy(self):\n        return %s%s(**vars(self))\n", prefix, pyClass(s.name))
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
	fn := &fnState{t: t, info: fi, env: map[string]typ{}, calls: map[string]int{}, root: fi.decl}
	// The parameters, the named results and the body's own declarations are one Go block.
	fn.push(fi.decl.Body.Pos(), fi.decl.Body.End())
	if fi.recvType != nil && fi.recvName != "" {
		fn.env[fi.recvName] = *fi.recvType
		fn.scopes[0].names[fi.recvName] = fi.decl.Pos()
	}
	var args []string
	if fi.recvType != nil && fi.recvName != "" && fi.recvType.kind == kindStruct && !t.structs[fi.recvType.name].zero {
		args = append(args, pyName(fi.recvName))
	}
	for _, p := range fi.params {
		fn.env[p.name] = p.typ
		fn.scopes[0].names[p.name] = fi.decl.Pos()
		if p.typ.kind == kindCtx {
			continue
		}
		if p.typ.variadic {
			args = append(args, "*"+pyName(p.name))
			continue
		}
		args = append(args, pyName(p.name))
	}
	var sb strings.Builder
	fmt.Fprintf(&sb, "def %s(%s):\n", t.pyFunc(fi), strings.Join(args, ", "))
	fn.indent = 1
	if fi.named {
		for _, r := range fi.results {
			fn.env[r.name] = r.typ
			fn.scopes[0].names[r.name] = fi.decl.Pos()
			if r.typ.kind == kindSlice {
				empty, err := r.typ.emptyList()
				if err != nil {
					return "", err
				}
				fn.line(&sb, "%s = %s", pyName(r.name), empty)
				continue
			}
			zero, err := r.typ.zero()
			if err != nil {
				return "", err
			}
			fn.line(&sb, "%s = %s", pyName(r.name), zero)
		}
	}
	if err := fn.copyParams(&sb, fi.params); err != nil {
		return "", err
	}
	if err := fn.stmts(&sb, fi.decl.Body.List); err != nil {
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
	// multi lets a call return several values, as a return of another call does.
	multi bool
	// calls counts the creating calls made so far, per operation, to key the roles table.
	calls map[string]int
	// scopes are the Go blocks open at the statement being translated, innermost last: see define.
	scopes []scope
	// root is the declaration of the Go function being translated, which a closure shares with its function.
	root ast.Node
	// breaks lists what a break leaves in the statements open at the one being translated, innermost last.
	breaks []breakKind
	// remZero lists the remainders that are only compared with zero, where Python's % agrees with Go's.
	remZero []*ast.BinaryExpr
	// counters are the loop variables that never go below zero: range keys and counted loops that start at a
	// constant that is not negative and step up by one. Python's % agrees with Go's on them.
	counters map[string]struct{}
}

// nonNegative records that the loop variable name never goes below zero.
func (f *fnState) nonNegative(name string) {
	if f.counters == nil {
		f.counters = map[string]struct{}{}
	}
	f.counters[name] = struct{}{}
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

// pyFunc is the Python name of a translated Go function or method. An exported Go name is
// public in Python and an unexported one carries the prefix. A method's Python name starts
// with its receiver type, but a name in Spec.Funcs names one Go function, the package-level
// one or else the first method of that name (gallery.Model.Callable), and the translation keys
// functions by that bare name (transpiler.funcs). Two methods of one name on different
// receivers cannot both be translated.
func (t *transpiler) pyFunc(fi *funcInfo) string {
	name := pySnake(fi.name)
	if fi.recvType != nil && fi.recvType.kind == kindStruct {
		name = pySnake(fi.recvType.name) + "_" + name
	}
	if fi.name != "" && unicode.IsUpper([]rune(fi.name)[0]) {
		return name
	}
	return t.prefix + name
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

// arrayFields renders the keyword arguments that give each array field of a struct its zero values, for a struct
// the Go code declares without a value.
func (s *structInfo) arrayFields() string {
	var kw []string
	for _, f := range s.fields {
		if t := s.types[f]; t.kind == kindSlice && t.length != "" {
			kw = append(kw, f+"="+s.defaultOf(f))
		}
	}
	return strings.Join(kw, ", ")
}

// global is a package-level value the translation renders as a module-level Python name.
type global struct {
	py string
	t  typ
}

// defaultOf is the Python default of a field of a class: the zero value of a number or bool, the zero values of an
// array, else None.
func (s *structInfo) defaultOf(field string) string {
	switch t := s.types[field]; t.kind {
	case kindSlice:
		if t.length != "" {
			if empty, err := t.emptyList(); err == nil {
				return empty
			}
		}
	case kindFloat, kindInt, kindUntyped, kindUntypedFloat, kindBool:
		if zero, err := t.zero(); err == nil {
			return zero
		}
	}
	return "None"
}
