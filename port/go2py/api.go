package go2py

import (
	"fmt"
	"go/ast"
	"sort"
	"strings"
)

// apiSig describes one operation of the decad, sketch and r3 packages the Python runtime
// implements. The translator calls it by name and never looks inside.
type apiSig struct {
	// res is the result type: "" for none, a scalar ("float", "int", "bool", "string"), or an
	// opaque type such as "decad.Body" or "[]decad.Body".
	res string
	// named marks an operation that creates a Fusion object. Its call site needs a role name.
	named bool
	// recv marks a sketchcheck helper whose first argument is the receiver.
	recv bool
}

// apiFuncs are the package-level functions a translated function may call.
var apiFuncs = map[string]apiSig{
	"decad.Extrude":       {res: "decad.Body", named: true},
	"decad.PatternCopies": {res: "[]decad.Body", named: true},
	"decad.Union":         {res: "decad.Body", named: true},
	"decad.Cut":           {res: "decad.Body", named: true},
	"decad.Fillet":        {res: "decad.Body", named: true},
	"decad.Chamfer":       {res: "decad.Body", named: true},
	"decad.Circular":      {res: "decad.Predicate"},
	"decad.LongerThan":    {res: "decad.Predicate"},
	"units.Millimeters":   {res: "units.Value"},
	"decad.Loft":          {res: "decad.Body", named: true},
	"decad.MirroredCopy":  {res: "decad.Body", named: true},
	"decad.Edges":         {res: "decad.EdgeQuery"},
	"decad.ParallelTo":    {res: "decad.Predicate"},
	"decad.EndpointAt":    {res: "decad.Predicate"},
	"decad.Concave":       {res: "decad.Predicate"},
	"decad.OuterLoopOf":   {res: "decad.Predicate"},
	"decad.CapStart":      {res: "decad.EdgeQuery"},
	"decad.CapEnd":        {res: "decad.EdgeQuery"},
	"sketch.NewWorld":     {res: "sketch.World"},
	"sketch.NewRadius":    {res: "sketch.Constraint"},
	"sketch.NewDiameter":  {res: "sketch.Constraint"},
	"r3.NewFrame":         {res: "r3.Frame"},
	"r3.NewVec":           {res: "r3.Vec"},

	"sketchcheck.DimensionPoint":         {res: "sketch.Point", recv: true},
	"sketchcheck.DimensionArcEnd":        {res: "sketch.Point", recv: true},
	"sketchcheck.VerifyFullyConstrained": {},
}

// apiMethods are the methods of the opaque types, keyed "Type.Method".
var apiMethods = map[string]apiSig{
	"decad.Document.Param":        {res: "units.Value"},
	"units.Value.Base":            {res: "float"},
	"decad.Body.IsSolid":          {res: "bool"},
	"decad.EdgeQuery.Or":          {res: "decad.EdgeQuery"},
	"decad.EdgeQuery.Exactly":     {res: "decad.EdgeQuery"},
	"decad.EdgeQuery.SelectEdges": {res: "[]decad.Edge"},
	"decad.Edge.Start":            {res: "decad.Vertex"},
	"decad.Edge.End":              {res: "decad.Vertex"},
	"decad.Edge.Curve":            {res: "decad.Curve"},
	"decad.Body.Faces":            {res: "[]decad.Face"},
	"decad.Body.Centroid":         {res: "decad.Centroid"},
	"r3.Vec.Dot":                  {res: "float"},
	"r3.Vec.Cross":                {res: "r3.Vec"},
	"decad.Face.Surface":          {res: "decad.Surface"},
	"decad.Face.Edges":            {res: "[]decad.Edge"},
	"decad.Vertex.Position":       {res: "decad.Position"},

	"sketch.World.CreateSketch":               {res: "sketch.Sketch", named: true},
	"sketch.World.CreatePlaneFromFrame":       {res: "sketch.Plane", named: true},
	"sketch.World.Params":                     {res: "sketch.Table"},
	"sketch.World.XY":                         {res: "sketch.Plane"},
	"sketch.Sketch.CreateCircle":              {res: "sketch.Circle"},
	"sketch.Sketch.CreateLine":                {},
	"sketch.Sketch.CreateArc":                 {},
	"sketch.Sketch.CreateFitSpline":           {res: "sketch.Entity"},
	"sketch.Sketch.CreatePoint":               {res: "sketch.Point"},
	"sketch.Sketch.CreatePatternCircularRing": {res: "sketch.Entity"},
	"sketch.Sketch.AddConstraint":             {},
	"sketch.Sketch.Bind":                      {},
	"sketch.Sketch.Origin":                    {res: "sketch.Point"},
	"sketch.Sketch.Profiles":                  {res: "[]sketch.Profile"},
	"sketch.Sketch.Solve":                     {},

	"r3.Frame.Origin": {res: "r3.Vec"},
	"r3.Frame.N":      {res: "r3.Vec"},
	"r3.Frame.U":      {res: "r3.Vec"},
	"r3.Frame.V":      {res: "r3.Vec"},
	"r3.Vec.Add":      {res: "r3.Vec"},
	"r3.Vec.Sub":      {res: "r3.Vec"},
	"r3.Vec.Scale":    {res: "r3.Vec"},
	"r3.Vec.Len":      {res: "float"},
}

// apiFields are the fields of the opaque types, keyed "Type.Field".
var apiFields = map[string]string{
	"decad.Position.Value":  "r3.Vec",
	"decad.Cylinder.Radius": "units.Value",
	"decad.Centroid.Value":  "r3.Vec",
	"sketch.Profile.Holes":  "[]sketch.Edge",
	"sketch.Circle.Center":  "sketch.Point",
	"r3.Vec.X":              "float",
	"r3.Vec.Y":              "float",
	"r3.Vec.Z":              "float",
	"sketch.Profile.Valid":  "bool",
	"sketch.Profile.Outer":  "[]sketch.Edge",
}

// apiConsts are the package-level constants of the opaque packages.
var apiConsts = map[string]string{
	"decad.Along": "int",
}

// apiLiterals are the struct types of the opaque packages a translated function may build
// with keyed fields. Each becomes a call of the runtime function with keyword arguments.
var apiLiterals = map[string]string{
	"decad.CircularPattern": "decad.CircularPattern",
	"decad.Distance":        "decad.Distance",
	"decad.MirrorFrame":     "decad.MirrorFrame",
	"decad.TwoSided":        "decad.TwoSided",
	"decad.DistanceSide":    "decad.DistanceSide",
}

// apiTypeTests are the types a translated function may test a value against, with a type
// assertion. Each becomes a runtime function named is_<type>.
var apiTypeTests = map[string]bool{"decad.Cylinder": true, "decad.Line3": true}

// typeTestName is the runtime function that tests a value against an API type.
func typeTestName(qualified string) string {
	pkg, name, _ := strings.Cut(qualified, ".")
	return pkg + "_is_" + pySnake(name)
}

// parseAPIType converts an API result type string.
func parseAPIType(s string) typ {
	switch {
	case s == "":
		return typ{kind: kindOpaque, name: "void"}
	case s == "float":
		return floatType
	case s == "int":
		return intType
	case s == "bool":
		return boolType
	case s == "string":
		return stringType
	case strings.HasPrefix(s, "[]"):
		elem := parseAPIType(s[2:])
		return typ{kind: kindSlice, elem: &elem}
	}
	return typ{kind: kindOpaque, name: s}
}

// Role is the Fusion name of an object a translated call creates.
type Role struct {
	// Feature names the sketch, plane or timeline feature.
	Feature string
	// Body names the body a feature creates, when it creates one.
	Body string
	// Qualify names a string variable of the translated function that is appended to Feature, so a
	// helper that serves several callers names its result after the caller's label.
	Qualify string
}

// runtimeName is the runtime function for a package-level API function.
func runtimeName(qualified string) string {
	pkg, fn, _ := strings.Cut(qualified, ".")
	return pkg + "_" + pySnake(fn)
}

// apiCall renders a call of the API tables. It reports false when the call is not an API call.
func (f *fnState) apiCall(e *ast.CallExpr, sel *ast.SelectorExpr) (string, typ, bool, error) {
	if pkg, ok := sel.X.(*ast.Ident); ok && opaquePackages[pkg.Name] || isIdent(sel.X, "sketchcheck") {
		name := sel.X.(*ast.Ident).Name + "." + sel.Sel.Name
		sig, ok := apiFuncs[name]
		if !ok {
			return "", typ{}, true, fmt.Errorf("%s: %s is not in the API table", f.pos(e), name)
		}
		args, err := f.argList(e)
		if err != nil {
			return "", typ{}, true, err
		}
		if sig.recv {
			return f.finishCall(e, name, sig, args[0]+"."+pyKeyword(pySnake(sel.Sel.Name)), args[1:])
		}
		return f.finishCall(e, name, sig, f.t.runtime+"."+runtimeName(name), args)
	}
	recvText, rt, err := f.expr(sel.X)
	if err != nil {
		return "", typ{}, false, nil
	}
	if rt.kind == kindStruct && f.t.structs[rt.name] != nil && f.t.structs[rt.name].zero {
		return "", typ{}, false, nil
	}
	if rt.kind != kindOpaque {
		return "", typ{}, false, nil
	}
	name := rt.name + "." + sel.Sel.Name
	sig, ok := apiMethods[name]
	if !ok {
		return "", typ{}, true, fmt.Errorf("%s: %s is not in the API table", f.pos(e), name)
	}
	args, err := f.argList(e)
	if err != nil {
		return "", typ{}, true, err
	}
	return f.finishCall(e, name, sig, recvText+"."+pyKeyword(pySnake(sel.Sel.Name)), args)
}

// argList renders the arguments of a call. A context argument is dropped, and a final
// spread argument becomes a star argument.
func (f *fnState) argList(e *ast.CallExpr) ([]string, error) {
	var out []string
	for i, a := range e.Args {
		text, at, err := f.expr(a)
		if err != nil {
			return nil, err
		}
		if at.kind == kindCtx {
			continue
		}
		if e.Ellipsis.IsValid() && i == len(e.Args)-1 {
			text = "*" + text
		}
		out = append(out, text)
	}
	return out, nil
}

// finishCall adds the role name of a creating call and renders it.
func (f *fnState) finishCall(e *ast.CallExpr, name string, sig apiSig, callee string, args []string) (string, typ, bool, error) {
	if sig.named {
		f.calls[name]++
		key := fmt.Sprintf("%s:%s#%d", f.info.name, name, f.calls[name])
		role, ok := f.t.roles[key]
		if !ok {
			return "", typ{}, true, fmt.Errorf("%s: no role name for %s: add it to the roles table", f.pos(e), key)
		}
		if role.Qualify != "" {
			args = append(args, "name="+quote(role.Feature+" ")+" + "+pyName(role.Qualify))
		} else {
			args = append(args, "name="+quote(role.Feature))
		}
		if role.Body != "" {
			args = append(args, "body="+quote(role.Body))
		}
	}
	return callee + "(" + strings.Join(args, ", ") + ")", parseAPIType(sig.res), true, nil
}

// quote renders a Python string literal.
func quote(s string) string { return fmt.Sprintf("%q", s) }

// APIEntry is one operation of the API table and the Python name the runtime defines for it.
type APIEntry struct {
	// Go is the qualified Go name, such as decad.Extrude or decad.Body.IsSolid.
	Go string
	// Python is the name of the runtime function, or the method name on the runtime class.
	Python string
	// Receiver is the Go type of a method's receiver, empty for a runtime function.
	Receiver string
}

// API lists every operation, field and constant the translator accepts, so a test can check
// that the Python runtime defines each one.
func API() []APIEntry {
	var out []APIEntry
	for name, sig := range apiFuncs {
		if strings.HasSuffix(name, "#v") {
			continue
		}
		if sig.recv {
			_, fn, _ := strings.Cut(name, ".")
			out = append(out, APIEntry{Go: name, Python: pySnake(fn), Receiver: "sketch.Sketch"})
			continue
		}
		out = append(out, APIEntry{Go: name, Python: runtimeName(name)})
	}
	for name := range apiMethods {
		i := strings.LastIndex(name, ".")
		out = append(out, APIEntry{Go: name, Python: pyKeyword(pySnake(name[i+1:])), Receiver: name[:i]})
	}
	for name := range apiFields {
		i := strings.LastIndex(name, ".")
		out = append(out, APIEntry{Go: name, Python: name[i+1:], Receiver: name[:i]})
	}
	for name := range apiConsts {
		pkg, c, _ := strings.Cut(name, ".")
		out = append(out, APIEntry{Go: name, Python: strings.ToUpper(pkg) + "_" + strings.ToUpper(pySnake(c))})
	}
	for name := range apiTypeTests {
		out = append(out, APIEntry{Go: name, Python: typeTestName(name)})
	}
	for name := range apiLiterals {
		out = append(out, APIEntry{Go: name, Python: runtimeName(name)})
	}
	sort.Slice(out, func(i, j int) bool { return out[i].Go < out[j].Go })
	return out
}
