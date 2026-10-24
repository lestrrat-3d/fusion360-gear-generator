// Package gallery reads a decad-gallery model package and extracts the facts the
// Fusion port needs: the inputs and their defaults, the parameter table, and the
// decad calls the build makes. It parses the Go source and never compiles it.
package gallery

// Model is what the reader extracts from one gallery model package.
type Model struct {
	pkg *parsedPackage
	// Package is the Go package name, for example "spurgear".
	Package string
	// ParamsAlias is the type Params aliases, for example "helicalgear.Params", or empty
	// when Params is a struct the package declares.
	ParamsAlias string
	// Inputs lists the fields of the Params struct in declaration order.
	Inputs []Input
	// Table lists the parameter table entries in the order defineParameters writes them.
	Table []Parameter
	// TableErr is why Table could not be read, for a model whose defineParameters is
	// not in the form the reader accepts (string-literal Set calls and no loops). A port
	// that needs the table must report it; a port that does not is unaffected.
	TableErr error
}

// Input is one field of the model's Params struct.
type Input struct {
	// Name is the Go field name, for example "PressureAngle".
	Name string
	// Type is the Go type as written, for example "float64".
	Type string
	// Default is the value DefaultParams gives the field. HasDefault is false when
	// DefaultParams leaves the field at its zero value.
	Default    float64
	HasDefault bool
	// DefaultText is the default of a string or bool field, as written without quotes.
	// HasDefault is false for these fields.
	DefaultText string
}

// ParameterKind tells how a table entry gets its value.
type ParameterKind string

const (
	// KindInput is an entry set from a Params field with a typed value.
	KindInput ParameterKind = "input"
	// KindCount is an entry set from a Params field as a dimensionless number.
	KindCount ParameterKind = "count"
	// KindExpr is an entry computed from an expression over other entries.
	KindExpr ParameterKind = "expr"
)

// Parameter is one entry of the parameter table.
type Parameter struct {
	// Name is the table name, for example "root_radius".
	Name string
	Kind ParameterKind
	// Unit is the unit constructor name used by the Go source, for example
	// "Millimeters", "Millimeter", "Radians" or "One".
	Unit string
	// Field is the Params field an input or count entry reads, for example "Module".
	Field string
	// Expr is the expression text of an expression entry.
	Expr string
}
