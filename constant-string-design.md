# Constant String Consolidation Design

## Overview

This design specifies the creation of a centralized constants system for the fusion360-gear-generator codebase. The goal is to replace hardcoded string literals with named constants to improve maintainability, prevent typos, and enable future internationalization.

**Current Problem**: String literals like `"Module"`, `"ToothNumber"`, and `"Generation error"` are scattered throughout the codebase. Typos cause silent runtime failures. Changes require hunting down all occurrences.

**Solution**: Create a `lib/geargen/constants/` package with four modules organizing all repeated strings by category. Use constants throughout the codebase.

## Module Structure

```
lib/geargen/
├── constants/
│   ├── __init__.py          # Public API - exports everything
│   ├── params.py            # Parameter & input field ID constants
│   ├── messages.py          # Error, UI, and validation messages
│   ├── entities.py          # Component, body, sketch, and feature names
│   └── units.py             # Unit string constants
├── param_expression.py      # Helper for building parameter expressions
```

### Design Principles

1. **Namespace Isolation**: Each category of constants lives in its own module
2. **Flat Import**: Import from `constants` package: `from lib.geargen.constants import PARAM_MODULE`
3. **Prefix Convention**: Use prefixes (PARAM_, INPUT_, ERR_, etc.) to avoid collisions
4. **Immutability**: All constants are module-level string variables
5. **Documentation**: Inline comments explain each constant's purpose

## Constant Definitions

### 1. params.py - Parameter and Input Field IDs

Parameter constants (PARAM_*) are used when creating and accessing user parameters in Fusion 360.
Input constants (INPUT_*) are used when creating command inputs and retrieving their values.

Note the casing conventions:
- PARAM_MODULE = "Module" (PascalCase)
- INPUT_MODULE = "module" (camelCase)

```python
"""Parameter and input field ID constants for gear generation."""

# Core Parameter Names & Input IDs
PARAM_MODULE = "Module"  # Module parameter - fundamental gear sizing unit (metric)
INPUT_MODULE = "module"  # Input field ID for module parameter

PARAM_TOOTH_NUMBER = "ToothNumber"  # Number of teeth on the gear
INPUT_TOOTH_NUMBER = "toothNumber"  # Input field ID for tooth number

PARAM_MATING_TOOTH_NUMBER = "MatingToothNumber"  # Number of teeth on mating gear (bevel gears)
INPUT_MATING_TOOTH_NUMBER = "matingToothNumber"  # Input field ID for mating tooth number

PARAM_PRESSURE_ANGLE = "PressureAngle"  # Pressure angle - angle of gear tooth face
INPUT_PRESSURE_ANGLE = "pressureAngle"  # Input field ID for pressure angle

PARAM_HELIX_ANGLE = "HelixAngle"  # Helix angle - angle of helical twist
INPUT_HELIX_ANGLE = "helixAngle"  # Input field ID for helix angle

PARAM_BORE_DIAMETER = "BoreDiameter"  # Diameter of central bore/shaft hole
INPUT_BORE_DIAMETER = "boreDiameter"  # Input field ID for bore diameter

PARAM_THICKNESS = "Thickness"  # Thickness/width of the gear
INPUT_THICKNESS = "thickness"  # Input field ID for gear thickness

PARAM_CHAMFER_TOOTH = "ChamferTooth"  # Chamfer size for tooth edges
INPUT_CHAMFER_TOOTH = "chamferTooth"  # Input field ID for chamfer size

PARAM_SKETCH_ONLY = "SketchOnly"  # Flag: 1 for sketch-only, 0 for full 3D body
INPUT_SKETCH_ONLY = "sketchOnly"  # Input field ID for sketch-only checkbox

# Selection Input Field IDs
INPUT_ANCHOR_POINT = "anchorPoint"  # Input field ID for anchor point selection
INPUT_PLANE = "plane"  # Input field ID for plane selection
INPUT_PARENT_COMPONENT = "parentComponent"  # Input field ID for parent component selection

# Derived Circle Parameters
PARAM_PITCH_CIRCLE_DIAMETER = "PitchCircleDiameter"  # Pitch circle diameter
PARAM_PITCH_CIRCLE_RADIUS = "PitchCircleRadius"  # Pitch circle radius
PARAM_BASE_CIRCLE_DIAMETER = "BaseCircleDiameter"  # Base circle diameter
PARAM_BASE_CIRCLE_RADIUS = "BaseCircleRadius"  # Base circle radius
PARAM_ROOT_CIRCLE_DIAMETER = "RootCircleDiameter"  # Root circle diameter
PARAM_ROOT_CIRCLE_RADIUS = "RootCircleRadius"  # Root circle radius
PARAM_TIP_CIRCLE_DIAMETER = "TipCircleDiameter"  # Tip circle diameter
PARAM_TIP_CIRCLE_RADIUS = "TipCircleRadius"  # Tip circle radius

# Tooth Profile Parameters
PARAM_INVOLUTE_STEPS = "InvoluteSteps"  # Number of line segments for involute curve
PARAM_FILLET_THRESHOLD = "FilletThreshold"  # Maximum fillet radius at tooth root
PARAM_FILLET_RADIUS = "FilletRadius"  # Actual fillet radius at tooth root

# Bevel Gear Specific Parameters
PARAM_GEAR_HEIGHT = "GearHeight"  # Height/pitch radius of main bevel gear
PARAM_MATING_GEAR_HEIGHT = "MatingGearHeight"  # Height/pitch radius of mating gear
PARAM_VIRTUAL_TEETH_NUMBER = "VirtualTeethNumber"  # Virtual tooth count for bevel gears
PARAM_DRIVING_GEAR_BASE_THICKNESS = "DrivingGearBaseThickness"  # Base thickness for driving gear
PARAM_TEETH_LENGTH = "TeethLength"  # Distance between diagonal lines in gear extension
```

### 2. messages.py - User-Facing Messages

All user-facing strings including error messages, UI labels, tooltips, prompts, and validation messages.

```python
"""User-facing message constants for gear generation."""

# Error Messages - Generic
ERR_GENERATION = "Generation error"  # Generic gear generation error
ERR_GENERATION_FAILED = "Gear generation failed"  # Generic failure message

# Error Messages - Input Validation
ERR_INVALID_MODULE = "Invalid module value"  # Module out of range
ERR_INVALID_PRESSURE_ANGLE = "Invalid pressure angle value"  # Pressure angle out of range
ERR_INVALID_TOOTH_NUMBER = "Invalid tooth number value"  # Tooth number out of range
ERR_INVALID_HELIX_ANGLE = "Invalid helix angle value"  # Helix angle out of range
ERR_INVALID_THICKNESS = "Invalid thickness value"  # Thickness out of range

# Error Messages - Parameters
ERR_MODULE_PARAM_NOT_FOUND = "Module parameter not found"  # Module param doesn't exist
ERR_PARAM_NOT_FOUND = "Parameter not found"  # Generic missing parameter
ERR_PARAM_CREATE_FAILED = "Failed to create parameter"  # Failed to add parameter

# Error Messages - Sketch and Geometry
ERR_ANCHOR_POINT_PROJECTION = "Failed to project anchor point"  # Anchor projection failed
ERR_SKETCH_CREATION = "Failed to create sketch"  # Sketch creation failed
ERR_PROFILE_NOT_FOUND = "Profile not found"  # Expected profile missing
ERR_TOOTH_PROFILE_NOT_FOUND = "could not find tooth profile"  # Tooth profile missing
ERR_GEAR_BODY_PROFILE_NOT_FOUND = "could not find gear body profile"  # Gear body profile missing

# Error Messages - 3D Operations
ERR_EXTRUDE_FAILED = "Failed to extrude profile"  # Extrusion failed
ERR_LOFT_CREATION = "Cannot create lofted tooth"  # Loft creation failed
ERR_REVOLVE_FAILED = "Failed to revolve profile"  # Revolve failed
ERR_JOIN_BODIES = "Cannot join bodies"  # Body combine failed
ERR_PATTERN_FAILED = "Failed to create pattern"  # Circular pattern failed

# Error Messages - Component and Features
ERR_COMPONENT_CREATION = "Failed to create component"  # Component creation failed
ERR_PLANE_CREATION = "Failed to create construction plane"  # Plane creation failed
ERR_AXIS_CREATION = "Failed to create construction axis"  # Axis creation failed

# Error Messages - Bevel Gear Phases
ERR_PHASE1_INCOMPLETE = "Phase 1 incomplete"  # Foundation sketch not complete
ERR_PHASE2_INCOMPLETE = "Phase 2 incomplete"  # Tooth profile not complete
ERR_FOUNDATION_SKETCH_NOT_FOUND = "Foundation sketch not found"  # Can't find foundation sketch

# Error Messages - State
ERR_MISSING_STATE_ATTR = "Missing required state attribute"  # State missing attribute
ERR_INVALID_STATE = "Invalid generation state"  # State is inconsistent

# UI Labels
LABEL_MODULE = "Module"  # Label for module input field
LABEL_TOOTH_NUMBER = "Number of Teeth"  # Label for tooth number input
LABEL_PRESSURE_ANGLE = "Pressure Angle"  # Label for pressure angle input
LABEL_HELIX_ANGLE = "Helix Angle"  # Label for helix angle input
LABEL_BORE_DIAMETER = "Bore Diameter"  # Label for bore diameter input
LABEL_THICKNESS = "Thickness"  # Label for thickness input
LABEL_CHAMFER_TOOTH = "Chamfer"  # Label for chamfer input
LABEL_SKETCH_ONLY = "Sketch Only"  # Label for sketch-only checkbox

# Tooltips
TOOLTIP_MODULE = "The module of the gear (metric sizing unit)"  # Module tooltip
TOOLTIP_TOOTH_NUMBER = "The number of teeth on the gear"  # Tooth number tooltip
TOOLTIP_PRESSURE_ANGLE = "The pressure angle of the gear teeth"  # Pressure angle tooltip
TOOLTIP_SKETCH_ONLY = "Generate sketches, but do not build body"  # Sketch-only tooltip
TOOLTIP_PLANE = "Select a plane to position the gear"  # Plane selection tooltip
TOOLTIP_ANCHOR_POINT = "Select a point to anchor the gear"  # Anchor point tooltip
TOOLTIP_PARENT_COMPONENT = "Select the parent component for the gear"  # Parent component tooltip

# Selection Prompts
PROMPT_SELECT_PLANE = "Select Plane"  # Prompt for plane selection
PROMPT_SELECT_POINT = "Select Point"  # Prompt for point selection
PROMPT_SELECT_COMPONENT = "Select Component"  # Prompt for component selection
PROMPT_SELECT_FACE = "Select Face"  # Prompt for face selection
PROMPT_SELECT_EDGE = "Select Edge"  # Prompt for edge selection

# Validation Messages
VALIDATION_POSITIVE = "must be > 0"  # Value must be positive
VALIDATION_NON_NEGATIVE = "must be >= 0"  # Value must be non-negative
VALIDATION_MIN_ONE = "must be at least 1"  # Value must be at least 1
VALIDATION_BETWEEN_0_AND_1 = "must be between 0 and 1"  # Value in range [0,1]
VALIDATION_INTEGER_REQUIRED = "must be an integer"  # Value must be integer
VALIDATION_NUMERIC_REQUIRED = "must be a number"  # Value must be numeric
VALIDATION_MISSING_STATE = "Missing required state attribute"  # State missing attribute
VALIDATION_INVALID_STATE = "Invalid generation state"  # State is invalid
VALIDATION_NO_SELECTION = "No selection made"  # No entity selected
VALIDATION_INVALID_SELECTION = "Invalid selection"  # Wrong entity type selected
```

### 3. entities.py - Fusion 360 Entity Names

Names for all Fusion 360 entities created during gear generation (components, bodies, sketches, planes, axes, features).

```python
"""Fusion 360 entity name constants for gear generation."""

# Component Names
COMPONENT_DESIGN = "Design"  # Root design component for bevel gears
COMPONENT_DRIVING_GEAR = "Driving Gear"  # Driving (main) bevel gear component
COMPONENT_MATING_GEAR = "Mating Gear"  # Mating (secondary) bevel gear component

# Body Names
BODY_NAME_GEAR = "Gear Body"  # Main gear body (shaft with teeth)
BODY_NAME_TOOTH = "Tooth Body"  # Individual tooth body before patterning
BODY_NAME_BASE = "Base Body"  # Base gear body (before adding teeth)
BODY_NAME_BORE = "Bore"  # Bore/shaft hole body (for cutting operation)

# Sketch Names
SKETCH_FOUNDATION = "Foundation Sketch"  # Foundation sketch for bevel gear (pitch cone)
SKETCH_TOOTH_PROFILE = "Tooth Profile"  # Tooth profile for extrusion/lofting
SKETCH_GEAR_PROFILE = "Gear Profile"  # Full gear profile (spur/helical gears)
SKETCH_BORE = "Bore Sketch"  # Bore/shaft hole sketch
SKETCH_TOOLS = "Tools"  # Utility sketch for construction geometry

# Construction Plane Names
PLANE_FOUNDATION = "Foundation Plane"  # Construction plane for bevel gear foundation
PLANE_TOOTH_PROFILE = "Tooth Profile Plane"  # Construction plane for tooth profile
PLANE_BOTTOM = "Bottom Plane"  # Bottom plane (for helical gears)
PLANE_TOP = "Top Plane"  # Top plane (for helical gears)

# Construction Axis Names
AXIS_GEAR_CENTER = "Gear Center"  # Gear center axis (axis of rotation)
AXIS_TOOTH = "Tooth Axis"  # Individual tooth axis

# Feature Names
FEATURE_PATTERN = "Tooth Pattern"  # Circular pattern for teeth duplication
FEATURE_EXTRUDE_GEAR = "Gear Extrude"  # Extrusion feature for gear body
FEATURE_EXTRUDE_TOOTH = "Tooth Extrude"  # Extrusion feature for tooth
FEATURE_LOFT_TOOTH = "Tooth Loft"  # Loft feature for helical tooth
FEATURE_REVOLVE_BORE = "Bore Revolve"  # Revolve feature for bore
```

### 4. units.py - Unit String Constants

Unit strings used when creating parameters in Fusion 360.

```python
"""Unit string constants for gear generation."""

# Length units
UNIT_MM = "mm"  # Millimeters - primary length unit
UNIT_CM = "cm"  # Centimeters
UNIT_M = "m"  # Meters
UNIT_IN = "in"  # Inches

# Angular units
UNIT_RAD = "rad"  # Radians - primary angular unit
UNIT_DEG = "deg"  # Degrees

# Dimensionless
UNIT_NONE = ""  # Empty string for dimensionless parameters
```

### 5. constants/__init__.py - Public API

```python
"""
String constants for the gear generator library.

This package provides centralized string constants used throughout the gear
generation code. Constants are organized by category and prefixed for clarity:

Parameters and Inputs (params.py):
- PARAM_*: Parameter names (Module, ToothNumber, etc.)
- INPUT_*: Input field IDs (module, toothNumber, etc.)

Messages (messages.py):
- ERR_*: Error messages
- LABEL_*: UI labels
- TOOLTIP_*: UI tooltips
- PROMPT_*: UI prompts
- VALIDATION_*: Validation messages

Entities (entities.py):
- COMPONENT_*: Component names
- BODY_*: Body names
- SKETCH_*: Sketch names
- PLANE_*: Construction plane names
- AXIS_*: Construction axis names
- FEATURE_*: Feature names

Units (units.py):
- UNIT_*: Unit strings

Usage:
    from lib.geargen.constants import PARAM_MODULE, INPUT_MODULE, ERR_INVALID_MODULE
"""

# Import and re-export all constants
from .params import *
from .messages import *
from .entities import *
from .units import *

# Define __all__ for controlled exports
__all__ = [
    # All exported constant names will be listed here
    # This is populated during implementation
]
```

## ParamExpression Helper Class

**Problem**: Parameter expressions embed parameter names as strings in f-strings:
```python
# BAD - defeats constant usage
expression = f'{make_param_name(prefix, "ToothNumber")} * {make_param_name(prefix, "Module")}'
```

**Solution**: `ParamExpression` helper builds expressions safely from constants.

```python
"""lib/geargen/param_expression.py - Helper for building parameter expressions."""

class ParamExpression:
    """
    Helper class for building Fusion 360 parameter expressions using constants.

    This ensures parameter names in expressions use constants rather than
    hardcoded strings, maintaining consistency and enabling refactoring.
    """

    def __init__(self, prefix: str):
        """Initialize with parameter prefix."""
        self.prefix = prefix

    def param(self, name: str) -> str:
        """Get fully-qualified parameter name for use in expressions."""
        from lib.geargen.utilities import make_param_name
        return make_param_name(self.prefix, name)

    def expr(self, template: str, **params) -> str:
        """
        Build an expression from a template and parameter constants.

        Args:
            template: Expression template with {param_name} placeholders
            **params: Keyword arguments mapping placeholder names to PARAM_* constants

        Returns:
            Expression string with fully-qualified parameter references

        Example:
            expr = ParamExpression(prefix)
            expression = expr.expr(
                "{tooth_num} * {module}",
                tooth_num=PARAM_TOOTH_NUMBER,
                module=PARAM_MODULE
            )
            # Returns: "prefix_ToothNumber * prefix_Module"
        """
        resolved = {key: self.param(value) for key, value in params.items()}
        return template.format(**resolved)
```

## Migration Patterns

### Pattern 1: Parameter Creation

```python
# Before:
add_parameter(design, prefix, "Module", value, "mm", "Module parameter")

# After:
from lib.geargen.constants import PARAM_MODULE, UNIT_MM
add_parameter(design, prefix, PARAM_MODULE, value, UNIT_MM, "Module parameter")
```

### Pattern 2: Parameter Expressions

```python
# Before:
add_parameter(
    design, prefix, 'PitchCircleDiameter',
    adsk.core.ValueInput.createByString(
        f'{make_param_name(prefix, "ToothNumber")} * {make_param_name(prefix, "Module")}'
    ),
    'mm', 'Pitch circle diameter'
)

# After:
from lib.geargen.constants import PARAM_PITCH_CIRCLE_DIAMETER, PARAM_TOOTH_NUMBER, PARAM_MODULE, UNIT_MM
from lib.geargen.param_expression import ParamExpression

expr = ParamExpression(prefix)
add_parameter(
    design, prefix, PARAM_PITCH_CIRCLE_DIAMETER,
    adsk.core.ValueInput.createByString(
        expr.expr("{teeth} * {module}", teeth=PARAM_TOOTH_NUMBER, module=PARAM_MODULE)
    ),
    UNIT_MM, 'Pitch circle diameter'
)
```

### Pattern 3: Input Retrieval

```python
# Before:
module = get_value(inputs, "module", "mm")

# After:
from lib.geargen.constants import INPUT_MODULE, UNIT_MM
module = get_value(inputs, INPUT_MODULE, UNIT_MM)
```

### Pattern 4: Error Handling

```python
# Before:
raise ValueError("Invalid module value")

# After:
from lib.geargen.constants import ERR_INVALID_MODULE
raise ValueError(ERR_INVALID_MODULE)
```

### Pattern 5: Component Creation

```python
# Before:
driving_gear.component.name = "Driving Gear"

# After:
from lib.geargen.constants import COMPONENT_DRIVING_GEAR
driving_gear.component.name = COMPONENT_DRIVING_GEAR
```

## Migration Strategy

### Step 1: Find Hardcoded Strings

Use grep to identify all hardcoded parameter, input, and message strings:

```bash
# Find parameter names (PascalCase strings)
grep -rn '"Module"' lib/geargen/
grep -rn '"ToothNumber"' lib/geargen/
grep -rn '"PressureAngle"' lib/geargen/

# Find input IDs (camelCase strings)
grep -rn '"module"' lib/geargen/
grep -rn '"toothNumber"' lib/geargen/

# Find error messages
grep -rn '"Generation error"' lib/geargen/
grep -rn '"Invalid module"' lib/geargen/
```

### Step 2: Replace with Constants

For each file containing hardcoded strings:

1. Add import statement at top of file
2. Replace each string literal with corresponding constant
3. Verify file still works (run tests if available)

### Step 3: Update Parameter Expressions

For files using `make_param_name()` in f-strings:

1. Import `ParamExpression` helper
2. Create `expr = ParamExpression(prefix)` instance
3. Replace f-string expressions with `expr.expr()` calls
4. Ensure all parameter names use constants

### Migration Order

1. **Core utilities first**: `utilities.py`, `inputs.py`, `parameters.py`
2. **Gear generators**: `spurgear.py`, `helicalgear.py`, `herringbonegear.py`, `bevelgear.py`
3. **Command entry points**: `commands/*/entry.py`

## Verification Strategy

### AST-Based Testing

Create a test that uses Python's AST parser to find any remaining hardcoded strings:

```python
# tests/test_no_hardcoded_strings.py

import ast
from pathlib import Path

# Comprehensive list of strings that MUST be constants
FORBIDDEN_PARAM_STRINGS = {
    'Module', 'ToothNumber', 'MatingToothNumber', 'PressureAngle',
    'HelixAngle', 'BoreDiameter', 'Thickness', 'ChamferTooth',
    'SketchOnly', 'PitchCircleDiameter', 'PitchCircleRadius',
    'BaseCircleDiameter', 'BaseCircleRadius', 'RootCircleDiameter',
    'RootCircleRadius', 'TipCircleDiameter', 'TipCircleRadius',
    'InvoluteSteps', 'FilletThreshold', 'FilletRadius',
    'GearHeight', 'MatingGearHeight', 'VirtualTeethNumber',
    'DrivingGearBaseThickness', 'TeethLength',
}

FORBIDDEN_INPUT_STRINGS = {
    'module', 'toothNumber', 'matingToothNumber', 'pressureAngle',
    'helixAngle', 'boreDiameter', 'thickness', 'chamferTooth',
    'sketchOnly', 'anchorPoint', 'plane', 'parentComponent',
}

def test_no_hardcoded_strings():
    """Use AST to find hardcoded parameter/input strings in production code."""
    production_files = [
        f for f in Path('lib/geargen').rglob('*.py')
        if 'constants' not in str(f) and 'test' not in str(f)
    ]

    violations = []

    for filepath in production_files:
        with open(filepath) as f:
            try:
                tree = ast.parse(f.read(), filename=str(filepath))
            except SyntaxError:
                continue

        for node in ast.walk(tree):
            if isinstance(node, ast.Constant) and isinstance(node.value, str):
                if node.value in FORBIDDEN_PARAM_STRINGS or node.value in FORBIDDEN_INPUT_STRINGS:
                    violations.append({
                        'file': str(filepath),
                        'line': node.lineno,
                        'string': node.value
                    })

    if violations:
        error_msg = "Found hardcoded strings (should use constants):\n\n"
        for v in violations:
            error_msg += f"  {v['file']}:{v['line']}: '{v['string']}'\n"
        assert False, error_msg
```

### Integration Testing

After migration, verify each gear type still works:

1. Load add-in in Fusion 360
2. Test Spur Gear creation
3. Test Helical Gear creation
4. Test Herringbone Gear creation
5. Test Bevel Gear creation (if applicable)
6. Verify parameters are created with correct names
7. Verify components/sketches have correct names

## Integration with Existing Utilities

The constants work seamlessly with existing utility functions:

```python
# With get_value() from inputs.py
from lib.geargen.constants import INPUT_MODULE, UNIT_MM
module = get_value(inputs, INPUT_MODULE, UNIT_MM)

# With get_selection() from inputs.py
from lib.geargen.constants import INPUT_PLANE
plane = get_selection(inputs, INPUT_PLANE)

# With add_parameter() from utilities.py
from lib.geargen.constants import PARAM_MODULE, UNIT_MM
add_parameter(design, prefix, PARAM_MODULE, value_input, UNIT_MM, "Module parameter")

# With parameter lookup
from lib.geargen.constants import PARAM_MODULE
from lib.geargen.utilities import make_param_name
module_param = design.userParameters.itemByName(make_param_name(prefix, PARAM_MODULE))
```

## Benefits

1. **Single Source of Truth**: Each string defined once, changes propagate everywhere
2. **Typo Prevention**: Constants are validated by Python; typos cause immediate errors
3. **Searchability**: Find all uses of a concept with "Find References"
4. **Refactoring Support**: Renaming a constant updates all usages
5. **Consistency**: Impossible to have inconsistent strings across the codebase
6. **i18n Ready**: Constants can be replaced with localization keys in the future

## Implementation Checklist

- [ ] Create `lib/geargen/constants/` directory
- [ ] Implement `params.py` with all parameter/input constants
- [ ] Implement `messages.py` with all message constants
- [ ] Implement `entities.py` with all entity name constants
- [ ] Implement `units.py` with all unit constants
- [ ] Implement `constants/__init__.py` with exports and `__all__`
- [ ] Implement `param_expression.py` helper class
- [ ] Migrate `utilities.py` to use constants
- [ ] Migrate `inputs.py` to use constants
- [ ] Migrate `parameters.py` to use constants and ParamExpression
- [ ] Migrate all gear generator modules
- [ ] Migrate all command entry points
- [ ] Create AST-based test for hardcoded strings
- [ ] Run AST test to verify no hardcoded strings remain
- [ ] Test all gear types in Fusion 360
- [ ] Verify parameters/components have correct names
