"""
Data types for functional gear generation.

This module contains all dataclasses used for gear generation. These are pure
data containers with no methods (except __post_init__ for validation).
"""

from dataclasses import dataclass, field
from typing import Optional
import math
import adsk.core
import adsk.fusion


@dataclass
class GenerationState:
    """
    Holds all state during gear generation.

    This dataclass is passed through the generation pipeline and updated
    as new components are created.
    """
    design: adsk.fusion.Design
    parent_component: adsk.fusion.Component
    component: adsk.fusion.Component
    occurrence: adsk.fusion.Occurrence
    param_prefix: str

    # Optional fields populated during generation
    plane: Optional[adsk.fusion.ConstructionPlane] = None
    anchor_point: Optional[adsk.fusion.SketchPoint] = None
    sketch: Optional[adsk.fusion.Sketch] = None
    gear_body: Optional[adsk.fusion.BRepBody] = None

    # Additional optional fields for specific generation stages
    tooth_body: Optional[adsk.fusion.BRepBody] = None
    center_axis: Optional[adsk.fusion.ConstructionAxis] = None
    extrusion_extent: Optional[adsk.core.Surface] = None
    extrusion_end_plane: Optional[adsk.fusion.ConstructionPlane] = None
    tooth_profile_is_embedded: bool = False

    # Helical gear specific fields
    helix_plane: Optional[adsk.fusion.ConstructionPlane] = None
    twisted_sketch: Optional[adsk.fusion.Sketch] = None

    # Bevel gear specific fields
    foundation_sketch: Optional[adsk.fusion.Sketch] = None
    foundation_plane: Optional[adsk.fusion.ConstructionPlane] = None
    apex_point: Optional[adsk.fusion.SketchPoint] = None
    gear_base_corner: Optional[adsk.fusion.SketchPoint] = None
    diagonal: Optional[adsk.fusion.SketchLine] = None
    design_component: Optional[adsk.fusion.Component] = None
    gear_component: Optional[adsk.fusion.Component] = None
    gear_occurrence: Optional[adsk.fusion.Occurrence] = None  # Occurrence for Driving Gear (for activation)
    mating_gear_component: Optional[adsk.fusion.Component] = None
    # Extension points for dual-trapezoid foundation sketch
    # Gear side extension points (P5-P10): towards lower Y, closes to P1
    p5: Optional[adsk.fusion.SketchPoint] = None
    p6: Optional[adsk.fusion.SketchPoint] = None
    p7: Optional[adsk.fusion.SketchPoint] = None
    p7_mid: Optional[adsk.fusion.SketchPoint] = None  # Intermediate point on P5->P7 line
    p8: Optional[adsk.fusion.SketchPoint] = None
    p9: Optional[adsk.fusion.SketchPoint] = None
    p10: Optional[adsk.fusion.SketchPoint] = None  # Diagonal profile line point on gear side
    # Mating side extension points (P11-P16): towards higher Y, closes to P3
    p11: Optional[adsk.fusion.SketchPoint] = None  # First mating extension point
    p12: Optional[adsk.fusion.SketchPoint] = None  # Second mating extension point (first vertical endpoint)
    p13: Optional[adsk.fusion.SketchPoint] = None  # Extended point
    p14_mid: Optional[adsk.fusion.SketchPoint] = None  # Intermediate point on P11->P13 line
    p14: Optional[adsk.fusion.SketchPoint] = None  # Outer vertical endpoint (closure point)
    p15: Optional[adsk.fusion.SketchPoint] = None  # Diagonal profile line start point on mating side
    p16: Optional[adsk.fusion.SketchPoint] = None  # Diagonal profile line end point on mating side
    large_end_sketch: Optional[adsk.fusion.Sketch] = None
    large_end_profile: Optional[adsk.fusion.Profile] = None
    small_end_sketch: Optional[adsk.fusion.Sketch] = None
    small_end_profile: Optional[adsk.fusion.Profile] = None
    cone_axis: Optional[adsk.fusion.ConstructionAxis] = None
    tooth_body: Optional[adsk.fusion.BRepBody] = None


@dataclass
class SpurGearSpec:
    """
    Specification for spur gear parameters.

    Contains all input parameters and computed derived parameters for
    generating a spur gear.
    """
    # Input parameters
    module: float
    tooth_number: int
    pressure_angle: float
    thickness: float
    bore_diameter: float
    chamfer_tooth: float
    sketch_only: bool

    # Derived parameters (computed in __post_init__)
    pitch_diameter: float = field(init=False)
    pitch_circle_radius: float = field(init=False)
    base_circle_diameter: float = field(init=False)
    base_circle_radius: float = field(init=False)
    root_circle_diameter: float = field(init=False)
    root_circle_radius: float = field(init=False)
    tip_circle_diameter: float = field(init=False)
    tip_circle_radius: float = field(init=False)
    involute_steps: int = field(init=False, default=15)
    fillet_threshold: float = field(init=False)
    fillet_radius: float = field(init=False)

    def __post_init__(self):
        """Compute derived parameters based on input parameters."""
        # Pitch circle calculations
        self.pitch_diameter = self.module * self.tooth_number
        self.pitch_circle_radius = self.pitch_diameter / 2

        # Base circle calculations
        # https://khkgears.net/new/gear_knowledge/gear-nomenclature/base-circle.html
        self.base_circle_diameter = self.pitch_diameter * math.cos(self.pressure_angle)
        self.base_circle_radius = self.base_circle_diameter / 2

        # Root circle calculations
        # https://khkgears.net/new/gear_knowledge/gear-nomenclature/root-diameter.html
        self.root_circle_diameter = self.pitch_diameter - 2.5 * self.module
        self.root_circle_radius = self.root_circle_diameter / 2

        # Tip circle calculations
        self.tip_circle_diameter = self.pitch_diameter + 2 * self.module
        self.tip_circle_radius = self.tip_circle_diameter / 2

        # Fillet calculations
        self.fillet_threshold = (self.base_circle_diameter * math.pi) / (self.tooth_number * 2) * 0.4
        self.fillet_radius = self.fillet_threshold

        # Validation
        if self.tooth_number < 1:
            raise ValueError(f"tooth_number must be at least 1, got {self.tooth_number}")
        if self.module <= 0:
            raise ValueError(f"module must be positive, got {self.module}")
        if self.thickness <= 0:
            raise ValueError(f"thickness must be positive, got {self.thickness}")


@dataclass
class HelicalGearSpec:
    """
    Specification for helical gear parameters.

    Contains all input parameters and computed derived parameters for
    generating a helical gear. Helical gears extend spur gears by adding
    a helix angle that causes the teeth to twist along the gear's axis.
    """
    # Input parameters
    module: float
    tooth_number: int
    pressure_angle: float
    thickness: float
    bore_diameter: float
    chamfer_tooth: float
    helix_angle: float
    sketch_only: bool

    # Derived parameters (computed in __post_init__)
    pitch_diameter: float = field(init=False)
    pitch_circle_radius: float = field(init=False)
    base_circle_diameter: float = field(init=False)
    base_circle_radius: float = field(init=False)
    root_circle_diameter: float = field(init=False)
    root_circle_radius: float = field(init=False)
    tip_circle_diameter: float = field(init=False)
    tip_circle_radius: float = field(init=False)
    involute_steps: int = field(init=False, default=15)
    fillet_threshold: float = field(init=False)
    fillet_radius: float = field(init=False)

    def __post_init__(self):
        """Compute derived parameters based on input parameters."""
        # Pitch circle calculations
        self.pitch_diameter = self.module * self.tooth_number
        self.pitch_circle_radius = self.pitch_diameter / 2

        # Base circle calculations
        # https://khkgears.net/new/gear_knowledge/gear-nomenclature/base-circle.html
        self.base_circle_diameter = self.pitch_diameter * math.cos(self.pressure_angle)
        self.base_circle_radius = self.base_circle_diameter / 2

        # Root circle calculations
        # https://khkgears.net/new/gear_knowledge/gear-nomenclature/root-diameter.html
        self.root_circle_diameter = self.pitch_diameter - 2.5 * self.module
        self.root_circle_radius = self.root_circle_diameter / 2

        # Tip circle calculations
        self.tip_circle_diameter = self.pitch_diameter + 2 * self.module
        self.tip_circle_radius = self.tip_circle_diameter / 2

        # Fillet calculations
        # Use a smaller fillet radius for helical/herringbone gears (0.2 vs 0.4 for spur)
        # because the lofted geometry is more complex and large fillets can fail
        self.fillet_threshold = (self.base_circle_diameter * math.pi) / (self.tooth_number * 2) * 0.2
        self.fillet_radius = self.fillet_threshold

        # Validation
        if self.tooth_number < 1:
            raise ValueError(f"tooth_number must be at least 1, got {self.tooth_number}")
        if self.module <= 0:
            raise ValueError(f"module must be positive, got {self.module}")
        if self.thickness <= 0:
            raise ValueError(f"thickness must be positive, got {self.thickness}")
        if self.helix_angle <= 0:
            raise ValueError(f"helix_angle must be positive, got {self.helix_angle}")


# Type alias: Herringbone gears use the same spec as helical gears
HerringboneGearSpec = HelicalGearSpec


@dataclass
class BevelGearSpec:
    """
    Specification for bevel gear parameters.

    Contains all input parameters and computed derived parameters for
    generating a bevel gear.
    """
    # Input parameters
    module: float
    tooth_number: int
    mating_tooth_number: int
    pressure_angle: float
    shaft_angle: float
    face_width: Optional[float] = None
    bore_diameter: Optional[float] = None
    driving_gear_base_thickness: float = 5.0
    teeth_length: float = 10.0  # Distance between diagonal profile lines for tooth extrusion
    sketch_only: bool = False

    # Derived parameters (computed in __post_init__)
    pitch_diameter: float = field(init=False)
    mating_pitch_diameter: float = field(init=False)
    pitch_cone_angle: float = field(init=False)
    mating_pitch_cone_angle: float = field(init=False)
    cone_distance: float = field(init=False)
    addendum: float = field(init=False)
    dedendum: float = field(init=False)
    whole_depth: float = field(init=False)
    addendum_angle: float = field(init=False)
    dedendum_angle: float = field(init=False)
    root_cone_angle: float = field(init=False)
    tip_cone_angle: float = field(init=False)
    driving_gear_virtual_teeth_number: int = field(init=False)

    def __post_init__(self):
        """Compute derived parameters based on input parameters."""
        # Pitch calculations
        self.pitch_diameter = self.module * self.tooth_number
        self.mating_pitch_diameter = self.module * self.mating_tooth_number

        # Cone angle calculations
        self.pitch_cone_angle = math.atan(self.tooth_number / self.mating_tooth_number)
        self.mating_pitch_cone_angle = self.shaft_angle - self.pitch_cone_angle
        self.cone_distance = self.pitch_diameter / (2 * math.sin(self.pitch_cone_angle))

        # Auto-calculate face width if not provided
        if self.face_width is None:
            self.face_width = self.cone_distance / 4

        # Tooth depth calculations
        self.addendum = self.module
        self.dedendum = 1.25 * self.module
        self.whole_depth = self.addendum + self.dedendum

        # Additional cone angles
        self.addendum_angle = math.atan(self.addendum / self.cone_distance)
        self.dedendum_angle = math.atan(self.dedendum / self.cone_distance)
        self.root_cone_angle = self.pitch_cone_angle - self.dedendum_angle
        self.tip_cone_angle = self.pitch_cone_angle + self.addendum_angle

        # Calculate virtual teeth number for 45-degree pitch cone angle
        # Zv = roundup(Z / cos(delta)) where delta is the pitch cone angle
        # For equal tooth numbers (1:1 ratio), delta = 45 degrees
        # For general case, we use the actual pitch cone angle
        self.driving_gear_virtual_teeth_number = math.ceil(self.tooth_number / math.cos(self.pitch_cone_angle))

        # Validation
        if self.tooth_number < 1:
            raise ValueError(f"tooth_number must be at least 1, got {self.tooth_number}")
        if self.mating_tooth_number < 1:
            raise ValueError(f"mating_tooth_number must be at least 1, got {self.mating_tooth_number}")
        if self.module <= 0:
            raise ValueError(f"module must be positive, got {self.module}")
        if self.face_width is not None and self.face_width > self.cone_distance / 3:
            import warnings
            warnings.warn(f"Face width {self.face_width} exceeds recommended maximum {self.cone_distance/3}")


@dataclass
class ComponentCleanupInfo:
    """
    Information needed to clean up a component and its parameters.

    Used for error handling to ensure components and parameters are
    properly deleted if generation fails.
    """
    param_prefix: str
    occurrence: adsk.fusion.Occurrence
