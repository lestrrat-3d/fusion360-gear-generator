"""Helper for building Fusion 360 parameter expressions using constants."""

from lib.geargen.utilities import make_param_name


class ParamExpression:
    """
    Helper class for building Fusion 360 parameter expressions using constants.

    This ensures parameter names in expressions use constants rather than
    hardcoded strings, maintaining consistency and enabling refactoring.

    Example:
        from lib.geargen.constants import PARAM_TOOTH_NUMBER, PARAM_MODULE
        from lib.geargen.param_expression import ParamExpression

        expr = ParamExpression(prefix)
        expression = expr.expr(
            "{tooth_num} * {module}",
            tooth_num=PARAM_TOOTH_NUMBER,
            module=PARAM_MODULE
        )
        # Returns: "prefix_ToothNumber * prefix_Module"
    """

    def __init__(self, prefix: str):
        """
        Initialize with parameter prefix.

        Args:
            prefix: The parameter prefix to use for all parameter names
        """
        self.prefix = prefix

    def param(self, name: str) -> str:
        """
        Get fully-qualified parameter name for use in expressions.

        Args:
            name: The parameter name constant (e.g., PARAM_MODULE)

        Returns:
            Fully-qualified parameter name with prefix (e.g., "prefix_Module")

        Example:
            expr = ParamExpression("gear1")
            param_name = expr.param(PARAM_MODULE)
            # Returns: "gear1_Module"
        """
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
            expr = ParamExpression("gear1")
            expression = expr.expr(
                "{tooth_num} * {module}",
                tooth_num=PARAM_TOOTH_NUMBER,
                module=PARAM_MODULE
            )
            # Returns: "gear1_ToothNumber * gear1_Module"
        """
        resolved = {key: self.param(value) for key, value in params.items()}
        return template.format(**resolved)
