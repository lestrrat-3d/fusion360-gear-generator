"""Errors of the decad operations that a translated gallery model tells apart, as the gallery does with errors.Is.

The translation of errors.Is(err, decad.X) is isinstance(err, X) for the classes here (port/go2py, Spec.Catch), so
the runtime raises one of them where decad returns the matching error value, and a plain RuntimeError for any other
failure.
"""


class Degenerate(RuntimeError):
    """decad.ErrDegenerate: the operation has no proper result, as when a split sheet misses the body it should cut."""
