"""L4 model-predictive-control experiments for the Isaac adaptation.

The production baseline remains the original C++ OpenLoong Convex MPC.  The
Python modules in this package are explicit reference implementations used for
formula checks and for the L4 Convex-MPC/NMPC comparison.
"""

from .srbd_model import SrbdParameters

__all__ = ["SrbdParameters"]
