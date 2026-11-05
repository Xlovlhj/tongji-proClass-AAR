"""Report the exact Python dependencies used by the L4 controller comparison."""
from __future__ import annotations

import importlib
import json
import platform


def main():
    modules = {}
    passed = True
    for name in ("numpy", "scipy", "osqp"):
        try:
            module = importlib.import_module(name)
            modules[name] = {"available": True,
                             "version": getattr(module, "__version__", "unknown")}
        except Exception as exc:
            passed = False
            modules[name] = {"available": False, "error": str(exc)}
    report = {"python": platform.python_version(), "modules": modules,
              "passed": passed,
              "casadi_required": False,
              "solver_note": (
                  "Convex QP and online SQP-RTI NMPC use OSQP; "
                  "the offline full-NLP reference uses SciPy SLSQP."
              )}
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
