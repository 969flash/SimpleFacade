# -*- coding:utf-8 -*-
"""
Grasshopper runner for a single building, mirroring previous main.py usage.
Place this script in a GhPython component and wire inputs accordingly.
"""

try:
    from typing import List
except ImportError:
    pass

import importlib
import facade_plan
import facade_generator as fg

importlib.reload(facade_plan)
importlib.reload(fg)


def _flatten(f_list: List[facade_plan.Facade]):
    g, w, f, s = [], [], [], []
    for fc in f_list or []:
        g.extend(fc.glasses or [])
        w.extend(fc.walls or [])
        f.extend(fc.frames or [])
        s.extend(fc.slabs or [])
    return g, w, f, s


if __name__ == "__main__":
    # Build inputs directly from GH globals and run
    inputs = fg.FGInputs.from_globals(globals())
    generator = fg.FacadeGenerator(inputs)
    _pattern_type = int(globals().get("pattern_type", 1))
    result = generator.generate(_pattern_type)

    # Outputs for GH
    glasses, walls, frames, slabs = _flatten(result.facades)
    parapets = result.parapets
    roof_cores = result.roof_cores
