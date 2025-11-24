#!/usr/bin/env python
# -*- coding:utf-8 -*-

try:
    from typing import List, Optional
except ImportError:
    pass

import Rhino.Geometry as geo  # type: ignore
import importlib
import random
import time

import facade_generator as fg
import facade_plan
import utils

# 모듈 새로고침
importlib.reload(fg)
importlib.reload(facade_plan)
importlib.reload(utils)

from facade_plan import Facade


class FGInputsExpander:
    """기본 FGInputs와 variation_factor만으로
    빌딩별 FGInputs 리스트로 확장하는 유틸리티"""

    @staticmethod
    def expand(
        base: fg.FGInputs,
        building_breps: List[geo.Brep],
        variation_factor: float = 0.0,
    ) -> List[fg.FGInputs]:
        rnd_seed = int(time.time() * 1000) % 10000
        random.seed(rnd_seed)
        base = base.coerce()

        results: List[fg.FGInputs] = []
        for brep in building_breps or []:
            mult = 1.0 + random.uniform(0.0, max(0.0, float(variation_factor)))
            results.append(
                fg.FGInputs(
                    building_brep=brep,
                    floor_height=base.floor_height,
                    pattern_length=float(base.pattern_length) * mult,
                    pattern_depth=float(base.pattern_depth) * mult,
                    pattern_ratio=float(base.pattern_ratio) * mult,
                    facade_offset=float(base.facade_offset) * mult,
                    slab_height=float(base.slab_height) * mult,
                    slab_offset=float(base.slab_offset) * mult,
                    bake_block=bool(base.bake_block),
                )
            )
        return results

    @staticmethod
    def normalize(
        values: Optional[List[int]], count: int, default_val: int = 1
    ) -> List[int]:
        if not values:
            return [default_val] * max(0, count)
        if len(values) < count:
            return values + [values[-1]] * (count - len(values))
        return values[:count]


def _flatten(fs: List[Facade]):
    g, w, f, s = [], [], [], []
    for fc in fs or []:
        g.extend(fc.glasses or [])
        w.extend(fc.walls or [])
        f.extend(fc.frames or [])
        s.extend(fc.slabs or [])
    return g, w, f, s


def merge_building_breps(
    building_breps: List[geo.Brep], xy_tol: float = 0.1
) -> List[geo.Brep]:
    """층별 Brep들을 XY 중심 근접도로 묶어 한 빌딩당 하나의 Brep으로 단순 병합."""
    if not building_breps:
        return []

    # bbox 캐시
    items = [(b, b.GetBoundingBox(True)) for b in building_breps]

    # 그룹화 (그리드 버킷팅)
    buckets = {}
    inv = 1.0 / float(xy_tol)
    for i, (_, bb) in enumerate(items):
        cx = (bb.Min.X + bb.Max.X) * 0.5
        cy = (bb.Min.Y + bb.Max.Y) * 0.5
        key = (int(round(cx * inv)), int(round(cy * inv)))
        buckets.setdefault(key, []).append(i)
    groups = list(buckets.values())

    merged: List[geo.Brep] = []
    for idxs in groups:
        if len(idxs) == 1:
            merged.append(items[idxs[0]][0])
            continue

        # 그룹 Z 범위와 기준 Brep (최저 Z)
        min_z = min(items[i][1].Min.Z for i in idxs)
        max_z = max(items[i][1].Max.Z for i in idxs)
        base_i = min(idxs, key=lambda i: items[i][1].Min.Z)
        base_brep, _ = items[base_i]

        # 바닥 윤곽
        crv = utils.get_outline_from_closed_brep(base_brep, geo.Plane.WorldXY)
        h = max_z - min_z
        extr = geo.Extrusion.Create(crv, h, True)
        brep_ex = extr.ToBrep() if extr else None
        merged.append(brep_ex if isinstance(brep_ex, geo.Brep) else base_brep)

    return merged


# Grasshopper 컴포넌트 입력이 있을 때만 실행되도록 안전 가드
if __name__ == "__main__" or "building_breps" in globals():
    # 입력 수집 및 전처리
    _building_breps = globals().get("building_breps", [])
    merged_breps = merge_building_breps(_building_breps) if _building_breps else []
    base = fg.FGInputs.from_globals(globals())
    variation_factor = float(globals().get("variation_factor", 0.0))
    pattern_types = globals().get("pattern_types", None)

    # 빌딩별 FGInputs 확장 및 패턴 타입 정규화
    per_bldg_inputs = FGInputsExpander.expand(
        base, merged_breps, variation_factor=variation_factor
    )
    pattern_types = FGInputsExpander.normalize(pattern_types, len(per_bldg_inputs), 1)

    # 실행 및 결과 수집
    facades: List[Facade] = []
    parapets_all: List[geo.Brep] = []
    roof_cores_all: List[geo.Brep] = []
    for inp, ptype in zip(per_bldg_inputs, pattern_types):
        gen = fg.FacadeGenerator(inp)
        result = gen.generate(int(ptype))
        facades.extend(result.facades)
        parapets_all.extend(result.parapets or [])
        roof_cores_all.extend(result.roof_cores or [])

    # GH 출력 변수 설정
    glasses, walls, frames, slabs = _flatten(facades)
    print("Generated {} facades.".format(len(facades)))
    print(
        "Generated {} glasses, {} walls, {} frames, {} slabs.".format(
            len(glasses), len(walls), len(frames), len(slabs)
        )
    )
    print(
        "Generated {} parapets, {} roof cores.".format(
            len(parapets_all), len(roof_cores_all)
        )
    )
    parapets = parapets_all
    roof_cores = roof_cores_all
