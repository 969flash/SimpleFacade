# -*- coding:utf-8 -*-
try:
    from typing import List, Tuple, Dict, Any, Optional
except ImportError:
    pass

import Rhino.Geometry as geo  # type: ignore
import scriptcontext as sc  # type: ignore
import Rhino  # type: ignore
import utils
import facade_plan
import importlib
import random
from dataclasses import dataclass

# 모듈 새로고침
importlib.reload(utils)
importlib.reload(facade_plan)

# facade_plan에서 Facade 클래스를 가져옴
from facade_plan import Facade


@dataclass
class FGInputs:
    building_brep: geo.Brep
    floor_height: float = 3.0
    pattern_length: float = 4.0
    pattern_depth: float = 1.0
    pattern_ratio: float = 0.8
    facade_offset: float = 0.2
    slab_height: float = 0.0
    slab_offset: float = 0.0
    bake_block: bool = False

    def coerce(self) -> "FGInputs":
        # Ensure numeric types are proper Python floats/ints
        self.floor_height = float(self.floor_height)
        self.pattern_length = float(self.pattern_length)
        self.pattern_depth = float(self.pattern_depth)
        self.pattern_ratio = float(self.pattern_ratio)
        self.facade_offset = float(self.facade_offset)
        self.slab_height = float(self.slab_height)
        self.slab_offset = float(self.slab_offset)
        self.bake_block = bool(self.bake_block)
        return self

    @staticmethod
    def from_globals(globs: dict) -> "FGInputs":
        return FGInputs(
            building_brep=globs.get("building_brep", None),
            floor_height=float(globs.get("floor_height", 3.0)),
            pattern_length=float(globs.get("pattern_length", 4.0)),
            pattern_depth=float(globs.get("pattern_depth", 1.0)),
            pattern_ratio=float(globs.get("pattern_ratio", 0.8)),
            facade_offset=float(globs.get("facade_offset", 0.2)),
            slab_height=float(globs.get("slab_height", 0.0)),
            slab_offset=float(globs.get("slab_offset", 0.0)),
            bake_block=bool(globs.get("bake_block", False)),
        ).coerce()


@dataclass
class FacadeGenerationResult:
    facades: list[Facade]
    parapets: list[geo.Brep]
    roof_cores: list[geo.Brep]


####
# input : 3D Building  Brep, Facade Parameters
# output : Building Facade Breps
####


class FacadeGenerator:
    def __init__(self, inputs: FGInputs):
        inputs = inputs.coerce()
        self.inputs = inputs

        # Unpack into attributes (keeps backward compatibility inside this class)
        building_brep = inputs.building_brep
        self.building_brep = building_brep
        self.floor_height = inputs.floor_height
        self.pattern_length = inputs.pattern_length
        self.pattern_depth = inputs.pattern_depth
        self.pattern_ratio = inputs.pattern_ratio
        self.slab_height = inputs.slab_height
        self.slab_offset = inputs.slab_offset
        self.bake_block = inputs.bake_block

        self.building_curve = utils.get_outline_from_closed_brep(
            building_brep, geo.Plane.WorldXY
        )
        self.building_curve = utils.offset_regions_outward(
            self.building_curve, inputs.facade_offset
        )[0]

        self.building_height = self._get_building_height()
        # 층고 및 층수 계산 (반올림 사용). 유효하지 않은 층고 입력은 1층 처리
        if self.floor_height > 0:
            self.building_floor = max(
                1, int(round(self.building_height / self.floor_height))
            )
        else:
            self.building_floor = 1

    def _get_building_height(self) -> float:
        # 건물의 높이를 계산하는 로직을 구현합니다.
        self.building_bbox = self.building_brep.GetBoundingBox(True)
        self.base_z = self.building_bbox.Min.Z
        self.top_z = self.building_bbox.Max.Z
        return self.top_z - self.base_z

    def generate(self, pattern_type: int = 1) -> FacadeGenerationResult:
        if self.slab_height >= self.floor_height:
            raise ValueError(
                f"slab_height ({self.slab_height}) must be less than floor_height ({self.floor_height})"
            )

        building_segs = utils.explode_curve(self.building_curve)
        facade_type_obj = facade_plan.FacadeTypeRegistry.create_facade_type(
            pattern_type,
            self.pattern_length,
            self.pattern_depth,
            self.pattern_ratio,
            self.building_curve,
        )

        use_bottom_facade = self.building_floor >= 4
        bottom_floor_span = 0
        bottom_base_facade: Optional[Facade] = None
        if use_bottom_facade:
            bottom_type = random.choice(
                facade_plan.BottomFacadeTypeRegistry.get_available_types()
            )
            bottom_floor_span = random.randint(1, 2)
            bottom_floor_span = min(bottom_floor_span, self.building_floor)
            bottom_obj = facade_plan.BottomFacadeTypeRegistry.create_facade_type(
                bottom_type,
                self.pattern_length,
                self.pattern_depth * 0.5,
                self.pattern_ratio,
                self.building_curve,
            )
        else:
            bottom_obj = None

        # 기준층(0층) 파사드/슬래브를 한 번만 생성
        base_floor_height = min(self.floor_height, self.building_height)
        facade_height = base_floor_height - self.slab_height
        base_facade = self._generate_base_floor(
            building_segs, facade_type_obj, facade_height
        )
        if bottom_obj is not None:
            bottom_base_facade = self._generate_base_floor(
                building_segs, bottom_obj, facade_height
            )

        floor_facades: list[Facade] = []
        if self.bake_block:
            self._bake_facade_blocks(base_facade)
        else:
            for floor in range(self.building_floor):
                base_z = floor * self.floor_height
                if base_z >= self.building_height:
                    break
                template = base_facade
                if bottom_base_facade and floor < bottom_floor_span:
                    template = bottom_base_facade
                moved = self._move_facade(template, geo.Vector3d(0, 0, base_z))
                floor_facades.append(moved)

        parapets = self._create_parapet()
        roof_cores = self._create_rooftop_core()
        if self.bake_block:
            self._bake_rooftop_blocks(parapets, roof_cores)
            parapets = []
            roof_cores = []
        return FacadeGenerationResult(
            facades=floor_facades,
            parapets=parapets,
            roof_cores=roof_cores,
        )

    # ===== Blocks =====
    def _bake_facade_blocks(self, base_facade: Facade) -> None:
        """기준층 파사드를 블록 정의로 만들고, 각 층에 인스턴스를 배치"""
        try:
            prev_doc = sc.doc
        except Exception:
            prev_doc = None
        sc.doc = Rhino.RhinoDoc.ActiveDoc

        try:
            doc = sc.doc
            # 레이어 준비: Facade / Facade::Glass / Facade::Wall / Facade::Frame
            facade_idx, glass_idx, wall_idx, frame_idx = self._ensure_facade_layers(doc)

            # 블록 이름 결정: simple_facade_N (중복 시 N++)
            block_name = self._next_block_name(doc, base="simple_facade")

            # 블록 정의 생성 (기준층 파사드만 포함; 슬래브 제외)
            geom, attrs = self._facade_geom_attrs(
                base_facade, glass_idx, wall_idx, frame_idx
            )
            if not geom:
                return
            idefs = doc.InstanceDefinitions
            base_pt = geo.Point3d.Origin
            def_index = idefs.Add(block_name, "Simple Facade", base_pt, geom, attrs)
            if def_index < 0:
                return

            # 층별 인스턴스 배치
            for floor in range(self.building_floor):
                base_z = floor * self.floor_height
                if base_z >= self.building_height:
                    break
                xform = geo.Transform.Translation(0, 0, base_z)
                doc.Objects.AddInstanceObject(def_index, xform)
        finally:
            if prev_doc is not None:
                sc.doc = prev_doc

    def _ensure_facade_layers(self, doc) -> tuple[int, int, int, int]:
        import Rhino.DocObjects as rdo

        def find_layer_index(name: str, parent_id=None) -> int:
            for i in range(doc.Layers.Count):
                layer = doc.Layers[i]
                if layer.Name == name and (
                    parent_id is None or layer.ParentLayerId == parent_id
                ):
                    return i
            return -1

        # Facade
        facade_idx = find_layer_index("Facade")
        if facade_idx < 0:
            lay = rdo.Layer()
            lay.Name = "Facade"
            facade_idx = doc.Layers.Add(lay)
        facade_id = doc.Layers[facade_idx].Id

        # Sub layers
        def ensure_sublayer(name: str) -> int:
            idx = -1
            for i in range(doc.Layers.Count):
                layer = doc.Layers[i]
                if layer.Name == name and layer.ParentLayerId == facade_id:
                    idx = i
                    break
            if idx < 0:
                sub = rdo.Layer()
                sub.Name = name
                sub.ParentLayerId = facade_id
                idx = doc.Layers.Add(sub)
            return idx

        glass_idx = ensure_sublayer("Glass")
        wall_idx = ensure_sublayer("Wall")
        frame_idx = ensure_sublayer("Frame")
        return facade_idx, glass_idx, wall_idx, frame_idx

    def _next_block_name(self, doc, base: str = "simple_facade") -> str:
        name = f"{base}_1"
        n = 1
        while True:
            existing = doc.InstanceDefinitions.Find(name, True)
            if existing is None or existing.Index < 0:
                return name
            n += 1
            name = f"{base}_{n}"

    def _facade_geom_attrs(
        self,
        facade: Facade,
        glass_layer_idx: int,
        wall_layer_idx: int,
        frame_layer_idx: int,
    ) -> tuple[list[geo.GeometryBase], list[Rhino.DocObjects.ObjectAttributes]]:
        import Rhino.DocObjects as rdo

        geom: list[geo.GeometryBase] = []
        attrs: list[rdo.ObjectAttributes] = []

        def add_many(items, layer_idx):
            for b in items or []:
                if not b:
                    continue
                geom.append(b)
                a = rdo.ObjectAttributes()
                a.LayerIndex = layer_idx
                attrs.append(a)

        add_many(facade.glasses, glass_layer_idx)
        add_many(facade.walls, wall_layer_idx)
        add_many(facade.frames, frame_layer_idx)
        return geom, attrs

    def _generate_floor_facade(
        self, building_segs, facade_type_obj, base_z: float, facade_height: float
    ) -> dict:
        """층별 파사드를 생성하는 메서드"""
        glasses, walls, frames = [], [], []

        facade_z_offset = base_z + (self.slab_height if self.slab_height > 0 else 0)

        for seg in building_segs:
            seg_facade = facade_type_obj.generate(seg, facade_height)
            if not seg_facade:
                continue

            # Z 위치로 이동
            if facade_z_offset > 0:
                seg_facade = self._move_facade(
                    seg_facade, geo.Vector3d.ZAxis * facade_z_offset
                )

            glasses.extend(seg_facade.glasses)
            walls.extend(seg_facade.walls)
            frames.extend(seg_facade.frames)

        return {"glasses": glasses, "walls": walls, "frames": frames}

    def _generate_base_floor(
        self, building_segs, facade_type_obj, facade_height: float
    ) -> Facade:
        """기준층(0층)의 슬래브와 파사드를 한 번 생성하여 반환"""
        # 기준층 슬래브 (Z=0~slab_height)
        slabs: list[geo.Brep] = []
        if self.slab_height > 0:
            slab_brep = self._create_slab(0)
            if slab_brep:
                slabs.append(slab_brep)

        # 기준층 파사드 (슬래브 상단부터 facade_height 만큼)
        glasses, walls, frames = [], [], []
        if facade_height > 0:
            facades = self._generate_floor_facade(
                building_segs, facade_type_obj, 0, facade_height
            )
            glasses.extend(facades["glasses"])
            walls.extend(facades["walls"])
            frames.extend(facades["frames"])

        return Facade(glasses, walls, frames, slabs)

    def _move_facade(self, facade: Facade, vector: geo.Vector3d) -> Facade:
        def _mv(b: geo.Brep) -> geo.Brep:
            return utils.move_brep(b, vector)

        glasses = [_mv(b) for b in facade.glasses if b]
        walls = [_mv(b) for b in facade.walls if b]
        frames = [_mv(b) for b in facade.frames if b]
        slabs = [_mv(b) for b in facade.slabs if b]
        return Facade(glasses, walls, frames, slabs)

    def _offset_slab_curve(self) -> Optional[geo.Curve]:
        """슬래브 커브를 오프셋하는 메서드"""
        if self.slab_offset == 0:
            return self.building_curve

        if self.slab_offset < 0:
            slab_curves = utils.offset_regions_outward(
                self.building_curve, self.slab_offset
            )
        else:
            slab_curves = utils.offset_regions_inward(
                self.building_curve, self.slab_offset
            )

        if not slab_curves:
            print("Failed to offset slab curve.")
            print("slab_offset:", self.slab_offset)
            print(
                "slab_offset 값을 building_curve에 적절한 수치로 지정하거나 building_curve가 적합한 형태인지 확인하세요."
            )
            return None

        return slab_curves[0]

    def _create_slab(self, base_z: float = 0) -> Optional[geo.Brep]:
        """슬래브를 생성하는 메서드 (Z=0에서 생성, 외부에서 이동 처리)"""
        # 슬래브용 커브 생성 (slab_offset만큼 오프셋)
        slab_curve = self.building_curve
        if self.slab_offset != 0:
            slab_curve = self._offset_slab_curve()

        extruded_slab = geo.Extrusion.Create(slab_curve, self.slab_height, True)
        slab_brep_final = extruded_slab.ToBrep()

        # base_z가 0이 아닌 경우에만 이동 (하위 호환성 유지)
        if base_z != 0:
            slab_brep_final = utils.move_brep(
                slab_brep_final, geo.Vector3d.ZAxis * base_z
            )
        return slab_brep_final

    def _create_parapet(self) -> list[geo.Brep]:
        """최상층 파라펫 생성"""
        if self.building_height <= 0:
            return []
        thickness = 1.0
        parapet_height = 1.0
        inner_candidates = utils.offset_regions_inward(self.building_curve, thickness)
        if not inner_candidates:
            return []

        inner_curve = max(inner_candidates, key=lambda c: c.GetLength())

        outer_curve = self.building_curve.DuplicateCurve()
        inner_curve = inner_curve.DuplicateCurve()
        inner_curve.Reverse()

        roof_lift = self.building_height
        move_vec = geo.Vector3d(0, 0, roof_lift)
        outer_curve.Translate(move_vec)
        inner_curve.Translate(move_vec)

        planar = geo.Brep.CreatePlanarBreps([outer_curve, inner_curve])
        if not planar:
            return []

        face = planar[0].Faces[0]
        start_pt = outer_curve.PointAtStart
        end_pt = start_pt + geo.Vector3d(0, 0, parapet_height)
        path = geo.LineCurve(start_pt, end_pt)
        parapet_brep = face.CreateExtrusion(path, True)
        if not parapet_brep:
            return []

        tol = self._get_model_tolerance()
        parapet_brep.CapPlanarHoles(tol)
        return [parapet_brep]

    def _create_rooftop_core(self) -> list[geo.Brep]:
        """옥상 코어(기계실 등) 생성"""
        if self.building_height <= 0:
            return []

        core_candidates = utils.offset_regions_inward(self.building_curve, 12.0)
        if not core_candidates:
            return []
        core_curve_base = max(core_candidates, key=lambda c: c.GetLength())
        core_height = 3.0
        roof_lift = self.building_height
        core_curve = core_curve_base.DuplicateCurve()
        core_curve.Translate(geo.Vector3d(0, 0, roof_lift))

        extr = geo.Extrusion.Create(core_curve, -core_height, True)
        if not extr:
            return []

        brep = extr.ToBrep()
        return [brep] if brep else []

    def _bake_rooftop_blocks(
        self, parapets: list[geo.Brep], roof_cores: list[geo.Brep]
    ) -> None:
        if not parapets and not roof_cores:
            return

        try:
            prev_doc = sc.doc
        except Exception:
            prev_doc = None
        sc.doc = Rhino.RhinoDoc.ActiveDoc

        try:
            doc = sc.doc
            _, parapet_idx, core_idx = self._ensure_rooftop_layers(doc)
            geom, attrs = self._rooftop_geom_attrs(
                parapets, roof_cores, parapet_idx, core_idx
            )
            if not geom:
                return

            block_name = self._next_block_name(doc, base="rooftop")
            base_pt = geo.Point3d.Origin
            idefs = doc.InstanceDefinitions
            def_index = idefs.Add(block_name, "Rooftop Elements", base_pt, geom, attrs)
            if def_index < 0:
                return

            xform = geo.Transform.Identity
            doc.Objects.AddInstanceObject(def_index, xform)
        finally:
            if prev_doc is not None:
                sc.doc = prev_doc

    def _ensure_rooftop_layers(self, doc) -> tuple[int, int, int]:
        import Rhino.DocObjects as rdo

        def find_layer(name: str, parent_id=None) -> int:
            for i in range(doc.Layers.Count):
                layer = doc.Layers[i]
                if layer.Name == name and (
                    parent_id is None or layer.ParentLayerId == parent_id
                ):
                    return i
            return -1

        rooftop_idx = find_layer("Rooftop")
        if rooftop_idx < 0:
            lay = rdo.Layer()
            lay.Name = "Rooftop"
            rooftop_idx = doc.Layers.Add(lay)
        rooftop_id = doc.Layers[rooftop_idx].Id

        def ensure_child(name: str) -> int:
            idx = find_layer(name, rooftop_id)
            if idx >= 0:
                return idx
            child = rdo.Layer()
            child.Name = name
            child.ParentLayerId = rooftop_id
            return doc.Layers.Add(child)

        core_idx = ensure_child("루프탑코어")
        parapet_idx = ensure_child("파라펫")
        return rooftop_idx, parapet_idx, core_idx

    def _rooftop_geom_attrs(
        self,
        parapets: list[geo.Brep],
        roof_cores: list[geo.Brep],
        parapet_layer_idx: int,
        core_layer_idx: int,
    ) -> tuple[list[geo.GeometryBase], list[Rhino.DocObjects.ObjectAttributes]]:
        import Rhino.DocObjects as rdo

        geom: list[geo.GeometryBase] = []
        attrs: list[rdo.ObjectAttributes] = []

        def add_many(items, layer_idx):
            for b in items or []:
                if not b:
                    continue
                geom.append(b)
                attr = rdo.ObjectAttributes()
                attr.LayerIndex = layer_idx
                attrs.append(attr)

        add_many(parapets, parapet_layer_idx)
        add_many(roof_cores, core_layer_idx)
        return geom, attrs

    def _get_model_tolerance(self) -> float:
        try:
            doc = Rhino.RhinoDoc.ActiveDoc
            if doc is not None:
                return doc.ModelAbsoluteTolerance
        except Exception:
            pass
        return 0.01
