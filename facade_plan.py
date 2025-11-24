# -*- coding:utf-8 -*-
try:
    from typing import List, Tuple, Dict, Any, Optional
except ImportError:
    pass

import Rhino.Geometry as geo  # type: ignore
import utils
from abc import ABC, abstractmethod
import math


class Facade:
    """파사드 클래스 - facade_plan에서 정의하여 순환 참조 방지"""

    def __init__(
        self,
        glasses: list[geo.Brep],
        walls: list[geo.Brep],
        frames: list[geo.Brep] = None,
        slabs: list[geo.Brep] = None,
    ) -> None:
        self.glasses = glasses
        self.walls = walls
        self.frames = frames if frames is not None else []
        self.slabs = slabs if slabs is not None else []


class BaseFacadeType(ABC):
    """파사드 타입의 베이스 클래스"""

    def __init__(
        self,
        pattern_length: float,
        pattern_depth: float,
        pattern_ratio: float,
        building_curve: geo.Curve,
    ):
        self.pattern_length = pattern_length
        self.pattern_depth = pattern_depth
        self.pattern_ratio = pattern_ratio
        self.building_curve = building_curve

    @abstractmethod
    def generate(self, seg: geo.Curve, extrude_height: float) -> Optional[Facade]:
        """파사드를 생성하는 추상 메서드"""
        pass

    def _extrude_to_facade(
        self,
        glass_segs: list[geo.Curve],
        wall_segs: list[geo.Curve],
        frame_segs: list[geo.Curve],
        extrude_height: float,
    ) -> Optional[Facade]:
        """커브 세트를 층 높이만큼 압출하여 Facade로 변환"""

        def _ext_to_brep(line: geo.Curve) -> Optional[geo.Brep]:
            ext = geo.Extrusion.Create(line, extrude_height, False)
            return ext.ToBrep() if ext else None

        glass_breps = [b for b in (_ext_to_brep(c) for c in glass_segs) if b]
        wall_breps = [b for b in (_ext_to_brep(c) for c in wall_segs) if b]
        frame_breps = [b for b in (_ext_to_brep(c) for c in frame_segs) if b]

        if not glass_breps and not wall_breps and not frame_breps:
            return None

        return Facade(glass_breps, wall_breps, frame_breps, [])


class FacadeType1(BaseFacadeType):
    """기본 파사드 - 패턴 파라미터 기반"""

    def generate(self, seg: geo.Curve, extrude_height: float) -> Optional[Facade]:
        pts_from_seg = utils.get_pts_by_length(
            seg, self.pattern_length, include_start=True
        )
        if not pts_from_seg or len(pts_from_seg) < 2:
            return None

        glass_segs: list[geo.Curve] = []
        wall_segs: list[geo.Curve] = []
        frame_segs: list[geo.Curve] = []

        for pt, next_pt in zip(pts_from_seg, pts_from_seg[1:]):
            vector = utils.get_vector_from_pts(pt, next_pt)
            if hasattr(vector, "IsZero") and vector.IsZero:
                continue
            vector.Unitize()

            div_pt = pt + vector * (self.pattern_length * self.pattern_ratio)
            out_vec = utils.get_outside_perp_vec_from_pt(div_pt, self.building_curve)

            if out_vec is not None:
                partition_pt = div_pt + out_vec * self.pattern_depth
                glass_segs.append(geo.LineCurve(pt, partition_pt))
                wall_segs.append(geo.LineCurve(partition_pt, next_pt))
            else:
                glass_segs.append(geo.LineCurve(pt, div_pt))
                wall_segs.append(geo.LineCurve(div_pt, next_pt))

        if len(pts_from_seg) >= 2:
            wall_segs.append(geo.LineCurve(pts_from_seg[-1], seg.PointAtEnd))

        return self._extrude_to_facade(
            glass_segs, wall_segs, frame_segs, extrude_height
        )


class FacadeType2(BaseFacadeType):
    """직선형 파사드"""

    def generate(self, seg: geo.Curve, extrude_height: float) -> Optional[Facade]:
        pts_from_seg = utils.get_pts_by_length(
            seg, self.pattern_length, include_start=True
        )
        if not pts_from_seg or len(pts_from_seg) < 2:
            return None

        glass_segs: list[geo.Curve] = []
        wall_segs: list[geo.Curve] = []
        frame_segs: list[geo.Curve] = []

        for pt, next_pt in zip(pts_from_seg, pts_from_seg[1:]):
            vector = utils.get_vector_from_pts(pt, next_pt)
            if hasattr(vector, "IsZero") and vector.IsZero:
                continue
            vector.Unitize()

            div_pt = pt + vector * (self.pattern_length * self.pattern_ratio)
            glass_segs.append(geo.LineCurve(pt, div_pt))
            wall_segs.append(geo.LineCurve(div_pt, next_pt))

        if len(pts_from_seg) >= 2:
            wall_segs.append(geo.LineCurve(pts_from_seg[-1], seg.PointAtEnd))

        return self._extrude_to_facade(
            glass_segs, wall_segs, frame_segs, extrude_height
        )


class FacadeType3(BaseFacadeType):
    """프레임이 있는 파사드"""

    def generate(self, seg: geo.Curve, extrude_height: float) -> Optional[Facade]:
        pts_from_seg = utils.get_pts_by_length(
            seg, self.pattern_length, include_start=True
        )
        if not pts_from_seg or len(pts_from_seg) < 2:
            return None

        glass_segs: list[geo.Curve] = []
        wall_segs: list[geo.Curve] = []
        frame_segs: list[geo.Curve] = []

        for pt, next_pt in zip(pts_from_seg, pts_from_seg[1:]):
            vector = utils.get_vector_from_pts(pt, next_pt)
            if hasattr(vector, "IsZero") and vector.IsZero:
                continue
            vector.Unitize()

            div_pt = pt + vector * (self.pattern_length * self.pattern_ratio)
            out_vec = utils.get_outside_perp_vec_from_pt(div_pt, self.building_curve)

            if out_vec is not None:
                out_mid = div_pt + out_vec * self.pattern_depth
                frame_segs.append(geo.LineCurve(div_pt, out_mid))

            glass_segs.append(geo.LineCurve(pt, div_pt))
            wall_segs.append(geo.LineCurve(div_pt, next_pt))

        if len(pts_from_seg) >= 2:
            wall_segs.append(geo.LineCurve(pts_from_seg[-1], seg.PointAtEnd))

        return self._extrude_to_facade(
            glass_segs, wall_segs, frame_segs, extrude_height
        )


class FacadeType4(BaseFacadeType):
    """지그재그 패턴 파사드"""

    def generate(self, seg: geo.Curve, extrude_height: float) -> Optional[Facade]:
        pts_from_seg = utils.get_pts_by_length(
            seg, self.pattern_length, include_start=True
        )
        if not pts_from_seg or len(pts_from_seg) < 2:
            return None

        glass_segs: list[geo.Curve] = []
        wall_segs: list[geo.Curve] = []
        frame_segs: list[geo.Curve] = []

        for i, (pt, next_pt) in enumerate(zip(pts_from_seg, pts_from_seg[1:])):
            vector = utils.get_vector_from_pts(pt, next_pt)
            if hasattr(vector, "IsZero") and vector.IsZero:
                continue
            vector.Unitize()

            div_pt = pt + vector * (self.pattern_length * self.pattern_ratio)
            out_vec = utils.get_outside_perp_vec_from_pt(div_pt, self.building_curve)

            if out_vec is not None:
                direction = 1 if i % 2 == 0 else -1
                zigzag_pt = div_pt + out_vec * (self.pattern_depth * direction)

                if direction > 0:
                    frame_end = div_pt + out_vec * (
                        self.pattern_depth * direction * 0.6
                    )
                    frame_segs.append(geo.LineCurve(div_pt, frame_end))

                glass_segs.append(geo.LineCurve(pt, zigzag_pt))
                wall_segs.append(geo.LineCurve(zigzag_pt, next_pt))
            else:
                glass_segs.append(geo.LineCurve(pt, div_pt))
                wall_segs.append(geo.LineCurve(div_pt, next_pt))

        if len(pts_from_seg) >= 2:
            wall_segs.append(geo.LineCurve(pts_from_seg[-1], seg.PointAtEnd))

        return self._extrude_to_facade(
            glass_segs, wall_segs, frame_segs, extrude_height
        )


class FacadeType5(BaseFacadeType):
    """웨이브 패턴 파사드"""

    def generate(self, seg: geo.Curve, extrude_height: float) -> Optional[Facade]:
        pts_from_seg = utils.get_pts_by_length(
            seg, self.pattern_length, include_start=True
        )
        if not pts_from_seg or len(pts_from_seg) < 2:
            return None

        glass_segs: list[geo.Curve] = []
        wall_segs: list[geo.Curve] = []
        frame_segs: list[geo.Curve] = []

        for i, (pt, next_pt) in enumerate(zip(pts_from_seg, pts_from_seg[1:])):
            vector = utils.get_vector_from_pts(pt, next_pt)
            if hasattr(vector, "IsZero") and vector.IsZero:
                continue
            vector.Unitize()

            div_pt = pt + vector * (self.pattern_length * self.pattern_ratio)
            out_vec = utils.get_outside_perp_vec_from_pt(div_pt, self.building_curve)

            if out_vec is not None:
                wave_factor = math.sin(i * math.pi / 2)
                wave_pt = div_pt + out_vec * (self.pattern_depth * wave_factor)

                if abs(wave_factor) > 0.7:
                    frame_depth = self.pattern_depth * abs(wave_factor) * 0.5
                    frame_end = div_pt + out_vec * frame_depth
                    frame_segs.append(geo.LineCurve(div_pt, frame_end))

                glass_segs.append(geo.LineCurve(pt, wave_pt))
                wall_segs.append(geo.LineCurve(wave_pt, next_pt))
            else:
                glass_segs.append(geo.LineCurve(pt, div_pt))
                wall_segs.append(geo.LineCurve(div_pt, next_pt))

        if len(pts_from_seg) >= 2:
            wall_segs.append(geo.LineCurve(pts_from_seg[-1], seg.PointAtEnd))

        return self._extrude_to_facade(
            glass_segs, wall_segs, frame_segs, extrude_height
        )


class FacadeType6(BaseFacadeType):
    """피라미드 패턴 파사드"""

    def generate(self, seg: geo.Curve, extrude_height: float) -> Optional[Facade]:
        pts_from_seg = utils.get_pts_by_length(
            seg, self.pattern_length, include_start=True
        )
        if not pts_from_seg or len(pts_from_seg) < 2:
            return None

        glass_segs: list[geo.Curve] = []
        wall_segs: list[geo.Curve] = []
        frame_segs: list[geo.Curve] = []

        total_segments = len(pts_from_seg) - 1

        for i, (pt, next_pt) in enumerate(zip(pts_from_seg, pts_from_seg[1:])):
            vector = utils.get_vector_from_pts(pt, next_pt)
            if hasattr(vector, "IsZero") and vector.IsZero:
                continue
            vector.Unitize()

            div_pt = pt + vector * (self.pattern_length * self.pattern_ratio)
            out_vec = utils.get_outside_perp_vec_from_pt(div_pt, self.building_curve)

            if out_vec is not None:
                center_distance = abs(i - total_segments / 2) / (total_segments / 2)
                pyramid_factor = 1.0 - center_distance
                pyramid_pt = div_pt + out_vec * (self.pattern_depth * pyramid_factor)

                if pyramid_factor > 0.3:
                    frame_end = div_pt + out_vec * (
                        self.pattern_depth * pyramid_factor * 0.5
                    )
                    frame_segs.append(geo.LineCurve(div_pt, frame_end))

                glass_segs.append(geo.LineCurve(pt, pyramid_pt))
                wall_segs.append(geo.LineCurve(pyramid_pt, next_pt))
            else:
                glass_segs.append(geo.LineCurve(pt, div_pt))
                wall_segs.append(geo.LineCurve(div_pt, next_pt))

        if len(pts_from_seg) >= 2:
            wall_segs.append(geo.LineCurve(pts_from_seg[-1], seg.PointAtEnd))

        return self._extrude_to_facade(
            glass_segs, wall_segs, frame_segs, extrude_height
        )


class FacadeType7(BaseFacadeType):
    """헥사곤 패턴 파사드"""

    def generate(self, seg: geo.Curve, extrude_height: float) -> Optional[Facade]:
        pts_from_seg = utils.get_pts_by_length(
            seg, self.pattern_length * 0.5, include_start=True
        )
        if not pts_from_seg or len(pts_from_seg) < 2:
            return None

        glass_segs: list[geo.Curve] = []
        wall_segs: list[geo.Curve] = []
        frame_segs: list[geo.Curve] = []

        for i, (pt, next_pt) in enumerate(zip(pts_from_seg, pts_from_seg[1:])):
            vector = utils.get_vector_from_pts(pt, next_pt)
            if hasattr(vector, "IsZero") and vector.IsZero:
                continue
            vector.Unitize()

            div_pt = pt + vector * ((self.pattern_length * 0.5) * self.pattern_ratio)
            out_vec = utils.get_outside_perp_vec_from_pt(div_pt, self.building_curve)

            if out_vec is not None:
                hex_angle = (i % 6) * math.pi / 3
                hex_factor = math.cos(hex_angle) * 0.8 + 0.2
                hex_pt = div_pt + out_vec * (self.pattern_depth * hex_factor)

                if abs(hex_factor) > 0.5:
                    frame_end = div_pt + out_vec * (
                        self.pattern_depth * hex_factor * 0.3
                    )
                    frame_segs.append(geo.LineCurve(div_pt, frame_end))

                glass_segs.append(geo.LineCurve(pt, hex_pt))
                wall_segs.append(geo.LineCurve(hex_pt, next_pt))
            else:
                glass_segs.append(geo.LineCurve(pt, div_pt))
                wall_segs.append(geo.LineCurve(div_pt, next_pt))

        if len(pts_from_seg) >= 2:
            wall_segs.append(geo.LineCurve(pts_from_seg[-1], seg.PointAtEnd))

        return self._extrude_to_facade(
            glass_segs, wall_segs, frame_segs, extrude_height
        )


class FacadeType8(BaseFacadeType):
    """스파이럴 패턴 파사드"""

    def generate(self, seg: geo.Curve, extrude_height: float) -> Optional[Facade]:
        pts_from_seg = utils.get_pts_by_length(
            seg, self.pattern_length * 0.8, include_start=True
        )
        if not pts_from_seg or len(pts_from_seg) < 2:
            return None

        glass_segs: list[geo.Curve] = []
        wall_segs: list[geo.Curve] = []
        frame_segs: list[geo.Curve] = []

        for i, (pt, next_pt) in enumerate(zip(pts_from_seg, pts_from_seg[1:])):
            vector = utils.get_vector_from_pts(pt, next_pt)
            if hasattr(vector, "IsZero") and vector.IsZero:
                continue
            vector.Unitize()

            div_pt = pt + vector * ((self.pattern_length * 0.8) * self.pattern_ratio)
            out_vec = utils.get_outside_perp_vec_from_pt(div_pt, self.building_curve)

            if out_vec is not None:
                spiral_angle = i * math.pi / 4
                spiral_radius = (i % 8 + 1) / 8.0
                spiral_factor = math.sin(spiral_angle) * spiral_radius
                spiral_pt = div_pt + out_vec * (self.pattern_depth * spiral_factor)

                if abs(spiral_factor) > 0.4:
                    frame_depth = self.pattern_depth * abs(spiral_factor) * 0.6
                    frame_end = div_pt + out_vec * frame_depth
                    frame_segs.append(geo.LineCurve(div_pt, frame_end))

                glass_segs.append(geo.LineCurve(pt, spiral_pt))
                wall_segs.append(geo.LineCurve(spiral_pt, next_pt))
            else:
                glass_segs.append(geo.LineCurve(pt, div_pt))
                wall_segs.append(geo.LineCurve(div_pt, next_pt))

        if len(pts_from_seg) >= 2:
            wall_segs.append(geo.LineCurve(pts_from_seg[-1], seg.PointAtEnd))

        return self._extrude_to_facade(
            glass_segs, wall_segs, frame_segs, extrude_height
        )


class FacadeType9(BaseFacadeType):
    """랜덤 노이즈 패턴 파사드"""

    def generate(self, seg: geo.Curve, extrude_height: float) -> Optional[Facade]:
        pts_from_seg = utils.get_pts_by_length(
            seg, self.pattern_length * 0.6, include_start=True
        )
        if not pts_from_seg or len(pts_from_seg) < 2:
            return None

        glass_segs: list[geo.Curve] = []
        wall_segs: list[geo.Curve] = []
        frame_segs: list[geo.Curve] = []

        for i, (pt, next_pt) in enumerate(zip(pts_from_seg, pts_from_seg[1:])):
            vector = utils.get_vector_from_pts(pt, next_pt)
            if hasattr(vector, "IsZero") and vector.IsZero:
                continue
            vector.Unitize()

            div_pt = pt + vector * ((self.pattern_length * 0.6) * self.pattern_ratio)
            out_vec = utils.get_outside_perp_vec_from_pt(div_pt, self.building_curve)

            if out_vec is not None:
                noise1 = math.sin(i * 0.7) * 0.5
                noise2 = math.cos(i * 1.3) * 0.3
                noise3 = math.sin(i * 2.1) * 0.2
                noise_factor = noise1 + noise2 + noise3

                noise_pt = div_pt + out_vec * (self.pattern_depth * noise_factor)

                if abs(noise_factor) > 0.6:
                    frame_end = div_pt + out_vec * (
                        self.pattern_depth * noise_factor * 0.4
                    )
                    frame_segs.append(geo.LineCurve(div_pt, frame_end))

                glass_segs.append(geo.LineCurve(pt, noise_pt))
                wall_segs.append(geo.LineCurve(noise_pt, next_pt))
            else:
                glass_segs.append(geo.LineCurve(pt, div_pt))
                wall_segs.append(geo.LineCurve(div_pt, next_pt))

        if len(pts_from_seg) >= 2:
            wall_segs.append(geo.LineCurve(pts_from_seg[-1], seg.PointAtEnd))

        return self._extrude_to_facade(
            glass_segs, wall_segs, frame_segs, extrude_height
        )


class FacadeType10(BaseFacadeType):
    """복합 패턴 파사드"""

    def generate(self, seg: geo.Curve, extrude_height: float) -> Optional[Facade]:
        pts_from_seg = utils.get_pts_by_length(
            seg, self.pattern_length * 0.4, include_start=True
        )
        if not pts_from_seg or len(pts_from_seg) < 2:
            return None

        glass_segs: list[geo.Curve] = []
        wall_segs: list[geo.Curve] = []
        frame_segs: list[geo.Curve] = []

        total_segments = len(pts_from_seg) - 1

        for i, (pt, next_pt) in enumerate(zip(pts_from_seg, pts_from_seg[1:])):
            vector = utils.get_vector_from_pts(pt, next_pt)
            if hasattr(vector, "IsZero") and vector.IsZero:
                continue
            vector.Unitize()

            div_pt = pt + vector * ((self.pattern_length * 0.4) * self.pattern_ratio)
            out_vec = utils.get_outside_perp_vec_from_pt(div_pt, self.building_curve)

            if out_vec is not None:
                # 1. 지그재그 성분
                zigzag_factor = 1 if i % 2 == 0 else -1
                # 2. 웨이브 성분
                wave_factor = math.sin(i * math.pi / 3) * 0.7
                # 3. 피라미드 성분
                center_distance = (
                    abs(i - total_segments / 2) / (total_segments / 2)
                    if total_segments > 0
                    else 0
                )
                pyramid_factor = (1.0 - center_distance) * 0.6
                # 4. 노이즈 성분
                noise_factor = math.sin(i * 0.9) * math.cos(i * 1.7) * 0.4

                # 모든 성분 결합
                combined_factor = (
                    zigzag_factor * 0.3
                    + wave_factor * 0.3
                    + pyramid_factor * 0.2
                    + noise_factor * 0.2
                )

                complex_pt = div_pt + out_vec * (self.pattern_depth * combined_factor)

                # 복잡한 프레임 시스템
                if abs(combined_factor) > 0.5:
                    frame_end = div_pt + out_vec * (
                        self.pattern_depth * combined_factor * 0.5
                    )
                    frame_segs.append(geo.LineCurve(div_pt, frame_end))

                    if abs(combined_factor) > 0.8:
                        side_frame_end = div_pt + out_vec * (
                            self.pattern_depth * combined_factor * 0.3
                        )
                        frame_segs.append(geo.LineCurve(div_pt, side_frame_end))

                glass_segs.append(geo.LineCurve(pt, complex_pt))
                wall_segs.append(geo.LineCurve(complex_pt, next_pt))
            else:
                glass_segs.append(geo.LineCurve(pt, div_pt))
                wall_segs.append(geo.LineCurve(div_pt, next_pt))

        if len(pts_from_seg) >= 2:
            wall_segs.append(geo.LineCurve(pts_from_seg[-1], seg.PointAtEnd))

        return self._extrude_to_facade(
            glass_segs, wall_segs, frame_segs, extrude_height
        )


class BottomFacadeType1(BaseFacadeType):
    """저층부 파사드 타입 1 - 두꺼운 프레임과 긴 벽체"""

    def generate(self, seg: geo.Curve, extrude_height: float) -> Optional[Facade]:
        pts_from_seg = utils.get_pts_by_length(
            seg, self.pattern_length * 0.9, include_start=True
        )
        if not pts_from_seg or len(pts_from_seg) < 2:
            return None

        glass_segs: list[geo.Curve] = []
        wall_segs: list[geo.Curve] = []
        frame_segs: list[geo.Curve] = []

        for pt, next_pt in zip(pts_from_seg, pts_from_seg[1:]):
            vector = utils.get_vector_from_pts(pt, next_pt)
            if hasattr(vector, "IsZero") and vector.IsZero:
                continue
            vector.Unitize()

            glass_len = self.pattern_length * 0.3
            glass_pt = pt + vector * glass_len
            glass_segs.append(geo.LineCurve(pt, glass_pt))

            wall_segs.append(geo.LineCurve(glass_pt, next_pt))

            out_vec = utils.get_outside_perp_vec_from_pt(glass_pt, self.building_curve)
            if out_vec is not None:
                frame_depth = self.pattern_depth * 1.3
                frame_end = glass_pt + out_vec * frame_depth
                frame_segs.append(geo.LineCurve(glass_pt, frame_end))

        if len(pts_from_seg) >= 2:
            wall_segs.append(geo.LineCurve(pts_from_seg[-1], seg.PointAtEnd))

        return self._extrude_to_facade(
            glass_segs, wall_segs, frame_segs, extrude_height
        )


class BottomFacadeType2(BaseFacadeType):
    """저층부 파사드 타입 2 - 벽체 비중이 높은 필라 구조"""

    def generate(self, seg: geo.Curve, extrude_height: float) -> Optional[Facade]:
        pts_from_seg = utils.get_pts_by_length(
            seg, self.pattern_length, include_start=True
        )
        if not pts_from_seg or len(pts_from_seg) < 2:
            return None

        glass_segs: list[geo.Curve] = []
        wall_segs: list[geo.Curve] = []
        frame_segs: list[geo.Curve] = []

        for pt, next_pt in zip(pts_from_seg, pts_from_seg[1:]):
            vector = utils.get_vector_from_pts(pt, next_pt)
            if hasattr(vector, "IsZero") and vector.IsZero:
                continue
            vector.Unitize()

            first_div = pt + vector * (self.pattern_length * 0.2)
            second_div = pt + vector * (self.pattern_length * 0.8)

            glass_segs.append(geo.LineCurve(pt, first_div))
            wall_segs.append(geo.LineCurve(first_div, second_div))
            wall_segs.append(geo.LineCurve(second_div, next_pt))

            out_vec = utils.get_outside_perp_vec_from_pt(
                second_div, self.building_curve
            )
            if out_vec is not None:
                frame_depth = self.pattern_depth * 1.1
                frame_start = first_div - out_vec * (self.pattern_depth * 0.3)
                frame_end = second_div + out_vec * frame_depth
                frame_segs.append(geo.LineCurve(frame_start, frame_end))

        if len(pts_from_seg) >= 2:
            wall_segs.append(geo.LineCurve(pts_from_seg[-1], seg.PointAtEnd))

        return self._extrude_to_facade(
            glass_segs, wall_segs, frame_segs, extrude_height
        )


class BottomFacadeType3(BaseFacadeType):
    """저층부 파사드 타입 3 - 대칭 프레임과 짧은 창호"""

    def generate(self, seg: geo.Curve, extrude_height: float) -> Optional[Facade]:
        pts_from_seg = utils.get_pts_by_length(
            seg, self.pattern_length * 0.75, include_start=True
        )
        if not pts_from_seg or len(pts_from_seg) < 2:
            return None

        glass_segs: list[geo.Curve] = []
        wall_segs: list[geo.Curve] = []
        frame_segs: list[geo.Curve] = []

        for pt, next_pt in zip(pts_from_seg, pts_from_seg[1:]):
            vector = utils.get_vector_from_pts(pt, next_pt)
            if hasattr(vector, "IsZero") and vector.IsZero:
                continue
            vector.Unitize()

            center = pt + vector * (self.pattern_length * 0.5)
            glass_start = center - vector * (self.pattern_length * 0.15)
            glass_end = center + vector * (self.pattern_length * 0.15)

            wall_segs.append(geo.LineCurve(pt, glass_start))
            glass_segs.append(geo.LineCurve(glass_start, glass_end))
            wall_segs.append(geo.LineCurve(glass_end, next_pt))

            out_vec = utils.get_outside_perp_vec_from_pt(center, self.building_curve)
            if out_vec is not None:
                frame_depth = self.pattern_depth * 1.4
                left_frame_start = glass_start
                left_frame_end = glass_start + out_vec * frame_depth
                right_frame_start = glass_end
                right_frame_end = glass_end + out_vec * frame_depth
                frame_segs.append(geo.LineCurve(left_frame_start, left_frame_end))
                frame_segs.append(geo.LineCurve(right_frame_start, right_frame_end))

        if len(pts_from_seg) >= 2:
            wall_segs.append(geo.LineCurve(pts_from_seg[-1], seg.PointAtEnd))

        return self._extrude_to_facade(
            glass_segs, wall_segs, frame_segs, extrude_height
        )


class FacadeTypeRegistry:
    """파사드 타입 레지스트리 - 팩토리 패턴"""

    _facade_types = {
        1: FacadeType1,
        2: FacadeType2,
        3: FacadeType3,
        4: FacadeType4,
        5: FacadeType5,
        6: FacadeType6,
        7: FacadeType7,
        8: FacadeType8,
        9: FacadeType9,
        10: FacadeType10,
    }

    @classmethod
    def create_facade_type(
        cls,
        type_num: int,
        pattern_length: float,
        pattern_depth: float,
        pattern_ratio: float,
        building_curve: geo.Curve,
    ) -> Optional[BaseFacadeType]:
        """지정된 타입의 파사드 객체를 생성"""
        facade_class = cls._facade_types.get(type_num)
        if facade_class:
            return facade_class(
                pattern_length, pattern_depth, pattern_ratio, building_curve
            )
        return None

    @classmethod
    def get_available_types(cls) -> list[int]:
        """사용 가능한 파사드 타입 번호 목록을 반환"""
        return list(cls._facade_types.keys())


class BottomFacadeTypeRegistry:
    """저층부 전용 파사드 타입 레지스트리"""

    _facade_types = {
        1: BottomFacadeType1,
        2: BottomFacadeType2,
        3: BottomFacadeType3,
    }

    @classmethod
    def create_facade_type(
        cls,
        type_num: int,
        pattern_length: float,
        pattern_depth: float,
        pattern_ratio: float,
        building_curve: geo.Curve,
    ) -> Optional[BaseFacadeType]:
        facade_class = cls._facade_types.get(type_num)
        if facade_class:
            return facade_class(
                pattern_length, pattern_depth, pattern_ratio, building_curve
            )
        return None

    @classmethod
    def get_available_types(cls) -> list[int]:
        return list(cls._facade_types.keys())
