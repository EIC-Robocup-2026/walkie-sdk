"""
Grasp - GraspNet grasp-pose service client.

Wraps the unified GraspNet server (the walkie_perception ``grasp_server`` node),
which loads one model and exposes three grasp services plus a shared
load/unload lifecycle:

  from_mask  — grasp from a YOLO segmentation mask over the live camera view
  from_cloud — grasp from a segmented object PointCloud2 supplied in the request
  pos        — "best": live crop + GraspNet + antipodal validation vs the
               supplied object cloud

All three return grasp poses in the planning frame (``base_footprint``). The
``from_cloud`` / ``pos`` clouds are plain PointCloud2 dicts — exactly what
``bot.point_cloud.get_once()`` returns — so they slot straight in.

This module works with any transport implementation (rosbridge, zenoh) and
degrades gracefully: service/validation errors surface as ``None`` (grasp
calls) or ``False`` (set_standby), never exceptions.
"""

from typing import Any, Dict, List, Optional, Sequence

from walkie_sdk.core.interfaces import ROSTransportInterface
from walkie_sdk.utils import converters
from walkie_sdk.utils.namespace import apply_namespace
from walkie_sdk.config.ros_topics import GRASP_SERVICES


def _vec3(v: Optional[Sequence[float]]) -> Dict[str, float]:
    """[x, y, z] (or None → zeros) → geometry_msgs/Vector3|Point dict."""
    if v is None:
        return {"x": 0.0, "y": 0.0, "z": 0.0}
    return {"x": float(v[0]), "y": float(v[1]), "z": float(v[2])}


def _empty_image() -> Dict[str, Any]:
    """An empty sensor_msgs/Image (no data) — signals the grasp server to use
    the bbox region at the depth/camera resolution instead of a mask."""
    return {
        "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""},
        "height": 0,
        "width": 0,
        "encoding": "mono8",
        "is_bigendian": 0,
        "step": 0,
        "data": [],
    }


class Grasp:
    """
    Controller for the unified GraspNet grasp service.

    Example:
        ```python
        # Cloud-based grasp (the natural SDK fit):
        cloud = bot.point_cloud.get_once(timeout=10.0)
        result = bot.grasp.from_cloud(cloud)
        if result and result["grasps"]:
            best = result["grasps"][0]
            print(best["position"], best["orientation"], best["score"])

        # Free / reload the GPU model between tasks:
        bot.grasp.set_standby(False)   # unload, free VRAM
        bot.grasp.set_standby(True)    # reload
        print(bot.grasp.status())
        ```
    """

    def __init__(
        self,
        transport: ROSTransportInterface,
        namespace: str = "",
    ):
        self._transport = transport
        self._namespace = namespace

    @property
    def namespace(self) -> str:
        """Current ROS namespace."""
        return self._namespace

    @namespace.setter
    def namespace(self, value: str) -> None:
        """Set ROS namespace."""
        self._namespace = value

    # ── internal helpers ─────────────────────────────────────────────

    def _svc(self, key: str) -> str:
        """Namespaced service name for a GRASP_SERVICES key."""
        return apply_namespace(GRASP_SERVICES[key], self._namespace)

    def _call(
        self, name_key: str, type_key: str, request: Dict[str, Any], timeout: float
    ) -> Optional[Dict[str, Any]]:
        """Call a grasp service and return the parsed result, or None on error."""
        try:
            response = self._transport.call_service(
                service_name=self._svc(name_key),
                service_type=GRASP_SERVICES[type_key],
                request=request,
                timeout=timeout,
            )
        except TimeoutError:
            print(f"[Grasp] {name_key} timed out after {timeout}s")
            return None
        except Exception as e:
            print(f"[Grasp] Error calling {name_key}: {e}")
            return None
        return self._parse(response)

    @staticmethod
    def _xyz(p: Dict[str, Any]) -> List[float]:
        pos = p.get("position", {})
        return [float(pos.get("x", 0.0)), float(pos.get("y", 0.0)), float(pos.get("z", 0.0))]

    @staticmethod
    def _quat(p: Dict[str, Any]) -> List[float]:
        q = p.get("orientation", {})
        return [
            float(q.get("x", 0.0)), float(q.get("y", 0.0)),
            float(q.get("z", 0.0)), float(q.get("w", 1.0)),
        ]

    @classmethod
    def _parse(cls, response: Dict[str, Any]) -> Dict[str, Any]:
        """Flatten a grasp service response into a friendly result dict.

        Combines the parallel response arrays (poses_base, approach_poses_base,
        scores, widths, heights, antipodal_scores) into one list of per-grasp
        dicts, ranked best-first as the server returned them.
        """
        poses = response.get("poses_base", {}).get("poses", []) or []
        approach = response.get("approach_poses_base", {}).get("poses", []) or []
        scores = response.get("scores", []) or []
        widths = response.get("widths", []) or []
        h_below = response.get("height_below_grasp", []) or []
        h_above = response.get("height_above_grasp", []) or []
        antipodal = response.get("antipodal_scores", []) or []  # GraspPos only

        grasps: List[Dict[str, Any]] = []
        for i, p in enumerate(poses):
            g: Dict[str, Any] = {
                "position": cls._xyz(p),
                "orientation": cls._quat(p),
                "score": float(scores[i]) if i < len(scores) else None,
                "width": float(widths[i]) if i < len(widths) else None,
                "height_below": float(h_below[i]) if i < len(h_below) else None,
                "height_above": float(h_above[i]) if i < len(h_above) else None,
            }
            if i < len(approach):
                g["approach_position"] = cls._xyz(approach[i])
            if i < len(antipodal):
                g["antipodal_score"] = float(antipodal[i])
            grasps.append(g)

        size = response.get("object_size", {})
        return {
            "success": bool(response.get("success", False)),
            "message": response.get("message", ""),
            "planning_frame": response.get("planning_frame", ""),
            "grasps": grasps,
            "object_size": [
                float(size.get("x", 0.0)),
                float(size.get("y", 0.0)),
                float(size.get("z", 0.0)),
            ],
            "object_bbox_pose": response.get("object_bbox_pose", {}),
        }

    # ── grasp services ───────────────────────────────────────────────

    def from_mask(
        self,
        mask: Optional[Dict[str, Any]] = None,
        tracker_id: int = 0,
        bbox: Optional[List[float]] = None,
        num_frames: int = 0,
        score_threshold: float = 0.0,
        max_grasps: int = 5,
        timeout: float = 10.0,
    ) -> Optional[Dict[str, Any]]:
        """
        Grasp from a YOLO segmentation mask over the live camera view.

        Args:
            mask: sensor_msgs/Image dict (mono8/16UC1). Build one from a numpy
                  array with converters.numpy_to_mono8_image(). If None, the
                  server falls back to the bbox region (bbox required then).
            tracker_id: >0 selects that label in a multi-object mask; 0 = use
                  all non-zero pixels.
            bbox: [cx, cy, w, h] fallback region used when the mask is empty.
            num_frames: frames to accumulate (0 = adaptive 1→3→5).
            score_threshold: minimum grasp score (0 = no filter).
            max_grasps: cap on returned poses.
            timeout: service timeout (seconds).

        Returns:
            Parsed result dict (see module docstring), or None on failure.
        """
        if mask is None and bbox is None:
            print("[Grasp] from_mask needs either a mask or a bbox")
            return None
        # bbox-only: send a TRULY EMPTY image (no data). A non-empty dummy mask
        # would make the node use the mask's tiny dimensions for the region and
        # build the bbox on that grid → empty region → 0 points. Empty data lets
        # the node take the real bbox path at the depth/camera resolution.
        request = {
            "mask": mask if mask is not None else _empty_image(),
            "tracker_id": int(tracker_id),
            "bbox": converters.bbox_to_bounding_box2d(bbox) if bbox is not None
            else converters.bbox_to_bounding_box2d([0, 0, 0, 0]),
            "num_frames": int(num_frames),
            "score_threshold": float(score_threshold),
            "max_grasps": int(max_grasps),
        }
        return self._call("from_mask", "from_mask_type", request, timeout)

    def from_cloud(
        self,
        cloud: Dict[str, Any],
        score_threshold: float = 0.0,
        max_grasps: int = 20,
        preferred_approach: Optional[Sequence[float]] = None,
        approach_weight: float = 0.0,
        preferred_position: Optional[Sequence[float]] = None,
        position_weight: float = 0.0,
        position_falloff_m: float = 0.0,
        preferred_width: float = 0.0,
        width_weight: float = 0.0,
        region_frac: float = 0.0,
        region_weight: float = 0.0,
        quality_weight: float = 0.0,
        timeout: float = 15.0,
    ) -> Optional[Dict[str, Any]]:
        """
        Grasp from a segmented object PointCloud2 supplied in the request.

        ``cloud`` is a PointCloud2 dict (e.g. from bot.point_cloud.get_once()).
        Its header.frame_id is the source frame; grasps come back in the
        planning frame. All ``*_weight`` default to 0 → poses are ranked purely
        by GraspNet quality (the soft preferences are an opt-in re-rank).

        Returns the parsed result dict, or None on failure.
        """
        request = {
            "cloud": cloud,
            "score_threshold": float(score_threshold),
            "max_grasps": int(max_grasps),
            "preferred_approach": _vec3(preferred_approach),
            "approach_weight": float(approach_weight),
            "preferred_position": _vec3(preferred_position),
            "position_weight": float(position_weight),
            "position_falloff_m": float(position_falloff_m),
            "preferred_width": float(preferred_width),
            "width_weight": float(width_weight),
            "region_frac": float(region_frac),
            "region_weight": float(region_weight),
            "quality_weight": float(quality_weight),
        }
        return self._call("from_cloud", "from_cloud_type", request, timeout)

    def from_pos(
        self,
        object_cloud: Dict[str, Any],
        crop_margin_m: float = 0.0,
        score_threshold: float = 0.0,
        max_grasps: int = 20,
        preferred_approach: Optional[Sequence[float]] = None,
        approach_weight: float = 0.0,
        preferred_position: Optional[Sequence[float]] = None,
        position_weight: float = 0.0,
        position_falloff_m: float = 0.0,
        preferred_width: float = 0.0,
        width_weight: float = 0.0,
        region_frac: float = 0.0,
        region_weight: float = 0.0,
        quality_weight: float = 0.0,
        antipodal_weight: float = 0.0,
        timeout: float = 15.0,
    ) -> Optional[Dict[str, Any]]:
        """
        "Best" grasp: crop the live view to the supplied object cloud's box,
        run GraspNet on that in-distribution crop, and validate each grasp's
        antipodal quality against the object cloud's surface normals.

        ``object_cloud`` is a segmented, near-complete PointCloud2 dict (its
        header.frame_id may be map/odom/camera…). With all weights 0 the server
        ranks by GraspNet quality + antipodal quality. Each returned grasp also
        carries an ``antipodal_score``.

        Returns the parsed result dict, or None on failure.
        """
        request = {
            "object_cloud": object_cloud,
            "crop_margin_m": float(crop_margin_m),
            "score_threshold": float(score_threshold),
            "max_grasps": int(max_grasps),
            "preferred_approach": _vec3(preferred_approach),
            "approach_weight": float(approach_weight),
            "preferred_position": _vec3(preferred_position),
            "position_weight": float(position_weight),
            "position_falloff_m": float(position_falloff_m),
            "preferred_width": float(preferred_width),
            "width_weight": float(width_weight),
            "region_frac": float(region_frac),
            "region_weight": float(region_weight),
            "quality_weight": float(quality_weight),
            "antipodal_weight": float(antipodal_weight),
        }
        return self._call("pos", "pos_type", request, timeout)

    # ── model lifecycle ──────────────────────────────────────────────

    def set_standby(self, load: bool, timeout: float = 30.0) -> bool:
        """
        Load (True) or unload (False) the GraspNet model on the GPU.

        Unloading frees VRAM for other tasks; loading restores it. Returns True
        if the server reported success, else False.
        """
        try:
            response = self._transport.call_service(
                service_name=self._svc("standby"),
                service_type=GRASP_SERVICES["standby_type"],
                request={"data": bool(load)},
                timeout=timeout,
            )
            ok = bool(response.get("success", False))
            if not ok:
                print(f"[Grasp] standby: {response.get('message', 'failed')}")
            return ok
        except Exception as e:
            print(f"[Grasp] Error calling standby: {e}")
            return False

    def status(self, timeout: float = 5.0) -> Optional[str]:
        """
        Query the server state (model state, VRAM, cached frames).

        Returns the status message string, or None on failure.
        """
        try:
            response = self._transport.call_service(
                service_name=self._svc("status"),
                service_type=GRASP_SERVICES["status_type"],
                request={},
                timeout=timeout,
            )
            return response.get("message", "")
        except Exception as e:
            print(f"[Grasp] Error calling status: {e}")
            return None
