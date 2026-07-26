"""ROS 의존성 없이 PointCloud2 buffer에서 XYZ만 안전하게 추출한다."""

from typing import Any, Iterable, Mapping, Sequence

import numpy as np


_DTYPES = {
    7: "f4",  # sensor_msgs/PointField.FLOAT32
    8: "f8",  # sensor_msgs/PointField.FLOAT64
}


def _field_value(field: Any, name: str) -> Any:
    if isinstance(field, Mapping):
        return field[name]
    return getattr(field, name)


def pointcloud2_to_xyz(
    data: bytes,
    fields: Sequence[Any],
    point_step: int,
    width: int,
    height: int = 1,
    row_step: int = 0,
    is_bigendian: bool = False,
) -> np.ndarray:
    """PointCloud2의 x/y/z field를 decode하고 NaN/Inf point를 제외한다."""

    selected = {str(_field_value(field, "name")): field for field in fields}
    specs = []
    for name in ("x", "y", "z"):
        if name not in selected:
            raise ValueError(f"PointCloud2 is missing {name!r} field")
        field = selected[name]
        datatype = int(_field_value(field, "datatype"))
        if datatype not in _DTYPES:
            raise ValueError(f"unsupported {name} datatype: {datatype}")
        specs.append((name, _DTYPES[datatype], int(_field_value(field, "offset"))))
    endian = ">" if is_bigendian else "<"
    dtype = np.dtype({
        "names": [item[0] for item in specs],
        "formats": [endian + item[1] for item in specs],
        "offsets": [item[2] for item in specs],
        "itemsize": int(point_step),
    })
    row_step = int(row_step) or int(width) * int(point_step)
    rows = []
    for row in range(int(height)):
        offset = row * row_step
        structured = np.frombuffer(data, dtype=dtype, count=int(width), offset=offset)
        rows.append(np.column_stack((structured["x"], structured["y"], structured["z"])))
    if not rows:
        return np.empty((0, 3), dtype=np.float64)
    points = np.concatenate(rows, axis=0).astype(np.float64, copy=False)
    return points[np.isfinite(points).all(axis=1)]
