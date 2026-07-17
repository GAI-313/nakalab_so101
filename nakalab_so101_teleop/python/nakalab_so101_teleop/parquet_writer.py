"""Write LeRobot frame metadata to a parquet file."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def _load_pyarrow() -> tuple[Any, Any]:
    try:
        import pyarrow as pa
        import pyarrow.parquet as pq
    except ImportError as exc:
        raise RuntimeError(
            "pyarrow is required to write LeRobot parquet files. "
            "Install dependencies from src/nakalab_so101/requirements.txt."
        ) from exc
    return pa, pq


def write_lerobot_parquet(input_json: str, output_parquet: str) -> None:
    pa, pq = _load_pyarrow()

    input_path = Path(input_json)
    output_path = Path(output_parquet)

    with input_path.open("r", encoding="utf-8") as stream:
        rows = json.load(stream)

    schema = pa.schema(
        [
            pa.field("index", pa.int64()),
            pa.field("timestamp", pa.float64()),
            pa.field("episode_index", pa.int64()),
            pa.field("observation.state", pa.list_(pa.float32())),
            pa.field("action", pa.list_(pa.float32())),
        ]
    )

    columns = {
        "index": [row["index"] for row in rows],
        "timestamp": [row["timestamp"] for row in rows],
        "episode_index": [row["episode_index"] for row in rows],
        "observation.state": [row["observation.state"] for row in rows],
        "action": [row["action"] for row in rows],
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    table = pa.Table.from_pydict(columns, schema=schema)
    pq.write_table(table, output_path)
