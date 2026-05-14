from __future__ import annotations

from pathlib import Path

from .mesh_types import MeshLayer


def write_obj_bundle(root_name: str, output_dir: str | Path, layers: list[MeshLayer]) -> tuple[Path, Path]:
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    obj_path = output_path / f"{root_name}.obj"
    mtl_path = output_path / f"{root_name}.mtl"

    with mtl_path.open("w", encoding="utf-8") as handle:
        for layer in layers:
            handle.write(f"newmtl {layer.name}\n")
            handle.write("Ka 0.0 0.0 0.0\n")
            handle.write("Kd 0.8 0.8 0.8\n")
            handle.write("Ks 0.0 0.0 0.0\n")
            handle.write("d 1.0\n\n")

    vertex_offset = 1
    with obj_path.open("w", encoding="utf-8") as handle:
        handle.write(f"mtllib {mtl_path.name}\n")
        for layer in layers:
            handle.write(f"\no {layer.name}\n")
            handle.write(f"usemtl {layer.name}\n")
            for vertex in layer.mesh.vertices:
                handle.write(f"v {vertex.x:.9f} {vertex.y:.9f} {vertex.z:.9f}\n")
            for a, b, c in layer.mesh.faces:
                handle.write(
                    f"f {a + vertex_offset} {b + vertex_offset} {c + vertex_offset}\n"
                )
            vertex_offset += len(layer.mesh.vertices)
    return obj_path, mtl_path
