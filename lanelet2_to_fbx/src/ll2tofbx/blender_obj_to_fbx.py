from __future__ import annotations

import argparse
import sys
from pathlib import Path

import bpy

try:
    from ll2tofbx.layer_names import LAYER_NAMES, OPTIONAL_LAYER_NAMES
except ImportError:  # pragma: no cover - Blender may run this file directly.
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from layer_names import LAYER_NAMES, OPTIONAL_LAYER_NAMES


def main(argv: list[str]) -> int:
    args = _parse_args(argv)
    _reset_scene()
    imported = _import_obj(Path(args.input_obj))
    _prepare_imported_meshes(imported)

    root = bpy.data.objects.new(args.root_name, None)
    bpy.context.scene.collection.objects.link(root)

    imported_by_name = {obj.name: obj for obj in imported if obj.type == "MESH"}
    for layer_name in LAYER_NAMES:
        obj = imported_by_name.get(layer_name)
        if obj is None:
            if layer_name in OPTIONAL_LAYER_NAMES:
                continue
            mesh = bpy.data.meshes.new(layer_name)
            obj = bpy.data.objects.new(layer_name, mesh)
            bpy.context.scene.collection.objects.link(obj)
        obj.name = layer_name
        obj.parent = root
        if obj.type == "MESH":
            material = bpy.data.materials.get(layer_name)
            if material is None:
                material = bpy.data.materials.new(name=layer_name)
            if not obj.data.materials:
                obj.data.materials.append(material)
            else:
                obj.data.materials[0] = material

    bpy.context.scene.unit_settings.system = "METRIC"
    bpy.context.scene.unit_settings.scale_length = 1.0
    bpy.ops.export_scene.fbx(
        filepath=str(Path(args.output_fbx)),
        use_selection=False,
        object_types={"MESH", "EMPTY"},
        axis_forward="X",
        axis_up="Z",
        mesh_smooth_type="OFF",
    )
    return 0


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-obj", required=True)
    parser.add_argument("--output-fbx", required=True)
    parser.add_argument("--root-name", required=True)
    return parser.parse_args(argv)


def _reset_scene() -> None:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for collection in (
        bpy.data.meshes,
        bpy.data.materials,
        bpy.data.images,
    ):
        for item in list(collection):
            if item.users == 0:
                collection.remove(item)


def _import_obj(path: Path):
    if hasattr(bpy.ops.wm, "obj_import"):
        bpy.ops.wm.obj_import(filepath=str(path), forward_axis="Y", up_axis="Z")
    else:  # pragma: no cover - older Blender
        bpy.ops.import_scene.obj(filepath=str(path), axis_forward="Y", axis_up="Z")
    return list(bpy.context.selected_objects)


def _prepare_imported_meshes(objects) -> None:
    for obj in objects:
        if obj.type != "MESH":
            continue
        _prepare_mesh_object(obj)


def _prepare_mesh_object(obj) -> None:
    bpy.ops.object.select_all(action="DESELECT")
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.mesh.normals_make_consistent(inside=False)
    bpy.ops.object.mode_set(mode="OBJECT")
    obj.data.update()


if __name__ == "__main__":
    arguments = sys.argv
    if "--" in arguments:
        arguments = arguments[arguments.index("--") + 1 :]
    else:
        arguments = []
    raise SystemExit(main(arguments))
