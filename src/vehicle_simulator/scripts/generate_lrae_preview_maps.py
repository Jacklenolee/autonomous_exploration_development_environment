#!/usr/bin/env python3

import math
import os
import shutil
import struct
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh


PKG_DIR = Path(__file__).resolve().parents[1]
MESH_DIR = PKG_DIR / "mesh"
WORLD_DIR = PKG_DIR / "world"
SCENES = ("lrae_scene_1", "lrae_scene_2", "lrae_scene_3", "lrae_scene_4")

SAMPLE_RESOLUTION = 0.28
GROUND_RESOLUTION = 0.35
MAX_MESH_POINTS = 80000
MIN_MESH_POINTS = 80


def parse_values(text, count, default=0.0):
  if text is None:
    values = []
  else:
    values = [float(v) for v in text.split()]
  while len(values) < count:
    values.append(default)
  return values[:count]


def rotation_matrix(roll, pitch, yaw):
  cr, sr = math.cos(roll), math.sin(roll)
  cp, sp = math.cos(pitch), math.sin(pitch)
  cy, sy = math.cos(yaw), math.sin(yaw)

  rx = np.array([[1, 0, 0],
                 [0, cr, -sr],
                 [0, sr, cr]], dtype=float)
  ry = np.array([[cp, 0, sp],
                 [0, 1, 0],
                 [-sp, 0, cp]], dtype=float)
  rz = np.array([[cy, -sy, 0],
                 [sy, cy, 0],
                 [0, 0, 1]], dtype=float)
  return rz @ ry @ rx


def pose_matrix(text):
  x, y, z, roll, pitch, yaw = parse_values(text, 6)
  matrix = np.eye(4)
  matrix[:3, :3] = rotation_matrix(roll, pitch, yaw)
  matrix[:3, 3] = [x, y, z]
  return matrix


def apply_transform(points, matrix):
  if len(points) == 0:
    return points
  homogeneous = np.ones((len(points), 4), dtype=float)
  homogeneous[:, :3] = points
  return (homogeneous @ matrix.T)[:, :3]


def grid_2d(width, height, spacing):
  nx = max(2, int(math.ceil(width / spacing)) + 1)
  ny = max(2, int(math.ceil(height / spacing)) + 1)
  xs = np.linspace(-width / 2.0, width / 2.0, nx)
  ys = np.linspace(-height / 2.0, height / 2.0, ny)
  xv, yv = np.meshgrid(xs, ys)
  return xv.ravel(), yv.ravel()


def sample_box(size, spacing=SAMPLE_RESOLUTION):
  sx, sy, sz = size
  faces = []

  y, z = grid_2d(sy, sz, spacing)
  faces.append(np.column_stack((np.full_like(y, sx / 2.0), y, z)))
  faces.append(np.column_stack((np.full_like(y, -sx / 2.0), y, z)))

  x, z = grid_2d(sx, sz, spacing)
  faces.append(np.column_stack((x, np.full_like(x, sy / 2.0), z)))
  faces.append(np.column_stack((x, np.full_like(x, -sy / 2.0), z)))

  x, y = grid_2d(sx, sy, spacing)
  faces.append(np.column_stack((x, y, np.full_like(x, sz / 2.0))))
  faces.append(np.column_stack((x, y, np.full_like(x, -sz / 2.0))))

  return np.vstack(faces)


def sample_plane(size, spacing=GROUND_RESOLUTION):
  sx, sy = size
  x, y = grid_2d(sx, sy, spacing)
  return np.column_stack((x, y, np.zeros_like(x)))


def sample_cylinder(radius, length, spacing=SAMPLE_RESOLUTION):
  circumference = max(2.0 * math.pi * radius, spacing)
  n_theta = max(12, int(math.ceil(circumference / spacing)))
  n_z = max(2, int(math.ceil(length / spacing)) + 1)
  theta = np.linspace(0, 2.0 * math.pi, n_theta, endpoint=False)
  z = np.linspace(-length / 2.0, length / 2.0, n_z)
  tt, zz = np.meshgrid(theta, z)
  side = np.column_stack((radius * np.cos(tt).ravel(),
                          radius * np.sin(tt).ravel(),
                          zz.ravel()))

  n_r = max(2, int(math.ceil(radius / spacing)) + 1)
  r = np.linspace(0, radius, n_r)
  rr, tt = np.meshgrid(r, theta)
  disks = []
  for z_value in (-length / 2.0, length / 2.0):
    disks.append(np.column_stack((rr.ravel() * np.cos(tt).ravel(),
                                  rr.ravel() * np.sin(tt).ravel(),
                                  np.full(rr.size, z_value))))
  return np.vstack([side] + disks)


def sample_sphere(radius, spacing=SAMPLE_RESOLUTION):
  sphere = trimesh.creation.icosphere(subdivisions=3, radius=radius)
  count = max(MIN_MESH_POINTS, int(math.ceil(sphere.area / (spacing * spacing))))
  return sphere.sample(count)


def resolve_model_uri(uri):
  if uri is None:
    return None
  uri = uri.strip()
  if uri.startswith("model://"):
    relative = uri[len("model://"):]
    return MESH_DIR / relative
  if uri.startswith("file://"):
    return Path(uri[len("file://"):])
  return Path(uri)


def scene_to_mesh(loaded):
  if isinstance(loaded, trimesh.Scene):
    return loaded.to_geometry()
  return loaded


def load_mesh(mesh_path, cache_dir, cache):
  key = str(mesh_path)
  if key in cache:
    return cache[key].copy()

  suffix = mesh_path.suffix.lower()
  converted = mesh_path
  swap_dae_axes = suffix == ".dae"

  if suffix == ".dae":
    converted = cache_dir / (mesh_path.stem + ".obj")
    subprocess.run(
      ["assimp", "export", str(mesh_path), str(converted)],
      check=True,
      stdout=subprocess.DEVNULL,
      stderr=subprocess.DEVNULL,
    )

  loaded = trimesh.load(str(converted), force="scene")
  mesh = scene_to_mesh(loaded)
  if mesh is None or len(mesh.vertices) == 0:
    raise RuntimeError(f"empty mesh: {mesh_path}")

  mesh = mesh.copy()
  if swap_dae_axes:
    vertices = mesh.vertices.copy()
    mesh.vertices = vertices[:, [0, 2, 1]]

  cache[key] = mesh.copy()
  return mesh


def sample_mesh(mesh, scale, spacing=SAMPLE_RESOLUTION):
  mesh = mesh.copy()
  scale_matrix = np.diag([scale[0], scale[1], scale[2], 1.0])
  mesh.apply_transform(scale_matrix)

  area = max(float(mesh.area), spacing * spacing)
  count = int(math.ceil(area / (spacing * spacing) * 2.5))
  count = max(MIN_MESH_POINTS, min(MAX_MESH_POINTS, count))
  return mesh.sample(count)


def local_geometry_points(geometry, cache_dir, cache):
  if geometry.find("box") is not None:
    size = parse_values(geometry.findtext("box/size"), 3, 1.0)
    return sample_box(size)

  if geometry.find("plane") is not None:
    size = parse_values(geometry.findtext("plane/size"), 2, 100.0)
    return sample_plane(size)

  if geometry.find("cylinder") is not None:
    cylinder = geometry.find("cylinder")
    radius = float(cylinder.findtext("radius", "0.5"))
    length = float(cylinder.findtext("length", "1.0"))
    return sample_cylinder(radius, length)

  if geometry.find("sphere") is not None:
    radius = float(geometry.find("sphere").findtext("radius", "0.5"))
    return sample_sphere(radius)

  mesh_element = geometry.find("mesh")
  if mesh_element is not None:
    mesh_path = resolve_model_uri(mesh_element.findtext("uri"))
    if mesh_path is None or not mesh_path.exists():
      raise RuntimeError(f"missing mesh uri: {mesh_element.findtext('uri')}")
    scale = parse_values(mesh_element.findtext("scale"), 3, 1.0)
    return sample_mesh(load_mesh(mesh_path, cache_dir, cache), scale)

  return np.empty((0, 3), dtype=float)


def iter_collision_geometries(element, parent_matrix, cache_dir, cache):
  current_matrix = parent_matrix
  if element.tag in ("model", "link", "collision"):
    current_matrix = parent_matrix @ pose_matrix(element.findtext("pose"))

  if element.tag == "collision":
    geometry = element.find("geometry")
    if geometry is not None:
      geometry_matrix = current_matrix @ pose_matrix(geometry.findtext("pose"))
      points = local_geometry_points(geometry, cache_dir, cache)
      if len(points):
        yield apply_transform(points, geometry_matrix)

  for child in list(element):
    if child.tag in ("model", "link", "collision"):
      yield from iter_collision_geometries(child, current_matrix, cache_dir, cache)


def voxel_downsample(points, voxel_size=0.12):
  if len(points) == 0:
    return points

  keys = np.floor(points / voxel_size).astype(np.int64)
  _, index = np.unique(keys, axis=0, return_index=True)
  return points[np.sort(index)]


def write_binary_ply(path, points):
  path.parent.mkdir(parents=True, exist_ok=True)
  header = (
    "ply\n"
    "format binary_little_endian 1.0\n"
    f"element vertex {len(points)}\n"
    "property float x\n"
    "property float y\n"
    "property float z\n"
    "end_header\n"
  ).encode("ascii")

  with path.open("wb") as f:
    f.write(header)
    for x, y, z in points.astype(np.float32):
      f.write(struct.pack("<fff", x, y, z))


def generate_scene(scene_name, cache_dir, cache):
  world_file = WORLD_DIR / f"{scene_name}.world"
  root = ET.parse(world_file).getroot()
  world = root.find("world")
  if world is None:
    raise RuntimeError(f"missing world element in {world_file}")

  chunks = []
  for model in world.findall("model"):
    chunks.extend(iter_collision_geometries(model, np.eye(4), cache_dir, cache))

  if not chunks:
    raise RuntimeError(f"no collision geometry found in {world_file}")

  points = np.vstack(chunks)
  points = points[np.isfinite(points).all(axis=1)]
  points = voxel_downsample(points)

  output = MESH_DIR / scene_name / "preview" / "pointcloud.ply"
  write_binary_ply(output, points)
  print(f"{scene_name}: wrote {len(points)} points to {output}")


def main():
  if shutil.which("assimp") is None:
    raise RuntimeError("assimp is required to convert DAE meshes")

  np.random.seed(7)
  with tempfile.TemporaryDirectory(prefix="lrae_preview_meshes_") as temp_dir:
    cache_dir = Path(temp_dir)
    cache = {}
    for scene_name in SCENES:
      generate_scene(scene_name, cache_dir, cache)


if __name__ == "__main__":
  main()
