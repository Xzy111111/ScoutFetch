"""Generate a Fortress bookstore with supported, accessible bottle stations.

Scene truth is used only to build fixtures and the fixed navigation map. The
robot still obtains its target solely from RGB YOLO detections and depth/TF.
"""

from copy import deepcopy
from functools import lru_cache
import json
import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET

import cv2
import numpy as np


RESOLUTION = 0.05
ORIGIN = (-10.0, -10.0)
SIZE = 400
SUPPORT_HEIGHT = 0.46
# Positions stay in the deliberately cleared central work area. The entire
# bookshop and perimeter furnishings remain available for manual exploration.
STATIONS = tuple((r*math.cos(a), r*math.sin(a), a)
                 for r, a in ((1.70, -5*math.pi/6), (1.90, -math.pi/2),
                              (2.10, -math.pi/6), (1.70, math.pi/6),
                              (2.00, math.pi/2), (1.90, 5*math.pi/6)))


def pose_matrix(text):
    x, y, z, roll, pitch, yaw = [float(v) for v in (text or '0 0 0 0 0 0').split()]
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    transform = np.eye(4)
    transform[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                         [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                         [-sp, cp*sr, cp*cr]]
    transform[:3, 3] = (x, y, z)
    return transform


@lru_cache(maxsize=64)
def mesh_triangles(filename):
    """Read the vendored COLLADA triangle meshes, including scene transforms."""
    tree = ET.parse(filename)
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    unit = float(tree.find('c:asset/c:unit', ns).get('meter', '1'))
    geometries = {}
    for geometry in tree.findall('.//c:geometry', ns):
        mesh = geometry.find('c:mesh', ns)
        sources = {}
        for source in mesh.findall('c:source', ns):
            values = np.fromstring(source.findtext('c:float_array', namespaces=ns), sep=' ')
            stride = int(source.find('c:technique_common/c:accessor', ns).get('stride', '3'))
            sources[source.get('id')] = values.reshape(-1, stride)[:, :3]
        vertices = {v.get('id'): v.find("c:input[@semantic='POSITION']", ns).get('source')[1:]
                    for v in mesh.findall('c:vertices', ns)}
        triangles = []
        for primitive in list(mesh):
            kind = primitive.tag.split('}')[-1]
            if kind not in ('triangles', 'polylist'):
                continue
            inputs = primitive.findall('c:input', ns)
            stride = max(int(i.get('offset', '0')) for i in inputs)+1
            vertex_input = next(i for i in inputs if i.get('semantic') == 'VERTEX')
            offset = int(vertex_input.get('offset', '0'))
            points = sources[vertices[vertex_input.get('source')[1:]]]
            indices = np.fromstring(primitive.findtext('c:p', namespaces=ns), sep=' ', dtype=int).reshape(-1, stride)[:, offset]
            counts = ([3]*(len(indices)//3) if kind == 'triangles' else
                      [int(v) for v in primitive.findtext('c:vcount', namespaces=ns).split()])
            start = 0
            for count in counts:
                for k in range(1, count-1):
                    triangles.append(points[indices[[start, start+k, start+k+1]]])
                start += count
        geometries[geometry.get('id')] = np.array(triangles)
    output = []

    def visit(node, parent):
        transform = parent.copy()
        for child in node:
            kind = child.tag.split('}')[-1]
            if kind == 'matrix':
                # COLLADA serializes these exporter matrices row by row;
                # translations occupy the last column, not the last row.
                transform = transform @ np.fromstring(child.text, sep=' ').reshape(4, 4)
            elif kind == 'translate':
                m = np.eye(4); m[:3, 3] = np.fromstring(child.text, sep=' '); transform = transform @ m
            elif kind == 'scale':
                m = np.eye(4); m[range(3), range(3)] = np.fromstring(child.text, sep=' '); transform = transform @ m
        for instance in node.findall('c:instance_geometry', ns):
            triangles = geometries[instance.get('url')[1:]]
            output.append((triangles @ transform[:3, :3].T + transform[:3, 3])*unit)
        for child in node.findall('c:node', ns):
            visit(child, transform)

    for node in tree.findall('.//c:visual_scene/c:node', ns):
        visit(node, np.eye(4))
    return np.concatenate(output) if output else np.concatenate(list(geometries.values()))*unit


def transform_triangles(triangles, matrix):
    return triangles @ matrix[:3, :3].T + matrix[:3, 3]


def model_triangles(model, parent=None, geometry_kind='collision'):
    transform = (np.eye(4) if parent is None else parent) @ pose_matrix(model.findtext('pose'))
    output = []
    for link in model.findall('link'):
        link_transform = transform @ pose_matrix(link.findtext('pose'))
        for collision in link.findall(geometry_kind):
            mesh = collision.find('geometry/mesh')
            if mesh is None:
                continue
            path = mesh.findtext('uri').removeprefix('file://')
            scale = np.fromstring(mesh.findtext('scale', '1 1 1'), sep=' ')
            output.append(transform_triangles(mesh_triangles(path)*scale,
                          link_transform @ pose_matrix(collision.findtext('pose'))))
    for child in model.findall('model'):
        output.extend(model_triangles(child, transform, geometry_kind))
    return output


def surface_below(triangles, x, y, ceiling):
    """Highest visual mesh surface below a point, using vertical ray casting."""
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    v0, v1, v2 = b[:, :2]-a[:, :2], c[:, :2]-a[:, :2], np.array([x, y])-a[:, :2]
    determinant = v0[:, 0]*v1[:, 1]-v0[:, 1]*v1[:, 0]
    valid = np.abs(determinant) > 1e-10
    u = np.divide(v2[:, 0]*v1[:, 1]-v2[:, 1]*v1[:, 0], determinant,
                  out=np.zeros_like(determinant), where=valid)
    v = np.divide(v0[:, 0]*v2[:, 1]-v0[:, 1]*v2[:, 0], determinant,
                  out=np.zeros_like(determinant), where=valid)
    z = a[:, 2]+u*(b[:, 2]-a[:, 2])+v*(c[:, 2]-a[:, 2])
    inside = valid & (u >= -1e-5) & (v >= -1e-5) & (u+v <= 1+1e-5) & (z <= ceiling)
    return float(z[inside].max()) if np.any(inside) else None


def _clip_z(polygon, z, above):
    result = []
    for a, b in zip(polygon, polygon[1:]+polygon[:1]):
        inside_a, inside_b = (a[2] >= z, b[2] >= z) if above else (a[2] <= z, b[2] <= z)
        if inside_a:
            result.append(a)
        if inside_a != inside_b:
            result.append(a + (b-a)*(z-a[2])/(b[2]-a[2]))
    return result


def _pixels(points):
    points = np.asarray(points)
    return np.round(np.column_stack(((points[:, 0]-ORIGIN[0])/RESOLUTION,
                                    SIZE-1-(points[:, 1]-ORIGIN[1])/RESOLUTION))).astype(np.int32)


def _rasterize(grid, triangles, z_min=.145, z_max=.18):
    # Match the horizontal laser's world height. This keeps AMCL consistent
    # with what the actual simulated scanner sees (rather than a table's top).
    selected = triangles[(triangles[:, :, 2].min(axis=1) <= z_max) &
                         (triangles[:, :, 2].max(axis=1) >= z_min)]
    polygons = []
    for triangle in selected:
        polygon = _clip_z(list(triangle), z_min, True)
        if polygon:
            polygon = _clip_z(polygon, z_max, False)
        if len(polygon) >= 3:
            polygons.append(_pixels(polygon))
    # Draw each projected triangle separately: a combined fillPoly call can
    # use even/odd winding and punch holes where two triangles overlap.
    for polygon in polygons:
        cv2.fillConvexPoly(grid, polygon, 0)


def base_free(grid, x, y, yaw):
    # Exact rectangular footprint plus 5 cm base margin and 5 cm nav error.
    corners = np.array([[-.375, -.30], [.375, -.30], [.375, .30], [-.375, .30]])
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    pixels = _pixels(corners @ rotation.T + (x, y))
    if pixels.min() < 0 or pixels.max() >= SIZE:
        return False
    mask = np.zeros_like(grid)
    cv2.fillConvexPoly(mask, pixels, 1)
    return bool(np.all(grid[mask.astype(bool)] == 254))


def _stand(index, x, y, yaw):
    model = ET.Element('model', name=f'pickup_table_{index}')
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = f'{x} {y} 0 0 0 {yaw}'
    link = ET.SubElement(model, 'link', name='table')
    for name, size, z, color in (
            ('pedestal', '0.42 0.45 0.38', .19, '.30 .19 .11 1'),
            ('top', '0.55 0.60 0.08', .42, '.63 .40 .20 1')):
        for kind in ('collision', 'visual'):
            element = ET.SubElement(link, kind, name=f'{name}_{kind}')
            ET.SubElement(element, 'pose').text = f'0 0 {z} 0 0 0'
            ET.SubElement(ET.SubElement(ET.SubElement(element, 'geometry'), 'box'), 'size').text = size
            if kind == 'visual':
                material = ET.SubElement(element, 'material')
                for channel in ('ambient', 'diffuse'):
                    ET.SubElement(material, channel).text = color
    return model


@lru_cache(maxsize=4)
def _bookstore_layout(assets, laboratory):
    """Import/audit static furniture once; random trials reuse an immutable template."""
    assets = Path(assets)
    source = ET.parse(assets/'bookstore_original.world').getroot().find('world')
    laboratory = ET.parse(laboratory).getroot().find('world')
    root = ET.Element('sdf', version='1.9')
    world = ET.SubElement(root, 'world', name='pick_place')
    for element in laboratory:
        if element.tag in ('physics', 'plugin', 'scene', 'light'):
            world.append(deepcopy(element))
    # Keep illumination broad enough for the RGB detector in all six directions.
    for light in source.findall('light'):
        world.append(deepcopy(light))
    grid = np.full((SIZE, SIZE), 205, dtype=np.uint8)
    cv2.rectangle(grid, tuple(_pixels([[-7.7, 7.25]])[0]),
                  tuple(_pixels([[7.7, -7.15]])[0]), 254, -1)
    navigation_grid = grid.copy()
    removed, kept, records = [], [], []
    for original in source.findall('model'):
        uri = original.findtext('include/uri')
        if not uri:
            raise ValueError('expected an included bookstore mesh model')
        asset = deepcopy(ET.parse(assets/'models'/uri[8:]/'model.sdf').getroot().find('model'))
        for element in asset.findall('.//uri'):
            if element.text.startswith('file://models/'):
                path = assets/element.text[7:]
                if not path.exists():
                    raise ValueError(f'missing bookstore asset: {path}')
                element.text = path.as_uri()
        for visual in asset.findall('.//visual'):
            meta = visual.find('meta')
            if meta is not None:
                visual.remove(meta)
        wrapper = ET.Element('model', name=original.get('name'))
        ET.SubElement(wrapper, 'static').text = 'true'
        wrapper.append(deepcopy(original.find('pose')))
        wrapper.append(asset)
        triangles = model_triangles(wrapper)
        all_points = np.concatenate(triangles).reshape(-1, 3)
        lo, hi = all_points.min(axis=0), all_points.max(axis=0)
        visual = np.concatenate(model_triangles(wrapper, geometry_kind='visual'))
        visual_points = visual.reshape(-1, 3)
        visual_lo, visual_hi = visual_points.min(axis=0), visual_points.max(axis=0)
        lo, hi = np.minimum(lo, visual_lo), np.maximum(hi, visual_hi)
        structural = any(word in uri for word in ('Floor', 'Wall', 'Column', 'Door', 'Window'))
        intersects_arena = lo[0] < 3.5 and hi[0] > -3.5 and lo[1] < 2.7 and hi[1] > -2.8
        records.append({'wrapper': wrapper, 'asset': asset, 'uri': uri,
                        'triangles': triangles, 'visual': visual,
                        'visual_lo': visual_lo, 'visual_hi': visual_hi,
                        'remove': not structural and intersects_arena and lo[2] < 1.8})

    # Props belong to their original supporting furniture, even when a tabletop
    # extends past the cleared rectangle. Removing the table must remove all of
    # its books/tablet holders together, not just props inside the rectangle.
    supports = [r for r in records if r['wrapper'].get('name').startswith(('Desk', 'Bookshelf', 'InfoDesk'))]
    support_audit, orphaned = {}, []
    for record in records:
        name = record['wrapper'].get('name')
        is_prop = name.startswith(('BookE_', 'BookD_', 'BookF_', 'Tablet', 'Holder', 'MagicCube', 'Computer', 'SmartSpeaker'))
        if not is_prop:
            continue
        lo, hi = record['visual_lo'], record['visual_hi']
        center = (lo+hi)/2
        candidates = []
        for support in supports:
            height = surface_below(support['visual'], center[0], center[1], lo[2]+.04)
            if height is not None and -.04 <= lo[2]-height <= .30:
                candidates.append((height, support))
        if candidates:
            height, support = max(candidates, key=lambda item: item[0])
            parent_name = support['wrapper'].get('name')
            support_audit[name] = {'support': parent_name, 'surface_z': height,
                                   'bottom_z': float(lo[2]), 'support_retained': not support['remove']}
            if support['remove']:
                record['remove'] = True
                orphaned.append(name)
        elif lo[2] > .08:
            record['remove'] = True
            orphaned.append(name)
            support_audit[name] = {'support': None, 'bottom_z': float(lo[2]), 'support_retained': False}

    for record in records:
        wrapper, asset, uri, triangles = (record[k] for k in ('wrapper', 'asset', 'uri', 'triangles'))
        if record['remove']:
            removed.append(wrapper.get('name'))
            continue
        if 'Floor' in uri:
            # The original rendered floor is retained. A flat contact plane
            # avoids small imported mesh offsets affecting the wheel calibration.
            for collision in asset.findall('.//collision'):
                geometry = collision.find('geometry')
                for child in list(geometry):
                    geometry.remove(child)
                ET.SubElement(ET.SubElement(geometry, 'box'), 'size').text = '15.4 14.3 0.05'
                ET.SubElement(collision, 'pose').text = '0 0 0.251618 0 0 0'
                for friction in collision.findall('surface/friction/ode/*'):
                    friction.text = '1.0'
        else:
            for mesh in triangles:
                _rasterize(grid, mesh)
                # Navigation covers the folded robot's height, including
                # tabletops/overhangs invisible to a single horizontal beam.
                _rasterize(navigation_grid, mesh, .04, .90)
        world.append(wrapper)
        kept.append(wrapper.get('name'))

    # Preserve rasterized laser-slice boundary cells as well: clipping the
    # same mesh at a different height can round edges into adjacent pixels.
    navigation_grid[grid == 0] = 0
    return root, grid, navigation_grid, kept, removed, support_audit, orphaned


def generate_bookstore(assets, laboratory, destination, seed, station=-1, include_bottle=True):
    template, static_grid, static_navigation_grid, kept, removed, support_audit, orphaned = _bookstore_layout(
        str(Path(assets).resolve()), str(Path(laboratory).resolve()))
    root = deepcopy(template)
    world = root.find('world')
    grid = static_grid.copy()
    navigation_grid = static_navigation_grid.copy()
    laboratory = ET.parse(laboratory).getroot().find('world')
    destination = Path(destination)
    directory = destination.parent

    rng = random.Random(seed)
    if station == -1:
        station = rng.randrange(len(STATIONS))
    if station not in range(len(STATIONS)):
        raise ValueError('bookstore station must be -1 or 0..5')
    for index, (x, y, angle) in enumerate(STATIONS):
        c, s = math.cos(angle), math.sin(angle)
        world.append(_stand(index, x+.12*c, y+.12*s, angle))
        corners = np.array([[-.21, -.225], [.21, -.225], [.21, .225], [-.21, .225]])
        rotation = np.array([[c, -s], [s, c]])
        cv2.fillConvexPoly(grid, _pixels(corners @ rotation.T + (x+.12*c, y+.12*s)), 0)
        # These pickup stands are intentionally approached with the chassis
        # below the tabletop; their pedestal is the base obstruction. The
        # task's arm approach is checked separately by scoutfetch_grasp.
        cv2.fillConvexPoly(navigation_grid, _pixels(corners @ rotation.T + (x+.12*c, y+.12*s)), 0)

    x, y, angle = STATIONS[station]
    c, s = math.cos(angle), math.sin(angle)
    radial, lateral = rng.uniform(-.025, .025), rng.uniform(-.035, .035)
    x, y = x+radial*c-lateral*s, y+radial*s+lateral*c
    # Validate multiple robust reachable-range approach poses, with a connected
    # collision-free corridor from the spawn, before creating the physical bottle.
    clearance = cv2.distanceTransform((navigation_grid == 254).astype(np.uint8), cv2.DIST_L2, 5)*RESOLUTION
    traversable = (clearance >= .36).astype(np.uint8)
    _, labels = cv2.connectedComponents(traversable, connectivity=8)
    start_pixel = _pixels([[0, 0]])[0]
    start_label = labels[start_pixel[1], start_pixel[0]]
    approaches = []
    for distance in (.50, .55, .60):
        bx, by = x-distance*c, y-distance*s
        pixel = _pixels([[bx, by]])[0]
        if (start_label and labels[pixel[1], pixel[0]] == start_label and
                all(base_free(navigation_grid, bx, by, angle+dyaw) for dyaw in (-.10, 0, .10))):
            approaches.append([bx, by, angle, distance])
    if not approaches or not base_free(navigation_grid, 0, 0, 0):
        raise ValueError(f'station {station} lacks a robust connected base approach')
    bottle = deepcopy(laboratory.find("model[@name='water_bottle']"))
    bottle.find('pose').text = f'{x} {y} {SUPPORT_HEIGHT} 0 0 {angle}'
    if include_bottle:
        world.append(bottle)
    fixture = ET.Element('sdf', version='1.9'); fixture.append(deepcopy(bottle))
    ET.ElementTree(fixture).write(directory/'bottle.sdf', encoding='utf-8', xml_declaration=True)
    ET.ElementTree(root).write(destination, encoding='utf-8', xml_declaration=True)
    image = directory/'bookstore.pgm'
    cv2.imwrite(str(image), grid)
    (directory/'bookstore.yaml').write_text(
        f'image: {image.name}\nmode: trinary\nresolution: {RESOLUTION}\n'
        f'origin: [{ORIGIN[0]}, {ORIGIN[1]}, 0]\nnegate: 0\n'
        'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
    nav_image = directory/'bookstore_navigation.pgm'
    cv2.imwrite(str(nav_image), navigation_grid)
    (directory/'bookstore_navigation.yaml').write_text(
        f'image: {nav_image.name}\nmode: trinary\nresolution: {RESOLUTION}\n'
        f'origin: [{ORIGIN[0]}, {ORIGIN[1]}, 0]\nnegate: 0\n'
        'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
    audit = {'seed': seed, 'station': station, 'bottle_xyz': [x, y, SUPPORT_HEIGHT],
             'support_height_m': SUPPORT_HEIGHT, 'bottle_radius_m': .032,
             'surface_edge_margin_m': min(.275-abs(radial-.12), .30-abs(lateral))-.032,
             'connected_robust_approaches': approaches, 'retained_models': kept,
             'cleared_central_models': removed,
             'removed_orphan_props': orphaned, 'decor_supports': support_audit,
             'target_source': 'RGB YOLO + aligned depth + TF; fixture truth is not published'}
    (directory/'placement_audit.json').write_text(json.dumps(audit, indent=2)+'\n')
    return audit
