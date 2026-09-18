"""Explicit map projection and deterministic fixed road-route construction."""
from __future__ import annotations

import json
from pathlib import Path

import networkx as nx
import numpy as np
from pyproj import Transformer
from shapely.geometry import shape, Polygon
from shapely.ops import transform, unary_union

from .route import PolylineRoute

ROOT = Path(__file__).resolve().parents[1]


def load_scenario(path=ROOT / 'scenarios/tamu.json'):
    config = json.loads(Path(path).read_text())
    source = ROOT / config['map_path']
    data = json.loads(source.read_text())
    projection = Transformer.from_crs('EPSG:4326', config['projected_crs'], always_xy=True)
    geometries = [transform(projection.transform, shape(f['geometry'])) for f in data['features']]
    aoi = next(g for g in geometries if isinstance(g, Polygon))
    if not aoi.is_valid:
        raise ValueError('Input AOI polygon is invalid')
    origin = np.array(aoi.bounds[:2])
    def local(x, y, z=None):
        return np.asarray(x)-origin[0], np.asarray(y)-origin[1]
    aoi = transform(local, aoi)
    roads = [transform(local, g) for g in geometries if g.geom_type == 'LineString']
    # Exact intersections split the supplied linework; no artificial connectors.
    noded = unary_union(roads)
    graph = nx.Graph()
    lines = list(noded.geoms) if hasattr(noded, 'geoms') else [noded]
    for line in lines:
        coordinates = list(line.coords)
        for a, b in zip(coordinates, coordinates[1:]):
            length = float(np.linalg.norm(np.array(a)-b))
            if length > 1e-8:
                graph.add_edge(a, b, weight=length)
    components = list(nx.connected_components(graph))
    component = max(components, key=lambda c: graph.subgraph(c).size(weight='weight'))
    graph = graph.subgraph(component).copy()
    pair, maximum = None, -1
    for a in sorted(graph):
        distances = nx.single_source_dijkstra_path_length(graph, a)
        for b, distance in sorted(distances.items()):
            if distance > maximum:
                pair, maximum = (a, b), distance
    chain = nx.shortest_path(graph, *pair, weight='weight')
    route = PolylineRoute(tuple(chain + chain[-2::-1]))
    metadata = {'origin_utm_m': origin.tolist(), 'road_components': len(components),
                'route_vertices_m': route.vertices, 'route_length_m': route.length_m,
                'aoi_area_m2': aoi.area,
                'road_policy': config['road_route']}
    return config, aoi, roads, route, metadata
