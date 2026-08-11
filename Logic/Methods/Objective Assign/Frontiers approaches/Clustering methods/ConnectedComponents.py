"""Clustering de fronteras mediante componentes conexos de ocho vecinos."""

from __future__ import annotations

from Logic.Map.grid_geometry import GridCell, GridGeometry
from Logic.Robot.Track import CoordinateMatrix
from .ClusteringMethod import ClusteringMethod, FrontierCluster


class ConnectedComponentsClustering(ClusteringMethod):
    def __init__(self, *, min_cluster_size: int = 1) -> None:
        if min_cluster_size < 1:
            raise ValueError("min_cluster_size debe ser al menos uno")
        self.min_cluster_size = int(min_cluster_size)

    def cluster(
        self,
        frontiers: CoordinateMatrix,
        *,
        cell_size: float,
    ) -> list[FrontierCluster]:
        geometry = GridGeometry(cell_size)
        pending = {
            geometry.world_to_cell(float(point[0]), float(point[1]))
            for point in frontiers
        }
        components: list[set[GridCell]] = []
        while pending:
            seed = min(pending, key=lambda cell: (cell.row, cell.column))
            pending.remove(seed)
            component, queue = {seed}, [seed]
            while queue:
                current = queue.pop()
                for row_offset in (-1, 0, 1):
                    for column_offset in (-1, 0, 1):
                        if row_offset == 0 and column_offset == 0:
                            continue
                        neighbor = GridCell(
                            current.column + column_offset,
                            current.row + row_offset,
                        )
                        if neighbor in pending:
                            pending.remove(neighbor)
                            component.add(neighbor)
                            queue.append(neighbor)
            if len(component) >= self.min_cluster_size:
                components.append(component)

        clusters: list[FrontierCluster] = []
        for identifier, component in enumerate(components):
            cells = tuple(
                geometry.cell_to_world(cell)
                for cell in sorted(component, key=lambda item: (item.row, item.column))
            )
            centroid = (
                sum(point[0] for point in cells) / len(cells),
                sum(point[1] for point in cells) / len(cells),
            )
            representative = min(
                cells,
                key=lambda point: (
                    (point[0] - centroid[0]) ** 2
                    + (point[1] - centroid[1]) ** 2,
                    point[1],
                    point[0],
                ),
            )
            clusters.append(
                FrontierCluster(identifier, cells, centroid, representative)
            )
        return clusters
