from __future__ import annotations

import importlib
import unittest

from Controllers.clustering_controller import ClusteringController


ConnectedComponentsClustering = importlib.import_module(
    "Logic.Methods.Objective Assign.Frontiers approaches."
    "Clustering methods.ConnectedComponents"
).ConnectedComponentsClustering


class FrontierClusteringTests(unittest.TestCase):
    def test_eight_connected_cells_form_one_cluster(self) -> None:
        method = ConnectedComponentsClustering()
        clusters = method.cluster(
            [[0.0, 0.0], [1.0, 1.0], [2.0, 1.0]],
            cell_size=1.0,
        )
        self.assertEqual(len(clusters), 1)
        self.assertEqual(clusters[0].size, 3)
        self.assertIn(clusters[0].representative, clusters[0].cells)

    def test_disconnected_frontiers_form_distinct_clusters(self) -> None:
        clusters = ConnectedComponentsClustering().cluster(
            [[0.0, 0.0], [1.0, 0.0], [8.0, 8.0]],
            cell_size=1.0,
        )
        self.assertEqual([cluster.size for cluster in clusters], [2, 1])

    def test_controller_can_disable_or_select_clustering(self) -> None:
        controller = ClusteringController()
        self.assertIsNone(controller.create())
        controller.select("Componentes conectados")
        self.assertIsInstance(controller.create(), ConnectedComponentsClustering)


if __name__ == "__main__":
    unittest.main()
