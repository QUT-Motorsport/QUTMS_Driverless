import copy

import matplotlib
from matplotlib.axes import Axes
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

import rclpy
from rclpy.node import Node

from driverless_msgs.msg import DoubleMatrix

from driverless_common.common import QOS_ALL


def mypause(interval):
    backend = plt.rcParams["backend"]
    if backend in matplotlib.rcsetup.interactive_bk:
        figManager = matplotlib._pylab_helpers.Gcf.get_active()
        if figManager is not None:
            canvas = figManager.canvas
            if canvas.figure.stale:
                canvas.draw()
            canvas.start_event_loop(interval)
            return


class NodeMatrixVisualisation(Node):
    fig: Figure
    heatmap_ax: Axes
    cbar_ax: Axes

    def __init__(self):
        super().__init__("matrix_visualisation")

        self.create_subscription(DoubleMatrix, "matrix", self.matrix_callback, QOS_ALL)

        self.fig, (self.heatmap_ax, self.cbar_ax) = plt.subplots(1, 2, gridspec_kw=dict(width_ratios=[0.9, 0.1]))
        plt.show(block=False)

    def matrix_callback(self, msg: DoubleMatrix):
        matrix = np.reshape(np.array(msg.values), (msg.rows, msg.columns))
        self.heatmap_ax.clear()
        self.cbar_ax.clear()
        sns.heatmap(
            matrix,
            ax=copy.copy(self.heatmap_ax),
            cbar_ax=copy.copy(self.cbar_ax),
            vmin=-0.5,
            vmax=0.5,
            cmap="seismic",
        )
        mypause(0.001)


def main():
    rclpy.init()
    node = NodeMatrixVisualisation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
