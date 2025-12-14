# offline_plot_contour2.py
#
#   This is a helper script to plot 3d points from csv data
#
__author__ = "Christian Pedrigal"
__email__ = "cpedrigal@scu.edu"
__date__ = "2025/12/12"
__version__ = "0.0.1"
__license__ = "CC BY-NC-SA 4.0"
__description__ = "This is a helper script to plot 3d points from csv data"

import argparse
import os
import shutil
import matplotlib.pyplot as plt
import numpy as np
import yaml
from random import random

from typing import Union, LiteralString, List, Set, Tuple, Dict
import pandas as pd

from pprint import pprint

import matplotlib.tri as tri
from mpl_toolkits.mplot3d import Axes3D  # enables 3D plotting
from matplotlib import cm

from scipy.interpolate import griddata
from scipy.ndimage import gaussian_filter
from mpl_toolkits.mplot3d import Axes3D

from scipy.spatial import cKDTree

FileName = Union[LiteralString, str]
Numeric = Union[int, float]
DEBUG: bool = True
VERBOSE: bool = False

class DataFrameWrapper:

    def __init__(
                self, 
                 data: Union[pd.DataFrame, FileName] = None, 
                 is_offline: bool = True,
                 number_cluster_vars: int = 6,
                 perform_smoothen: bool = False
                 ) -> None:
        
        # Attributes
        self.is_offline: bool = is_offline
        self.number_cluster_vars: int = number_cluster_vars
        self.perform_smoothen: bool = perform_smoothen

        # Create Panda Frame
        if data != None:
            self._data = self.create_panda_dataframe(data)

            self.define_col_headers()    
    
    @property
    def data(self) -> pd.DataFrame:
        return self._data
    
    @data.setter
    def data(self, data: Union[pd.DataFrame, FileName]) -> None:
        self._data = self.create_panda_dataframe(data)

        self.define_col_headers()    

    def define_col_headers(self) -> None:
        
        # For each column in the data frame
        for c in self._data.columns:

            # Create a class attribute and assign the column
            # NOTE: The .strip() method removes leading and trailing
            # whitespace.
            # NOTE: The .replace(' ', '_') replaces inner spaces with
            # underscores
            self.__setattr__(c.strip()\
                            .replace(' ', '_')\
                            .replace('/', '_')\
                            .replace('.', '')
                            , self._data[c]
                            )

        # Prints all the keys
        if VERBOSE: print(self.__dict__.keys())  


    @staticmethod
    def create_panda_dataframe(data) -> pd.DataFrame:
        
        # Get datatype
        typ = type(data)

        # TODO: Check which type of string
        if typ == str or typ == LiteralString:
            df = pd.read_csv(data)
            # df.dropna(inplace=False)
            return df
        elif typ == pd.DataFrame:
            return data
        

class FigLabels:
    def __init__(self, title: str = None, xlabel: str = None, ylabel: str = None, zlabel: str = None):
        self.title: str = title
        self.xlabel: str = xlabel
        self.ylabel: str = ylabel
        self.zlabel: str = zlabel
        
class ContourPlotter(DataFrameWrapper):

    def __init__(
                self, 
                 data: Union[pd.DataFrame, FileName]  = None
                 ) -> None:

        # Add data
        super().__init__(data)

        # Properties
        self.linewidths: float = 0.5
        self.levels: int = 15
        self.color: str = 'k'
        self.colors: str = 'blue'
        self.cmap: str = 'coolwarm'
        self.figlabels: FigLabels = FigLabels()

    def _preprocess_data(self, x = None, y = None, z = None):

        if x is not None: self._x = x
        if y is not None: self._y = y
        if z is not None: self._z = z 

        # Remove NaN
        self._x = self._x.dropna() if not isinstance(self._x, np.ndarray) else self._x[~np.isnan(self._x)]
        self._y = self._y.dropna() if not isinstance(self._y, np.ndarray) else self._y[~np.isnan(self._y)]
        self._z = self._z.dropna() if not isinstance(self._z, np.ndarray) else self._z[~np.isnan(self._z    )]

        # Match lengths as needed
        if len(self._x) != len(self._y) or \
        len(self._x) != len(self._z) or \
        len(self._y) != len(self._z):

            # Get minimum length
            size: int = min(min(len(self._x), len(self._y)),len(self._z))
            if DEBUG: print(size)

            # Match minimum length
            self._x = self._x[:size]
            self._y = self._y[:size]
            self._z = self._z[:size]


    def create_simple_3d_plot(self, x = None, y = None, z = None):

        if x is None or y is None or z is None:
            self._preprocess_data()
            x = self._x
            y = self._y
            z = self._z
        
        if not(len(x) >= 3 or len(y) >=3 or len(z) > 3):
            print("Not enough data points!")
            return
        
        plt.tricontour(x, y, z, self.levels, linewidths=self.linewidths, colors=self.color)
        plt.tricontourf(x, y, z, self.levels)
        plt.show()

    def create_another_3d_plot(self, x = None, y = None, z = None):

        if x is None or y is None or z is None:
            self._preprocess_data()
            x = self._x
            y = self._y
            z = self._z

        if not(len(x) >= 3 or len(y) >=3 or len(z) > 3):
            print("Not enough data points!")
            return

        # Create triangulation for irregular data
        triang = tri.Triangulation(x, y)

        # Plot contour lines and filled contours
        fig, ax = plt.subplots()
        contour_lines = ax.tricontour(triang, z, levels=self.levels, linewidths=self.linewidths, colors=self.color)
        contour_filled = ax.tricontourf(triang, z, levels=self.levels, cmap=self.cmap)

        # Add colorbar
        fig.colorbar(contour_filled, ax=ax)

        # Add labels
        ax.set_title(self.figlabels.title)
        ax.set_xlabel(self.figlabels.xlabel)
        ax.set_ylabel(self.figlabels.ylabel)
        ax.set_label(self.figlabels.zlabel)
        plt.show()

    def create_blurred_3d_plot(self, x=None, y=None, z=None, sigma=0.05):

        if x is None or y is None or z is None:
            self._preprocess_data()
            x = self._x
            y = self._y
            z = self._z

        if not(len(x) >= 3 or len(y) >=3 or len(z) > 3):
            print("Not enough data points!")
            return

        # Interpolate to a regular grid
        xi = np.linspace(np.min(x), np.max(x), 200)
        yi = np.linspace(np.min(y), np.max(y), 200)
        xi, yi = np.meshgrid(xi, yi)

        # Cubic interpolation for smoothness
        zi = griddata((x, y), z, (xi, yi), method='cubic')

        # Apply Gaussian filter for additional smoothing
        zi_smooth = gaussian_filter(zi, sigma=sigma)


        # Create plot
        fig, ax = plt.subplots()
        contour_lines = ax.contour(xi, yi, zi_smooth, levels=self.levels, linewidths=self.linewidths, colors=self.color)
        contour_filled = ax.contourf(xi, yi, zi_smooth, levels=self.levels, cmap=self.cmap)

        # Add colorbar
        fig.colorbar(contour_filled, ax=ax)

        # Add labels
        ax.set_title(self.figlabels.title)
        ax.set_xlabel(self.figlabels.xlabel)
        ax.set_ylabel(self.figlabels.ylabel)
        ax.set_label(self.figlabels.zlabel)
        plt.show()


    def create_3d_scatter(self, x = None, y = None, z = None):

        if x is None or y is None or z is None:
            self._preprocess_data()
            x = self._x
            y = self._y
            z = self._z

        try:

            # Create 3D figure
            fig = plt.figure()
            ax = fig.add_subplot(projection='3d')

            # Set the view: elevation=90 looks from top, azimuth=270 or 0 rotates view horizontally
            ax.view_init(elev=90, azim=270)

            # Create scatter plot
            ax.clear()
            sc = ax.scatter(x, y, z, c=z, cmap=self.cmap)

            # Add labels
            ax.set_title(self.figlabels.title)
            ax.set_xlabel(self.figlabels.xlabel)
            ax.set_ylabel(self.figlabels.ylabel)

            # Add colorbar
            cbar = fig.colorbar(sc, ax=ax)
            cbar.set_label(self.figlabels.zlabel)

            plt.show()

        except Exception as e:
            print(f"Plot error: {e}")

    def create_3d_scatter_zoh_fast(self, x=None, y=None, z=None, res=50):

        if x is None or y is None or z is None:
            self._preprocess_data()
            x = np.array(self._x)
            y = np.array(self._y)
            z = np.array(self._z)
        else:
            x = np.array(x)
            y = np.array(y)
            z = np.array(z)

        try:
            # Create grid
            xi = np.linspace(np.min(x), np.max(x), res)
            yi = np.linspace(np.min(y), np.max(y), res)
            X, Y = np.meshgrid(xi, yi)

            # Flatten grid points for KDTree query
            grid_points = np.c_[X.ravel(), Y.ravel()]

            # Build KD-Tree from original points
            tree = cKDTree(np.c_[x, y])

            # Find nearest neighbor index for each grid point
            _, idx = tree.query(grid_points)

            # Assign Z values using nearest neighbors
            Z = z[idx].reshape(X.shape)

            # Create 3D figure
            fig = plt.figure()
            ax = fig.add_subplot(projection='3d')

            # View from top
            ax.view_init(elev=90, azim=270)

            Z_norm = (Z - np.min(Z)) / (np.max(Z) - np.min(Z))

            # Plot surface with plateau values
            surf = ax.plot_surface(
                X, Y, Z,
                facecolors=cm.coolwarm(Z_norm),
                rstride=1, cstride=1,
                linewidth=0, antialiased=False
            )

            self.norm_x = X
            self.norm_y = Y
            self.norm_z = Z

            # Labels
            ax.set_title(self.figlabels.title)
            ax.set_xlabel(self.figlabels.xlabel)
            ax.set_ylabel(self.figlabels.ylabel)

            # Colorbar
            mappable = cm.ScalarMappable(cmap=cm.coolwarm)
            mappable.set_array(Z)
            cbar = fig.colorbar(mappable, ax=ax)
            cbar.set_label(self.figlabels.zlabel)

            plt.show()

        except Exception as e:
            print(f"Plot error: {e}")



def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", "-f", type=str, required=True, help="File location")
    parser.add_argument("--levels", "-l", type=int, required=False, default=6, help="Number of contour levels")
    parser.add_argument("--sigma", "-s", type=float, required=False, default=10e-03, help="Number of contour levels")
    parser.add_argument("--res", "-r", type=int, required=False, default=100, help="Number of contour levels")   
    return parser.parse_args()

    
def main() -> None:

    # Parse command-line arguments
    args = parse_args()

    # Create plotter object
    plotter: ContourPlotter = ContourPlotter(data=args.file)

    # Set x, y, and z from data columns
    try:
        plotter._x = plotter._contour_x
        plotter._y = plotter._contour_y
        plotter._z = plotter._contour_z
    except AttributeError as e:
        try:
            print("Using alternate values!")
            plotter._x = plotter._p1_pose2D_x
            plotter._y = plotter._p1_pose2D_y 
            plotter._z = plotter._p1_rssi_data 
        except:
            try:
                print("Using alternate values!")
                plotter._x = plotter.y
                plotter._y = plotter.x
                plotter._z = plotter.rssi
            except:
                print("Here are the current attributes!")
                pprint(plotter.__dict__.keys())
                return
    
    # Set levels
    plotter.levels = args.levels

    # Set figure labels
    plotter.figlabels = FigLabels("Mow the Lawn", "X (m)", "Y (m)", "RSSI (dBm)")

    # # Create plot
    # plotter.create_simple_3d_plot()

    # # Create second plot
    # plotter.create_another_3d_plot()

    # Create blutted 3d plot
    # plotter.create_blurred_3d_plot(sigma=args.sigma)

    # Create interactive 3d plot
    plotter.create_3d_scatter()

    # Create zoh of 3d plot
    plotter.create_3d_scatter_zoh_fast(res=args.res)

def christian_additional_plots():

    # Parse command-line arguments
    args = parse_args()

    file1 = "~/MYUSB/10_09_25/good_output_10_09_25_123128.csv"
    file2 = "~/MYUSB/10_09_25/good_output_10_09_25_131934.csv"
    file3 = "~/MYUSB/10_09_25/good_stay_and_spin_output_10_09_25_132720.csv"

    plotters = [ContourPlotter(data=file1), ContourPlotter(data=file2), ContourPlotter(data=file3)]

    for ind, plotter in enumerate(plotters) :
            # Set x, y, and z from data columns
        try:
            plotter._x = plotter._contour_x
            plotter._y = plotter._contour_y
            plotter._z = plotter._contour_z
        except AttributeError as e:
            try:
                print("Using alternate values!")
                plotter._x = plotter._p1_pose2D_x
                plotter._y = plotter._p1_pose2D_y 
                plotter._z = plotter._p1_rssi_data 
            except:
                try:
                    print("Using alternate values!")
                    plotter._x = plotter.y
                    plotter._y = plotter.x
                    plotter._z = plotter.rssi
                except:
                    print("Here are the current attributes!")
                    pprint(plotter.__dict__.keys())
                    return
                
        # Add Labels
        plotter.figlabels = FigLabels(f"Scalar Field Validation Test No. {ind + 1}", "X (m)", "Y (m)", "RSSI (dBM)")

        # Create interactive 3d plot
        plotter.create_3d_scatter()

        # Create zoh of 3d plot
        plotter.create_3d_scatter_zoh_fast(res=args.res)

    # Create plot of the residuals    
    plotter3 = ContourPlotter()

    # Get residuals
    x = plotters[1].norm_x
    y = plotters[1].norm_y
    z = plotters[1].norm_z - plotters[0].norm_z

    # Further preprocessing (e.g. remove NaN)
    plotter3._preprocess_data(x, y, z)

    # Plot residuals
    plotter3.figlabels = FigLabels("Difference in RSSI between 2 maps", "X (m)", "Y (m)", "RSSI (dbM)")
    plotter3.create_3d_scatter_zoh_fast(res=20)
    plotter3.create_3d_scatter()


    # Create a boxplot of the residuals
    plt.boxplot(z, vert=True, patch_artist=True, 
                boxprops=dict(facecolor='lightblue', color='blue'),
                medianprops=dict(color='red'))

    plt.title(plotter3.figlabels.title)
    plt.ylabel(plotter3.figlabels.zlabel)
    plt.xticks(rotation=90)
    plt.tight_layout()
    plt.grid()
    plt.show()

    print(z.shape)

    # Create plot of RSSI value based on orientation while staying in place
    # Identify timestamp when X, Y are approximately 0 but theta is moving rapidly



    xy_tol: float = 0.3 # m
    theta_tol: float = 0.1 # rad

    # Find timestamps when pioneer is in place
    def find_timestamps_when_in_place(x, tol):
        
        _min : int = 0
        _max : int = 0
        largest_res: int = 0

        # Range
        res: List[int] = list()

        j = _min
        while j <= len(x) - 2:

            
            print(j)

        
            # Check if value has not moved within tolerance
            if abs(x[j+1] - x[j]) <= tol:
                _max = j+1
            
            # Update largest range
            if _max - _min > largest_res:
                largest_res = _max - _min
                j += 1
            
            else:
                res.append([_min, _max])
                _min = _max
                largest_res = 0
                j = _min + 1
                # Start checking new min index
        
        return res

    print(np.array(plotters[2]._x))
    x_range = find_timestamps_when_in_place(np.array(plotters[2]._x), xy_tol)
    y_range = find_timestamps_when_in_place(np.array(plotters[2]._y), xy_tol)
    
    ind_min = max(x_range[0][0], y_range[0][0])
    ind_max = min(x_range[0][1], y_range[0][1])

    indices = np.linspace(ind_min, ind_max, ind_max - ind_min)

    # Compute IQR for RSSI
    q1 = np.percentile(np.array(plotters[2]._z)[ind_min:ind_max], 25)
    q3 = np.percentile(np.array(plotters[2]._z)[ind_min:ind_max], 75)
    print(q1, q3)

    # Create subplots
    fig, (ax1, ax2) = plt.subplots(
        2, 1, sharex=True, figsize=(8, 6)
    )

    # Top plot: Rotation
    ax1.plot(indices, np.array(plotters[2]._p1_pose2D_theta)[ind_min:ind_max])
    ax1.set_ylabel("Rotation (rad)")
    ax1.set_title("Rotation and RSSI vs Index")
    ax1.grid(True)

    # Bottom plot: RSSI
    ax2.plot(indices, np.array(plotters[2]._z)[ind_min:ind_max])
    ax2.set_ylabel("RSSI (dBm)")
    ax2.set_xlabel("Index")
    ax2.grid(True)

    ax2.fill_between(
    indices,
    q1,
    q3,
    alpha=0.3,
    label="IQR (25–75%)"
    )

    plt.tight_layout()
    plt.show()

            






if __name__ == "__main__":
    # main()
    christian_additional_plots()
    

        
