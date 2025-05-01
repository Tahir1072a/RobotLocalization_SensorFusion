#!/usr/bin/env python3
import rclpy
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from rclpy.node import Node
from typing import Tuple

class DataVisualizer(Node):
    def __init__(self):
        super().__init__("data_visualizer")
        self.get_logger().info("Data Visualization Node Started")
        
        try:
            imu_data, odom_data = self.load_data(
                ("imu_saved_data.xlsx", "pose_saved_data.xlsx")
            )
            
            ekf_err = self.process_data(imu_data, ["time", "estimated_error_x", "estimated_error_y"])
            fused_err = self.process_data(odom_data, ["time", "estimated_error_x", "estimated_error_y"])
            
            self.plot_comparison(
                data_pairs=[
                    (ekf_err, "EKF", ["X Error", "Y Error"]),
                    (fused_err, "Fused", ["X Error", "Y Error"])
                ],
                title="Position Estimation Error Comparison",
                output_file="error_comparison.png"
            )
            
        except Exception as e:
            self.get_logger().error(f"Error in visualization: {str(e)}")
            raise

    def load_data(self, filenames: Tuple[str, str]) -> Tuple[pd.DataFrame, pd.DataFrame]:
        try:
            return (
                pd.read_excel(filenames[0]),
                pd.read_excel(filenames[1])
            )
        except FileNotFoundError as e:
            raise RuntimeError(f"Data file not found: {e.filename}") from e

    def process_data(self, df: pd.DataFrame, columns: list) -> pd.DataFrame:
        df_clean = df[columns].copy()
        df_clean["time"] -= df_clean["time"].min()  # Zamanı normalize et
        return df_clean.dropna()

    def plot_comparison(self, data_pairs: list, title: str, output_file: str = None):
        plt.figure(figsize=(12, 8))
        
        for df, label, metrics in data_pairs:
            time = df["time"].values
            for i, metric in enumerate(metrics):
                plt.plot(
                    time,
                    df.iloc[:, i+1].values,
                    label=f"{label} {metric}",
                    linestyle="--" if i%2 else "-"
                )
        
        plt.xlabel("Time (s)", fontsize=12)
        plt.ylabel("Error (m)", fontsize=12)
        plt.title(title, fontsize=14)
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        if output_file:
            plt.savefig(output_file, dpi=300, bbox_inches="tight")
            self.get_logger().info(f"Graph saved to {output_file}")
        
        plt.show()

def main():
    rclpy.init()
    visualizer = DataVisualizer()
    
    try:
        rclpy.spin_once(visualizer, timeout_sec=1)
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()