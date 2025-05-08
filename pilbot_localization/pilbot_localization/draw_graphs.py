#!/usr/bin/env python3
import rclpy
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from rclpy.node import Node
from typing import Tuple, List, Dict, Optional

# Matplotlib stilini ve Türkçe karakter desteğini ayarlama (isteğe bağlı)
# plt.style.use('seaborn-v0_8-darkgrid') # Daha modern bir stil deneyebilirsiniz
plt.rcParams['font.family'] = ['sans-serif'] # Varsa Türkçe karakter destekleyen bir font
plt.rcParams['axes.unicode_minus'] = False # Eksi işaretlerinin düzgün görünmesi için


class DataVisualizer(Node):
    def __init__(self):
        super().__init__("data_visualizer")
        self.get_logger().info("Veri Görselleştirme Düğümü Başlatıldı")

        filename_imu = "imu_saved_data.xlsx"
        filename_pose = "pose_saved_data.xlsx"
        filename_virtual = "virtual_imu_data.xlsx"

        # Ortalama almak için DataFrame'leri saklayacak listeler
        self.all_ekf_x_errors: List[pd.DataFrame] = []
        self.all_fused_x_errors: List[pd.DataFrame] = []
        self.all_virtual_x_errors: List[pd.DataFrame] = []
        self.all_ekf_y_errors: List[pd.DataFrame] = []
        self.all_fused_y_errors: List[pd.DataFrame] = []
        self.all_virtual_y_errors: List[pd.DataFrame] = []

        num_file_sets_to_process = 10 # İşlenecek dosya seti sayısı (örn: _1.xlsx'den _10.xlsx'ye kadar)
        self.get_logger().info(f"Toplam {num_file_sets_to_process} dosya seti işlenecek ve ardından ortalamaları alınacak.")

        for i in range(1, num_file_sets_to_process + 1):
            try:
                current_imu_file = filename_imu.replace(".xlsx", f"_{i}.xlsx")
                current_pose_file = filename_pose.replace(".xlsx", f"_{i}.xlsx")
                current_virtual_file = filename_virtual.replace(".xlsx", f"_{i}.xlsx")

                self.get_logger().info(f"--- Set {i} işleniyor ---")
                # self.get_logger().info(f"IMU dosyası: {current_imu_file}")
                # self.get_logger().info(f"Pose dosyası: {current_pose_file}")
                # self.get_logger().info(f"Virtual IMU dosyası: {current_virtual_file}")

                imu_data, odom_data, virtual_data = self.load_data(
                    (current_imu_file, current_pose_file, current_virtual_file)
                )
                # self.get_logger().info(f"Veri dosyaları (set {i}) yüklenme durumu kontrol ediliyor.")

                # --- X Pozisyon Hatası İşleme ve Görselleştirme (Set i için) ---
                pos_cols_x = ["time", "estimated_error_x"]
                data_for_x_plot_set_i = []

                # EKF (IMU) X Hatası
                if imu_data is not None and all(col in imu_data.columns for col in pos_cols_x):
                    ekf_pos_err_x = self.process_data(imu_data.copy(), pos_cols_x)
                    if not ekf_pos_err_x.empty:
                        data_for_x_plot_set_i.append((ekf_pos_err_x, "EKF (IMU)", "estimated_error_x"))
                        self.all_ekf_x_errors.append(ekf_pos_err_x)
                elif imu_data is not None:
                    self.get_logger().warning(f"EKF (IMU) verisinde ({current_imu_file}) X pozisyon hatası için gerekli sütunlar bulunamadı.")

                # Fused (Odom) X Hatası
                if odom_data is not None and all(col in odom_data.columns for col in pos_cols_x):
                    fused_pos_err_x = self.process_data(odom_data.copy(), pos_cols_x)
                    if not fused_pos_err_x.empty:
                        data_for_x_plot_set_i.append((fused_pos_err_x, "Fused (Local EKF)", "estimated_error_x"))
                        self.all_fused_x_errors.append(fused_pos_err_x)
                elif odom_data is not None:
                    self.get_logger().warning(f"Fused (Local EKF) verisinde ({current_pose_file}) X pozisyon hatası için gerekli sütunlar bulunamadı.")

                # Virtual IMU X Hatası
                if virtual_data is not None and all(col in virtual_data.columns for col in pos_cols_x):
                    virtual_pos_err_x = self.process_data(virtual_data.copy(), pos_cols_x)
                    if not virtual_pos_err_x.empty:
                        data_for_x_plot_set_i.append((virtual_pos_err_x, "Virtual IMU", "estimated_error_x"))
                        self.all_virtual_x_errors.append(virtual_pos_err_x)
                elif virtual_data is not None:
                    self.get_logger().warning(f"Virtual IMU verisinde ({current_virtual_file}) X pozisyon hatası için gerekli sütunlar bulunamadı.")

                if len(data_for_x_plot_set_i) > 0:
                    self.plot_specific_error(
                        data_to_plot=data_for_x_plot_set_i,
                        title=f"X Pozisyon Tahmin Hatası Karşılaştırması - Set {i}",
                        y_label="Hata X (m)",
                        output_file=f"position_error_x_comparison_set_{i}.png"
                    )
                else:
                    self.get_logger().warning(f"Set {i} için çizilecek X pozisyon hatası verisi bulunamadı.")

                # --- Y Pozisyon Hatası İşleme ve Görselleştirme (Set i için) ---
                pos_cols_y = ["time", "estimated_error_y"]
                data_for_y_plot_set_i = []

                if imu_data is not None and all(col in imu_data.columns for col in pos_cols_y):
                    ekf_pos_err_y = self.process_data(imu_data.copy(), pos_cols_y)
                    if not ekf_pos_err_y.empty:
                        data_for_y_plot_set_i.append((ekf_pos_err_y, "EKF (IMU)", "estimated_error_y"))
                        self.all_ekf_y_errors.append(ekf_pos_err_y)
                elif imu_data is not None:
                     self.get_logger().warning(f"EKF (IMU) verisinde ({current_imu_file}) Y pozisyon hatası için gerekli sütunlar bulunamadı.")

                if odom_data is not None and all(col in odom_data.columns for col in pos_cols_y):
                    fused_pos_err_y = self.process_data(odom_data.copy(), pos_cols_y)
                    if not fused_pos_err_y.empty:
                        data_for_y_plot_set_i.append((fused_pos_err_y, "Fused (Local EKF)", "estimated_error_y"))
                        self.all_fused_y_errors.append(fused_pos_err_y)
                elif odom_data is not None:
                     self.get_logger().warning(f"Fused (Local EKF) verisinde ({current_pose_file}) Y pozisyon hatası için gerekli sütunlar bulunamadı.")
                
                if virtual_data is not None and all(col in virtual_data.columns for col in pos_cols_y):
                    virtual_pos_err_y = self.process_data(virtual_data.copy(), pos_cols_y)
                    if not virtual_pos_err_y.empty:
                        data_for_y_plot_set_i.append((virtual_pos_err_y, "Virtual IMU", "estimated_error_y"))
                        self.all_virtual_y_errors.append(virtual_pos_err_y)
                elif virtual_data is not None:
                     self.get_logger().warning(f"Virtual IMU verisinde ({current_virtual_file}) Y pozisyon hatası için gerekli sütunlar bulunamadı.")

                if len(data_for_y_plot_set_i) > 0:
                    self.plot_specific_error(
                        data_to_plot=data_for_y_plot_set_i,
                        title=f"Y Pozisyon Tahmin Hatası Karşılaştırması - Set {i}",
                        y_label="Hata Y (m)",
                        output_file=f"position_error_y_comparison_set_{i}.png"
                    )
                else:
                    self.get_logger().warning(f"Set {i} için çizilecek Y pozisyon hatası verisi bulunamadı.")

            except FileNotFoundError as e: # load_data None döndüreceği için bu direkt yakalanmayabilir
                                          # ama load_data içinde bir dosya bile eksikse uyarı loglanacak
                self.get_logger().info(f"Set {i} için bazı veri dosyaları eksik olabilir, logları kontrol edin. Bu setin bazı kısımları atlanmış olabilir.")
            except Exception as e:
                self.get_logger().error(f"Set {i} için genel bir hata oluştu: {str(e)}", exc_info=True)
            finally:
                self.get_logger().info(f"--- Set {i} işlenmesi tamamlandı ---")
        
        # --- Tüm Setlerin Ortalamasının Hesaplanması ve Çizdirilmesi ---
        self.get_logger().info("--- Tüm setlerin ortalama hataları hesaplanıyor ve çizdiriliyor ---")

        # Ortalama X Hataları
        avg_ekf_x_err = self.average_error_dataframes(self.all_ekf_x_errors, "estimated_error_x", "EKF X")
        avg_fused_x_err = self.average_error_dataframes(self.all_fused_x_errors, "estimated_error_x", "Fused X")
        avg_virtual_x_err = self.average_error_dataframes(self.all_virtual_x_errors, "estimated_error_x", "Virtual X")

        data_for_avg_x_plot = []
        if avg_ekf_x_err is not None and not avg_ekf_x_err.empty:
            data_for_avg_x_plot.append((avg_ekf_x_err, "Ort. EKF (IMU)", "estimated_error_x"))
        if avg_fused_x_err is not None and not avg_fused_x_err.empty:
            data_for_avg_x_plot.append((avg_fused_x_err, "Ort. Fused (Local EKF)", "estimated_error_x"))
        if avg_virtual_x_err is not None and not avg_virtual_x_err.empty:
            data_for_avg_x_plot.append((avg_virtual_x_err, "Ort. Virtual IMU", "estimated_error_x"))

        if len(data_for_avg_x_plot) > 0:
            self.plot_specific_error(
                data_to_plot=data_for_avg_x_plot,
                title=f"Ortalama X Pozisyon Tahmin Hatası ({len(self.all_ekf_x_errors)} set)",
                y_label="Ortalama Hata X (m)",
                output_file="average_position_error_x_comparison.png"
            )
        else:
            self.get_logger().warning("Çizilecek ortalama X pozisyon hatası verisi bulunamadı.")

        # Ortalama Y Hataları
        avg_ekf_y_err = self.average_error_dataframes(self.all_ekf_y_errors, "estimated_error_y", "EKF Y")
        avg_fused_y_err = self.average_error_dataframes(self.all_fused_y_errors, "estimated_error_y", "Fused Y")
        avg_virtual_y_err = self.average_error_dataframes(self.all_virtual_y_errors, "estimated_error_y", "Virtual Y")

        data_for_avg_y_plot = []
        if avg_ekf_y_err is not None and not avg_ekf_y_err.empty:
            data_for_avg_y_plot.append((avg_ekf_y_err, "Ort. EKF (IMU)", "estimated_error_y"))
        if avg_fused_y_err is not None and not avg_fused_y_err.empty:
            data_for_avg_y_plot.append((avg_fused_y_err, "Ort. Fused (Local EKF)", "estimated_error_y"))
        if avg_virtual_y_err is not None and not avg_virtual_y_err.empty:
            data_for_avg_y_plot.append((avg_virtual_y_err, "Ort. Virtual IMU", "estimated_error_y"))

        if len(data_for_avg_y_plot) > 0:
            self.plot_specific_error(
                data_to_plot=data_for_avg_y_plot,
                title=f"Ortalama Y Pozisyon Tahmin Hatası ({len(self.all_ekf_y_errors)} set)",
                y_label="Ortalama Hata Y (m)",
                output_file="average_position_error_y_comparison.png"
            )
        else:
            self.get_logger().warning("Çizilecek ortalama Y pozisyon hatası verisi bulunamadı.")


    def load_data(self, filenames: Tuple[str, str, str]) -> Tuple[Optional[pd.DataFrame], Optional[pd.DataFrame], Optional[pd.DataFrame]]:
        dfs = []
        for filename in filenames:
            try:
                df = pd.read_excel(filename)
                self.get_logger().info(f"'{filename}' başarıyla yüklendi.")
                dfs.append(df)
            except FileNotFoundError:
                self.get_logger().warning(f"Veri dosyası bulunamadı: {filename}. Bu dosya için None kullanılacak.")
                dfs.append(None)
            except Exception as e:
                self.get_logger().error(f"'{filename}' yüklenirken bir hata oluştu: {str(e)}. Bu dosya için None kullanılacak.")
                dfs.append(None)
        return tuple(dfs) #type: ignore


    def process_data(self, df: pd.DataFrame, columns: list) -> pd.DataFrame:
        if not all(col in df.columns for col in columns):
             missing_cols = [col for col in columns if col not in df.columns]
             self.get_logger().error(f"process_data: DataFrame'de şu sütunlar eksik: {missing_cols}")
             return pd.DataFrame(columns=columns)

        df_clean = df[columns].copy()
        if "time" in df_clean.columns and pd.api.types.is_numeric_dtype(df_clean["time"]):
            if not df_clean["time"].empty:
                 df_clean["time"] -= df_clean["time"].min()
            else:
                self.get_logger().warning("Zaman sütunu boş, normalleştirme atlanıyor.")
        elif "time" in df_clean.columns:
             self.get_logger().warning("Zaman sütunu sayısal değil, normalleştirme atlanıyor.")
        
        return df_clean.dropna()

    def average_error_dataframes(self, list_of_dfs: List[pd.DataFrame], error_column_name: str, data_label_for_log: str, num_points_common_time: int = 500) -> Optional[pd.DataFrame]:
        """ Birden fazla DataFrame'deki belirli bir hata sütununun ortalamasını alır. """
        if not list_of_dfs:
            self.get_logger().info(f"Ortalaması alınacak '{data_label_for_log}' için DataFrame listesi boş.")
            return None

        valid_dfs = []
        for df_idx, df_item in enumerate(list_of_dfs):
            if isinstance(df_item, pd.DataFrame) and not df_item.empty and \
               'time' in df_item.columns and error_column_name in df_item.columns and \
               not df_item['time'].empty and not df_item[error_column_name].empty and \
               len(df_item['time']) == len(df_item[error_column_name]):
                valid_dfs.append(df_item.sort_values(by='time')) # Zamanı sıralı ekle
            else:
                self.get_logger().warning(f"'{data_label_for_log}' için {df_idx}. DataFrame geçersiz veya eksik veri içeriyor, ortalamaya dahil edilmeyecek.")

        if not valid_dfs:
            self.get_logger().info(f"Ortalaması alınacak '{data_label_for_log}' için geçerli DataFrame bulunamadı.")
            return None

        self.get_logger().info(f"{len(valid_dfs)} adet DataFrame '{data_label_for_log}' için ortalamaya dahil edilecek.")

        max_duration = 0
        for df in valid_dfs:
            max_duration = max(max_duration, df['time'].max())
        
        if max_duration < 1e-6: # Neredeyse sıfır süre
            self.get_logger().warning(f"'{data_label_for_log}' için maksimum süre çok küçük ({max_duration}). Anlamlı ortalama alınamayabilir.")
            return None # Veya boş bir DataFrame

        common_time = np.linspace(0, max_duration, num_points_common_time)
        interpolated_errors_list = []

        for df in valid_dfs:
            interpolated = np.interp(common_time, df['time'].values, df[error_column_name].values, left=np.nan, right=np.nan)
            interpolated_errors_list.append(interpolated)

        if not interpolated_errors_list:
            self.get_logger().warning(f"'{data_label_for_log}' için interpolasyon sonrası veri yok.")
            return None
            
        stacked_errors = np.vstack(interpolated_errors_list)
        mean_errors = np.nanmean(stacked_errors, axis=0) # NaN değerleri göz ardı ederek ortalama al

        avg_df = pd.DataFrame({'time': common_time, error_column_name: mean_errors})
        avg_df.dropna(subset=[error_column_name], inplace=True) # Ortalama NaN ise o satırı kaldır

        if avg_df.empty:
            self.get_logger().warning(f"Ortalaması alınmış '{data_label_for_log}' verisi boş.")
            return None
        
        self.get_logger().info(f"'{data_label_for_log}' için ortalama başarıyla hesaplandı, {len(avg_df)} satır.")
        return avg_df

    def plot_specific_error(self, data_to_plot: list, title: str, y_label: str, output_file: str = None):
        plt.figure(figsize=(12, 8))
        plot_successful = False

        for df, label_prefix, metric_column_name in data_to_plot:
            if "time" not in df.columns or metric_column_name not in df.columns:
                self.get_logger().error(f"'{label_prefix}' için '{metric_column_name}' veya 'time' sütunu DataFrame'de bulunamadı. Çizim atlanıyor.")
                continue
            if df.empty or df[[metric_column_name, "time"]].isnull().all().all():
                self.get_logger().warning(f"'{label_prefix}' için '{metric_column_name}' veya 'time' sütununda çizilecek geçerli veri yok.")
                continue

            time_values = df["time"].values
            data_values = df[metric_column_name].values

            plt.plot(time_values, data_values, label=f"{label_prefix}", linewidth=1.5)
            plot_successful = True

        if not plot_successful:
            self.get_logger().warning(f"'{title}' başlıklı grafik için çizilecek veri bulunamadı. Grafik oluşturulmayacak.")
            plt.close() 
            return

        plt.xlabel("Zaman (s)", fontsize=12)
        plt.ylabel(y_label, fontsize=12)
        plt.title(title, fontsize=14, weight='bold')
        plt.grid(True, which='both', linestyle='--', linewidth=0.5, alpha=0.7)
        plt.legend(fontsize=10)
        plt.tight_layout()

        if output_file:
            try:
                plt.savefig(output_file, dpi=300, bbox_inches="tight")
                self.get_logger().info(f"Grafik şuraya kaydedildi: {output_file}")
            except Exception as e:
                self.get_logger().error(f"Grafik kaydedilemedi: {output_file}. Hata: {str(e)}")
        plt.show()

def main():
    rclpy.init()
    visualizer = None
    try:
        visualizer = DataVisualizer()
    except Exception as e:
         if visualizer: # Düğüm oluşturulduysa onun logger'ını kullan
              visualizer.get_logger().error(f"DataVisualizer başlatılırken/çalışırken bir hata oluştu: {str(e)}", exc_info=True)
         else: # Düğüm oluşturulamadıysa print ile fallback
              print(f"rclpy başlatılırken veya DataVisualizer oluşturulurken hata: {str(e)}")
    finally:
        if visualizer and rclpy.ok(): # Sadece düğüm başarıyla oluşturulduysa ve rclpy çalışıyorsa destroy et
            visualizer.destroy_node()
            visualizer.get_logger().info("Veri Görselleştirme Düğümü Kapatıldı.")
        if rclpy.ok():
            rclpy.shutdown()
        print("ROS Kapatıldı.")

if __name__ == "__main__":
    main()