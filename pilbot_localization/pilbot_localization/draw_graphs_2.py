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


class AverageDataPlotterNode(Node):
    def __init__(self):
        super().__init__("average_data_plotter_50s_rmse_node") # Düğüm adını güncelleyelim
        self.get_logger().info("Sadece Ortalama Veri Çizdirme Düğümü Başlatıldı (İlk 50sn, RMSE ile)")

        filename_imu = "imu_saved_data.xlsx"
        filename_pose = "pose_saved_data.xlsx"
        filename_virtual = "virtual_imu_data.xlsx"

        self.all_ekf_x_errors: List[pd.DataFrame] = []
        self.all_fused_x_errors: List[pd.DataFrame] = []
        self.all_virtual_x_errors: List[pd.DataFrame] = []
        self.all_ekf_y_errors: List[pd.DataFrame] = []
        self.all_fused_y_errors: List[pd.DataFrame] = []
        self.all_virtual_y_errors: List[pd.DataFrame] = []

        num_file_sets_to_process = 10
        self.get_logger().info(f"Toplam {num_file_sets_to_process} dosya setinden veri okunacak ve ortalamalarının ilk 50 saniyesi RMSE ile çizdirilecek.")

        for i in range(1, num_file_sets_to_process + 1):
            try:
                current_imu_file = filename_imu.replace(".xlsx", f"_{i}.xlsx")
                current_pose_file = filename_pose.replace(".xlsx", f"_{i}.xlsx")
                current_virtual_file = filename_virtual.replace(".xlsx", f"_{i}.xlsx")

                self.get_logger().info(f"--- Set {i} verileri yükleniyor ve işleniyor ---")
                imu_data, odom_data, virtual_data = self.load_data(
                    (current_imu_file, current_pose_file, current_virtual_file)
                )

                pos_cols_x = ["time", "estimated_error_x"]
                if imu_data is not None and all(col in imu_data.columns for col in pos_cols_x):
                    ekf_pos_err_x = self.process_data(imu_data.copy(), pos_cols_x)
                    if not ekf_pos_err_x.empty: self.all_ekf_x_errors.append(ekf_pos_err_x)
                elif imu_data is not None: self.get_logger().warning(f"EKF (IMU) ({current_imu_file}) X hata sütunları eksik.")

                if odom_data is not None and all(col in odom_data.columns for col in pos_cols_x):
                    fused_pos_err_x = self.process_data(odom_data.copy(), pos_cols_x)
                    if not fused_pos_err_x.empty: self.all_fused_x_errors.append(fused_pos_err_x)
                elif odom_data is not None: self.get_logger().warning(f"Fused (Local EKF) ({current_pose_file}) X hata sütunları eksik.")

                if virtual_data is not None and all(col in virtual_data.columns for col in pos_cols_x):
                    virtual_pos_err_x = self.process_data(virtual_data.copy(), pos_cols_x)
                    if not virtual_pos_err_x.empty: self.all_virtual_x_errors.append(virtual_pos_err_x)
                elif virtual_data is not None: self.get_logger().warning(f"Virtual IMU ({current_virtual_file}) X hata sütunları eksik.")

                pos_cols_y = ["time", "estimated_error_y"]
                if imu_data is not None and all(col in imu_data.columns for col in pos_cols_y):
                    ekf_pos_err_y = self.process_data(imu_data.copy(), pos_cols_y)
                    if not ekf_pos_err_y.empty: self.all_ekf_y_errors.append(ekf_pos_err_y)
                elif imu_data is not None: self.get_logger().warning(f"EKF (IMU) ({current_imu_file}) Y hata sütunları eksik.")

                if odom_data is not None and all(col in odom_data.columns for col in pos_cols_y):
                    fused_pos_err_y = self.process_data(odom_data.copy(), pos_cols_y)
                    if not fused_pos_err_y.empty: self.all_fused_y_errors.append(fused_pos_err_y)
                elif odom_data is not None: self.get_logger().warning(f"Fused (Local EKF) ({current_pose_file}) Y hata sütunları eksik.")
                
                if virtual_data is not None and all(col in virtual_data.columns for col in pos_cols_y):
                    virtual_pos_err_y = self.process_data(virtual_data.copy(), pos_cols_y)
                    if not virtual_pos_err_y.empty: self.all_virtual_y_errors.append(virtual_pos_err_y)
                elif virtual_data is not None: self.get_logger().warning(f"Virtual IMU ({current_virtual_file}) Y hata sütunları eksik.")

            except Exception as e:
                self.get_logger().error(f"Set {i} verileri işlenirken genel bir hata oluştu: {str(e)}", exc_info=True)
        
        self.get_logger().info(f"Tüm ({num_file_sets_to_process}) set için veri okuma ve işleme tamamlandı.")
        
        time_limit_for_average_plot = 49.5
        self.get_logger().info(f"--- Tüm setlerin ortalama hatalarının İLK {time_limit_for_average_plot:.0f} SANİYESİ RMSE ile hesaplanıyor ve çizdiriliyor ---")

        # Ortalama X Hataları
        avg_ekf_x_err_full = self.average_error_dataframes(self.all_ekf_x_errors, "estimated_error_x", "EKF X")
        avg_fused_x_err_full = self.average_error_dataframes(self.all_fused_x_errors, "estimated_error_x", "Fused X (Local EKF)")
        avg_virtual_x_err_full = self.average_error_dataframes(self.all_virtual_x_errors, "estimated_error_x", "Virtual X")

        data_for_avg_x_plot_filtered = []
        if avg_ekf_x_err_full is not None and not avg_ekf_x_err_full.empty:
            avg_ekf_x_err_filtered = avg_ekf_x_err_full[avg_ekf_x_err_full['time'] <= time_limit_for_average_plot].copy()
            if not avg_ekf_x_err_filtered.empty: data_for_avg_x_plot_filtered.append((avg_ekf_x_err_filtered, "Ort. EKF (IMU)", "estimated_error_x"))

        if avg_fused_x_err_full is not None and not avg_fused_x_err_full.empty:
            avg_fused_x_err_filtered = avg_fused_x_err_full[avg_fused_x_err_full['time'] <= time_limit_for_average_plot].copy()
            if not avg_fused_x_err_filtered.empty: data_for_avg_x_plot_filtered.append((avg_fused_x_err_filtered, "Ort. Fused (Local EKF)", "estimated_error_x"))

        if avg_virtual_x_err_full is not None and not avg_virtual_x_err_full.empty:
            avg_virtual_x_err_filtered = avg_virtual_x_err_full[avg_virtual_x_err_full['time'] <= time_limit_for_average_plot].copy()
            if not avg_virtual_x_err_filtered.empty: data_for_avg_x_plot_filtered.append((avg_virtual_x_err_filtered, "Ort. Virtual IMU", "estimated_error_x"))

        num_valid_sets_for_avg_x = len(self.all_ekf_x_errors) 
        if len(data_for_avg_x_plot_filtered) > 0:
            self.plot_specific_error(
                data_to_plot=data_for_avg_x_plot_filtered,
                title=f"Ortalama X Pozisyon Tahmin Hatası (İlk {time_limit_for_average_plot:.0f}sn, ~{num_valid_sets_for_avg_x} set)",
                y_label="Ortalama Hata X (m)", # Bu etiket, grafikte sqrt(SE) çizildiğini varsayar
                output_file="ONLY_average_pos_error_x_first_50s_rmse.png"
            )
        else: self.get_logger().warning(f"Çizilecek ort. X hatası (ilk {time_limit_for_average_plot:.0f}sn) bulunamadı.")

        # Ortalama Y Hataları
        avg_ekf_y_err_full = self.average_error_dataframes(self.all_ekf_y_errors, "estimated_error_y", "EKF Y")
        avg_fused_y_err_full = self.average_error_dataframes(self.all_fused_y_errors, "estimated_error_y", "Fused Y (Local EKF)")
        avg_virtual_y_err_full = self.average_error_dataframes(self.all_virtual_y_errors, "estimated_error_y", "Virtual Y")

        data_for_avg_y_plot_filtered = []
        if avg_ekf_y_err_full is not None and not avg_ekf_y_err_full.empty:
            avg_ekf_y_err_filtered = avg_ekf_y_err_full[avg_ekf_y_err_full['time'] <= time_limit_for_average_plot].copy()
            if not avg_ekf_y_err_filtered.empty: data_for_avg_y_plot_filtered.append((avg_ekf_y_err_filtered, "Ort. EKF (IMU)", "estimated_error_y"))
        
        if avg_fused_y_err_full is not None and not avg_fused_y_err_full.empty:
            avg_fused_y_err_filtered = avg_fused_y_err_full[avg_fused_y_err_full['time'] <= time_limit_for_average_plot].copy()
            if not avg_fused_y_err_filtered.empty: data_for_avg_y_plot_filtered.append((avg_fused_y_err_filtered, "Ort. Fused (Local EKF)", "estimated_error_y"))

        if avg_virtual_y_err_full is not None and not avg_virtual_y_err_full.empty:
            avg_virtual_y_err_filtered = avg_virtual_y_err_full[avg_virtual_y_err_full['time'] <= time_limit_for_average_plot].copy()
            if not avg_virtual_y_err_filtered.empty: data_for_avg_y_plot_filtered.append((avg_virtual_y_err_filtered, "Ort. Virtual IMU", "estimated_error_y"))
        
        num_valid_sets_for_avg_y = len(self.all_ekf_y_errors)
        if len(data_for_avg_y_plot_filtered) > 0:
            self.plot_specific_error(
                data_to_plot=data_for_avg_y_plot_filtered,
                title=f"Ortalama Y Pozisyon Tahmin Hatası (İlk {time_limit_for_average_plot:.0f}sn, ~{num_valid_sets_for_avg_y} set)",
                y_label="Ortalama Hata Y (m)", # Bu etiket, grafikte sqrt(SE) çizildiğini varsayar
                output_file="ONLY_average_pos_error_y_first_50s_rmse.png"
            )
        else: self.get_logger().warning(f"Çizilecek ort. Y hatası (ilk {time_limit_for_average_plot:.0f}sn) bulunamadı.")


    def load_data(self, filenames: Tuple[str, str, str]) -> Tuple[Optional[pd.DataFrame], Optional[pd.DataFrame], Optional[pd.DataFrame]]:
        dfs = []
        for filename in filenames:
            try:
                df = pd.read_excel(filename)
                self.get_logger().debug(f"'{filename}' başarıyla yüklendi.")
                dfs.append(df)
            except FileNotFoundError:
                self.get_logger().warning(f"Veri dosyası bulunamadı: {filename}. Bu dosya için None kullanılacak.")
                dfs.append(None)
            except Exception as e:
                self.get_logger().error(f"'{filename}' yüklenirken bir hata oluştu: {str(e)}. Bu dosya için None kullanılacak.")
                dfs.append(None)
        return tuple(dfs)


    def process_data(self, df: pd.DataFrame, columns: list) -> pd.DataFrame:
        if not all(col in df.columns for col in columns):
             missing_cols = [col for col in columns if col not in df.columns]
             self.get_logger().error(f"process_data: DataFrame'de şu sütunlar eksik: {missing_cols}")
             return pd.DataFrame(columns=columns)

        df_clean = df[columns].copy()
        if "time" in df_clean.columns and pd.api.types.is_numeric_dtype(df_clean["time"]):
            if not df_clean["time"].empty:
                 df_clean["time"] -= df_clean["time"].min()
            else: self.get_logger().warning("Zaman sütunu (time) boş, normalleştirme atlanıyor.")
        elif "time" in df_clean.columns: self.get_logger().warning("Zaman sütunu (time) sayısal değil.")
        
        return df_clean.dropna()

    def average_error_dataframes(self, list_of_dfs: List[pd.DataFrame], error_column_name: str, data_label_for_log: str, num_points_common_time: int = 500) -> Optional[pd.DataFrame]:
        if not list_of_dfs:
            self.get_logger().info(f"Ort. alınacak '{data_label_for_log}' için DF listesi boş.")
            return None

        valid_dfs = []
        for df_idx, df_item in enumerate(list_of_dfs):
            if isinstance(df_item, pd.DataFrame) and not df_item.empty and \
               'time' in df_item.columns and error_column_name in df_item.columns and \
               not df_item['time'].empty and not df_item[error_column_name].empty and \
               len(df_item['time']) == len(df_item[error_column_name]):
                valid_dfs.append(df_item.sort_values(by='time'))
            else: self.get_logger().debug(f"'{data_label_for_log}' için {df_idx}. DF geçersiz/eksik/uyumsuz.")

        if not valid_dfs:
            self.get_logger().warning(f"Ort. alınacak '{data_label_for_log}' için geçerli DF bulunamadı.")
            return None

        self.get_logger().info(f"{len(valid_dfs)} adet DF '{data_label_for_log}' için ortalamaya dahil edilecek.")
        max_duration = 0.0
        for df in valid_dfs:
            if not df['time'].empty: max_duration = max(max_duration, df['time'].max())
        
        if max_duration < 1e-6: 
            self.get_logger().warning(f"'{data_label_for_log}' için max. süre çok küçük ({max_duration}).")
            return pd.DataFrame(columns=['time', error_column_name])

        common_time = np.linspace(0, max_duration, num_points_common_time)
        interpolated_errors_list = []
        for df in valid_dfs:
            if df['time'].values.size > 0 and df[error_column_name].values.size > 0 :
                interpolated = np.interp(common_time, df['time'].values, df[error_column_name].values, left=np.nan, right=np.nan)
                interpolated_errors_list.append(interpolated)
        if not interpolated_errors_list:
            self.get_logger().warning(f"'{data_label_for_log}' için interpolasyon sonrası veri yok.")
            return pd.DataFrame(columns=['time', error_column_name])
            
        stacked_errors = np.vstack(interpolated_errors_list)
        mean_errors = np.nanmean(stacked_errors, axis=0)
        avg_df = pd.DataFrame({'time': common_time, error_column_name: mean_errors})
        avg_df.dropna(subset=[error_column_name], inplace=True) 
        if avg_df.empty:
            self.get_logger().warning(f"Ort. alınmış '{data_label_for_log}' verisi (NaN olmayan) boş.")
            return pd.DataFrame(columns=['time', error_column_name])
        self.get_logger().info(f"'{data_label_for_log}' için ortalama hesaplandı, {len(avg_df)} satır.")
        return avg_df

    def plot_specific_error(self, data_to_plot: list, title: str, y_label: str, output_file: str = None):
        fig, ax = plt.subplots(figsize=(12, 8)) # Figür ve eksen nesnelerini alalım
        plot_successful = False
        rmse_info_texts = ["RMSE Değerleri:"]

        for df, label_prefix, metric_column_name in data_to_plot:
            # metric_column_name sütunu karesi alınmış hataları (SE) içerir.
            # Ortalama DataFrame'ler için bu, ortalama SE olur.
            
            if "time" not in df.columns or metric_column_name not in df.columns:
                self.get_logger().error(f"'{label_prefix}' için '{metric_column_name}' veya 'time' sütunu DF'de bulunamadı.")
                continue
            if df.empty or df["time"].isnull().all() or df[metric_column_name].isnull().all():
                self.get_logger().warning(f"'{label_prefix}' için '{metric_column_name}' veya 'time' sütununda çiz. geçerli veri yok.")
                continue

            time_values = df["time"].values
            squared_error_values = df[metric_column_name].values # Bunlar SE (veya ortalama SE)

            # RMSE Hesaplaması: sqrt(mean(SE))
            if squared_error_values.size > 0:
                mse = np.nanmean(squared_error_values) 
                if not np.isnan(mse) and mse >= 0: # MSE negatif olmamalı
                    rmse = np.sqrt(mse)
                    rmse_info_texts.append(f"{label_prefix}: {rmse:.4f}")
                else:
                    rmse_info_texts.append(f"{label_prefix}: Hesaplanamadı") # NaN veya negatif MSE durumu
            else:
                rmse_info_texts.append(f"{label_prefix}: Veri Yok")

            # Grafikte gerçek hatayı (sqrt(SE)) çizdirelim
            # Negatif değer olmadığından emin olmak için np.maximum(0, ...)
            actual_error_to_plot = np.sqrt(np.maximum(0, squared_error_values))
            
            ax.plot(time_values, actual_error_to_plot, label=f"{label_prefix}", linewidth=1.5)
            plot_successful = True

        if not plot_successful:
            self.get_logger().warning(f"'{title}' için çizilecek veri bulunamadı. Grafik oluşturulmayacak.")
            plt.close(fig) # Figürü kapat
            return

        # RMSE metnini grafiğe ekle
        full_rmse_text = "\n".join(rmse_info_texts)
        ax.text(0.98, 0.98, full_rmse_text,
                transform=ax.transAxes, # Eksenlere göre % koordinat
                fontsize=9,
                verticalalignment='top', # Metin kutusunun üst kenarı y pozisyonuna hizalanır
                horizontalalignment='right', # Metin kutusunun sağ kenarı x pozisyonuna hizalanır
                bbox=dict(boxstyle='round,pad=0.4', fc='ivory', alpha=0.8)) # Arkaplan kutusu

        ax.set_xlabel("Zaman (s)", fontsize=12)
        ax.set_ylabel(y_label, fontsize=12) # y_label "Ortalama Hata X/Y (m)" olmalı
        ax.set_title(title, fontsize=14, weight='bold')
        ax.grid(True, which='both', linestyle='--', linewidth=0.5, alpha=0.7)
        ax.legend(fontsize=10, loc='lower right') # RMSE metniyle çakışmaması için legend konumu ayarlanabilir
        
        plt.tight_layout(rect=[0, 0, 0.95, 1]) # RMSE metni için sağda biraz boşluk bırakabiliriz, gerekirse ayarlanır

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
        visualizer = AverageDataPlotterNode()
    except Exception as e:
         if visualizer: visualizer.get_logger().error(f"Düğüm başlatılırken/çalışırken hata: {str(e)}", exc_info=True)
         else: print(f"rclpy/Düğüm oluşturulurken hata: {str(e)}")
    finally:
        if visualizer and rclpy.ok():
            visualizer.destroy_node()
            visualizer.get_logger().info("Ortalama Veri Çizdirme Düğümü Kapatıldı.")
        if rclpy.ok(): rclpy.shutdown()
        print("ROS Kapatıldı.")

if __name__ == "__main__":
    main()