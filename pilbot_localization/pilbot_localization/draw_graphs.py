#!/usr/bin/env python3
import rclpy
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from rclpy.node import Node
from typing import Tuple, List, Dict

# Matplotlib stilini ve Türkçe karakter desteğini ayarlama (isteğe bağlı)
# plt.style.use('seaborn-v0_8-darkgrid') # Daha modern bir stil deneyebilirsiniz
plt.rcParams['font.family'] = ['sans-serif'] # Varsa Türkçe karakter destekleyen bir font
plt.rcParams['axes.unicode_minus'] = False # Eksi işaretlerinin düzgün görünmesi için


class DataVisualizer(Node):
    def __init__(self):
        super().__init__("data_visualizer")
        self.get_logger().info("Veri Görselleştirme Düğümü Başlatıldı")

        # --- Ayarlar ---
        # Excel dosyalarınızdaki Z yönelim (Yaw) hatası sütununun adını buraya yazın
        self.yaw_error_column_name = "estimated_yaw_error"
        # Yaw hatasının birimi (grafik etiketi için)
        self.yaw_error_unit = "deg" # veya "deg" (derece) ise değiştirin

        try:
            imu_data_file = "imu_saved_data.xlsx"
            odom_data_file = "pose_saved_data.xlsx"

            # 1. Veri Yükleme
            imu_data, odom_data = self.load_data(
                (imu_data_file, odom_data_file)
            )
            self.get_logger().info("Veri dosyaları başarıyla yüklendi.")

            # --- 2. Pozisyon Hatası İşleme ve Görselleştirme ---
            pos_cols = ["time", "estimated_error_x", "estimated_error_y"]
            if all(col in imu_data.columns for col in pos_cols) and \
               all(col in odom_data.columns for col in pos_cols):

                ekf_pos_err = self.process_data(imu_data.copy(), pos_cols)
                fused_pos_err = self.process_data(odom_data.copy(), pos_cols)

                self.plot_data(
                    data_pairs=[
                        (ekf_pos_err, "EKF Pos", ["X Hatası", "Y Hatası"]),
                        (fused_pos_err, "Fused Pos", ["X Hatası", "Y Hatası"])
                    ],
                    title="Pozisyon Tahmin Hatası Karşılaştırması (X, Y)",
                    y_label="Hata (m)",
                    output_file="position_error_comparison.png"
                )
            else:
                self.get_logger().warning("Pozisyon hatası sütunları (time, estimated_error_x, estimated_error_y) bulunamadı. Pozisyon grafiği atlanıyor.")


            # --- 3. Z Yönelim (Yaw) Hatası İşleme ve Görselleştirme ---
            yaw_cols = ["time", self.yaw_error_column_name]
            if all(col in imu_data.columns for col in yaw_cols) and \
               all(col in odom_data.columns for col in yaw_cols):

                ekf_yaw_err = self.process_data(imu_data.copy(), yaw_cols)
                fused_yaw_err = self.process_data(odom_data.copy(), yaw_cols)

                self.plot_data(
                    data_pairs=[
                        (ekf_yaw_err, "EKF Yaw", ["Yaw Hatası"]),
                        (fused_yaw_err, "Fused Yaw", ["Yaw Hatası"])
                    ],
                    title="Yönelim Tahmin Hatası Karşılaştırması (Yaw / Z Ekseni)",
                    y_label=f"Hata ({self.yaw_error_unit})", # Birimi etikete ekle
                    output_file="orientation_error_comparison.png"
                )
            else:
                self.get_logger().warning(f"Yaw hatası sütunları (time, {self.yaw_error_column_name}) bulunamadı. Yönelim grafiği atlanıyor.")

        except FileNotFoundError as e:
             self.get_logger().error(f"Veri dosyası bulunamadı: {e.filename}")
        except KeyError as e:
             self.get_logger().error(f"Excel dosyasında beklenen sütun bulunamadı: {str(e)}")
        except Exception as e:
            self.get_logger().error(f"Görselleştirme sırasında hata oluştu: {str(e)}")
            # raise # Hatanın programı durdurmasını istiyorsanız uncomment yapın

    def load_data(self, filenames: Tuple[str, str]) -> Tuple[pd.DataFrame, pd.DataFrame]:
        """İki Excel dosyasını yükler."""
        try:
            df1 = pd.read_excel(filenames[0])
            df2 = pd.read_excel(filenames[1])
            return df1, df2
        except FileNotFoundError as e:
            # Hata mesajını daha spesifik hale getir
            raise FileNotFoundError(f"Veri dosyası bulunamadı: {e.filename}") from e

    def process_data(self, df: pd.DataFrame, columns: list) -> pd.DataFrame:
        """Belirtilen sütunları seçer, zamanı normalleştirir ve NaN değerleri kaldırır."""
        if not all(col in df.columns for col in columns):
             missing_cols = [col for col in columns if col not in df.columns]
             raise KeyError(f"DataFrame'de şu sütunlar eksik: {missing_cols}")

        df_clean = df[columns].copy() # Sadece gerekli sütunları al ve kopyala
        # Zaman sütununun sayısal olup olmadığını kontrol et
        if pd.api.types.is_numeric_dtype(df_clean["time"]):
             df_clean["time"] -= df_clean["time"].min()  # Zamanı 0'dan başlat
        else:
             self.get_logger().warning("Zaman sütunu sayısal değil, normalleştirme atlanıyor.")
        return df_clean.dropna() # Eksik veri içeren satırları kaldır

    def plot_data(self, data_pairs: list, title: str, y_label: str, output_file: str = None):
        """Verilen veri çiftlerini karşılaştırmalı olarak çizer."""
        plt.figure(figsize=(12, 8)) # Grafik boyutunu ayarla

        for df, label_prefix, metrics in data_pairs:
            time_col = df.columns[0] # İlk sütunun zaman olduğunu varsay
            time_values = df[time_col].values

            # Veri sütunlarının (zamandan sonraki) sayısının metrics listesiyle eşleştiğini kontrol et
            if len(df.columns) -1 != len(metrics):
                self.get_logger().error(f"'{label_prefix}' için veri sütunu sayısı ({len(df.columns)-1}) ile metrik adı sayısı ({len(metrics)}) eşleşmiyor. Grafik çizimi atlanıyor.")
                continue # Bu veri çiftini atla

            # Metrikleri çizdir
            for i, metric_name in enumerate(metrics):
                data_col_index = i + 1 # Zaman sütunundan sonraki sütunlar
                data_values = df.iloc[:, data_col_index].values

                # Çizgi stillerini ayarla (örneğin X için düz, Y için kesikli)
                linestyle = "--" if i % 2 else "-"
                # Farklı metrikler için belki farklı renkler de eklenebilir
                # color = plt.cm.viridis(i / len(metrics)) # Örnek renk paleti

                plt.plot(
                    time_values,
                    data_values,
                    label=f"{label_prefix} - {metric_name}", # Etiketi daha açıklayıcı yap
                    linestyle=linestyle,
                    # color=color # Renk eklenecekse
                    linewidth=1.5 # Çizgi kalınlığını biraz artır
                )

        plt.xlabel("Zaman (s)", fontsize=12)
        plt.ylabel(y_label, fontsize=12) # Fonksiyona parametre olarak eklenen y etiketi
        plt.title(title, fontsize=14, weight='bold') # Başlığı kalın yap
        plt.grid(True, which='both', linestyle='--', linewidth=0.5, alpha=0.7) # Izgara görünümünü iyileştir
        plt.legend(fontsize=10) # Legend yazı boyutunu ayarla
        plt.tight_layout() # Grafik elemanlarının sıkışmasını önle

        if output_file:
            try:
                plt.savefig(output_file, dpi=300, bbox_inches="tight")
                self.get_logger().info(f"Grafik şuraya kaydedildi: {output_file}")
            except Exception as e:
                self.get_logger().error(f"Grafik kaydedilemedi: {output_file}. Hata: {str(e)}")

        plt.show() # Grafiği ekranda göster

def main():
    rclpy.init()
    visualizer = None # Hata durumunda destroy_node çağrılabilmesi için
    try:
        visualizer = DataVisualizer()
        # Bu düğüm sadece başlatılıp grafik çizdiği için spin'e gerek yok gibi,
        # ama ROS düğümü olarak kalması için spin_once kullanılabilir.
        # Eğer başka ROS işlevleri eklenecekse spin() gerekebilir.
        # rclpy.spin_once(visualizer, timeout_sec=1)
        # Veya grafik penceresi kapanana kadar beklemesi için:
        plt.show() # Bu satır plot_data içinde zaten var, ama burada da olabilir.
                  # Eğer plot_data'dan kaldırılırsa burası pencereyi açık tutar.

    except Exception as e:
         # Hatanın kaynağını daha iyi anlamak için log seviyesini artırabiliriz
         if visualizer:
              visualizer.get_logger().error(f"DataVisualizer başlatılırken veya çalışırken bir hata oluştu: {str(e)}", exc_info=True)
         else:
              print(f"rclpy başlatılırken veya DataVisualizer oluşturulurken hata: {str(e)}")

    finally:
        if visualizer:
            visualizer.destroy_node()
            visualizer.get_logger().info("Veri Görselleştirme Düğümü Kapatıldı.")
        if rclpy.ok():
            rclpy.shutdown()
        print("ROS Kapatıldı.")


if __name__ == "__main__":
    main()