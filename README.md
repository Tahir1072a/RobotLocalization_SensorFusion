# Çoklu IMU ve Tekerlek Enkoderi ile EKF Tabanlı Araç Konum Tahmini (Multi-IMU & Wheel Encoder EKF-based Vehicle Localization)

Bu proje, özellikle GPS sinyalinin zayıf veya olmadığı durumlarda, çoklu Ataletsel Ölçüm Birimi (IMU) ve tekerlek enkoderi (wheel encoder) verilerini Genişletilmiş Kalman Filtresi (EKF) kullanarak birleştirerek otonom bir aracın konumunu hassas bir şekilde tahmin etmeyi amaçlar. Proje, `ROS (Robot Operating System)` ve `Gazebo` simülasyon ortamında geliştirilmiş ve test edilmiştir.

## 🎯 Temel Amaç ve Motivasyon

Otonom sistemlerde güvenilir ve doğru konum bilgisi kritik öneme sahiptir. GPS, yaygın bir çözüm olmasına rağmen kapalı alanlar, şehir kanyonları gibi ortamlarda yetersiz kalmaktadır. Bu proje, düşük maliyetli MEMS IMU'ların çoklu kullanımıyla bu zorlukların üstesinden gelmeyi ve GPS kesintilerinde dahi kesintisiz ve daha doğru bir konum tahmini sağlamayı hedefler.

Bu çalışma, özellikle aşağıdaki sensör füzyonu yaklaşımlarını incelemektedir:
1.  **Sanal IMU (Virtual IMU):** Çoklu IMU'lardan gelen ham verilerin birleştirilerek tek bir sanal sensör oluşturulması ve bu verinin tek bir EKF'ye beslenmesi.
2.  **Yerel EKF'ler ile İstatistiksel Birleştirme (Local EKFs with Statistical Fusion):** Her bir IMU verisinin ayrı bir yerel EKF'ye beslenmesi ve elde edilen konum tahminlerinin istatistiksel olarak birleştirilerek nihai konumun elde edilmesi.

## ✨ Öne Çıkan Özellikler

* **Çoklu Sensör Füzyonu:** 3 adet IMU ve tekerlek enkoderi verilerinin EKF ile etkin bir şekilde birleştirilmesi.
* **İki Farklı Füzyon Mimarisi:** 'Sanal IMU' ve 'Yerel EKF ile İstatistiksel Birleştirme' yöntemlerinin uygulanması ve karşılaştırılması.
* **GPS Bağımsız Lokalizasyon:** Özellikle GPS'in kullanılamadığı senaryolarda konum doğruluğunu artırma.
* **ROS Entegrasyonu:** ROS ortamında düğümler (nodes), konular (topics) ve servisler (services) kullanılarak modüler bir yapı.
* **Gazebo Simülasyonu:** Farklı senaryoların ve sensör yapılandırmalarının kontrollü bir ortamda test edilebilmesi.

## 🛠️ Kullanılan Teknolojiler

* **Robot Operating System (ROS):** Ros2 Humble
* **Gazebo:** Robot simülasyonları için
* **Programlama Dili:** Python
* **Temel Algoritma:** Genişletilmiş Kalman Filtresi (EKF)
* **Sensörler (Simüle Edilmiş):** MEMS IMU, Tekerlek Enkoderi

## 👤 İletişim
tahirifdn@gmail.com

---
