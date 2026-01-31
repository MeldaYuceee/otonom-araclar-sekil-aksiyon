# Otonom Görev ve Şekil Aksiyonu  
*(Gazebo + ArduPilot SITL + DroneKit + ROS + OpenCV)*

Bu proje, **Otonom Araçlar Topluluğu** kapsamında verilen **2. Faz (Final) görevleri** doğrultusunda geliştirilmiştir.  
Çalışmanın amacı, Gazebo simülasyon ortamında bir drone’un **tamamen otonom** şekilde uçuş gerçekleştirmesi, kamerası aracılığıyla zemindeki şekilleri algılaması ve algılanan şekle göre **önceden tanımlanmış aksiyonları** yerine getirmesidir.

---

## Görev Senaryosu

Drone, görev boyunca herhangi bir manuel müdahale olmadan aşağıdaki adımları sırasıyla gerçekleştirir:

- **Otonom Kalkış**  
  Drone, sistem başlatıldıktan sonra otomatik olarak **10 metre irtifaya** kalkış yapar.

- **Seyrüsefer**  
  Simülasyon ortamında yer alan iki sanal direk (direk ↔ direk_0) arasından geçerek belirlenen güzergâhı takip eder.

- **Arama ve Tarama**  
  Seyir esnasında, altındaki zemin üzerinde rastgele yerleştirilmiş geometrik şekilleri kamerası ile tarar.

- **Şekle Göre Aksiyon**  
  Kamera görüntüsünden elde edilen verilere göre:
  - **Kırmızı Üçgen** tespit edilirse, drone şeklin tam üzerine konumlanır ve **LAND** komutu ile iniş yapar.
  - **Mavi Altıgen** tespit edilirse, drone **3 metre irtifaya alçalır**, **5 saniye bekler**, ardından tekrar **10 metreye yükselerek** görevine devam eder.

---

## Sistem Durumu Bildirimi

Görev boyunca drone’un mevcut durumu terminal üzerinden anlık olarak yazdırılmaktadır.  
Örnek durum çıktıları:

- `STATE=TAKEOFF`
- `STATE=NAVIGATION`
- `STATE=SEARCHING`
- `STATE=DESCENDING`
- `STATE=LANDING`

Bu çıktı mekanizması, **“sistemin durumunu gösterme”** gereksinimini karşılamak amacıyla eklenmiştir.

---

## Dosya Yapısı ve Açıklamaları

- `phase2_mission.py`  
  2. Faz final senaryosunun tamamını içeren ana görev dosyası  
  (10 m kalkış, direkler arası geçiş, tarama ve şekle göre aksiyonlar).

- `pole_nav_v2.py`  
  Birinci aşama testleri için kullanılan, direkler arası otonom geçiş senaryosu.

- `axis_probe_air3.py`  
  Gazebo koordinat sistemi ile drone’un local NED eksenleri arasındaki ilişkiyi doğrulamak için geliştirilmiş test dosyası.

- `hover_debug_shape.py`  
  Şekil tespiti sırasında HSV maskeleme ve piksel yoğunluğu (red_px / blue_px) analizlerinin yapıldığı debug dosyası.

- `RAPOR.md`  
  Şekil tespitinde kullanılan yöntemlerin (renk filtresi, kontur analizi) nedenleriyle birlikte açıklandığı kısa teknik rapor.

- `requirements.txt`  
  Python bağımlılıkları.

---

## Kullanılan Yöntem

Şekil tespiti için **OpenCV** kullanılarak:
- HSV renk uzayında **renk filtresi** uygulanmış,
- Filtrelenen alanlar üzerinde **kontur analizi** ile şekil doğrulaması yapılmıştır.

Bu yöntem, simülasyon ortamında ışık koşullarının sabit olması nedeniyle **hızlı, kararlı ve düşük hesaplama maliyetli** bir çözüm sunduğu için tercih edilmiştir.

---

## Gereksinimler

- Ubuntu  
- ROS Noetic  
- Gazebo (ROS API Plugin)  
- ArduPilot SITL (ArduCopter)  
- Python 3  

Kamera topic’i:
/iris_demo/gimbal_camera/image_raw


Python bağımlılıkları:
```bash
pip3 install -r requirements.txt





