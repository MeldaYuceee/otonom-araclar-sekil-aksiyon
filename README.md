# Otonom Görev ve Şekil Aksiyonu

Bu proje, **Otonom Araçlar Topluluğu** kapsamında verilen **2. Faz (Final)** görevi için hazırlanmıştır.  
Gazebo simülasyon ortamında bir drone’un kendi başına kalkış yapması, belirlenen rotayı takip etmesi ve kamerasıyla yerdeki şekilleri algılayarak buna göre aksiyon alması amaçlanmıştır.

---

## Görev Nasıl İlerliyor?

Görev başladığında drone herhangi bir manuel kontrol olmadan havalanır ve **10 metre irtifaya** çıkar.  
Ardından simülasyon ortamında bulunan **iki sanal direk** arasından geçerek uçuşuna devam eder.

Uçuş sırasında kamera sürekli olarak zemini tarar. Drone, tespit ettiği şekle göre farklı davranacak şekilde programlanmıştır:

- **Kırmızı bir üçgen** görüldüğünde, şeklin tam üzerine gider ve iniş yapar (LAND).
- **Mavi bir altıgen** görüldüğünde, **3 metre irtifaya alçalır**, **5 saniye bekler** ve ardından tekrar **10 metreye çıkarak** görevine devam eder.

---

## Şekilleri Nasıl Tanıyor?

Şekil tespiti için, simülasyon ortamına uygun, sade ve güvenilir bir yöntem tercih edilmiştir.

Öncelikle kamera görüntüsü **HSV renk uzayına** çevrilmiş ve kırmızı ile mavi renkler maske kullanılarak ayrılmıştır.  
Daha sonra bu maskeler üzerinde **kontur analizi** yapılmış ve küçük, anlamlı olmayan bölgeler elenmiştir.

Son aşamada konturlar sadeleştirilmiş ve köşe sayılarına bakılarak şekil ayrımı yapılmıştır:
- 3 köşe → **Üçgen**
- 5–7 köşe → **Altıgen**

Bu yaklaşım, simülasyon ortamında renklerin net olması sayesinde hızlı ve kararlı sonuçlar vermiştir. Ayrıca ek bir veri seti veya eğitim sürecine ihtiyaç duyulmamaktadır.

---

## Ek Görev (Melda İçin)

Ek görev kapsamında, sistemin görev sırasında hangi aşamada olduğunu göstermek için **iki seçenek** sunulmuştur:  
**basit bir PyQt arayüzü** veya **terminal çıktısı**.

Bu projede, görev akışını daha sade ve doğrudan takip edebilmek amacıyla **terminal üzerinden durum çıktısı verme yöntemi** tercih edilmiştir.  
Drone’un yaptığı tüm işlemler terminalde `STATE=...` formatında yazdırılmaktadır.

Örnek durumlar:
- TAKEOFF  
- NAVIGATION  
- SEARCHING  
- DESCENDING  
- LANDING  

Bu sayede sistemin hangi aşamada olduğu anlık olarak takip edilebilmekte ve debug süreci kolaylaşmaktadır.

---

## Çalıştırma

Gazebo ve ArduPilot SITL çalışır durumdayken görev aşağıdaki komut ile başlatılır:

```bash
cd ~/ardupilot/ArduCopter
python3 -u phase2_mission.py --connect 127.0.0.1:14550
