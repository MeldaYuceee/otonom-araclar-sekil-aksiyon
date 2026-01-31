# Otonom Görev ve Şekil Aksiyonu

Bu proje, Otonom Araçlar Topluluğu kapsamında verilen 2. Faz (Final) görevi için hazırlanmıştır.  
Gazebo simülasyon ortamında bir drone’un kendi başına kalkış yapması, belirlenen rotayı takip etmesi ve kamerasıyla yerdeki şekilleri algılayarak buna göre hareket etmesi hedeflenmiştir.

---

## Görev Nasıl İlerliyor?

Görev başladığında drone herhangi bir manuel kontrol olmadan havalanır ve 10 metre irtifaya çıkar.  
Ardından simülasyon ortamında bulunan iki sanal direk arasından geçerek uçuşuna devam eder.

Uçuş sırasında kamera sürekli olarak zemini tarar. Drone, gördüğü şekle göre farklı davranacak şekilde programlanmıştır:

- **Kırmızı bir üçgen** tespit edildiğinde, şeklin tam üzerine doğru gider ve iniş yapar.
- **Mavi bir altıgen** tespit edildiğinde, 3 metre irtifaya alçalır, 5 saniye bekler ve tekrar 10 metreye çıkarak görevine kaldığı yerden devam eder.

---

## Şekilleri Nasıl Tanıyor?

Şekil tespiti için karmaşık yöntemler yerine, simülasyon ortamına uygun ve güvenilir bir yaklaşım tercih edildi.

Önce kamera görüntüsü HSV renk uzayına çevrildi ve kırmızı ile mavi renkler maske kullanılarak ayrıldı.  
Daha sonra bu alanlar üzerinde kontur analizi yapıldı ve küçük, anlamlı olmayan bölgeler elendi.

Son aşamada konturlar sadeleştirildi ve köşe sayılarına bakılarak şekil ayrımı yapıldı:
- 3 köşe → üçgen  
- 5–7 köşe → altıgen  

Bu yöntem hem hızlı çalışıyor hem de simülasyon ortamında oldukça kararlı sonuçlar veriyor. Ayrıca ekstra bir veri seti ya da eğitim süreci gerektirmiyor.

---

## Ek Görev (Benim İçin)

Ek gereksinim olarak, drone’un görev sırasında hangi aşamada olduğunu göstermek gerekiyordu.  
Bunun için PyQt gibi bir arayüz eklemek yerine, durumu doğrudan terminal üzerinden yazdırmayı tercih ettim.

Görev ilerledikçe terminalde şu tür çıktılar görülüyor:
- TAKEOFF  
- NAVIGATION  
- SEARCHING  
- DESCENDING  
- LANDING  

Bu sayede drone’un ne yaptığını anlık olarak takip etmek mümkün oluyor ve debug süreci de kolaylaşıyor.

---

## Çalıştırma

Gazebo ve ArduPilot SITL çalışır durumdayken aşağıdaki komut ile görev başlatılır:

```bash
cd ~/ardupilot/ArduCopter
python3 -u phase2_mission.py --connect 127.0.0.1:14550
