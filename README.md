IKA Aracı Simülasyonu

Proje Açıklaması
Bu proje, MATLAB kullanarak bir diferansiyel tekerlekli robot (IKA Aracı) için yol planlama ve engellerden kaçınma simülasyonu gerçekleştirir. Simülasyon, robotun belirli yol noktalarını takip etmesini ve engellerden kaçınmasını sağlayan bir kontrol algoritması içerir. Lidar sensörü, robotun çevresindeki engelleri algılamak için kullanılır ve bu sensör verileri, robotun hareketini yönlendirmek için değerlendirilir.

Kurulum ve Gereksinimler

MATLAB: Bu proje, MATLAB 2018b veya daha yeni bir sürümünde çalıştırılmalıdır.

MATLAB Robotics System Toolbox: Yol planlama ve engellerden kaçınma algoritmalarını kullanmak için gereklidir.

Harita Verisi: Harita oluşturulması için gerekli parametreler ve engeller.

Dosya Yapısı

Simulasyon.m: Simülasyonu başlatan ve kontrol eden ana MATLAB dosyası.

DifferentialDrive: Tekerlek ve dingil mesafesi gibi robot özelliklerini tanımlayan bir sınıf.

LidarSensor: Lidar sensörü özelliklerini tanımlayan bir sınıf.

Visualizer2D: Robotun ve haritanın 2D görselleştirilmesini sağlayan bir sınıf.

controllerPurePursuit: Yol noktalarını takip etmek için kullanılan kontrolör.

controllerVFH: Engellerden kaçınma için kullanılan kontrolör.

Kullanım

Simülasyonu Başlatma:

MATLAB'da Simulasyon.m dosyasını açın.

Kodun içinde tanımlı olan parametreleri ve harita verilerini inceleyin.

Simülasyonu başlatmak için kodu çalıştırın.

Parametreler:

Tekerlek Yarıçapı (R): Tekerleklerin yarıçapı (örneğin, 0.1 m).

Dingil Mesafesi (L): Tekerlekler arasındaki mesafe (örneğin, 0.5 m).

Örnekleme Zamanı (sampleTime): Simülasyonun her örneklemesi arasındaki süre (örneğin, 0.1 s).

Yol Noktaları (waypoints): Robotun takip etmesi gereken yol noktaları.

Görselleştirme:

Simülasyon sırasında robotun hareketi, yol noktaları ve engeller 2D görselleştirici aracılığıyla görüntülenir.

Visualizer2D sınıfı, robotun ve haritanın güncellenmiş görünümlerini sağlar.

Kod Açıklaması

Simülasyon Kurulumu

Aracı Tanımla: Tekerlek yarıçapı ve dingil mesafesi tanımlanır.

Örnekleme Zamanı ve Zaman Dizisi: Simülasyon süresi ve örnekleme zamanı ayarlanır.

Başlangıç Koşulları: Robotun başlangıç pozisyonu ve açısı belirlenir.

Harita Yükle: Engeller içeren bir harita oluşturulur.

Lidar Sensörü Oluştur: Sensör özellikleri belirlenir.

Görselleştirici Oluştur: Görselleştirici nesnesi ve lidar sensörü bağlanır.

Yol Planlama ve Takip

Yol Noktaları Oluştur: Robotun takip etmesi gereken yol noktaları tanımlanır.

Saf Takip Kontrolörü: Yol noktalarını takip etmek için kullanılan kontrolör ayarlanır.

Vektör Alanı Histogramı (VFH): Engellerden kaçınmak için kullanılan kontrolör ayarlanır.

Simülasyon Döngüsü

Sensör Okumaları: Lidar sensöründen alınan mesafe verileri alınır.

Yol Takibi ve Engellerden Kaçınma: Hedef doğrusal ve açısal hızlar hesaplanır, engellerden kaçınma algoritmaları uygulanır.

Robot Kontrolü: Robotun vücut hızları hesaplanır ve dünya koordinatlarına dönüştürülür.

Görselleştirme Güncellemesi: Robotun güncellenmiş pozisyonu ve çevresi görselleştirilir.

İletişim

Sorularınız veya geri bildirimleriniz için benimle iletişime geçebilirsiniz:

E-posta: alitokmak3535@gmail.com

GitHub: AlITOKMAK35 - https://github.com/AlITOKMAK35 -
