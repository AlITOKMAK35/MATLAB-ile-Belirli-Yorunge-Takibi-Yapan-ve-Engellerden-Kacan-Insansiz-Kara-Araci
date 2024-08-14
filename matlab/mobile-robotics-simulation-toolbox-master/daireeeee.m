%% Simülasyon kurulumu
% Aracı Tanımla
R = 0.1;                        % Tekerlek yarıçapı [m]
L = 0.5;                        % Dingil mesafesi [m]
dd = DifferentialDrive(R,L);

% Örnekleme zamanı ve zaman dizisi
sampleTime = 0.1;              % Örnekleme zamanı [s]
tVec = 0:sampleTime:40;        % Zaman dizisi

% Başlangıç koşulları
initPose = [2.75;3.1;0];            % Başlangıç pozu (x y teta)
pose = zeros(3,numel(tVec));   % Poz matrisi
pose(:,1) = initPose;

% Harita yükle
close all
map = mapClutter(10,{'Box'},'MapSize',[12 12],'MapResolution',5);

% Lidar sensörü oluştur
lidar = LidarSensor;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-pi/2,pi/2,51);
lidar.maxRange = 1.5;

% Görselleştirici oluştur
viz = Visualizer2D;
viz.hasWaypoints = true;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);

%% Yol planlama ve takip
% Yol noktaları oluştur
waypoints = [2.75 3.1 ;
    2.75 3.23 ;
    2.35 4.53 ;
    2.22 6.2 ;
    2.32 7.35 ;
    3.16 8.95 ;
    4.28 9.43 ;
    5.52 9.63 ;
    7.07 9.66 ;
    7.99 9.18 ;
    8.72 8.36 ;
    9.26 6.91 ;
    9.54 5.09 ;
    8.9 3.59 ;
    8.01 2.85 ;
    6.59 2.34 ;
    5.34 2.21 ;
    4.38 2.39 ;
    3.36 2.7 ;
    2.91 3.03 ];
 
% Saf Takip Kontrolörü
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.5;
controller.DesiredLinearVelocity = 0.75;
controller.MaxAngularVelocity = 1.5;

% Vektör Alanı Histogramı (VFH) engellerden kaçınmak için
vfh = controllerVFH;
vfh.DistanceLimits = [0.05 3];
vfh.NumAngularSectors = 36;
vfh.HistogramThresholds = [5 10];
vfh.RobotRadius = L;
vfh.SafetyDistance = L;
vfh.MinTurningRadius = 0.25;

%% Simülasyon döngüsü
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec) 
    
% Sensör okumalarını al
curPose = pose(:,idx-1);
    ranges = lidar(curPose);
        
% Takip eden yolu ve engellerden kaçınma algoritmalarını çalıştırın
[vRef,wRef,lookAheadPt] = controller(curPose);
    targetDir = atan2(lookAheadPt(2)-curPose(2),lookAheadPt(1)-curPose(1)) - curPose(3);
    steerDir = vfh(ranges,lidar.scanAngles,targetDir);    
    if ~isnan(steerDir) && abs(steerDir-targetDir) > 0.1
        wRef = 0.5*steerDir;
    end
    
% Robotu kontrol et
velB = [vRef;0;wRef];                   % Vücut hızları [vx; vy; w]
    vel = bodyToWorld(velB,curPose);  % Bedenden dünyaya dönüştür
    
% İleri ayrık entegrasyon adımını gerçekleştirin
pose(:,idx) = curPose + vel*sampleTime; 
    
% Güncelleme görselleştirmesi
viz(pose(:,idx),waypoints,ranges)
    waitfor(r);
end
