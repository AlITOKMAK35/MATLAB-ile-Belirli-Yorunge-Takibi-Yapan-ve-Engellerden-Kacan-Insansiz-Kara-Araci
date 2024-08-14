%% Simülasyon kurulumu
% Aracı Tanımla
R = 0.1;                        % Tekerlek yarıçapı [m]
L = 0.5;                        % Dingil mesafesi [m]
dd = DifferentialDrive(R,L);

% Örnekleme zamanı ve zaman dizisi
sampleTime = 0.1;              % Örnekleme zamanı [s]
tVec = 0:sampleTime:60;        % Zaman dizisi

% Başlangıç koşulları
initPose = [2;2;0];            % Başlangıç pozu (x y teta)
pose = zeros(3,numel(tVec));   % Poz matrisi
pose(:,1) = initPose;

% Harita yükle
close all
map = mapClutter(12,{'Box'},'MapSize',[12 12],'MapResolution',5);

% Lidar sensörü oluştur
lidar = LidarSensor;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-pi/2,pi/2,51);
lidar.maxRange = 1.4;

% Görselleştirici oluştur
viz = Visualizer2D;
viz.hasWaypoints = true;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);





%% Yol planlama ve takip
% Yol noktaları oluştur
waypoints = [ 
    2.09 2.73;
    2.43 2.44 ;
    3.11 2.15 ;
    4.06 1.86 ;
    5 1.78 ;
    6.03 1.81 ;
    7.02 1.91 ;
    7.89 2.07 ;
    8.55 2.2 ;
    9.07 2.36 ;
    9.62 2.68 ;
    2.72 8.64 ;
    3.53 8.72 ;
    4.19 8.8 ;
    4.9 8.85 ;
    5.71 8.85 ;
    6.34 8.93 ;
    7.21 8.93 ;
    7.81 8.82 ;
    8.39 8.67 ;
    8.92 8.45 ;
    9.44 8.22 ;
    9.57 8.03 ;
    2.19 2.75 ];
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

%% Simulation loop
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
