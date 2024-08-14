%% Simülasyon kurulumu
% Aracı Tanımla
R = 0.1;                        % Tekerlek yarıçapı [m]
L = 0.5;                        % Tekerlek açıklığı [m]
dd = DifferentialDrive(R,L);

% Örnekleme zamanı ve zaman dizisi
sampleTime = 0.1;              % Örnekleme zamanı [s]
tVec = 0:sampleTime:40;        % Zaman dizisi

% Başlangıç koşulları
initPose = [2;2;0];            % Başlangıç pozu (x y teta)
pose = zeros(3,numel(tVec));   % Poz matrisi
pose(:,1) = initPose;

% Harita yükle
close all
map = mapClutter(7,{'Box'},'MapSize',[12 12],'MapResolution',5);

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
waypoints = [2 1.99 ;
    2 8 ;
    8 8 ;
    8 2 ;
    2 2];
 
% Takip Kontrolörü
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.5;
controller.DesiredLinearVelocity = 0.75;
controller.MaxAngularVelocity = 1.5;

% Vektör Alanı Histogramı (VAH) engellerden kaçınmak için
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

figure
plot(t,x,'r--',t,y,'b.',t,psi,'g-')
legend('x,[m]','y,[m]','\psi,[rad]');
set(gca,'fontsize',16)
xlabel('$t$,[m]','Interpreter','latex');
ylabel('$\eta$,[units]','Interpreter','latex');

figure
plot(t,eta_tilda)
legend('x,[m]','y,[e]','\psi,[rad]');
set(gca,'fontsize',8)
xlabel('t,[s]');
ylabel('$\tilde(\eta)$,[units]','Interpreter','latex');