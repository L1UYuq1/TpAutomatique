kobuki = importdata('data_kobukinumero1.txt',',')

[nl,nc] = size(kobuki)
tfin = kobuki(end,1) - kobuki(1,1);  
G_int = tf(1,[1,0]); % 1/S 积分器 PID控制系统中的I
vx = kobuki(:,11); %线速度
wz = kobuki(:,12); %角速度


theta0 = -pi/2; % -90°(弧度 = 角度 * pi / 180)
temps = linspace(0,tfin,nl);
theta = lsim(G_int,wz,temps,theta0);
lsim(G_int,wz,temps,theta0)

left_encoder = kobuki(:,3);
right_encoder = kobuki(:,4);
%通过控制轮子的对应时间的速度

R = 0.07 / 2; % rayon des roues en m
Ly = 0.23; % largeur essieu en m ( dist entre milieu des roues )
Lx=0.25+0.17; % distance mesure camera RGB suivant l'axe Xe du robot
reduction = 6545.0 / 132.0; % rapport de reduction
tickByTour = 52 * reduction; % nb pas codeur / tour de roue
encToRad =  2 * pi/ tickByTour ; % angles roues  gauche, droite= encToRad *  left_encoder , encToRad *  right_encoder

%condition initiale 
thetad0 = 0;
thetag0 = 0;
x0 = 1.5549;
y0 = 1.0608;

[A,B,C,D] = linmod('ShemaSimulation');
ss_Kobuki = ss(A,B,C,D);
tf_Kobuki = tf(ss_Kobuki);
Ho_wz = tf_Kobuki(3,1);


