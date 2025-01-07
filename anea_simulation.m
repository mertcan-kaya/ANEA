% clc
clear all
close all

%% Environmental features

GRAV = 9.81; % gravity constant
g = -[0;0;GRAV]; % gravitational acceleration vector

%% Robot Kinematic Parameters

kin.q_posLim = jointLimits();

[kin.a_i,kin.alpha_i,kin.d_i,kin.theta_i_O] = kinematicParameters(zeros(6,1));

% 0: non, 1: sensor, 2: sensor+gripper, 3: sensor+gripper+object
flange_att = 3;

obj_rec.mass = 20; % kg
% obj_rec.geo = [0.60;0.60;0.05]; % [length;width,height] m
obj_rec.geo = [0.5;0.05;0.3];
obj_rec.offset = [-0.2;0.0;-0.05];

kin.r_e = getEEpos(flange_att,obj_rec.geo);

kin.n = length(kin.theta_i_O); % number of joints
n = kin.n;

kin.g0 = g;
kin.zi_i = [0;0;1];

[~,Th_i_O] = forwardGeometry(kin,zeros(n,1));
Th_i_O(:,:,n) = Th_i_O(:,:,n)*[eye(3),kin.r_e;[0,0,0,1]];
    
kin.rh_h_i = zeros(3,1,n+1);
kin.rh_h_i(:,:,1:n) = Th_i_O(1:3,4,:);

%% Real Robot Dynamic Parameters

dyn.spring_on = 1;
dyn.friction_on = 1;

% Standard inertial parameters
[dyn.m_i,dyn.ri_i_ci,dyn.Ii_i] = stdInertialParameters(flange_att,obj_rec);

%% Control Parameters

% id=1 : Passivity known dynamics
% id=2 : Adaptive known dynamics
% id=3 : Adaptive unknown dynamics

% id = 3;

for id = 1:3

% 0: off, 1: on
if id == 1
    disp('Passivity known dynamics')
    dyn_estimate = 1;
elseif id == 2
    disp('Adaptive known dynamics')
    dyn_estimate = 1;
elseif id == 3
    disp('Adaptive unknown dynamics')
    dyn_estimate = 0;
else
    disp('ERROR!')
end

% 0: non, 1: sensor, 2: sensor+gripper, 3: sensor+gripper+object
ctr.flange_att = 2;

ctr.tcyc = 0.004; % Cycle time

% 0: pid, 1: idc, 2: passivity+
ctr.algo = 2;
if ctr.algo == 2
    % 0: non, 1: matrix, 2: kws, 3: yuan, 4: mnea, 5: anea
%     ctr.adaptive = 5;

    if id == 1
        ctr.adaptive = 0;
    elseif id == 2
        ctr.adaptive = 5;
    elseif id == 3
        ctr.adaptive = 5;
    else
        disp('ERROR!')
    end

    % 0: \O, 1: a, 2: b, 3: c
    ctr.star = 3;
else
    ctr.star = 0;
end
ctr.comp_spr = 1;
ctr.comp_frc = 0;
ctr.comp_grv = 0;

% Standard inertial parameters
ctr.pi_bar = zeros(n*10,1);
if dyn_estimate ~= 0
    [m_i,ri_i_ci,Ii_i] = stdInertialParameters(ctr.flange_att,obj_rec);

    di_i = zeros(3,1,n);
    pi_i = zeros(10,1,n);
    for i = 1:n
        di_i(:,:,i) = m_i(i)*ri_i_ci(:,:,i);

        pi_i(:,:,i) = [m_i(i);di_i(:,:,i);SymVec(Ii_i(:,:,i))];
        ctr.pi_bar((i-1)*10+1:i*10,1) = pi_i(:,:,i);
    end
end

if ctr.algo == 2
    ctr.Lm_jnt_psv = [100;100;100;80;60;40];
    ctr.Kp_jnt_psv = [1000;5000;2000;1500;500;100]*0.0;
    ctr.Kd_jnt_psv = [15;15;100;60;30;20];

    % Adaptation gain
    ctr.Pdiag = zeros(10*n,1);

    ctr.Pdiag = [10;0.2;0.2;0.2;0.0005;0.0005;0.0005;0.0005;0.0005;0.0005
                 10;0.2;0.2;0.2;0.0005;0.0005;0.0005;0.0005;0.0005;0.0005
                 10;0.2;0.2;0.2;0.0005;0.0005;0.0005;0.0005;0.0005;0.0005
                 6;0.1;0.1;0.1;0.0005;0.0005;0.0005;0.0005;0.0005;0.0005
                 6;0.1;0.1;0.1;0.0005;0.0005;0.0005;0.0005;0.0005;0.0005
                 6;0.1;0.1;0.1;0.0005;0.0005;0.0005;0.0005;0.0005;0.0005];

    ctr.Pdiag = [10;0.1;0.1;0.1;0.0005;0.0005;0.0005;0.0005;0.0005;0.0005
                 10;0.1;0.1;0.1;0.0005;0.0005;0.0005;0.0005;0.0005;0.0005
                 10;0.1;0.1;0.1;0.0005;0.0005;0.0005;0.0005;0.0005;0.0005
                 6;0.1;0.1;0.1;0.0005;0.0005;0.0005;0.0005;0.0005;0.0005
                 6;0.1;0.1;0.1;0.0005;0.0005;0.0005;0.0005;0.0005;0.0005
                 6;0.1;0.1;0.1;0.0005;0.0005;0.0005;0.0005;0.0005;0.0005];

elseif ctr.algo == 1
    ctr.Kp_jnt_idc = [4000;8000;6000;6000;6000;6000];
    ctr.Ki_jnt_idc = [10;10;10;10;10;10];
    ctr.Kd_jnt_idc = [30;50;60;30;7.5;5];
else
    ctr.Kp_jnt_pid = [4000;8000;6000;700;600;400];
    ctr.Ki_jnt_pid = [10;10;10;10;10;10];
    ctr.Kd_jnt_pid = [30;50;60;30;7.5;5];
end

ctr.errsum = zeros(6,1);

%% Simulation

tini = 0; % Start time
% tfin = 10; % Final time

tcyc = ctr.tcyc;
tstp = 0.0001; % Time step
        
% 0: minimum-jerk, 1: trapezoidal
trj_type = 0;

% 0: unconstrained mj, 1: constrained mj
mj_constrained = 0;

% tf = 2*ta + ts, ratio = ts/ta
trapez_ratio = 8;

T = [0 1 2 3 4 5 6];

Pos = deg2rad([ 90 150 70 10 120 50 90
                0 60 -60 30 -80 60 0
                90 0 20 -60 -10 0 90
                0 50 0 -10 40 0 0
                90 0 90 -90 90 -90 90
                90 120 90 30 90 -30 90]);
                
if mj_constrained == 1
    Vel = zeros(n,length(T));
    Acc = zeros(n,length(T));
else
    Vel = zeros(n,2);
    Acc = zeros(n,2);
end

Cj = zeros(6*n,6);
for j = 1:6
    if mj_constrained == 1
        Cj(:,j) = minimumJerkConstrCoefficient1DOF(T,Pos(j,:),Vel(j,:),Acc(j,:));
    else
        Cj(:,j) = minimumJerkCoefficient1DOF(T,Pos(j,:),Vel(j,:),Acc(j,:));
    end
end

tfin = T(end);

ttot = ceil(tfin/tcyc);

q_posDes = zeros(n,ttot);
q_velDes = zeros(n,ttot);
q_accDes = zeros(n,ttot);

q_posFbk = zeros(n,ttot);
q_velFbk = zeros(n,ttot);

des.q_pos = zeros(n,1);
des.q_vel = zeros(n,1);
des.q_acc = zeros(n,1);

q_posAct = Pos(:,1);
q_velAct = zeros(n,1);
q_accAct = zeros(n,1);

mass_est = zeros(n,ttot);

d = round(tcyc/tstp);
tloop = 0;
for k = 0:tfin/tstp
% for k = 0:0
    
    if rem(k, d) == 0
        
        kc = k/d+1;
        t = k*tstp;
        
        % Trajectory generation
        for j = 1:n
            if trj_type == 1
                [des.q_pos(j,1),des.q_vel(j,1),des.q_acc(j,1)] = p2pTrjTrap1DOF(t,T,Pos(j,:),trapez_ratio);
            else
                [des.q_pos(j,1),des.q_vel(j,1),des.q_acc(j,1)] = minimumJerkPolynomial1DOF(t,T,Cj(:,j));
            end
        end
            
        q_posDes(:,kc) = des.q_pos;
        q_velDes(:,kc) = des.q_vel;
        q_accDes(:,kc) = des.q_acc;
        
        fbk.q_vel = q_velAct;
        fbk.q_pos = q_posAct;
            
        q_posFbk(:,kc) = fbk.q_pos;
        q_velFbk(:,kc) = fbk.q_vel;
        
        % Control algorithm
        tic
        [tau,ctr.pi_bar] = controlAlgo(ctr,kin,des,fbk);
        tloop = tloop + toc;
        mass_est(1,kc) = ctr.pi_bar(1);
        mass_est(2,kc) = ctr.pi_bar(11);
        mass_est(3,kc) = ctr.pi_bar(21);
        mass_est(4,kc) = ctr.pi_bar(31);
        mass_est(5,kc) = ctr.pi_bar(41);
        mass_est(6,kc) = ctr.pi_bar(51);
%         ctr.pi_bar
    end

    % Computation of robot motion
    q_accAct = robotAlgo(kin,dyn,tau,q_posAct,q_velAct);
    q_velAct = q_velAct + q_accAct*tstp;
    q_posAct = q_posAct + q_velAct*tstp;
    for i = 1:6
        if q_posAct(i) < deg2rad(kin.q_posLim(i,1))
            q_posAct(i) = deg2rad(kin.q_posLim(i,1));
            q_velAct(i) = 0;
            q_accAct(i) = 0;
        elseif q_posAct(i) > deg2rad(kin.q_posLim(i,2))
            q_posAct(i) = deg2rad(kin.q_posLim(i,2));
            q_velAct(i) = 0;
            q_accAct(i) = 0;
        end
    end

end

% tau
% q_accAct

xd(id,:) = 0:tcyc:tfin;

yd1(id,:) = rad2deg(q_posDes(1,:));
yd2(id,:) = rad2deg(q_posDes(2,:));
yd3(id,:) = rad2deg(q_posDes(3,:));
yd4(id,:) = rad2deg(q_posDes(4,:));
yd5(id,:) = rad2deg(q_posDes(5,:));
yd6(id,:) = rad2deg(q_posDes(6,:));
y1(id,:) = rad2deg(q_posFbk(1,:));
y2(id,:) = rad2deg(q_posFbk(2,:));
y3(id,:) = rad2deg(q_posFbk(3,:));
y4(id,:) = rad2deg(q_posFbk(4,:));
y5(id,:) = rad2deg(q_posFbk(5,:));
y6(id,:) = rad2deg(q_posFbk(6,:));

error = rad2deg(q_posDes) - rad2deg(q_posFbk);
for i = 1:6
rmse12(i) = sqrt(mean(error(i,:).^2));
end 
rmse12
mean_rmse12 = mean(rmse12)
std_rmse12 = std(rmse12)

end % if

% paper Fig 1
% % 
h2 = figure(1);
ax(1) = subplot(6,1,1);
plot(xd(1,:),yd1(1,:),'b-', xd(1,:),y1(1,:),'r--', xd(1,:),y1(2,:),'k:',xd(1,:),y1(3,:),'g-.', 'LineWidth', 2); 
%title('Joint 1');ylabel('Joint 1 - Position [Deg]'); xlabel('Time [sec]'); 
legend('Desired', 'Passivity','Adapt. Est.','Adapt. Null.', 'Location', 'Best');
ax(2) = subplot(6,1,2);
plot(xd(1,:),yd2(1,:),'b-', xd(1,:),y2(1,:),'r--', xd(1,:),y2(2,:),'k:',xd(1,:),y2(3,:),'g-.', 'LineWidth', 2); 
%title('Joint 2'); ylabel('Position [Deg]'); xlabel('Time [sec]'); %legend('Desired', 'Passivity', 'Location', 'Best');
ax(3) = subplot(6,1,3);
plot(xd(1,:),yd3(1,:),'b-', xd(1,:),y3(1,:),'r--', xd(1,:),y3(2,:),'k:',xd(1,:),y3(3,:),'g-.', 'LineWidth', 2); 
%title('Joint 3'); ylabel('Position [Deg]'); xlabel('Time [sec]'); %legend('Desired', 'Passivity', 'Location', 'Best');
ax(4) = subplot(6,1,4);
plot(xd(1,:),yd4(1,:),'b-', xd(1,:),y4(1,:),'r--', xd(1,:),y4(2,:),'k:',xd(1,:),y4(3,:),'g-.', 'LineWidth', 2); 
%title('Joint 4'); ylabel('Position [Deg]'); xlabel('Time [sec]'); %legend('Desired', 'Passivity', 'Location', 'Best');
ax(5) = subplot(6,1,5);
plot(xd(1,:),yd5(1,:),'b-', xd(1,:),y5(1,:),'r--', xd(1,:),y5(2,:),'k:',xd(1,:),y5(3,:),'g-.', 'LineWidth', 2); 
%title('Joint 5'); ylabel('Position [Deg]'); xlabel('Time [sec]'); %legend('Desired', 'Passivity', 'Location', 'Best');
ax(6) = subplot(6,1,6);
plot(xd(1,:),yd6(1,:),'b-', xd(1,:),y6(1,:),'r--', xd(1,:),y6(2,:),'k:',xd(1,:),y6(3,:),'g-.', 'LineWidth', 2); 
%title('Joint 6'); ylabel('Position [Deg]'); xlabel('Time [sec]'); %legend('Desired', 'Passivity', 'Location', 'Best');

% Once happy with your figure, add an inset:
% Cosine inset:
MagInset(h2, ax(1), [2.3 3.05 -5 20], [2 3.44 86 155], {'SW','SW';'SE','SE'}, ':');
MagInset(h2, ax(1), [4.38 4.64 70.7 95.76], [3.5 4.5 -5 69.7], {'NW','NW';'SE','SE'}, ':');
MagInset(h2, ax(1), [5.6 6 81.1 92], [4.7 6 99 155], {'SW','SW';'SE','SE'}, ':');
% Sine insets:
% MagInset(h2, ax(2), [0.15 0.2 15 40], [0.38 0.78 55 90], {'NW','NW';'SE','SE'});
% MagInset(h2, ax(2), [1.8 1.85 15 40], [1.25 1.65 55 90], {'NE','NE';'SW','SW'});

% paper Fig 2
% 
figure (2)
sub1 = subplot(6,1,1);
plot(xd(1,:), yd1(1,:) - y1(1,:), 'b-', xd(1,:),yd1(2,:) - y1(2,:), 'r--', xd(1,:),yd1(3,:) - y1(3,:), 'k:', 'LineWidth', 2);
legend('Passivity', 'Adapt. Est.', 'Adapt. Null.', 'Location', 'Best');
sub2 = subplot(6,1,2);
plot(xd(1,:), yd2(1,:) - y2(1,:), 'b-', xd(1,:),yd2(2,:) - y2(2,:), 'r--', xd(1,:),yd2(3,:) - y2(3,:), 'k:', 'LineWidth', 2);
sub3 = subplot(6,1,3);
plot(xd(1,:), yd3(1,:) - y3(1,:), 'b-', xd(1,:),yd3(2,:) - y3(2,:), 'r--', xd(1,:),yd3(3,:) - y3(3,:), 'k:', 'LineWidth', 2);
sub4 = subplot(6,1,4);
plot(xd(1,:), yd4(1,:) - y4(1,:), 'b-', xd(1,:),yd4(2,:) - y4(2,:), 'r--', xd(1,:),yd4(3,:) - y4(3,:), 'k:', 'LineWidth', 2);
sub5 = subplot(6,1,5);
plot(xd(1,:), yd5(1,:) - y5(1,:), 'b-', xd(1,:),yd5(2,:) - y5(2,:), 'r--', xd(1,:),yd5(3,:) - y5(3,:), 'k:', 'LineWidth', 2);
sub6 = subplot(6,1,6);
plot(xd(1,:), yd6(1,:) - y6(1,:), 'b-', xd(1,:),yd6(2,:) - y6(2,:), 'r--', xd(1,:),yd6(3,:) - y6(3,:), 'k:', 'LineWidth', 2);
% 

figure
plot(xd(1,:),yd1(1,:),'b-', xd(1,:),y1(1,:),'r--', xd(1,:),y1(2,:),'k:',xd(1,:),y1(3,:),'g-.', 'LineWidth', 2); 
%title('Joint 1');ylabel('Joint 1 - Position [Deg]'); xlabel('Time [sec]'); 
legend('Desired', 'Passivity','Adapt. Est.','Adapt. Null.', 'Location', 'Best');

figure
plot(xd(1,:),yd2(1,:),'b-', xd(1,:),y2(1,:),'r--', xd(1,:),y2(2,:),'k:',xd(1,:),y2(3,:),'g-.', 'LineWidth', 2); 
%title('Joint 2'); ylabel('Position [Deg]'); xlabel('Time [sec]'); %legend('Desired', 'Passivity', 'Location', 'Best');
legend('Desired', 'Passivity','Adapt. Est.','Adapt. Null.', 'Location', 'Best');

figure
plot(xd(1,:),yd3(1,:),'b-', xd(1,:),y3(1,:),'r--', xd(1,:),y3(2,:),'k:',xd(1,:),y3(3,:),'g-.', 'LineWidth', 2); 
%title('Joint 3'); ylabel('Position [Deg]'); xlabel('Time [sec]'); %legend('Desired', 'Passivity', 'Location', 'Best');
legend('Desired', 'Passivity','Adapt. Est.','Adapt. Null.', 'Location', 'Best');

figure
plot(xd(1,:),yd4(1,:),'b-', xd(1,:),y4(1,:),'r--', xd(1,:),y4(2,:),'k:',xd(1,:),y4(3,:),'g-.', 'LineWidth', 2); 
%title('Joint 4'); ylabel('Position [Deg]'); xlabel('Time [sec]'); %legend('Desired', 'Passivity', 'Location', 'Best');
legend('Desired', 'Passivity','Adapt. Est.','Adapt. Null.', 'Location', 'Best');

figure
plot(xd(1,:),yd5(1,:),'b-', xd(1,:),y5(1,:),'r--', xd(1,:),y5(2,:),'k:',xd(1,:),y5(3,:),'g-.', 'LineWidth', 2); 
%title('Joint 5'); ylabel('Position [Deg]'); xlabel('Time [sec]'); %legend('Desired', 'Passivity', 'Location', 'Best');
legend('Desired', 'Passivity','Adapt. Est.','Adapt. Null.', 'Location', 'Best');

figure
plot(xd(1,:),yd6(1,:),'b-', xd(1,:),y6(1,:),'r--', xd(1,:),y6(2,:),'k:',xd(1,:),y6(3,:),'g-.', 'LineWidth', 2); 
%title('Joint 6'); ylabel('Position [Deg]'); xlabel('Time [sec]'); %legend('Desired', 'Passivity', 'Location', 'Best');
legend('Desired', 'Passivity','Adapt. Est.','Adapt. Null.', 'Location', 'Best');

function output = Rot_x(input)
    output = [	1	0           0           0
                0	cos(input)  -sin(input)	0
                0	sin(input)	cos(input)	0
                0	0           0           1 ];
end

function output = Rot_z(input)
    output = [  cos(input)	-sin(input)	0	0
                sin(input)	 cos(input)	0	0
                0            0          1	0
                0            0          0	1 ];
end

function output = Trn_x(input)
    output = [  1 0 0 input
                0 1 0 0
                0 0 1 0
                0 0 0 1     ];
end

function output = Trn_z(input)
    output = [  1 0 0 0
                0 1 0 0
                0 0 1 input
                0 0 0 1     ];
end

function [a_i,alpha_i,d_i,theta_i] = kinematicParameters(q_pos)

    piOver2 = pi/2;

    a1 = 0; a2 = 0.150; a3 = 0.825;
    d1 = 0.550; d2 = 0; d4 = 0.625; d6 = 0.110;

    A1 = 0; A2 = -piOver2; A3 = 0; A4 = piOver2; A5 = -piOver2; A6 = piOver2;
    T2 = -piOver2; T3 = piOver2;

    a4 = 0; a5 = 0; a6 = 0;
    d3 = 0; d5 = 0;
    T1 = 0; T4 = 0; T5 = 0; T6 = 0;

    % General DH
    a_i     = [a1;a2;a3;a4;a5;a6];
    alpha_i = [A1;A2;A3;A4;A5;A6];
    d_i     = [d1;d2;d3;d4;d5;d6];
    theta_i = [T1;T2;T3;T4;T5;T6] + q_pos;
    
end

function [m_i,ri_i_ci,Ii_i] = stdInertialParameters(flange_att,obj_rec)
    
    % end link dynamic parameters
    [m_6,r6_6_c6,I6_c6] = getEEdynPara(flange_att,obj_rec);
    
    % mass (PARAMETERS ARE NOT INCLUDED BECAUSE OF CONFIDENTIALITY!)
    m_i = [m_1; m_2; m_3; m_4; m_5; m_6];

    % center of mass (PARAMETERS ARE NOT INCLUDED BECAUSE OF CONFIDENTIALITY!)
    ri_i_ci(:,:,1) = r1_1_c1;
    ri_i_ci(:,:,2) = r2_2_c2;
    ri_i_ci(:,:,3) = r3_3_c3;
    ri_i_ci(:,:,4) = r4_4_c4;
    ri_i_ci(:,:,5) = r5_5_c5;
    ri_i_ci(:,:,6) = r6_6_c6;

    % Inertia tensor
    
    % Inertia tensors on CoM of links in joint frames (PARAMETERS ARE NOT INCLUDED BECAUSE OF CONFIDENTIALITY!)
    Ii_ci(:,:,1) = I1_c1;

    Ii_ci(:,:,2) = I2_c2;

    Ii_ci(:,:,3) = I3_c3;

    Ii_ci(:,:,4) = I4_c4;

    Ii_ci(:,:,5) = I5_c5;

    Ii_ci(:,:,6) = I6_c6;
    
    % Inertia tensors on frame origin of links in joint frames
    Ii_i = zeros(3,3,6);
    for i = 1:length(m_i)
        Ii_i(:,:,i) = Ii_ci(:,:,i)-m_i(i)*SkewSym(ri_i_ci(:,:,i))*SkewSym(ri_i_ci(:,:,i));
    end
    
end

function r_e = getEEpos(flange_att,obj_rec_geo)

    sensor_length  = 0.0333; % m
    gripper_length = 0.150; % m
%     object_length  = 0.020; % m
    object_length  = obj_rec_geo(3); % m

    if flange_att == 1
        d_e = sensor_length;
    elseif flange_att == 2
        d_e = sensor_length + gripper_length;
    elseif flange_att == 3
%         d_e = sensor_length + gripper_length + object_length;
        d_e = sensor_length + gripper_length;
    else
        d_e = 0;
    end

    r_e = [0;0;d_e];
    
end

function [m_6,r6_6_c6,I6_c6] = getEEdynPara(flange_att,obj_rec)

    % sixth link parameters
    m_6_O        = ;
    r6_6_c6_O   = [];
    I6_c6_O     = [];
    
    % attachment masses (kg)
    m_so = 0.804; % outter sensor mass (including inner adapter mass)
    m_si = 0.114; % inner sensor mass
    m_oa = 0.253; % outter adapter mass
    m_g = 1.120; % gripper mass
    m_o = obj_rec.mass; % object mass

    % object_parameters
    x_sqr = obj_rec.geo(1)^2; % length square
    y_sqr = obj_rec.geo(2)^2; % width square
    z_sqr = obj_rec.geo(3)^2; % height square
    
    I_x = (1/12)*m_o*(z_sqr+y_sqr);
    I_y = (1/12)*m_o*(x_sqr+z_sqr);
    I_z = (1/12)*m_o*(x_sqr+y_sqr);
    
    % inertia tensors at their respective CoMs
    Is_cs = [0.000592,0,0;0,0.000592,0;0,0,0.001014]; % r:47,h:33.4
    Ia_ca = [0.000150,0,0;0,0.000150,0;0,0,0.000291]; % r:48,h:15
    Ig_cg = [0.001531,0,0;0,0.000831,0;0,0,0.001167]; % l:50,w:100,h:80
    Io_co = [I_x,0,0;0,I_y,0;0,0,I_z]; % l,w,h

    % mount angle of sensor
    sensor_mount_ang = -pi/4; % or 0

    RotZ = [	cos(-sensor_mount_ang)	-sin(-sensor_mount_ang)	0
                sin(-sensor_mount_ang)	 cos(-sensor_mount_ang)	0
                0                       0                       1];

    % length vectors
    rs_6_s = [0;0;0.0333]; % sensor length vector (m)
    rs_s_a = [0;0;0.015]; % adapter length vector (m)
    rs_a_g = [0;0;0.080+0.05]; % gripper length vector (m)

    % CoM vectors
    rs_6_c6_O = r6_6_c6_O; % rotated at z (doesn't change)
    rs_6_cs = [0;0;0.01665]; % sensor CoM vector (m)
    rs_s_ca = [0;0;0.0075]; % adapter CoM vector (m)
    rs_a_cg = [0;0;0.040]; % gripper CoM vector (m)
    rs_g_co = [0;0;obj_rec.geo(3)/2] + obj_rec.offset; % add sensor to obj vec!!!!

    % sensor(outter) center of mass in sixth frame
    r6_6_cso = [0.000; 0.000;0.0136];
    
    % inertia tensor of sensor(whole) at frame at o6_c6
    m_s = m_so + m_si;
    rs_c6_cs = rs_6_cs - rs_6_c6_O; % not actual
    Is_s = Is_cs - m_s*SkewSym(rs_c6_cs)*SkewSym(rs_c6_cs); % not rotated

    % inertia tensor of adapter at frame at o6_c6
    rs_c6_ca = rs_s_ca + rs_6_s - rs_6_c6_O; % not actual
    Is_a = Ia_ca - m_oa*SkewSym(rs_c6_ca)*SkewSym(rs_c6_ca); % not rotated
    
    % inertia tensor of gripper at frame at o6_c6
    rs_c6_cg = rs_a_cg + rs_s_a + rs_6_s - rs_6_c6_O; % not actual
    Is_g = Ig_cg - m_g*SkewSym(rs_c6_cg)*SkewSym(rs_c6_cg); % not rotated
    
    % inertia tensor of object at frame at o6_c6
    rs_c6_co = rs_g_co + rs_a_g + rs_s_a + rs_6_s - rs_6_c6_O; % not actual
    Is_o = Io_co - m_o*SkewSym(rs_c6_co)*SkewSym(rs_c6_co); % not rotated

    if flange_att == 1   	% sensor
        m_l = m_si;
        rs_s_cl = [0.0;0.0;-0.005];
        Is_ct = Is_s;
    elseif  flange_att == 2	% sensor + gripper
        m_l = m_si + m_oa + m_g;
        rs_s_cl = [0.0117;0.0097;0.0622];
        Is_ct = Is_s + Is_a + Is_g;
    elseif  flange_att == 3	% sensor + gripper + object
        m_l = m_si + m_oa + m_g + m_o;
        rs_s_co = rs_g_co + rs_a_g + rs_s_a;
        rs_s_cl = (m_o*rs_s_co + (m_si + m_oa + m_g)*[0.0117;0.0097;0.0622])/m_l;
        Is_ct = Is_s + Is_a + Is_g + Is_o;
    else                    % none
        m_so = 0.0;
        m_l = 0.0;
        rs_s_cl = zeros(3,1);
        Is_ct = zeros(3);
    end

    % CoM of load for frame 6
    r6_6_cl = RotZ*(rs_6_s + rs_s_cl);

    % inertia tensor of all tools combined
    I6_ct = RotZ*Is_ct*RotZ';

    % finalization
    m_6 = m_6_O + m_so + m_l;
    r6_6_c6 = (m_6_O*r6_6_c6_O + m_so*r6_6_cso + m_l*r6_6_cl)/(m_6_O + m_so + m_l);
    I6_c6 = I6_c6_O + I6_ct;
    
end

function [T0_h,Th_i] = forwardGeometry(kin,q_pos)

    Th_i = zeros(4,4,kin.n);
    T0_h = zeros(4,4,kin.n+1);

    T0_h(:,:,1) = eye(4);

    for i = 1:kin.n
        Rot_x_h = Rot_x(kin.alpha_i(i));
        Trn_x_h	= Trn_x(kin.a_i(i));
        Rot_z_i = Rot_z(kin.theta_i_O(i) + q_pos(i));
        Trn_z_i = Trn_z(kin.d_i(i));

        Th_i(:,:,i) = Rot_x_h * Trn_x_h * Rot_z_i * Trn_z_i;
        T0_h(:,:,i+1) = T0_h(:,:,i) * Th_i(:,:,i);
    end

end

function S = SkewSym(r)
    S = [   0       , -r(3) , r(2)
            r(3)	, 0     , -r(1)
            -r(2)   , r(1)  , 0     ];
end

function D = DotMat(r)
    D = [   r(1)	, 0     , 0     , r(2) 	, r(3)	, 0
            0       , r(2)  , 0     , r(1)  , 0     , r(3)
            0       , 0     , r(3)  , 0     , r(1)  , r(2)  ];
end

function L = SymVec(I)
    L = [I(1,1);I(2,2);I(3,3);I(1,2);I(1,3);I(2,3)];
end

function tf = computeTrjTime(trj)
    
    % joint abs vel limits (rad/s)
    velLim = deg2rad([200;200;255;315;360;870]);
        
    % joint abs acc limits (rad/s^2)
    accLim = [7.9;6.5;10.5;25.2;19.6;41.7];

    v_prcnt = trj.prcnt(1);
    a_prcnt = trj.prcnt(2);

    kv = v_prcnt*velLim;
    ka = a_prcnt*accLim;

    D = trj.qf-trj.qi;

    switch trj.profile
        case 1
            tf = max(abs(D)./kv);
        case 2
            tf = max(max(3*abs(D)./(2*kv)),max(sqrt(6*abs(D)./ka)));
        case 3
            tf = max(max(15*abs(D)./(8*kv)),max(sqrt(10*abs(D)./(sqrt(3)*ka))));
        case 4
            tf = max(max(2*abs(D)./kv),max(sqrt(4*abs(D)./ka)));
        otherwise
            tf = 0;
    end

end

function [q_pos,q_vel,q_acc] = trjGeneration(trj,t)

    tf = trj.tf;
    
    if t > tf
        t = tf;
    end

    if trj.profile ~= 0
        % With interpolation
        
        % Joint space trajectory
        D = trj.qf - trj.qi;

        switch trj.profile
            case 1
                s_pos = t/tf;
                s_vel = 1/tf;
                s_acc = 0;
            case 2
                s_pos = 3*(t/tf)^2 - 2*(t/tf)^3;
                s_vel = 6*(t/tf^2) - 6*(t^2/tf^3);
                s_acc = 6/tf^2 - 12*(t/tf^3);
            case 3
                s_pos = 10*(t/tf)^3 - 15*(t/tf)^4 + 6*(t/tf)^5;
                s_vel = 30*(t^2/tf^3) - 60*(t^3/tf^4) + 30*(t^4/tf^5);
                s_acc = 60*(t/tf^3) - 180*(t^2/tf^4) + 120*(t^3/tf^5);
            case 4
                if t <= tf/2
                    s_pos = 2*(t/tf)^2;
                    s_vel = 4*(t/tf^2);
                    s_acc = 4/tf^2;
                else
                    s_pos = -1 + 4*(t/tf) - 2*(t/tf)^2;
                    s_vel = 4/tf - 4*(t/tf^2);
                    s_acc = -4/tf^2;
                end
            otherwise
                s_pos = 0;
                s_vel = 0;
                s_acc = 0;
        end
        
        q_pos = trj.qi + s_pos*D;
        q_vel = s_vel*D;
        q_acc = s_acc*D;
    else
        % Without interpolation
        q_pos = trj.qf;
        q_vel = zeros(kin.n,1);
        q_acc = zeros(kin.n,1);
    end
    
end

function [tau,pi_bar, step_num, elaps_time] = controlAlgo(ctr,kin,des,fbk)
persistent counter;

if isempty(counter)
counter = 0;
end
counter = counter + 1;
step_num = counter;
    %% Error Calculation
    
    if ctr.algo == 2 % Passivity
        err_m = fbk.q_pos - des.q_pos; % e
        errdot_m = fbk.q_vel - des.q_vel; % edot
%         errdot_r = errdot_m + ctr.Lm_jnt_psv.*err_m; %s
        ref.q_vel = des.q_vel - ctr.Lm_jnt_psv.*err_m;
        ref.q_acc = des.q_acc - ctr.Lm_jnt_psv.*errdot_m;
        errdot_r = fbk.q_pos - ref.q_vel; %s
    else
        err_m = des.q_pos - fbk.q_pos;
        ctr.errsum = ctr.errsum + err_m*ctr.tcyc;
        errdot_m = des.q_vel - fbk.q_vel;
    end
    
    %% Control Torque
    O_nx1 = zeros(kin.n,1);
    
    [~,Th_i] = forwardGeometry(kin,fbk.q_pos);
    Th_i(:,:,kin.n) = Th_i(:,:,kin.n)*[eye(3),kin.r_e;[0,0,0,1]];

    q_p.Rh_i = zeros(3,3,kin.n+1);
    q_p.Rh_i(:,:,1:kin.n) = Th_i(1:3,1:3,:);

    if ctr.algo == 2
        % Adaptive
        tau_pd = ctr.Kp_jnt_psv.*err_m - ctr.Kd_jnt_psv.*errdot_r;
        switch ctr.adaptive
            case 1
                tic
                [tau_pass,pi_bar] = mtrx_func(ctr,kin,q_p,fbk,ref);
                elaps_time = toc;
            case 2
                tic
                [tau_pass,pi_bar] = kwsk_func(ctr,kin,q_p,fbk,ref);
                elaps_time = toc;
            case 3
                tic
                [tau_pass,pi_bar] = yuan_func(ctr,kin,q_p,fbk,ref);
                elaps_time = toc;
            case 4
                tic
                [tau_pass,pi_bar] = mnea_func(ctr,kin,q_p,fbk,ref);
                elaps_time = toc;
            case 5
                tic
                [tau_pass,pi_bar] = ANEA(ctr,kin,q_p,fbk.q_vel,ref.q_vel,ref.q_acc,kin.g0,ctr.pi_bar,ctr.Pdiag);
                elaps_time = toc;
            otherwise
                tic
                [tau_pass,pi_bar] = ANEA(ctr,kin,q_p,fbk.q_vel,ref.q_vel,ref.q_acc,kin.g0,ctr.pi_bar,zeros(kin.n*10,1));
                elaps_time = toc;
        end
        tau_ctrl = tau_pass + tau_pd;
    elseif ctr.algo == 1
        pi_bar = ctr.pi_bar;
        a_ctrl = ctr.Kp_jnt_idc.*err_m + ctr.Ki_jnt_idc.*ctr.errsum + ctr.Kd_jnt_idc.*errdot_m;
        
        qdd_ctrl = a_ctrl + des.q_acc;
        tau_ctrl = MNEA(ctr,kin,q_p,fbk.q_vel,fbk.q_vel,qdd_ctrl,kin.g0,ctr.pi_bar);
    else
        % PID
        pi_bar = ctr.pi_bar; % for compansations
        tau_ctrl = ctr.Kp_jnt_pid.*err_m + ctr.Ki_jnt_pid.*ctr.errsum + ctr.Kd_jnt_pid.*errdot_m;
    end
    
    %% Compensation Torque
    
    if ctr.comp_spr == 1
        tau_spr = springModel(fbk.q_pos);
    else
        tau_spr = O_nx1;
    end

    if ctr.comp_frc == 1
%         if ctr.frc_fbk == 0
            tau_frc = frictionPWModel(fbk.q_vel);
%         else
%             tau_frc = frictionPWModel(des.q_vel);
%         end
    else
        tau_frc = O_nx1;
    end

    if ctr.comp_grv == 1 && ctr.algo == 0
        tau_grv = MNEA(ctr,kin,q_p,O_nx1,O_nx1,O_nx1,kin.g0,ctr.pi_bar);
    else
        tau_grv = O_nx1;
    end
    
    %% Command Torque
    
    tau = tau_ctrl + tau_spr + tau_frc + tau_grv;

end

function q_acc = robotAlgo(kin,dyn,tau,q_pos,q_vel)

    m_i         = dyn.m_i;
    ri_i_ci     = dyn.ri_i_ci;
    Ii_i        = dyn.Ii_i;
    
    n = kin.n;
    
    zzi_i   = [zeros(3,1);kin.zi_i];
    
    [~,Th_i] = forwardGeometry(kin,q_pos);
    Th_i(:,:,n) = Th_i(:,:,n)*[eye(3),kin.r_e;[0,0,0,1]];
    
    Rh_i          = zeros(3,3,n+1);
    Rh_i(:,:,1:6) = Th_i(1:3,1:3,:);

    rh_h_i  = zeros(3,1,n+1); % +1 because of prcNE
    ci_i    = zeros(3,1,n);
    XXi_h   = zeros(6,6,n);
    
    if dyn.spring_on == 1
        tau_spr = springModel(q_pos);
    else
        tau_spr = zeros(6,1);
    end

    if dyn.friction_on == 1
        tau_frc = frictionPWModel(q_vel);
%         tau_frc = frictionViscousModel(0,q_vel);
    else
        tau_frc = zeros(6,1);
    end

    tau_i = tau - (tau_spr + tau_frc);

    % i) First forward recursive computations for i = 1, ..., n
    vvh_h       = zeros(6,1);
    gammai_i    = zeros(6,1,n);
    betah_h     = zeros(6,1,n+1);
    IIh_h       = zeros(6,6,n+1);
    for i = 1:n
        rh_h_i(:,:,i)   = Th_i(1:3,4,i);
        ci_i(:,:,i)     = m_i(i) * ri_i_ci(:,:,i);
        XXi_h(:,:,i)    = [Rh_i(:,:,i)', -Rh_i(:,:,i)'*SkewSym(rh_h_i(:,:,i)); zeros(3,3), Rh_i(:,:,i)'];
        
        gammai_i(:,:,i) = [Rh_i(:,:,i)'*(cross(vvh_h(4:6), cross(vvh_h(4:6), rh_h_i(:,:,i)))) ...
                            ;cross(Rh_i(:,:,i)'*vvh_h(4:6),q_vel(i)*kin.zi_i)];
        vvh_h           = XXi_h(:,:,i)*vvh_h + q_vel(i)*zzi_i;
        betah_h(:,:,i+1) = -[ cross(vvh_h(4:6), cross(vvh_h(4:6), ci_i(:,:,i)));
                              cross(vvh_h(4:6), Ii_i(:,:,i)*vvh_h(4:6))];
        IIh_h(:,:,i+1)  = [m_i(i)*eye(3), -SkewSym(ci_i(:,:,i)); SkewSym(ci_i(:,:,i)), Ii_i(:,:,i)];
    end

    % ii) Backward recursive computations for i = n, ..., 1
    II2h_h              = zeros(6,6,n+1);
    II2h_h(:,:,n+1)     = IIh_h(:,:,n+1);
    beta2h_h            = zeros(6,1,n+1);
    beta2h_h(:,:,n+1)   = betah_h(:,:,n+1);
    H_i                 = zeros(n,1);
    alphai_i            = zeros(6,1,n);
    for i = n:-1:1
        H_i(i)          = zzi_i'*II2h_h(:,:,i+1)*zzi_i;
        KKi_i           = II2h_h(:,:,i+1) - II2h_h(:,:,i+1)*zzi_i*(1/H_i(i))*zzi_i'*II2h_h(:,:,i+1);
        alphai_i(:,:,i) = KKi_i*gammai_i(:,:,i) + II2h_h(:,:,i+1)*zzi_i*(1/H_i(i)) ...
                            *(tau_i(i)+zzi_i'*beta2h_h(:,:,i+1)) - beta2h_h(:,:,i+1);
        beta2h_h(:,:,i) = betah_h(:,:,i) - XXi_h(:,:,i)'*alphai_i(:,:,i);
        II2h_h(:,:,i)   = IIh_h(:,:,i) + XXi_h(:,:,i)'*KKi_i*XXi_h(:,:,i);
    end

    % iii) Second forward recursive computations for i = 1, ..., n
    q_acc = zeros(n,1);
    aah_h = [-kin.g0;zeros(3,1)];
    for i = 1:n
        aai_h       = XXi_h(:,:,i)*aah_h;
        q_acc(i)    = (1/H_i(i))*(-zzi_i'*II2h_h(:,:,i+1)*(aai_h ...
                    + gammai_i(:,:,i)) + tau_i(i) + zzi_i'*beta2h_h(:,:,i+1));
        aah_h       = aai_h + zzi_i*q_acc(i) + gammai_i(:,:,i);
    end
    
end

function tau = springModel(q)

    r = 0.08;
    L = 0.728;

    Pc  = 5314;
    a0  = deg2rad(-0.8458);
    k   = 23950;
        
    x0 = sqrt(r^2+(r+L)^2-2*r*(r+L)*cos(a0+q(2)))-L;

    B1E = r*sin(a0+q(2));
    A2G = ((L+r)*B1E)./(L+x0);

    % F = k*x+Pc;

    F1 = k*x0;
    F2 = Pc*ones(1,length(x0));

    tau1 = F1.*A2G;
    tau2 = F2.*A2G;

    tau = zeros(6,1);
    tau(2) = tau1 + tau2;

end

function tau_frc = frictionPWModel(qd)

    tau_frc = zeros(6,1);

    tau = zeros(7,1);
    vel = zeros(7,1);
    vel(1:6) = qd;
    vel(7) = qd(5)+qd(6);        

    v = [111.2  ;87.28  ;36.49  ;9.596  ;15.42  ;0.1652 ;6.564];    % Viscous
    c = [35.98  ;55.28  ;27.51  ;7.647  ;4.747  ;0.5175 ;2.805];    % Coulumb
    o = [0.9811 ;-4.794 ;-7.222 ;0.01566;0.7782 ;0.007565;-0.2884]; % Offset
    n = [461.2  ;688.0  ;383.8  ;85.91  ;55.11  ;5.265  ;37.50];     % Negative slop
    p = [480.8  ;592.1  ;239.4  ;86.22  ;70.67  ;5.416  ;31.73];     % Positive slop

    for i = 1:7
        if (vel(i) > -1e-1 && vel(i) <= 0)
            tau(i) = n(i)*vel(i);
        elseif  (vel(i) > 0 && vel(i) <= 1e-1)
            tau(i) = p(i)*vel(i);
        else
            tau(i) = v(i)*vel(i)+c(i)*sign(vel(i))+o(i);
        end
    end

    tau_frc(1) = tau(1);
    tau_frc(2) = tau(2);
    tau_frc(3) = tau(3);
    tau_frc(4) = tau(4);
    tau_frc(5) = tau(5) + tau(7);
    tau_frc(6) = tau(6) + tau(7);

end

function tau_frc = frictionViscousModel(coupling,qd)

    tau_frc = zeros(6,1);

    tau_frc(1) = 133.7*qd(1);
    tau_frc(2) = 121.8*qd(2);
    tau_frc(3) = 53.69*qd(3);
    tau_frc(4) = 14.38*qd(4);
    if coupling == 1
        % Coupling
        tau_frc_a = 18.39*qd(5);
        tau_frc_b = 0.4887*qd(6);
        tau_frc_c = 7.44*(qd(5)+qd(6));

        tau_frc(5) = tau_frc_a + tau_frc_c;
        tau_frc(6) = tau_frc_b + tau_frc_c;
    else
        tau_frc(5) = 18.39*qd(5);
        tau_frc(6) = 14.71*qd(6);
    end

end

function tau_frc = frictionVisCoulModel(qd)

    tau_frc = zeros(6,1);

    tau_frc(1) = 111.2*qd(1) + 35.98*sign(qd(1)) + 0.9811;
    tau_frc(2) = 87.28*qd(2) + 55.28*sign(qd(2)) - 4.794;
    tau_frc(3) = 36.49*qd(3) + 27.51*sign(qd(3)) - 7.222;
    tau_frc(4) = 9.596*qd(4) + 7.647*sign(qd(4)) + 0.01566;

    % Coupling
    tau_frc_a = 15.42*qd(5) + 4.747*sign(qd(5)) + 0.7782;
    tau_frc_b = 0.1652*qd(6) + 0.5175*sign(qd(6)) + 0.007565;
    tau_frc_c = 6.564*(qd(5)+qd(6)) + 2.805*sign(qd(5)+qd(6)) - 0.2884;

    tau_frc(5) = tau_frc_a + tau_frc_c;
    tau_frc(6) = tau_frc_b + tau_frc_c;
        
end

function q_posLim = jointLimits()

    q_posLim = [-160    ,160
                -137.5  ,137.5
                -150    ,150
                -270    ,270
                -105    ,120
                -270    ,270];

end

function tau_i = MNEA(ctr,kin,q,qd,qRd,qRdd,g,pi_bar)

    n = kin.n;
    
    Ri_h = zeros(3,3,n+1);
    
    % output declaration
    tau_i = zeros(n,1);
    
    % forward recursion (link 1 to n)
    wh_h = zeros(3,1);
    wRh_h = zeros(3,1);
    SSSh_h = zeros(3,3);
    wdh_h = zeros(3,1);
    Psifh_h = zeros(3,10,n+1);
    Psinh_h = zeros(3,10,n+1);
    muh_h = -g;
    for i = 1:n
        % take transpoze one time for each step
        Ri_h(:,:,i) = q.Rh_i(:,:,i)';
        
        if ctr.star == 1
            Upsi_i = qRdd(i)*kin.zi_i + cross(Ri_h(:,:,i)*wRh_h,qd(i)*kin.zi_i);
        elseif ctr.star == 2
            Upsi_i = qRdd(i)*kin.zi_i + cross(Ri_h(:,:,i)*wh_h,qRd(i)*kin.zi_i);
        elseif ctr.star == 3
            Upsi_i = qRdd(i)*kin.zi_i + (cross(Ri_h(:,:,i)*wRh_h,qd(i)*kin.zi_i) + cross(Ri_h(:,:,i)*wh_h,qRd(i)*kin.zi_i))/2;
        else
            Upsi_i = qRdd(i)*kin.zi_i + cross(Ri_h(:,:,i)*wh_h,qd(i)*kin.zi_i);
        end
        
        wdh_h = Ri_h(:,:,i)*wdh_h + Upsi_i;
        muh_h = Ri_h(:,:,i)*(muh_h + SSSh_h*kin.rh_h_i(:,:,i));

        % calculate normal and reference ang vel
        wh_h = Ri_h(:,:,i)*wh_h + qd(i)*kin.zi_i;
        wRh_h = Ri_h(:,:,i)*wRh_h + qRd(i)*kin.zi_i;

        if ctr.star == 1
            Gh_h = SkewSym(wh_h)*SkewSym(wRh_h);
            Dh_h = SkewSym(wRh_h)*DotMat(wh_h);
        elseif ctr.star == 2
            Gh_h = SkewSym(wRh_h)*SkewSym(wh_h);
            Dh_h = SkewSym(wh_h)*DotMat(wRh_h);
        elseif ctr.star == 3
            Gh_h = (SkewSym(wh_h)*SkewSym(wRh_h) + SkewSym(wRh_h)*SkewSym(wh_h))/2;
            Dh_h = (SkewSym(wRh_h)*DotMat(wh_h) + SkewSym(wh_h)*DotMat(wRh_h))/2;
        else
            Gh_h = SkewSym(wh_h)*SkewSym(wh_h);
            Dh_h = SkewSym(wh_h)*DotMat(wh_h);
        end
        
        SSSh_h = SkewSym(wdh_h) + Gh_h;

        Psifh_h(:,:,i+1) = [muh_h,SSSh_h,zeros(3,6)];
        Psinh_h(:,:,i+1) = [zeros(3,1),SkewSym(muh_h)',DotMat(wdh_h) + Dh_h];
    end
    
    % backward recursion recursion (link n to 1)
    fi_i = zeros(3,1);
    ni_i = zeros(3,1);
    for i = n:-1:1
        ni_i = Psinh_h(:,:,i+1)*pi_bar((i-1)*10+1:i*10,1) + q.Rh_i(:,:,i+1)*(SkewSym(Ri_h(:,:,i+1)*kin.rh_h_i(:,:,i+1))*fi_i + ni_i);
        fi_i = Psifh_h(:,:,i+1)*pi_bar((i-1)*10+1:i*10,1) + q.Rh_i(:,:,i+1)*fi_i;
        tau_i(i) = kin.zi_i'*ni_i;
    end
end

function [tau_f,phati_bar] = mtrx_func(ctr,kin,q_p,fbk,ref)

    n = kin.n;

    Ri_h = zeros(3,3,n+1);
    for i = 1:n
        Ri_h(:,:,i) = q_p.Rh_i(:,:,i)';
    end

    Phii_i = zeros(6,1,n);
    Phii_i_bar = zeros(6*n,n);
    for i = 1:n
        Phii_i(:,:,i) = [zeros(3,1);kin.zi_i];
        Phii_i_bar((i-1)*6+1:i*6,i) = Phii_i(:,:,i);
    end

    Ri_j = zeros(3,3,n,n);
    rj_ji = zeros(3,1,n,n);
    for i = n:-1:1
        for j = i:-1:1
            if j == i
                Ri_j(:,:,i,j) = eye(3);
            else
                Ri_j(:,:,i,j) = Ri_j(:,:,i,j+1)*Ri_h(:,:,j+1);
                if j == i-1
                    rj_ji(:,:,j,i) = kin.rh_h_i(:,:,j+1);
                else
                    rj_ji(:,:,j,i) = q_p.Rh_i(:,:,j+1)*rj_ji(:,:,j+1,i) + kin.rh_h_i(:,:,j+1);
                end
            end
        end
    end

    Xi_ij = zeros(6,6,n,n);
    Xi_bar = zeros(6*n,6*n);
    Ji_ij = zeros(6,n,n);
    for i = 1:n
        for j = 1:i
            Xi_ij(:,:,j,i) = [Ri_j(:,:,i,j),Ri_j(:,:,i,j)*SkewSym(rj_ji(:,:,j,i))';zeros(3),Ri_j(:,:,i,j)];
            Xi_bar(((i-1)*6+1:i*6),((j-1)*6+1:j*6)) = Xi_ij(:,:,j,i);

            Ji_ij(:,j,i) = Xi_ij(:,:,j,i)*Phii_i(:,:,j);
        end
    end

    Ji_bar = Xi_bar*Phii_i_bar;

    vwh_h = zeros(6,1,n+1);
    vh_h = zeros(3,1,n+1);
    wh_h = zeros(3,1,n+1);
    for i = 1:n
        vwh_h(:,:,i+1) = Ji_ij(:,:,i)*fbk.q_vel;

        vh_h(:,:,i+1) = vwh_h(1:3,:,i+1);
        wh_h(:,:,i+1) = vwh_h(4:6,:,i+1);
    end

    Xdi_ij = zeros(6,6,n,n);
    Jdi_ij = zeros(6,n,n);
    for i = 1:n
        for j = 1:i
            Xdi_ij(:,:,j,i) = [Ri_j(:,:,i,j)*SkewSym(wh_h(:,:,j+1)),Ri_j(:,:,i,j)*(SkewSym(rj_ji(:,:,j,i))'*SkewSym(wh_h(:,:,j+1))+SkewSym(Ri_j(:,:,i,j)'*vh_h(:,:,i+1)-vh_h(:,:,j+1))');zeros(3),Ri_j(:,:,i,j)*SkewSym(wh_h(:,:,j+1))];
            Jdi_ij(:,j,i) = Xdi_ij(:,:,j,i)*Phii_i(:,:,j);
        end
    end

    % Passivity

    s = fbk.q_vel - ref.q_vel;

    vwRh_h = zeros(6,1,n+1);
    vRh_h = zeros(3,1,n+1);
    wRh_h = zeros(3,1,n+1);
    for i = 1:n
        vwRh_h(:,:,i+1) = Ji_ij(:,:,i)*ref.q_vel;

        vRh_h(:,:,i+1) = vwRh_h(1:3,:,i+1);
        wRh_h(:,:,i+1) = vwRh_h(4:6,:,i+1);
    end

    XRdi_ij = zeros(6,6,n,n);
    JRdi_ij = zeros(6,n,n);
    for i = 1:n
        for j = 1:i
            XRdi_ij(:,:,j,i) = [Ri_j(:,:,i,j)*SkewSym(wRh_h(:,:,j+1)),Ri_j(:,:,i,j)*(SkewSym(rj_ji(:,:,j,i))'*SkewSym(wRh_h(:,:,j+1))+SkewSym(Ri_j(:,:,i,j)'*vRh_h(:,:,i+1)-vRh_h(:,:,j+1))');zeros(3),Ri_j(:,:,i,j)*SkewSym(wRh_h(:,:,j+1))];
            JRdi_ij(:,j,i) = XRdi_ij(:,:,j,i)*Phii_i(:,:,j);
        end
    end

    zeta = zeros(6,1,n);
    Gh_h = zeros(3,3,n+1);
    Dh_h = zeros(3,6,n+1);
    if ctr.star == 1
        % Type A
        for i = 1:n
            zeta(:,:,i) = JRdi_ij(:,:,i)*fbk.q_vel;
            Gh_h(:,:,i+1) = SkewSym(wh_h(:,:,i+1))*SkewSym(wRh_h(:,:,i+1));
            Dh_h(:,:,i+1) = SkewSym(wRh_h(:,:,i+1))*DotMat(wh_h(:,:,i+1));
        end
    elseif ctr.star == 2
        % Type B
        for i = 1:n
            zeta(:,:,i) = Jdi_ij(:,:,i)*ref.q_vel;
            Gh_h(:,:,i+1) = SkewSym(wRh_h(:,:,i+1))*SkewSym(wh_h(:,:,i+1));
            Dh_h(:,:,i+1) = SkewSym(wh_h(:,:,i+1))*DotMat(wRh_h(:,:,i+1));
        end
    else
        % Type C
        for i = 1:n
            zeta(:,:,i) = (JRdi_ij(:,:,i)*fbk.q_vel + Jdi_ij(:,:,i)*ref.q_vel)/2;
            Gh_h(:,:,i+1) = (SkewSym(wh_h(:,:,i+1))*SkewSym(wRh_h(:,:,i+1)) + SkewSym(wRh_h(:,:,i+1))*SkewSym(wh_h(:,:,i+1)))/2;
            Dh_h(:,:,i+1) = (SkewSym(wRh_h(:,:,i+1))*DotMat(wh_h(:,:,i+1)) + SkewSym(wh_h(:,:,i+1))*DotMat(wRh_h(:,:,i+1)))/2;
        end
    end

    %% general
    
    R0_h = zeros(3,3,n+1);
    R0_h(:,:,1) = eye(3);
    for i = 1:n
        R0_h(:,:,i+1) = R0_h(:,:,i)*q_p.Rh_i(:,:,i);
    end

    vwdh_h = zeros(6,1,n+1);
    vdh_h = zeros(3,1,n+1);
    wdh_h = zeros(3,1,n+1);
    muh_h(:,:,1) = -kin.g0;
    for i = 1:n
        vwdh_h(:,:,i+1) = Ji_ij(:,:,i)*ref.q_acc + zeta(:,:,i);

        vdh_h(:,:,i+1) = vwdh_h(1:3,:,i+1);
        wdh_h(:,:,i+1) = vwdh_h(4:6,:,i+1);

        muh_h(:,:,i+1) = vdh_h(:,:,i+1) - R0_h(:,:,i+1)'*kin.g0;
    end

    SSSh_h = zeros(3,3,n+1);
    LSLh_h = zeros(3,6,n+1);
    Psifh_h = zeros(3,10,n+1);
    Psinh_h = zeros(3,10,n+1);
    Psii_i = zeros(6,10,n);
    for i = 1:n
        SSSh_h(:,:,i+1) = SkewSym(wdh_h(:,:,i+1)) + Gh_h(:,:,i+1);
        LSLh_h(:,:,i+1) = DotMat(wdh_h(:,:,i+1)) + Dh_h(:,:,i+1);

        Psifh_h(:,:,i+1) = [muh_h(:,:,i+1),SSSh_h(:,:,i+1),zeros(3,6)];
        Psinh_h(:,:,i+1) = [zeros(3,1),SkewSym(muh_h(:,:,i+1))',LSLh_h(:,:,i+1)];
        Psii_i(:,:,i) = [Psifh_h(:,:,i+1);Psinh_h(:,:,i+1)];
    end

    Psii_bar = zeros(6*n,10*n);
    for i = 1:n
        Psii_bar((i-1)*6+1:i*6,(i-1)*10+1:i*10) = Psii_i(:,:,i);
    end

    Y = Ji_bar'*Psii_bar;

    sigma_bar = Y'*s;

    P_bar = zeros(n*10,n*10);
    for i = 1:n*10
        P_bar(i,i) = ctr.Pdiag(i);
    end

    %% tau_f

    phati_bar = ctr.pi_bar - P_bar*sigma_bar*ctr.tcyc;
    tau_f = Y*phati_bar;

end

function [tau_f,phati_bar] = kwsk_func(ctr,kin,q_p,fbk,ref)

    n = kin.n;

    Ri_h = zeros(3,3,n+1);
    for i = 1:n
        Ri_h(:,:,i) = q_p.Rh_i(:,:,i)';  
    end

    % velocity
    wh_h = zeros(3,1,n+1);
    for i = 1:n
        wh_h(:,:,i+1) = Ri_h(:,:,i)*wh_h(:,:,i) + fbk.q_vel(i)*kin.zi_i;
    end

    %% Passivity

    % velocity
    wRh_h = zeros(3,1,n+1);
    for i = 1:n
        wRh_h(:,:,i+1) = Ri_h(:,:,i)*wRh_h(:,:,i) + ref.q_vel(i)*kin.zi_i;
    end

    upsi_i = zeros(3,1,n);
    Gh_h = zeros(3,3,n+1);
    Dh_h = zeros(3,6,n+1);
    if ctr.star == 1
        % Type A
        for i = 1:n
            upsi_i(:,:,i) = ref.q_acc(i)*kin.zi_i + cross(Ri_h(:,:,i)*wRh_h(:,:,i),fbk.q_vel(i)*kin.zi_i);

            Gh_h(:,:,i+1) = SkewSym(wh_h(:,:,i+1))*SkewSym(wRh_h(:,:,i+1));
            Dh_h(:,:,i+1) = SkewSym(wRh_h(:,:,i+1))*DotMat(wh_h(:,:,i+1));
        end
    elseif ctr.star == 2
        % Type B
        for i = 1:n
            upsi_i(:,:,i) = ref.q_acc(i)*kin.zi_i + cross(Ri_h(:,:,i)*wh_h(:,:,i),ref.q_vel(i)*kin.zi_i);

            Gh_h(:,:,i+1) = SkewSym(wRh_h(:,:,i+1))*SkewSym(wh_h(:,:,i+1));
            Dh_h(:,:,i+1) = SkewSym(wh_h(:,:,i+1))*DotMat(wRh_h(:,:,i+1));
        end
    else
        % Type C
        for i = 1:n
            upsi_i(:,:,i) = ref.q_acc(i)*kin.zi_i + (cross(Ri_h(:,:,i)*wRh_h(:,:,i),fbk.q_vel(i)*kin.zi_i) + cross(Ri_h(:,:,i)*wh_h(:,:,i),ref.q_vel(i)*kin.zi_i))/2;

            Gh_h(:,:,i+1) = (SkewSym(wh_h(:,:,i+1))*SkewSym(wRh_h(:,:,i+1)) + SkewSym(wRh_h(:,:,i+1))*SkewSym(wh_h(:,:,i+1)))/2;
            Dh_h(:,:,i+1) = (SkewSym(wRh_h(:,:,i+1))*DotMat(wh_h(:,:,i+1)) + SkewSym(wh_h(:,:,i+1))*DotMat(wRh_h(:,:,i+1)))/2;
        end
    end

    % acceleration
    wdh_h = zeros(3,1,n+1);
    muh_h = zeros(3,1,n+1);
    muh_h(:,:,1) = -kin.g0;
    SSSh_h = zeros(3,3,n+1);
    LSLh_h = zeros(3,6,n+1);
    Psifh_h = zeros(3,10,n+1);
    Psinh_h = zeros(3,10,n+1);
    for i = 1:n
        wdh_h(:,:,i+1) = Ri_h(:,:,i)*wdh_h(:,:,i) + upsi_i(:,:,i);
        muh_h(:,:,i+1) = Ri_h(:,:,i)*(muh_h(:,:,i) + SSSh_h(:,:,i)*kin.rh_h_i(:,:,i));

        SSSh_h(:,:,i+1) = SkewSym(wdh_h(:,:,i+1)) + Gh_h(:,:,i+1);
        LSLh_h(:,:,i+1) = DotMat(wdh_h(:,:,i+1)) + Dh_h(:,:,i+1);

        Psifh_h(:,:,i+1) = [muh_h(:,:,i+1),SSSh_h(:,:,i+1),zeros(3,6)];
        Psinh_h(:,:,i+1) = [zeros(3,1),SkewSym(muh_h(:,:,i+1))',LSLh_h(:,:,i+1)];
    end

    s = fbk.q_vel - ref.q_vel;

    y_ij = zeros(n,10,n);
    sigma_i = zeros(10,n);
    for i = 1:6
        for j = i:n
            if i == j
                rj_ij = zeros(3,1);
                Jw = kin.zi_i;
            else
                rj_ij = Ri_h(:,:,j)*(rj_ij + kin.rh_h_i(:,:,j));
                Jw = Ri_h(:,:,j)*Jw;
            end
            Jv = cross(Jw, rj_ij);

            y_ij(i,:,j) = Jv'*Psifh_h(:,:,j+1) + Jw'*Psinh_h(:,:,j+1);
        end
        for j = 1:i
            sigma_i(:,i) = sigma_i(:,i) + s(j)*y_ij(j,:,i)';
        end
    end

    P_bar = zeros(n*10,n*10);
    for i = 1:n*10
        P_bar(i,i) = ctr.Pdiag(i);
    end

    P_i = zeros(10,10,n);
    for i = 1:n
        P_i(:,:,i) = P_bar((i-1)*10+1:i*10,(i-1)*10+1:i*10);
    end
    
    pi_i = zeros(10,1,n);
    for i = 1:n
        pi_i(:,:,i) = ctr.pi_bar((i-1)*10+1:i*10,:);
    end
    
    phati_i = zeros(10,1,n);
    for i = 1:n
        phati_i(:,:,i) = pi_i(:,:,i) - P_i(:,:,i)*sigma_i(:,i)*ctr.tcyc;
    end
    
    phati_bar = zeros(n*10,1);
    for i = 1:n
        phati_bar((i-1)*10+1:i*10,:) = phati_i(:,:,i);
    end
    
    tau_f = zeros(n,1);
    for i = 1:n
        for j = i:n
            tau_f(i,:) = tau_f(i,:) + y_ij(i,:,j)*phati_i(:,:,j);
        end
    end

end

function [tau_f,phati_bar] = yuan_func(ctr,kin,q_p,fbk,ref)
    
    n = kin.n;
    
    Phii_i = zeros(6,1,n);
    for i = 1:n
        Phii_i(:,:,i) = [zeros(3,1);kin.zi_i];
    end

    Ri_j = zeros(3,3,n,n);
    rj_ji = zeros(3,1,n,n);
    for i = n:-1:1
        for j = i:-1:1
            if j == i
                Ri_j(:,:,i,j) = eye(3);
            else
                Ri_j(:,:,i,j) = Ri_j(:,:,i,j+1)*q_p.Rh_i(:,:,j+1)';
                if j == i-1
                    rj_ji(:,:,j,i) = kin.rh_h_i(:,:,j+1);
                else
                    rj_ji(:,:,j,i) = q_p.Rh_i(:,:,j+1)*rj_ji(:,:,j+1,i) + kin.rh_h_i(:,:,j+1);
                end
            end
        end
    end

    Ji_i = zeros(6,n,n);
    for i = 1:n
        for j = 1:i
            Ji_i(:,j,i) = [Ri_j(:,:,i,j)*SkewSym(rj_ji(:,:,j,i))'*kin.zi_i;Ri_j(:,:,i,j)*kin.zi_i];
        end
    end

    vwh_h = zeros(6,1,n+1);
    vh_h = zeros(3,1,n+1);
    wh_h = zeros(3,1,n+1);
    for i = 1:n
        vwh_h(:,:,i+1) = Ji_i(:,:,i)*fbk.q_vel;

        vh_h(:,:,i+1) = vwh_h(1:3,:,i+1);
        wh_h(:,:,i+1) = vwh_h(4:6,:,i+1);
    end

    Jdi_i = zeros(6,n,n);
    for i = 1:n
        for j = 1:i
            Jdi_i(:,j,i) = [Ri_j(:,:,i,j)*(SkewSym(rj_ji(:,:,j,i))'*SkewSym(wh_h(:,:,j+1))+SkewSym(Ri_j(:,:,i,j)'*vh_h(:,:,i+1)-vh_h(:,:,j+1))')*kin.zi_i;Ri_j(:,:,i,j)*SkewSym(wh_h(:,:,j+1))*kin.zi_i];
        end
    end

    %% Passivity

    vwRh_h = zeros(6,1,n+1);
    vRh_h = zeros(3,1,n+1);
    wRh_h = zeros(3,1,n+1);
    for i = 1:n
        vwRh_h(:,:,i+1) = Ji_i(:,:,i)*ref.q_vel;

        vRh_h(:,:,i+1) = vwRh_h(1:3,:,i+1);
        wRh_h(:,:,i+1) = vwRh_h(4:6,:,i+1);
    end

    JRdi_i = zeros(6,n,n);
    for i = 1:n
        for j = 1:i
            JRdi_i(:,j,i) = [Ri_j(:,:,i,j)*(SkewSym(rj_ji(:,:,j,i))'*SkewSym(wRh_h(:,:,j+1))+SkewSym(Ri_j(:,:,i,j)'*vRh_h(:,:,i+1)-vRh_h(:,:,j+1))')*kin.zi_i;Ri_j(:,:,i,j)*SkewSym(wRh_h(:,:,j+1))*kin.zi_i];
        end
    end

    zeta = zeros(6,1,n);
    Gh_h = zeros(3,3,n+1);
    Dh_h = zeros(3,6,n+1);
    if ctr.star == 1
        % Type A
        for i = 1:n
            zeta(:,:,i) = JRdi_i(:,:,i)*fbk.q_vel;
            
            Gh_h(:,:,i+1) = SkewSym(wh_h(:,:,i+1))*SkewSym(wRh_h(:,:,i+1));
            Dh_h(:,:,i+1) = SkewSym(wRh_h(:,:,i+1))*DotMat(wh_h(:,:,i+1));
        end
    elseif ctr.star == 2
        % Type B
        for i = 1:n
            zeta(:,:,i) = Jdi_i(:,:,i)*ref.q_vel;
            
            Gh_h(:,:,i+1) = SkewSym(wRh_h(:,:,i+1))*SkewSym(wh_h(:,:,i+1));
            Dh_h(:,:,i+1) = SkewSym(wh_h(:,:,i+1))*DotMat(wRh_h(:,:,i+1));
        end
    else
        % Type C
        for i = 1:n
            zeta(:,:,i) = (JRdi_i(:,:,i)*fbk.q_vel + Jdi_i(:,:,i)*ref.q_vel)/2;

            Gh_h(:,:,i+1) = (SkewSym(wh_h(:,:,i+1))*SkewSym(wRh_h(:,:,i+1)) + SkewSym(wRh_h(:,:,i+1))*SkewSym(wh_h(:,:,i+1)))/2;
            Dh_h(:,:,i+1) = (SkewSym(wRh_h(:,:,i+1))*DotMat(wh_h(:,:,i+1)) + SkewSym(wh_h(:,:,i+1))*DotMat(wRh_h(:,:,i+1)))/2;
        end
    end

    R0_h = zeros(3,3,n+1);
    R0_h(:,:,1) = eye(3);
    for i = 1:n
        R0_h(:,:,i+1) = R0_h(:,:,i)*q_p.Rh_i(:,:,i);
    end

    vwdh_h = zeros(6,1,n+1);
    vdh_h = zeros(3,1,n+1);
    wdh_h = zeros(3,1,n+1);
    muh_h(:,:,1) = -kin.g0;
    for i = 1:n
        vwdh_h(:,:,i+1) = Ji_i(:,:,i)*ref.q_acc + zeta(:,:,i);

        vdh_h(:,:,i+1) = vwdh_h(1:3,:,i+1);
        wdh_h(:,:,i+1) = vwdh_h(4:6,:,i+1);

        muh_h(:,:,i+1) = vwdh_h(1:3,:,i+1) - R0_h(:,:,i+1)'*kin.g0;
    end

    SSSh_h = zeros(3,3,n+1);
    LSLh_h = zeros(3,6,n+1);
    Omegai_i = zeros(6,9,n);
    Psifh_h = zeros(3,10,n+1);
    Psinh_h = zeros(3,10,n+1);
    Psii_i = zeros(6,10,n);
    for i = 1:n
        SSSh_h(:,:,i+1) = SkewSym(wdh_h(:,:,i+1)) + Gh_h(:,:,i+1);
        LSLh_h(:,:,i+1) = DotMat(wdh_h(:,:,i+1)) + Dh_h(:,:,i+1);

        Omegai_i(:,:,i) = [Gh_h(:,:,i+1),zeros(3,6);zeros(3,3),Dh_h(:,:,i+1)];
        Psifh_h(:,:,i+1) = [muh_h(:,:,i+1),SSSh_h(:,:,i+1),zeros(3,6)];
        Psinh_h(:,:,i+1) = [zeros(3,1),SkewSym(muh_h(:,:,i+1))',LSLh_h(:,:,i+1)];
        Psii_i(:,:,i) = [Psifh_h(:,:,i+1);Psinh_h(:,:,i+1)];
    end

    s = fbk.q_vel - ref.q_vel;

    svwi_i = zeros(6,1,n);
    sigma_i = zeros(10,1,n);
    for i = 1:n
        svwi_i(:,:,i) = Ji_i(:,:,i)*s;

        sigma_i(:,:,i) = Psii_i(:,:,i)'*svwi_i(:,:,i);
    end

    P_bar = zeros(n*10,n*10);
    for i = 1:n*10
        P_bar(i,i) = ctr.Pdiag(i);
    end
    P_i = zeros(10,10,n);
    for i = 1:n
        P_i(:,:,i) = P_bar((i-1)*10+1:i*10,(i-1)*10+1:i*10);
    end

    pi_i = zeros(10,1,n);
    for i = 1:n
        pi_i(:,:,i) = ctr.pi_bar((i-1)*10+1:i*10,:);
    end
    
    phati_i = zeros(10,1,n);
    for i = 1:n
        phati_i(:,:,i) = pi_i(:,:,i) - P_i(:,:,i)*sigma_i(:,i)*ctr.tcyc;
    end

    phati_bar = zeros(n*10,1);
    for i = 1:n
        phati_bar((i-1)*10+1:i*10,:) = phati_i(:,:,i);
    end
    
    tau_f = zeros(n,1);
    for i = 1:n
        tau_f = tau_f + Ji_i(:,:,i)'*Psii_i(:,:,i)*phati_i(:,:,i);
    end

end

function [tau_f,phati_bar] = mnea_func(ctr,kin,q_p,fbk,ref)

    n = kin.n;

    Y = zeros(n,10*n);
    for i = 1:n*10                    
        w = zeros(n*10,1);
        w(i) = 1;
        Y(:,i) = MNEA(ctr,kin,q_p,fbk.q_vel,ref.q_vel,ref.q_acc,kin.g0,w);
    end
    
    s = fbk.q_vel - ref.q_vel;
    sigma_bar = Y'*s;
    
    P_bar = zeros(n*10,n*10);
    for i = 1:n*10
        P_bar(i,i) = ctr.Pdiag(i);
    end
    
    phati_bar = ctr.pi_bar - P_bar*sigma_bar*ctr.tcyc;
%     tau_f = Y*phati_bar;
    tau_f = MNEA(ctr,kin,q_p,fbk.q_vel,ref.q_vel,ref.q_acc,kin.g0,phati_bar);
    
end

function [tau_fi,phat_bar] = ANEA(ctr,kin,q,qd,qRd,qRdd,g,p,Pdiag)

    n = kin.n;
    zi_i = kin.zi_i;
    Ri_h = zeros(3,3,n+1);
    
    % output declaration
    tau_fi = zeros(n,1);
    phat_bar = zeros(10*n,1);
    
    % forward recursion (link 1 to n)
    wh_h = zeros(3,1);
    wRh_h = zeros(3,1);
    SSSh_h = zeros(3,3);
    wdh_h = zeros(3,1);
    Psifh_h = zeros(3,10,n+1);
    Psinh_h = zeros(3,10,n+1);
    swh_h = zeros(3,1);
    svh_h = zeros(3,1);
    phati_i = zeros(10,1,n);
    muh_h = -g;
    for i = 1:n
        % take transpoze one time for each step
        Ri_h(:,:,i) = q.Rh_i(:,:,i)';
        
        if ctr.star == 1
            Upsi_i = qRdd(i)*zi_i + cross(Ri_h(:,:,i)*wRh_h,qd(i)*zi_i);
        elseif ctr.star == 2
            Upsi_i = qRdd(i)*zi_i + cross(Ri_h(:,:,i)*wh_h,qRd(i)*zi_i);
        elseif ctr.star == 3
            Upsi_i = qRdd(i)*zi_i + (cross(Ri_h(:,:,i)*wRh_h,qd(i)*zi_i) + cross(Ri_h(:,:,i)*wh_h,qRd(i)*zi_i))/2;
        else
            Upsi_i = qRdd(i)*zi_i + cross(Ri_h(:,:,i)*wh_h,qd(i)*zi_i);
        end
        
        wdh_h = Ri_h(:,:,i)*wdh_h + Upsi_i;
        muh_h = Ri_h(:,:,i)*(muh_h + SSSh_h*kin.rh_h_i(:,:,i));

        % calculate normal and reference ang vel
        wh_h = Ri_h(:,:,i)*wh_h + qd(i)*zi_i;
        wRh_h = Ri_h(:,:,i)*wRh_h + qRd(i)*zi_i;

        if ctr.star == 1
            Gh_h = SkewSym(wh_h)*SkewSym(wRh_h);
            Dh_h = SkewSym(wRh_h)*DotMat(wh_h);
        elseif ctr.star == 2
            Gh_h = SkewSym(wRh_h)*SkewSym(wh_h);
            Dh_h = SkewSym(wh_h)*DotMat(wRh_h);
        elseif ctr.star == 3
            Gh_h = (SkewSym(wh_h)*SkewSym(wRh_h) + SkewSym(wRh_h)*SkewSym(wh_h))/2;
            Dh_h = (SkewSym(wRh_h)*DotMat(wh_h) + SkewSym(wh_h)*DotMat(wRh_h))/2;
        else
            Gh_h = SkewSym(wh_h)*SkewSym(wh_h);
            Dh_h = SkewSym(wh_h)*DotMat(wh_h);
        end
        
        SSSh_h = SkewSym(wdh_h) + Gh_h;

        Psifh_h(:,:,i+1) = [muh_h,SSSh_h,zeros(3,6)];
        Psinh_h(:,:,i+1) = [zeros(3,1),SkewSym(muh_h)',DotMat(wdh_h) + Dh_h];

        % calculate task space sliding variables
        svh_h = Ri_h(:,:,i)*(svh_h + cross(swh_h,kin.rh_h_i(:,:,i)));
        swh_h = wh_h - wRh_h;
        
        sigmai_i = Psifh_h(:,:,i+1)'*svh_h + Psinh_h(:,:,i+1)'*swh_h;
        Pi = Pdiag((i-1)*10+1:i*10);
        phati_iprev = p((i-1)*10+1:i*10,1);
        phati_i(:,:,i) = phati_iprev - Pi.*sigmai_i*ctr.tcyc;
        phat_bar((i-1)*10+1:i*10) = phati_i(:,:,i);
    end
    
    % backward recursion recursion (link n to 1)
    fi_fi = zeros(3,1);
    ni_fi = zeros(3,1);
    for i = n:-1:1
        ni_fi = Psinh_h(:,:,i+1)*phati_i(:,:,i) + q.Rh_i(:,:,i+1)*(SkewSym(Ri_h(:,:,i+1)*kin.rh_h_i(:,:,i+1))*fi_fi+ni_fi);
        fi_fi = Psifh_h(:,:,i+1)*phati_i(:,:,i) + q.Rh_i(:,:,i+1)*fi_fi;
        tau_fi(i) = zi_i'*ni_fi;
    end
end

function [Pj,Vj,Aj,Jj] = minimumJerkPolynomial1DOF(t,T,Cj)

    n = length(T)-1;  % n = number of trajectories
    
    % Trajectory Polynomial
    for j = n+1:-1:2
        if t <= T(j)
            i = j-1;
        end
    end
    
    Pj = Cj(1+6*(i-1))*t^5 + Cj(2+6*(i-1))*t^4 + Cj(3+6*(i-1))*t^3 + Cj(4+6*(i-1))*t^2 + Cj(5+6*(i-1))*t + Cj(6+6*(i-1));
    Vj = 5*Cj(1+6*(i-1))*t^4 + 4*Cj(2+6*(i-1))*t^3 + 3*Cj(3+6*(i-1))*t^2 + 2*Cj(4+6*(i-1))*t + Cj(5+6*(i-1));
    Aj = 5*4*Cj(1+6*(i-1))*t^3 + 4*3*Cj(2+6*(i-1))*t^2 + 3*2*Cj(3+6*(i-1))*t + 2*Cj(4+6*(i-1));
    Jj = 5*4*3*Cj(1+6*(i-1))*t^2 + 4*3*2*Cj(2+6*(i-1))*t + 3*2*Cj(3+6*(i-1));

end

function [q_pos,q_vel,q_acc,q_jer] = p2pTrjTrap1DOF(t,T,Pos,c)

    n = length(T)-1;  % n = number of trajectories
    
    for j = n+1:-1:2
        if t <= T(j)
            i = j-1;
        end
    end
    
    qi = Pos(i);
    qf = Pos(i+1);
    ti = t-T(i);
    tf = T(i+1)-T(i);
    
    D = qf - qi;
    signD = sign(D);

    ta = tf/(2+c);
    kv = abs(D)/((c+1)*ta);
    ka = abs(D)/((c+1)*ta^2);

    if (ti <= ta)
        q_pos = qi + 0.5*ti^2*ka*signD;
        q_vel = ti*ka*signD;
        q_acc = ka*signD;
        q_jer = 0.0;
    elseif (ti <= tf - ta)
        q_pos = qi + (ti-0.5*ta)*kv*signD;
        q_vel = kv*signD;
        q_acc = 0.0;
        q_jer = 0.0;
    else
        q_pos = qf - 0.5*(tf-ti)^2*ka*signD;
        q_vel = (tf-ti)*ka*signD;
        q_acc = -ka*signD;
        q_jer = 0.0;
    end

end

function Cj = minimumJerkCoefficient1DOF(T,Pos,Vel,Acc)

    n = length(T)-1;  % n = number of trajectories 

    % Waypoints' Matrix
    R = zeros(6*n,6*n); % Start with zero matrix for R, joint
    
    % Positions of all waypoints
    for i = 1:n
        R(1+2*(i-1),1+6*(i-1):6+6*(i-1)) = [T(i)^5 T(i)^4 T(i)^3 T(i)^2 T(i)^1 1];
        R(2+2*(i-1),1+6*(i-1):6+6*(i-1)) = [T(i+1)^5 T(i+1)^4 T(i+1)^3 T(i+1)^2 T(i+1)^1 1];
    end
    % Velocity boundary conditions (inital and final waypoints)        
    R(((1+2*n):(1+2*n)),(1:6))               = [  5*T(1)^4   4*T(1)^3   3*T(1)^2   2*T(1) 1 0];
    R(((2+2*n):(2+2*n)),((1+6*(n-1)):(6*n))) = [5*T(n+1)^4 4*T(n+1)^3 3*T(n+1)^2 2*T(n+1) 1 0];
    % Equal Accelaration boundary conditions (initial and final waypoints)
    R(((3+2*n):(3+2*n)),(1:6))               = [  20*T(1)^3   12*T(1)^2   6*T(1)^1 2 0 0];
    R(((4+2*n):(4+2*n)),((1+6*(n-1)):(6*n))) = [20*T(n+1)^3 12*T(n+1)^2 6*T(n+1)^1 2 0 0];
    %Equal velocity, accelaration , jerk, and snap at intermideate waypoints
    for i = 1:n-1
        R(((1+(i-1)+(0*(n-1))+4+(2*n)):(1+(i-1)+(0*(n-1))+4+(2*n))),(1+(6*(i-1)):6*(i+1))) = [  5*T(i+1)^4  4*T(i+1)^3 3*T(i+1)^2 2*T(i+1)^1 1 0   -5*T(i+1)^4  -4*T(i+1)^3 -3*T(i+1)^2 -2*T(i+1)^1 -1 0]; % Equal velocity at intermediate waypoints
        R(((1+(i-1)+(1*(n-1))+4+(2*n)):(1+(i-1)+(1*(n-1))+4+(2*n))),(1+(6*(i-1)):6*(i+1))) = [ 20*T(i+1)^3 12*T(i+1)^2 6*T(i+1)^1          2 0 0  -20*T(i+1)^3 -12*T(i+1)^2 -6*T(i+1)^1          -2  0 0]; % Equal acceleration at intermediate waypoints
        R(((1+(i-1)+(2*(n-1))+4+(2*n)):(1+(i-1)+(2*(n-1))+4+(2*n))),(1+(6*(i-1)):6*(i+1))) = [ 60*T(i+1)^2 24*T(i+1)^1          6          0 0 0  -60*T(i+1)^2 -24*T(i+1)^1          -6           0  0 0]; % Equal jerk at intermediate waypoints
        R(((1+(i-1)+(3*(n-1))+4+(2*n)):(1+(i-1)+(3*(n-1))+4+(2*n))),(1+(6*(i-1)):6*(i+1))) = [120*T(i+1)^1          24          0          0 0 0 -120*T(i+1)^1          -24           0           0  0 0]; % Equal snap at intermediate waypoints
    end
    
    % Boundary Conditions Matrix
    BC = zeros(6*n,1);
    BC(1,1) = Pos(1);     % Position of the first waypoint
    if n > 1
        PosInter = zeros(2*(n-1),1);
    end
    for i = 2:n
        PosInter(2*(i-2)+1:2*(i-2)+2) = [Pos(i);Pos(i)];
    end
    for i = 1:2*(n-1)
        BC(2+(i-1)) = PosInter(i);
    end
    BC(2+2*(n-1)) = Pos(n+1);   % Position of the final waypoint
    BC(3+2*(n-1)) = Vel(1);     % initial velocity
    BC(4+2*(n-1)) = Vel(2);     % final velocity
    BC(5+2*(n-1)) = Acc(1);     % initial acceleration
    BC(6+2*(n-1)) = Acc(2);     % final acceleration

    % Coefficient  Vector
    Cj = R\BC;
    
end

function Cj = minimumJerkConstrCoefficient1DOF(T,Pos,Vel,Acc)

    n = length(T)-1;  % n= number of trajectories 

    % Waypoints' Matrix
    R = zeros(6*n,6*n); % Start with zero matrix for R, joint
    
    % Positions of all waypoints
    for i = 1:n
        % Position
        R(2*i-1:2*i-1        ,6*i-5:6*i) = [T(i)^5      T(i)^4    T(i)^3    T(i)^2    T(i)   1];
        R(2*i-0:2*i+0        ,6*i-5:6*i) = [T(i+1)^5    T(i+1)^4  T(i+1)^3  T(i+1)^2  T(i+1) 1];
        % Velocity
        R(2*n+2*i-1:2*n+2*i-1,6*i-5:6*i) = [5*T(i)^4	4*T(i)^3	3*T(i)^2      2*T(i)	1 0];
        R(2*n+2*i-0:2*n+2*i-0,6*i-5:6*i) = [5*T(i+1)^4	4*T(i+1)^3	3*T(i+1)^2    2*T(i+1)  1 0];
        % Accelaration
        R(4*n+2*i-1:4*n+2*i-1,6*i-5:6*i) = [20*T(i)^3 	12*T(i)^2 	6*T(i)      2	0 0];
        R(4*n+2*i-0:4*n+2*i-0,6*i-5:6*i) = [20*T(i+1)^3	12*T(i+1)^2	6*T(i+1)	2	0 0];
    end
    
    % Boundary Conditions Matrix
    BC = zeros(6*n,1);
    BC(1,1) = Pos(:,1);      	% Position of the first waypoint
    BC(1+2*n:1+2*n) = Vel(:,1);	% Velocity of the first waypoint
    BC(1+4*n:1+4*n) = Acc(:,1);	% Acceleration of the first waypoint
    if n > 1
        PosInter = zeros(2*(n-1),1);
        VelInter = zeros(2*(n-1),1);
        AccInter = zeros(2*(n-1),1);
    end
    for i = 2:n
        PosInter(2*i-3:2*i-2,:) = [Pos(:,i)';Pos(:,i)'];
        VelInter(2*i-3:2*i-2,:) = [Vel(:,i)';Vel(:,i)'];
        AccInter(2*i-3:2*i-2,:) = [Acc(:,i)';Acc(:,i)'];
    end
    for i = 1:2*(n-1)
        % Position
        BC(1+i:i+1) = PosInter(i,:);
        % Velocity
        BC(1+2*n+i:2*n+i+1) = VelInter(i,:);
        % Acceelration
        BC(1+4*n+i:4*n+i+1) = AccInter(i,:);
    end
    BC(1+2*n-1:2*n+0) = Pos(:,n+1); % Position of the final waypoint
    BC(1+2*n+1:2*n+2) = Vel(:,n+1); % Velocity of the final waypoint
    BC(1+2*n+3:2*n+4) = Acc(:,n+1); % Acceleration of the final waypoint

    % Coefficient  Vector
    Cj = R\BC;
    
end

function MagInset(h, ax, ZoomArea, InsetPos, Lines, LineStyle)
    % ************ MagInset.m *************
    % * Created: January 1, 2015          *
    % * By: Damien Frost                  *
    % * Inspiration By: Robert Richardson *
    % *************************************
    % 
    % MagInset (Magnified Inset) places an axes inset into an existing 
    % figure with handle h. Re-size your figure and axes before running
    % MagInset.
    %
    % h - handle to a figure on which to place an inset
    %
    % ax - handle to an axes object. Set to -1 to use the default axes in the
    %      figure.
    %
    % ZoomArea - a 4x1 array which defines the zoom area that inset
    %            should have. It is of the form: 
    %                [xmin xmax ymin ymax]
    %            The values are NOT normalized values, but graph values. In
    %            this way, you can easily read off the zoom area you would like
    %            to specify.
    %
    % InsetPos - defines the position of the inset. It is of the form:
    %                [xmin xmax ymin ymax]
    %            The values are NOT normalized values, but graph values. In
    %            this way, you can easily read off the inset area you would
    %            like to specify
    %
    % Lines - defines a list of lines to connect the zoom area with the inset
    %         graph. It can be empty. It should be of the form:
    %             Lines = {'NW','SW'; 'NE','SE'};
    %         It can have as many rows as you wish
    %         The first colum is the corner of the Zoom Area. The second column is the
    %         corner of the Inset Axes
    BadInput = 0;
    axesObjs = get(h, 'Children');  %axes handles
    % Determine which axes the inset will be placed on:
    if(ax == -1)
        MainAx = axesObjs(end);
    else
        MainAx = -1;
        for ii=1:1:max(size(axesObjs))
            if(axesObjs(ii) == ax)
                MainAx = axesObjs(ii);
                break;
            end
        end
        if(MainAx == -1)
            % Could not find the desired axes:
            fprintf('\nMagInset Error: Could not find the desired axes in the figure.\n');
            BadInput = 1;
        end
    end
    if(BadInput == 0)
        % Get the plot data:
        dataObjs = get(MainAx, 'Children');
        % Annotation positions are of the form:
        % [x y length height]
        % And are normalized to the figure
        % Calculate the normalize rectangular coordinates for the zoom area:
        [zax, zay] = xy2norm(MainAx, ZoomArea(1:2), ZoomArea(3:4));
        % Create the rectangle around the area we are going to zoom into:
        annotation('rectangle',[zax(1) zay(1) (zax(2) - zax(1)) (zay(2) - zay(1))]);
        % Calculate the inset position in normalized coordinates;
        [ipx, ipy] = xy2norm(MainAx, InsetPos(1:2), InsetPos(3:4));
        if(nargin > 4)
            % Add the lines from the zoom area to the inset:
            numLine = size(Lines,1);
            if((numLine>0) && (size(Lines,2) == 2))
                lx = zeros(2,1);
                ly = zeros(2,1);
                for ii=1:1:numLine
                    jj = 1;
                    % Find the co-ordinate in the zoom area:
                    % y co-ordinates:
                    if(Lines{ii,jj}(1) == 'S')
                        ly(jj) = zay(1);
                    else
                        ly(jj) = zay(2);
                    end
                    % x co-ordinates:
                    if(Lines{ii,jj}(2) == 'W')
                        lx(jj) = zax(1);
                    else
                        lx(jj) = zax(2);
                    end
                    jj = 2;
                    % Find the co-ordinate in the inset axes:
                    % y co-ordinates:
                    if(Lines{ii,jj}(1) == 'S')
                        ly(jj) = ipy(1);
                    else
                        ly(jj) = ipy(2);
                    end
                    % x co-ordinates:
                    if(Lines{ii,jj}(2) == 'W')
                        lx(jj) = ipx(1);
                    else
                        lx(jj) = ipx(2);
                    end
                    % Add the line:
                    ln = annotation('line', lx,ly);
                    ln.LineStyle = LineStyle;
                end
            end
        end
        % Add the second set of axes on the same plot:
        iaxes = axes('position', [ipx(1) ipy(1) (ipx(2) - ipx(1)) (ipy(2) - ipy(1))]);
        hold on;
        box on;
        % Add the plots from the original axes onto the inset axes:
        copyobj(dataObjs,iaxes);
        % set the limits on the new axes:
        xlim(ZoomArea(1:2));
        ylim(ZoomArea(3:4));
        % Our work here is done.
    end
end
function [xn, yn] = xy2norm(axh, x, y)
    % ********* xy2norm.m *********
    % * Created: January 1, 2015  *
    % * By: Damien Frost          *
    % *****************************
    % This function takes a point (x, y) and returns the normalized
    % co-ordinates (xn, yn) given the axes with handle axh.
    %
    % *** Modifications: ***
    % 1) Added 'axh = handle(axh);' line to make the script backwards
    % compatible with MATLAB R2013a (Perhaps further back) - Thanks Shi
    % Zhao!
    axh = handle(axh);
    % Save some temporary variables about the axes:
    axPos = axh.Position;
    xlims = axh.XLim;
    ylims = axh.YLim;
    % Calculate the normalized co-ordinates:
    xn = axPos(1) + axPos(3) .* (x-xlims(1))./(xlims(2) - xlims(1));
    yn = axPos(2) + axPos(4) .* (y-ylims(1))./(ylims(2) - ylims(1));
    % GTFO:
end