clc
clear all
close all

robot_model = 2; % 1: Stäubli RX160 (Inertial parameters are removed!), 2: Franka Emika Panda
ee_att = 0;
static_pose = 0;
origin_static = 0;
animate = 1;

%% Animation

if animate == 1
    app.base_on = 1;
    app.coord_frame_on = 1;
    app.high_quality = 0;
    
    % Load CAD files
    [app.ms_1,app.ms] = loadMAT(robot_model,app);
    
    animFig = figure('Name','Animation');
    animAx = axes(animFig);
    
    view(animAx,3)  % Isometric view
    
    xlabel(animAx,'x')
    ylabel(animAx,'y')
    zlabel(animAx,'z')
    grid(animAx,'on')
    light(animAx,'Position',[-1 0 1])
    
    set(animAx,	'Projection','perspective', ...
                    'PlotBoxAspectRatio',[1 1 1], ...
                    'DataAspectRatio',[1 1 1]);
    
    app.robot_model = robot_model;
    app.ee_att = ee_att;
    
    app.animAx = animAx;
end

%% Environmental features

GRAV = 9.81; % gravity constant

%% Robot Kinematic Parameters

kin = kinematicParameters(robot_model); 

n = kin.n; % number of joints
np = 10;
kin.np = np;
kin.g0 = -[0;0;GRAV]; % gravitational acceleration vector

%% Real Robot Dynamic Parameters

dyn.spring_on = 1;
dyn.friction_on = 1;

% Standard inertial parameters
[dyn.m_j,rj_jcj,Ij_cj] = dynamicParameters(robot_model);

dyn.Ij_j = zeros(3,3,n);
dyn.dj_j = zeros(3,1,n);
pj_j = zeros(np,1,n);
pj_bar = zeros(n*np,1);
for j = 1:n
    S_rj_jcj = SkewSym(rj_jcj(:,:,j));
    dyn.Ij_j(:,:,j) = Ij_cj(:,:,j) - dyn.m_j(j)*S_rj_jcj*S_rj_jcj;
    dyn.dj_j(:,:,j) = dyn.m_j(j)*rj_jcj(:,:,j);

    pj_j(:,:,j) = [SymVec(dyn.Ij_j(:,:,j));dyn.dj_j(:,:,j);dyn.m_j(j)];
    pj_bar((j-1)*np+1:j*np,1) = pj_j(:,:,j);
end

%% Control Parameters

ctr.tcyc = 0.004; % Cycle time

if robot_model == 1
    ctr.Kp_jnt_pid = [400;800;600;70;60;40];
    ctr.Ki_jnt_pid = [1;1;1;1;1;1];
    ctr.Kd_jnt_pid = [3;5;6;3;0.75;0.5];
else
    % ctr.Kp_jnt_pid = [40;80;60;7;6;4;4];
    % ctr.Ki_jnt_pid = [1;1;1;1;1;1;1];
    % ctr.Kd_jnt_pid = [0.3;0.5;0.6;0.3;0.075;0.05;0.05];
    ctr.Kp_jnt_pid = [1;1;1;1;1;1;1];
    ctr.Ki_jnt_pid = [0;0;0;0;0;0;0];
    ctr.Kd_jnt_pid = [0;0;0;0;0;0;0];
end

ctr.comp_grv = 1;

ctr.errsum = zeros(n,1);

%% Simulation

tini = 0; % Start time
% tfin = 10; % Final time

tcyc = ctr.tcyc;
tstp = 0.0001; % Time step

if static_pose == 1
    joint_pos1 = 26.845443;
    joint_pos2 = 16.520273;
    joint_pos3 = 49.560818;
    joint_pos4 = -12.390205;
    joint_pos5 = -24.780409;
    joint_pos6 = 18.585307;
    joint_pos7 = -9.201434;

    joint_vel1 = 0;
    joint_vel2 = 0;
    joint_vel3 = 0;
    joint_vel4 = 0;
    joint_vel5 = 0;
    joint_vel6 = 0;
    joint_vel7 = 0;

    joint_acc1 = 0;
    joint_acc2 = 0;
    joint_acc3 = 0;
    joint_acc4 = 0;
    joint_acc5 = 0;
    joint_acc6 = 0;
    joint_acc7 = 0;

    if origin_static == 1
        if robot_model == 2
            q   = zeros(7,1);
            qd  = zeros(7,1);
            qdd = zeros(7,1);
        else
            q   = zeros(6,1);
            qd  = zeros(6,1);
            qdd = zeros(6,1);
        end
    else
        if robot_model == 2
            q   = deg2rad([joint_pos1;joint_pos2;joint_pos3;joint_pos4;joint_pos5;joint_pos6;joint_pos7]);
            qd  = deg2rad([joint_vel1;joint_vel2;joint_vel3;joint_vel4;joint_vel5;joint_vel6;joint_vel7]);
            qdd = deg2rad([joint_acc1;joint_acc2;joint_acc3;joint_acc4;joint_acc5;joint_acc6;joint_acc7]);
        else
            q   = deg2rad([joint_pos1;joint_pos2;joint_pos3;joint_pos4;joint_pos5;joint_pos6]);
            qd  = deg2rad([joint_vel1;joint_vel2;joint_vel3;joint_vel4;joint_vel5;joint_vel6]);
            qdd = deg2rad([joint_acc1;joint_acc2;joint_acc3;joint_acc4;joint_acc5;joint_acc6]);
        end
    end

    tfin = 5; % Final time

    q_posAct = q;
else
    % 0: unconstrained mj, 1: constrained mj
    mj_constrained = 1;
    
    T = [0 1 2 3 4 5 6];
    
    if robot_model == 1
        Pos = deg2rad([ 90  150 70  10  120 50  90
                        0   60  -60 30  -80 60  0 
                        90  0   20  -60 -10 0   90
                        0   50  0   -10 40  0   0 
                        90  0   90  -90 90  -90 90
                        90  120 90  30  90 -30  90]);
    else
        Pos = deg2rad([ 90  150 70  10  120 50  90
                        0   60  -60 30  -80 60  0 
                        90  0   20  -60 -10 0   90
                        -10 -60 -10 -20 -80 -10 -10 
                        90  0   90  -90 90  -90 90
                        90  120 90  30  90  0   90
                        90  30  -30 -30 -30 30  -60]);
    end
         
    if mj_constrained == 1
        Vel = zeros(n,length(T));
        Acc = zeros(n,length(T));
    else
        Vel = zeros(n,2);
        Acc = zeros(n,2);
    end
    
    Cj = zeros((length(T)-1)*6,6);
    for j = 1:n
        if mj_constrained == 1
            Cj(:,j) = minimumJerkConstrCoefficient1DOF(T,Pos(j,:),Vel(j,:),Acc(j,:));
        else
            Cj(:,j) = minimumJerkCoefficient1DOF(T,Pos(j,:),Vel(j,:),Acc(j,:));
        end
    end

    tfin = T(end);

    q_posAct = Pos(:,1);
end

ttot = ceil(tfin/tcyc)+1;

q_posDes = zeros(n,ttot);
q_velDes = zeros(n,ttot);
q_accDes = zeros(n,ttot);

q_posFbk = zeros(n,ttot);
q_velFbk = zeros(n,ttot);

des.q_pos = zeros(n,1);
des.q_vel = zeros(n,1);
des.q_acc = zeros(n,1);

q_velAct = zeros(n,1);
q_accAct = zeros(n,1);

d = round(tcyc/tstp);
draw_p = round(0.05/tstp);
tloop = 0;
for k = 0:tfin/tstp
    
    if rem(k,d) == 0
        
        kc = k/d+1;
        t = k*tstp;
        
        % Trajectory generation
        if static_pose == 0
            for j = 1:n
                [des.q_pos(j,1),des.q_vel(j,1),des.q_acc(j,1)] = minimumJerkPolynomial1DOF(t,T,Cj(:,j));
            end
        else
            des.q_pos(:,1) = q;
            des.q_vel(:,1) = qd;
            des.q_acc(:,1) = qdd;
        end
        
        q_posDes(:,kc) = des.q_pos;
        q_velDes(:,kc) = des.q_vel;
        q_accDes(:,kc) = des.q_acc;
        
        fbk.q_vel = q_velAct;
        fbk.q_pos = q_posAct;
        
        q_posFbk(:,kc) = fbk.q_pos;
        q_velFbk(:,kc) = fbk.q_vel;
        
        % Control algorithm
        tau = controlAlgo(ctr,kin,dyn,des,fbk);
    end

    % Computation of robot motion
    q_accAct = robotAlgo(robot_model,kin,dyn,tau,q_posAct,q_velAct);
    q_velAct = q_velAct + q_accAct*tstp;
    q_posAct = q_posAct + q_velAct*tstp;
    for j = 1:n
        if q_posAct(j) < deg2rad(kin.q_posLim(j,1))
            q_posAct(j) = deg2rad(kin.q_posLim(j,1));
            q_velAct(j) = 0;
            q_accAct(j) = 0;
        elseif q_posAct(j) > deg2rad(kin.q_posLim(j,2))
            q_posAct(j) = deg2rad(kin.q_posLim(j,2));
            q_velAct(j) = 0;
            q_accAct(j) = 0;
        end
    end
    
    if animate == 1
        if rem(k,draw_p) == 0
            app.q_posAni = q_posAct;
            app.q_posDes = des.q_pos;
            
            cla(app.animAx)                    
            light(app.animAx,'Position',[-1 0 1])
            % plotAxesInstant(app,kin)

            % Robot parameters
            % n = kin.n;
        
            % CAD properties
            if app.base_on == 1 && app.robot_model == 1
                Pobj_1.pB = patch(app.animAx,'Faces', app.ms.sB.F, 'Vertices', app.ms.sB.V);
                set(Pobj_1.pB, 'facec', 'flat');              % Set the face color flat
                set(Pobj_1.pB, 'FaceColor', app.ms.sB.C);         % Set the color (from file)
                set(Pobj_1.pB, 'EdgeColor', 'none');          % Set the edge color
            end
            Pobj_1.p0 = patch(app.animAx,'Faces', app.ms_1.s0.F, 'Vertices', app.ms_1.s0.V);
            set(Pobj_1.p0, 'facec', 'flat');                  % Set the face color flat
            set(Pobj_1.p0, 'FaceColor', app.ms_1.s0.C);           % Set the color (from file)
            set(Pobj_1.p0, 'EdgeColor', 'none');              % Set the edge color
            Pobj_1.p1 = patch(app.animAx,'Faces', app.ms_1.s1.F, 'Vertices', app.ms_1.s1.V);
            set(Pobj_1.p1, 'facec', 'flat');                  % Set the face color flat
            set(Pobj_1.p1, 'FaceColor', app.ms_1.s1.C);           % Set the color (from file)
            set(Pobj_1.p1, 'EdgeColor', 'none');              % Set the edge color
            Pobj_1.p2 = patch(app.animAx,'Faces', app.ms_1.s2.F, 'Vertices', app.ms_1.s2.V);
            set(Pobj_1.p2, 'facec', 'flat');                  % Set the face color flat
            set(Pobj_1.p2, 'FaceColor', app.ms_1.s2.C);           % Set the color (from file)
            set(Pobj_1.p2, 'EdgeColor', 'none');              % Set the edge color
            Pobj_1.p3 = patch(app.animAx,'Faces', app.ms_1.s3.F, 'Vertices', app.ms_1.s3.V);
            set(Pobj_1.p3, 'facec', 'flat');                  % Set the face color flat
            set(Pobj_1.p3, 'FaceColor', app.ms_1.s3.C);           % Set the color (from file)
            set(Pobj_1.p3, 'EdgeColor', 'none');              % Set the edge color
            Pobj_1.p4 = patch(app.animAx,'Faces', app.ms_1.s4.F, 'Vertices', app.ms_1.s4.V);
            set(Pobj_1.p4, 'facec', 'flat');                  % Set the face color flat
            set(Pobj_1.p4, 'FaceColor', app.ms_1.s4.C);           % Set the color (from file)
            set(Pobj_1.p4, 'EdgeColor', 'none');              % Set the edge color
            Pobj_1.p5 = patch(app.animAx,'Faces', app.ms_1.s5.F, 'Vertices', app.ms_1.s5.V);
            set(Pobj_1.p5, 'facec', 'flat');                  % Set the face color flat
            set(Pobj_1.p5, 'FaceColor', app.ms_1.s5.C);           % Set the color (from file)
            set(Pobj_1.p5, 'EdgeColor', 'none');              % Set the edge color
            Pobj_1.p6 = patch(app.animAx,'Faces', app.ms_1.s6.F, 'Vertices', app.ms_1.s6.V);
            set(Pobj_1.p6, 'facec', 'flat');                  % Set the face color flat
            set(Pobj_1.p6, 'FaceColor', app.ms_1.s6.C);           % Set the color (from file)
            set(Pobj_1.p6, 'EdgeColor', 'none');              % Set the edge color
            if app.robot_model == 2
                Pobj_1.p7 = patch(app.animAx,'Faces', app.ms_1.s7.F, 'Vertices', app.ms_1.s7.V);
                set(Pobj_1.p7, 'facec', 'flat');                  % Set the face color flat
                set(Pobj_1.p7, 'FaceColor', app.ms_1.s7.C);           % Set the color (from file)
                set(Pobj_1.p7, 'EdgeColor', 'none');              % Set the edge color
            end
            if app.coord_frame_on == 1
                Pobj_1.pF = patch(app.animAx,'Faces', app.ms.sF.F, 'Vertices', app.ms.sF.V);
                set(Pobj_1.pF, 'facec', 'flat');                % Set the face color flat
                set(Pobj_1.pF, 'FaceVertexCData', app.ms.sF.C);	% Set the color (from file)
                set(Pobj_1.pF, 'EdgeColor', 'none');            % Set the edge color
            end
        
            Pobj_1d.p1d = Pobj_1.p1;
            Pobj_1d.p2d = Pobj_1.p2;
            Pobj_1d.p3d = Pobj_1.p3;
            Pobj_1d.p4d = Pobj_1.p4;
            Pobj_1d.p5d = Pobj_1.p5;
            Pobj_1d.p6d = Pobj_1.p6;
            if app.robot_model == 2
                Pobj_1d.p7d = Pobj_1.p7;
            end
            if app.coord_frame_on == 1
                Pobj_1d.pFd = Pobj_1.pF;
            end
        
            sensor_angle_1 = 0;
        
            hold(app.animAx,'on')
        
            TI_0_1	= eye(4);
        
            TAniI_i_1 = getTransMatrix(TI_0_1,kin.a_j,kin.alpha_j,kin.d_j,kin.theta_O_j,app.q_posAni);
            TI_EFinAni = TAniI_i_1(:,:,n+1)*Trn_z(kin.d_j(n+1));
        
            TDesI_i_1 = getTransMatrix(TI_0_1,kin.a_j,kin.alpha_j,kin.d_j,kin.theta_O_j,app.q_posDes);
            TI_EFinDes = TDesI_i_1(:,:,n+1)*Trn_z(kin.d_j(n+1));
        
            if app.base_on == 1 && app.robot_model == 1, nvB = [app.ms.sB.V,ones(size(app.ms.sB.V(:,1)))]*TAniI_i_1(:,:,1)'; end
            nv1d = [app.ms_1.s1.V,ones(size(app.ms_1.s1.V(:,1)))]*TDesI_i_1(:,:,2)';
            nv2d = [app.ms_1.s2.V,ones(size(app.ms_1.s2.V(:,1)))]*TDesI_i_1(:,:,3)';
            nv3d = [app.ms_1.s3.V,ones(size(app.ms_1.s3.V(:,1)))]*TDesI_i_1(:,:,4)';
            nv4d = [app.ms_1.s4.V,ones(size(app.ms_1.s4.V(:,1)))]*TDesI_i_1(:,:,5)';
            nv5d = [app.ms_1.s5.V,ones(size(app.ms_1.s5.V(:,1)))]*TDesI_i_1(:,:,6)';
            nv6d = [app.ms_1.s6.V,ones(size(app.ms_1.s6.V(:,1)))]*TDesI_i_1(:,:,7)';
            if app.robot_model == 2
                nv7d = [app.ms_1.s7.V,ones(size(app.ms_1.s7.V(:,1)))]*TDesI_i_1(:,:,8)';
            end
            nv0 = [app.ms_1.s0.V,ones(size(app.ms_1.s0.V(:,1)))]*TAniI_i_1(:,:,1)';
            nv1 = [app.ms_1.s1.V,ones(size(app.ms_1.s1.V(:,1)))]*TAniI_i_1(:,:,2)';
            nv2 = [app.ms_1.s2.V,ones(size(app.ms_1.s2.V(:,1)))]*TAniI_i_1(:,:,3)';
            nv3 = [app.ms_1.s3.V,ones(size(app.ms_1.s3.V(:,1)))]*TAniI_i_1(:,:,4)';
            nv4 = [app.ms_1.s4.V,ones(size(app.ms_1.s4.V(:,1)))]*TAniI_i_1(:,:,5)';
            nv5 = [app.ms_1.s5.V,ones(size(app.ms_1.s5.V(:,1)))]*TAniI_i_1(:,:,6)';
            nv6 = [app.ms_1.s6.V,ones(size(app.ms_1.s6.V(:,1)))]*TAniI_i_1(:,:,7)';
            if app.robot_model == 2
                nv7 = [app.ms_1.s7.V,ones(size(app.ms_1.s7.V(:,1)))]*TAniI_i_1(:,:,8)';
            end
            if app.coord_frame_on == 1
                nvFd = [app.ms.sF.V,ones(size(app.ms.sF.V(:,1)))]*Rot_z(sensor_angle_1)'*TI_EFinDes';
                nvF = [app.ms.sF.V,ones(size(app.ms.sF.V(:,1)))]*Rot_z(sensor_angle_1)'*TI_EFinAni';
            end
        
            if app.base_on == 1 && app.robot_model == 1, set(Pobj_1.pB,'Vertices',nvB(:,1:3)), end
            set(Pobj_1.p0,'Vertices',nv0(:,1:3))
        
            set(Pobj_1d.p1d,'Vertices',nv1d(:,1:3),'FaceAlpha',1)
            set(Pobj_1d.p2d,'Vertices',nv2d(:,1:3),'FaceAlpha',1)
            set(Pobj_1d.p3d,'Vertices',nv3d(:,1:3),'FaceAlpha',1)
            set(Pobj_1d.p4d,'Vertices',nv4d(:,1:3),'FaceAlpha',1)
            set(Pobj_1d.p5d,'Vertices',nv5d(:,1:3),'FaceAlpha',1)
            set(Pobj_1d.p6d,'Vertices',nv6d(:,1:3),'FaceAlpha',1)
            if app.robot_model == 2
                set(Pobj_1d.p7d,'Vertices',nv7d(:,1:3),'FaceAlpha',1)
            end
        
            set(Pobj_1.p1,'Vertices',nv1(:,1:3))
            set(Pobj_1.p2,'Vertices',nv2(:,1:3))
            set(Pobj_1.p3,'Vertices',nv3(:,1:3))
            set(Pobj_1.p4,'Vertices',nv4(:,1:3))
            set(Pobj_1.p5,'Vertices',nv5(:,1:3))
            set(Pobj_1.p6,'Vertices',nv6(:,1:3))
            if app.robot_model == 2
                set(Pobj_1.p7,'Vertices',nv7(:,1:3))
            end
            if app.coord_frame_on == 1
                set(Pobj_1d.pFd,'Vertices',nvFd(:,1:3),'FaceAlpha',1)
                set(Pobj_1.pF,'Vertices',nvF(:,1:3))
            end
        
            hold(app.animAx,'off')
        
            drawnow

        end
    end
    
end

xd = 0:tcyc:tfin;

for j = 1:n
    yd(:,j) = rad2deg(q_posDes(j,:));
    yf(:,j) = rad2deg(q_posFbk(j,:));
end

% Fig 1
figure(1);
ax(1) = subplot(n,1,1);
plot(xd,yd(:,1),'b-',xd,yf(:,1),'r--','LineWidth',2); 
legend('Desired','Actual', 'Location', 'Best');
for j = 2:n
    ax(j) = subplot(n,1,j);
    plot(xd,yd(:,j),'b-',xd,yf(:,j),'r--','LineWidth',2); 
end

% Fig 2
figure(2);
sub(1) = subplot(n,1,1);
plot(xd,yd(:,1)-yf(:,1),'b-','LineWidth',2);
for j = 2:n
    sub(j) = subplot(n,1,j);
    plot(xd,yd(:,j)-yf(:,j),'b-','LineWidth',2);
end

%% Functions

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

function TI_i = getTransMatrix(TI_0,a_j,alpha_j,d_j,theta_j,q_j)
    
    k = length(q_j);

    TI_i = zeros(4,4,k+1);

    TI_i(:,:,1) = TI_0;

    for j = 1:k        
        Rot_x_i = Rot_x(alpha_j(j));
        Trn_x_i	= Trn_x(a_j(j));
        Rot_z_j = Rot_z(theta_j(j)+q_j(j));
        Trn_z_j = Trn_z(d_j(j));

        TI_i(:,:,j+1) = TI_i(:,:,j) * Rot_x_i * Trn_x_i * Rot_z_j * Trn_z_j;
    end
    
end

function tau = controlAlgo(ctr,kin,dyn,des,fbk)
    %% Error Calculation
    
    err_m = des.q_pos - fbk.q_pos; % e
    errdot_m = des.q_vel - fbk.q_vel; % edot
    ctr.errsum = ctr.errsum + err_m*ctr.tcyc;
    
    %% Control Torque

    % PID
    tau_ctrl = ctr.Kp_jnt_pid.*err_m + ctr.Ki_jnt_pid.*ctr.errsum + ctr.Kd_jnt_pid.*errdot_m;
    
    %% Compensation Torque
    
    if ctr.comp_grv == 1
        tau_grv = getTauG(kin,fbk.q_pos,kin.g0,dyn.m_j,dyn.dj_j);
    else
        tau_grv = zeros(kin.n,1);
    end
    
    %% Command Torque
    
    tau = tau_ctrl + tau_grv;

end

function q_acc = robotAlgo(robot_model,kin,dyn,tau,q_pos,q_vel)

    n = kin.n;
    
    Phij_j = [zeros(3,1);kin.zj_j];

    if dyn.spring_on == 1 && robot_model == 1
        tau_spr = springModel(q_pos);
    else
        tau_spr = zeros(n,1);
    end

    if dyn.friction_on == 1
        if robot_model == 2
            tau_frc = frictionFERModel(q_vel);
        else
            tau_frc = frictionPWModel(q_vel);
        end
    else
        tau_frc = zeros(n,1);
    end

    tau_j = tau - (tau_spr + tau_frc);

    % i) First forward recursive computations for i = 1, ..., n
    Rj_i        = zeros(3,3,n);
    wi_i        = zeros(3,1);
    gammaj_j    = zeros(6,1,n);
    betai_i     = zeros(6,1,n+1);
    Gammai_i    = zeros(6,6,n+1);
    for j = 1:n
        Rj_i(:,:,j)         = getRi_j(kin.alpha_j(j),kin.theta_O_j(j)+q_pos(j))';
        gammaj_j(:,:,j)     = [Rj_i(:,:,j)*(cross(wi_i,cross(wi_i,getri_j(kin.alpha_j(j),kin.a_j(j),kin.d_j(j)))))
                                cross(Rj_i(:,:,j)*wi_i,q_vel(j)*kin.zj_j)];
        wi_i                = Rj_i(:,:,j)*wi_i + q_vel(j)*kin.zj_j;
        betai_i(:,:,j+1)    = -[cross(wi_i,cross(wi_i,dyn.dj_j(:,:,j)));
                                cross(wi_i,dyn.Ij_j(:,:,j)*wi_i)];
        Gammai_i(:,:,j+1)   = [dyn.m_j(j)*eye(3),-SkewSym(dyn.dj_j(:,:,j));SkewSym(dyn.dj_j(:,:,j)),dyn.Ij_j(:,:,j)];
    end

    % ii) Backward recursive computations for i = n, ..., 1
    H_j                 = zeros(n,1);
    Xj_i                = zeros(6,6,n);
    betaSi_i            = zeros(6,1,n+1);
    betaSi_i(:,:,n+1)   = betai_i(:,:,n+1);
    GammaSi_i           = zeros(6,6,n+1);
    GammaSi_i(:,:,n+1)  = Gammai_i(:,:,n+1);
    for j = n:-1:1
        H_j(j)              = Phij_j'*GammaSi_i(:,:,j+1)*Phij_j;
        KKi_i               = GammaSi_i(:,:,j+1) - GammaSi_i(:,:,j+1)*Phij_j*(1/H_j(j))*Phij_j'*GammaSi_i(:,:,j+1);
        alphaj_j            = KKi_i*gammaj_j(:,:,j) + GammaSi_i(:,:,j+1)*Phij_j*(1/H_j(j)) ...
                            *(tau_j(j)+Phij_j'*betaSi_i(:,:,j+1)) - betaSi_i(:,:,j+1);
        Xj_i(:,:,j)         = SO3R3_R66_twist(Rj_i(:,:,j),getrj_i(kin.theta_O_j(j)+q_pos(j),kin.a_j(j),kin.d_j(j)));
        betaSi_i(:,:,j)     = betai_i(:,:,j) - Xj_i(:,:,j)'*alphaj_j;
        GammaSi_i(:,:,j)    = Gammai_i(:,:,j) + Xj_i(:,:,j)'*KKi_i*Xj_i(:,:,j);
    end

    % iii) Second forward recursive computations for i = 1, ..., n
    q_acc = zeros(n,1);
    aai_i = [-kin.g0;zeros(3,1)];
    for j = 1:n
        aaj_i       = Xj_i(:,:,j)*aai_i;
        q_acc(j)    = (1/H_j(j))*(-Phij_j'*GammaSi_i(:,:,j+1)*(aaj_i ...
                    + gammaj_j(:,:,j)) + tau_j(j) + Phij_j'*betaSi_i(:,:,j+1));
        aai_i       = aaj_i + Phij_j*q_acc(j) + gammaj_j(:,:,j);
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

function tau_frc = frictionFERModel(qd)

    phi1 = [5.4615e-01;0.87224;6.4068e-01;1.2794e+00;8.3904e-01;3.0301e-01;5.6489e-01];
    phi2 = [5.1181e+00;9.0657e+00;1.0136e+01;5.5903e+00;8.3469e+00;1.7133e+01;1.0336e+01];
    phi3 = [3.9533e-02;2.5882e-02;-4.6070e-02;3.6194e-02;2.6226e-02;-2.1047e-02;3.5526e-03];

    tau_frc = zeros(7,1);
    for j = 1:7
        tau_frc(j) = phi1(j)/(1+exp(-phi2(j)*(qd(j)+phi3(j))))-phi1(j)/(1+exp(-phi2(j)*phi3(j)));
    end

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

function output = rot_x(input)
    output = [	1	0           0
                0	cos(input)  -sin(input)
                0	sin(input)	cos(input)];
end

function output = rot_z(input)
    output = [  cos(input)	-sin(input)	0
                sin(input)	 cos(input)	0
                0            0          1];
end

function output = trn_x(input)
    output = [input
              0
              0];
end

function output = trn_z(input)
    output = [0
              0
              input];
end

function kin = kinematicParameters(robot_model)

    switch robot_model
        case 1
            DH = [ 0     0     0.550 0
                   0.150 -pi/2 0     -pi/2
                   0.825 0     0     pi/2
                   0     pi/2  0.625 0
                   0     -pi/2 0     0
                   0     pi/2  0     0
                   0     0     0.110 0    ];

            kin.j_type = [1;1;1;1;1;1];
        case 2
            DH = [ 0       0     0.333 0
                   0       -pi/2 0     0
                   0       pi/2  0.316 0
                   0.0825  pi/2  0     0
                   -0.0825 -pi/2 0.384 0
                   0       pi/2  0     0
                   0.088   pi/2  0     0
                   0       0     0.107 0    ];

            kin.j_type = [1;1;1;1;1;1;1];
        otherwise
    end

    kin.a_j = DH(:,1);
    kin.alpha_j = DH(:,2);
    kin.d_j = DH(:,3);
    kin.theta_O_j = DH(:,4);
    kin.n = size(DH,1)-1;

    kin.r1 = 1; % first revolute joint
    kin.r2 = 2; % subsequent revolute joint whose axis is not parallel to the axis of joint r1

    kin.q_posLim = jointLimits(robot_model);
    kin.zj_j = [0;0;1];

end

function Ri_j = getRi_j(alpha,theta)
    Ri_j = rot_x(alpha)*rot_z(theta);
end

function ri_j = getri_j(alpha,a,d)
    ri_j = [a;-d*sin(alpha);d*cos(alpha)];
end

function rj_i = getrj_i(theta,a,d)
    rj_i = [-a*cos(theta);a*sin(theta);-d];
end

function [m_j,rj_jcj,Ij_cj] = dynamicParameters(robot_model)

    if robot_model == 2
        m_j = [4.970684;0.646926;3.228604;3.587895;1.225946;1.666555;7.35522e-01];
    
        rj_jcj(:,:,1) = [3.875e-03;2.081e-03;0];
        rj_jcj(:,:,2) = [-3.141e-03;-2.872e-02;3.495e-03];
        rj_jcj(:,:,3) = [2.7518e-02;3.9252e-02;-6.6502e-02];
        rj_jcj(:,:,4) = [-5.317e-02;1.04419e-01;2.7454e-02];
        rj_jcj(:,:,5) = [-1.1953e-02;4.1065e-02;-3.8437e-02];
        rj_jcj(:,:,6) = [6.0149e-02;-1.4117e-02;-1.0517e-02];
        rj_jcj(:,:,7) = [1.0517e-02;-4.252e-03;6.1597e-02];

        Ij_cj(:,:,1) = symmetrize(7.0337e-01 ,-1.3900e-04 ,6.7720e-03 ...
                                             ,7.0661e-01  ,1.9169e-02 ...
                                                          ,9.1170e-03);
        Ij_cj(:,:,2) = symmetrize(7.9620e-03 ,-3.9250e-03 ,1.0254e-02 ...
                                             ,2.8110e-02  ,7.0400e-04 ...
                                                          ,2.5995e-02);
        Ij_cj(:,:,3) = symmetrize(3.7242e-02 ,-4.7610e-03 ,-1.1396e-02 ...
                                             ,-1.1396e-02 ,-1.2805e-02 ...
                                                          ,1.0830e-02);
        Ij_cj(:,:,4) = symmetrize(2.5853e-02 ,7.7960e-03  ,-1.3320e-03 ...
                                             ,1.9552e-02  ,8.6410e-03 ...
                                                          ,2.8323e-02);
        Ij_cj(:,:,5) = symmetrize(3.5549e-02 ,-2.1170e-03 ,-4.0370e-03 ...
                                             ,2.9474e-02  ,2.2900e-04 ...
                                                          ,8.6270e-03);
        Ij_cj(:,:,6) = symmetrize(1.9640e-03 ,1.0900e-04  ,-1.1580e-03 ...
                                             ,4.3540e-03  ,3.4100e-04 ...
                                                          ,5.4330e-03);
        Ij_cj(:,:,7) = symmetrize(1.2516e-02 ,-4.2800e-04 ,-1.1960e-03 ...
                                             ,1.0027e-02  ,-7.4100e-04 ...
                                                          ,4.8150e-03);
    else
        % (PARAMETERS ARE NOT INCLUDED BECAUSE OF CONFIDENTIALITY!)
    end

end

function I = symmetrize(Ixx,Ixy,Ixz,Iyy,Iyz,Izz)
    I = [Ixx Ixy Ixz
         Ixy Iyy Iyz
         Ixz Iyz Izz];
end

function q_posLim = jointLimits(robot_model)

    if robot_model == 2
        q_posLim = rad2deg([-2.8973 ,2.8973
                            -1.7628 ,1.7628
                            -2.8973 ,2.8973
                            -3.0718 ,-0.0698
                            -2.8973 ,2.8973
                            -0.0175 ,3.7525
                            -2.8973 ,2.8973]);
    else
        q_posLim = [-160    ,160
                    -137.5  ,137.5
                    -150    ,150
                    -270    ,270
                    -105    ,120
                    -270    ,270];
    end

end

function S = SkewSym(r)

    S = [   0       , -r(3) , r(2)
            r(3)	, 0     , -r(1)
            -r(2)   , r(1)  , 0     ];

end

function D = DotMat(r)
    D = [   r(1)	, r(2)  , r(3)  , 0 	, 0     , 0
            0       , r(1)  , 0     , r(2)  , r(3)  , 0
            0       , 0     , r(1)  , 0     , r(2)  , r(3)  ];
end

function L = SymVec(I)
    L = [I(1,1);I(1,2);I(1,3);I(2,2);I(2,3);I(3,3)];
end

function I = VecSym(L)
    I = [L(1),L(2),L(3)
         L(2),L(4),L(5)
         L(3),L(5),L(6)];
end

function M = R10_skew66(v)
    M = [v(10)*eye(3),-SkewSym(v(7:9));SkewSym(v(7:9)),VecSym(v(1:6))];
end

function T = SO3R3_SE3(R,r)
    T = [R,r;zeros(1,3),1];
end

function [R,r] = SE3_SO3R3(T)
    R = T(1:3,1:3);
    r = T(1:3,4);
end

function X = SO3R3_R66_twist(R,r)
    X = [R,SkewSym(r)*R;zeros(3),R];
end

function X = SO3R3_XR_twist(R)
    X = [R,zeros(3);zeros(3),R];
end

% function X = SO3R3_R66_wrench(R,r)
%     X = [R,zeros(3);SkewSym(r)*R,R];
% end

function J = SO3R3_J(z,r_ij)
    J = [SkewSym(r_ij)*z;z];
end

function Jd = get_Jd(z,r_ij,w,v_ij)
    Jd = [ (SkewSym(r_ij)*SkewSym(w) + SkewSym(v_ij))*z;SkewSym(w)*z ];
end

function Xdt = get_Xd_twist(R,r_ij,w,v_ij)
    Xd11 = SkewSym(w)*R;
    Xd12 = SkewSym(r_ij)*Xd11 + SkewSym(v_ij)*R;

    Xdt = [ Xd11    ,Xd12
            zeros(3),Xd11];
end

function tau_gj = getTauG(kin,q,g,m_j,dj_j)

    n = kin.n;
    Rj_i = zeros(3,3,n+1);
    
    % output declaration
    tau_gj = zeros(n,1);
    
    % forward recursion (link 1 to n)
    gi_i = zeros(3,1,n+1);
    gi_i(:,:,1) = -g;
    for j = 1:n
        % take transpoze one time for each step
        Rj_i(:,:,j) = getRi_j(kin.alpha_j(j),kin.theta_O_j(j)+q(j))';
        gi_i(:,:,j+1) = Rj_i(:,:,j)*gi_i(:,:,j);
    end
    
    % backward recursion recursion (link n to 1)
    ri_j = zeros(3,1);
    fi_fi = zeros(3,1);
    ni_fi = zeros(3,1);
    zj_j = kin.zj_j;
    for j = n:-1:1
        ni_fi = -SkewSym(gi_i(:,:,j+1))*dj_j(:,:,j) + Rj_i(:,:,j+1)'*(SkewSym(Rj_i(:,:,j+1)*ri_j)*fi_fi+ni_fi);
        fi_fi = gi_i(:,:,j+1)*m_j(j) + Rj_i(:,:,j+1)'*fi_fi;
        ri_j = getri_j(kin.alpha_j(j),kin.a_j(j),kin.d_j(j));
        tau_gj(j) = zj_j'*ni_fi;
    end
end

function [tau_fj,phatj_bar] = ANEA(ctr,kin,q,qd,qRd,qRdd,g,p,Pdiag)

    n = kin.n;
    np = kin.np;
    zj_j = kin.zj_j;
    Rj_i = zeros(3,3,n+1);
    ri_j = zeros(3,1,n+1);
    
    % output declaration
    tau_fj = zeros(n,1);
    phatj_bar = zeros(n*np,1);
    
    % forward recursion (link 1 to n)
    wi_i = zeros(3,1);
    wRi_i = zeros(3,1);
    Ui_i = zeros(3,3);
    wdi_i = zeros(3,1);
    Psifi_i = zeros(3,np,n+1);
    Psini_i = zeros(3,np,n+1);
    swi_i = zeros(3,1);
    svi_i = zeros(3,1);
    phatj_j = zeros(np,1,n);
    mui_i = -g;
    for j = 1:n
        % take transpoze one time for each step
        Rj_i(:,:,j) = getRi_j(kin.alpha_j(j),kin.theta_O_j(j)+q(j))';
        ri_j(:,:,j) = getri_j(kin.alpha_j(j),kin.a_j(j),kin.d_j(j));
        
        wdi_i = Rj_i(:,:,j)*wdi_i + (cross(Rj_i(:,:,j)*wRi_i,qd(j)*zj_j) + cross(Rj_i(:,:,j)*wi_i,qRd(j)*zj_j))/2 + qRdd(j)*zj_j;
        mui_i = Rj_i(:,:,j)*(mui_i + Ui_i*ri_j(:,:,j));

        % calculate normal and reference ang vel
        wi_i = Rj_i(:,:,j)*wi_i + qd(j)*zj_j;
        wRi_i = Rj_i(:,:,j)*wRi_i + qRd(j)*zj_j;

        Ui_i = SkewSym(wdi_i) + (SkewSym(wi_i)*SkewSym(wRi_i) + SkewSym(wRi_i)*SkewSym(wi_i))/2;
        Oh_h = DotMat(wdi_i) + (SkewSym(wRi_i)*DotMat(wi_i) + SkewSym(wi_i)*DotMat(wRi_i))/2;

        Psifi_i(:,:,j+1) = [zeros(3,6),Ui_i,mui_i];
        Psini_i(:,:,j+1) = [Oh_h,-SkewSym(mui_i),zeros(3,1)];

        % calculate task space sliding variables
        svi_i = Rj_i(:,:,j)*(svi_i + cross(swi_i,ri_j(:,:,j)));
        swi_i = wi_i - wRi_i;
        
        sigmai_i = Psifi_i(:,:,j+1)'*svi_i + Psini_i(:,:,j+1)'*swi_i;
        Pi = Pdiag((j-1)*np+1:j*np);
        phati_iprev = p((j-1)*np+1:j*np,1);
        phatj_j(:,:,j) = phati_iprev - Pi.*sigmai_i*ctr.tcyc;
        phatj_bar((j-1)*np+1:j*np) = phatj_j(:,:,j);
    end
    
    % backward recursion recursion (link n to 1)
    fi_fi = zeros(3,1);
    ni_fi = zeros(3,1);
    for j = n:-1:1
        ni_fi = Psini_i(:,:,j+1)*phatj_j(:,:,j) + Rj_i(:,:,j+1)'*(SkewSym(Rj_i(:,:,j+1)*ri_j(:,:,j+1))*fi_fi+ni_fi);
        fi_fi = Psifi_i(:,:,j+1)*phatj_j(:,:,j) + Rj_i(:,:,j+1)'*fi_fi;
        tau_fj(j) = zj_j'*ni_fi;
    end
end

function [Pj,Vj,Aj,Jj] = minimumJerkPolynomial1DOF(t,T,Cj)

    k = length(T)-1;  % k = number of trajectories
    
    % Trajectory Polynomial
    for j = k+1:-1:2
        if t <= T(j)
            i = j-1;
        end
    end
    
    Pj = Cj(1+6*(i-1))*t^5 + Cj(2+6*(i-1))*t^4 + Cj(3+6*(i-1))*t^3 + Cj(4+6*(i-1))*t^2 + Cj(5+6*(i-1))*t + Cj(6+6*(i-1));
    Vj = 5*Cj(1+6*(i-1))*t^4 + 4*Cj(2+6*(i-1))*t^3 + 3*Cj(3+6*(i-1))*t^2 + 2*Cj(4+6*(i-1))*t + Cj(5+6*(i-1));
    Aj = 5*4*Cj(1+6*(i-1))*t^3 + 4*3*Cj(2+6*(i-1))*t^2 + 3*2*Cj(3+6*(i-1))*t + 2*Cj(4+6*(i-1));
    Jj = 5*4*3*Cj(1+6*(i-1))*t^2 + 4*3*2*Cj(2+6*(i-1))*t + 3*2*Cj(3+6*(i-1));

end

function Cj = minimumJerkCoefficient1DOF(T,Pos,Vel,Acc)

    k = length(T)-1;  % k = number of trajectories 

    % Waypoints' Matrix
    R = zeros(6*k,6*k); % Start with zero matrix for R, joint
    
    % Positions of all waypoints
    for i = 1:k
        R(1+2*(i-1),1+6*(i-1):6+6*(i-1)) = [T(i)^5 T(i)^4 T(i)^3 T(i)^2 T(i)^1 1];
        R(2+2*(i-1),1+6*(i-1):6+6*(i-1)) = [T(i+1)^5 T(i+1)^4 T(i+1)^3 T(i+1)^2 T(i+1)^1 1];
    end
    % Velocity boundary conditions (inital and final waypoints)        
    R(((1+2*k):(1+2*k)),(1:6))               = [  5*T(1)^4   4*T(1)^3   3*T(1)^2   2*T(1) 1 0];
    R(((2+2*k):(2+2*k)),((1+6*(k-1)):(6*k))) = [5*T(k+1)^4 4*T(k+1)^3 3*T(k+1)^2 2*T(k+1) 1 0];
    % Equal Accelaration boundary conditions (initial and final waypoints)
    R(((3+2*k):(3+2*k)),(1:6))               = [  20*T(1)^3   12*T(1)^2   6*T(1)^1 2 0 0];
    R(((4+2*k):(4+2*k)),((1+6*(k-1)):(6*k))) = [20*T(k+1)^3 12*T(k+1)^2 6*T(k+1)^1 2 0 0];
    %Equal velocity, accelaration , jerk, and snap at intermideate waypoints
    for i = 1:k-1
        R(((1+(i-1)+(0*(k-1))+4+(2*k)):(1+(i-1)+(0*(k-1))+4+(2*k))),(1+(6*(i-1)):6*(i+1))) = [  5*T(i+1)^4  4*T(i+1)^3 3*T(i+1)^2 2*T(i+1)^1 1 0   -5*T(i+1)^4  -4*T(i+1)^3 -3*T(i+1)^2 -2*T(i+1)^1 -1 0]; % Equal velocity at intermediate waypoints
        R(((1+(i-1)+(1*(k-1))+4+(2*k)):(1+(i-1)+(1*(k-1))+4+(2*k))),(1+(6*(i-1)):6*(i+1))) = [ 20*T(i+1)^3 12*T(i+1)^2 6*T(i+1)^1          2 0 0  -20*T(i+1)^3 -12*T(i+1)^2 -6*T(i+1)^1          -2  0 0]; % Equal acceleration at intermediate waypoints
        R(((1+(i-1)+(2*(k-1))+4+(2*k)):(1+(i-1)+(2*(k-1))+4+(2*k))),(1+(6*(i-1)):6*(i+1))) = [ 60*T(i+1)^2 24*T(i+1)^1          6          0 0 0  -60*T(i+1)^2 -24*T(i+1)^1          -6           0  0 0]; % Equal jerk at intermediate waypoints
        R(((1+(i-1)+(3*(k-1))+4+(2*k)):(1+(i-1)+(3*(k-1))+4+(2*k))),(1+(6*(i-1)):6*(i+1))) = [120*T(i+1)^1          24          0          0 0 0 -120*T(i+1)^1          -24           0           0  0 0]; % Equal snap at intermediate waypoints
    end
    
    % Boundary Conditions Matrix
    BC = zeros(6*k,1);
    BC(1,1) = Pos(1);     % Position of the first waypoint
    if k > 1
        PosInter = zeros(2*(k-1),1);
    end
    for i = 2:k
        PosInter(2*(i-2)+1:2*(i-2)+2) = [Pos(i);Pos(i)];
    end
    for i = 1:2*(k-1)
        BC(2+(i-1)) = PosInter(i);
    end
    BC(2+2*(k-1)) = Pos(k+1);   % Position of the final waypoint
    BC(3+2*(k-1)) = Vel(1);     % initial velocity
    BC(4+2*(k-1)) = Vel(2);     % final velocity
    BC(5+2*(k-1)) = Acc(1);     % initial acceleration
    BC(6+2*(k-1)) = Acc(2);     % final acceleration

    % Coefficient  Vector
    Cj = R\BC;
    
end

function Cj = minimumJerkConstrCoefficient1DOF(T,Pos,Vel,Acc)

    k = length(T)-1;  % n= number of trajectories 

    % Waypoints' Matrix
    R = zeros(6*k,6*k); % Start with zero matrix for R, joint
    
    % Positions of all waypoints
    for i = 1:k
        % Position
        R(2*i-1:2*i-1        ,6*i-5:6*i) = [T(i)^5      T(i)^4    T(i)^3    T(i)^2    T(i)   1];
        R(2*i-0:2*i+0        ,6*i-5:6*i) = [T(i+1)^5    T(i+1)^4  T(i+1)^3  T(i+1)^2  T(i+1) 1];
        % Velocity
        R(2*k+2*i-1:2*k+2*i-1,6*i-5:6*i) = [5*T(i)^4	4*T(i)^3	3*T(i)^2      2*T(i)	1 0];
        R(2*k+2*i-0:2*k+2*i-0,6*i-5:6*i) = [5*T(i+1)^4	4*T(i+1)^3	3*T(i+1)^2    2*T(i+1)  1 0];
        % Accelaration
        R(4*k+2*i-1:4*k+2*i-1,6*i-5:6*i) = [20*T(i)^3 	12*T(i)^2 	6*T(i)      2	0 0];
        R(4*k+2*i-0:4*k+2*i-0,6*i-5:6*i) = [20*T(i+1)^3	12*T(i+1)^2	6*T(i+1)	2	0 0];
    end
    
    % Boundary Conditions Matrix
    BC = zeros(6*k,1);
    BC(1,1) = Pos(:,1);      	% Position of the first waypoint
    BC(1+2*k:1+2*k) = Vel(:,1);	% Velocity of the first waypoint
    BC(1+4*k:1+4*k) = Acc(:,1);	% Acceleration of the first waypoint
    if k > 1
        PosInter = zeros(2*(k-1),1);
        VelInter = zeros(2*(k-1),1);
        AccInter = zeros(2*(k-1),1);
    end
    for i = 2:k
        PosInter(2*i-3:2*i-2,:) = [Pos(:,i)';Pos(:,i)'];
        VelInter(2*i-3:2*i-2,:) = [Vel(:,i)';Vel(:,i)'];
        AccInter(2*i-3:2*i-2,:) = [Acc(:,i)';Acc(:,i)'];
    end
    for i = 1:2*(k-1)
        % Position
        BC(1+i:i+1) = PosInter(i,:);
        % Velocity
        BC(1+2*k+i:2*k+i+1) = VelInter(i,:);
        % Acceelration
        BC(1+4*k+i:4*k+i+1) = AccInter(i,:);
    end
    BC(1+2*k-1:2*k+0) = Pos(:,k+1); % Position of the final waypoint
    BC(1+2*k+1:2*k+2) = Vel(:,k+1); % Velocity of the final waypoint
    BC(1+2*k+3:2*k+4) = Acc(:,k+1); % Acceleration of the final waypoint

    % Coefficient  Vector
    Cj = R\BC;
    
end
function [ms_1,ms] = loadMAT(robot_model,app)

    if robot_model ~= 2
        if app.high_quality == 1
            temp = load('mesh/rx160/base_link.mat');  ms_1.s0 = temp.modelstruct;
            temp = load('mesh/rx160/link_1.mat');     ms_1.s1 = temp.modelstruct;
            temp = load('mesh/rx160/link_2.mat');     ms_1.s2 = temp.modelstruct;
            temp = load('mesh/rx160/link_3.mat');     ms_1.s3 = temp.modelstruct;
            if robot_model ~= 3
                temp = load('mesh/rx160/link_4.mat');
            else
                temp = load('mesh/rx160/link_4l.mat');
            end
            ms_1.s4 = temp.modelstruct;
            temp = load('mesh/rx160/link_5.mat');     ms_1.s5 = temp.modelstruct;
            temp = load('mesh/rx160/link_6.mat');     ms_1.s6 = temp.modelstruct;
        else
            temp = load('mesh/rx160/base_link_low.mat');  ms_1.s0 = temp.modelstruct;
            temp = load('mesh/rx160/link_1_low.mat');     ms_1.s1 = temp.modelstruct;
            temp = load('mesh/rx160/link_2_low.mat');     ms_1.s2 = temp.modelstruct;
            temp = load('mesh/rx160/link_3_low.mat');     ms_1.s3 = temp.modelstruct;
            if robot_model ~= 3
                temp = load('mesh/rx160/link_4_low.mat');
            else
                temp = load('mesh/rx160/link_4l_low.mat');
            end
            ms_1.s4 = temp.modelstruct;
            temp = load('mesh/rx160/link_5_low.mat');     ms_1.s5 = temp.modelstruct;
            temp = load('mesh/rx160/link_6_low.mat');     ms_1.s6 = temp.modelstruct;
        end
        
        if app.base_on == 1, temp = load('mesh/rx160/base_block.mat');  ms.sB = temp.modelstruct; end
    else
        if app.high_quality == 1
            temp = load('mesh/fer/visual/link0.mat');     ms_1.s0 = temp.modelstruct;
            temp = load('mesh/fer/visual/link1.mat');     ms_1.s1 = temp.modelstruct;
            temp = load('mesh/fer/visual/link2.mat');     ms_1.s2 = temp.modelstruct;
            temp = load('mesh/fer/visual/link3.mat');     ms_1.s3 = temp.modelstruct;
            temp = load('mesh/fer/visual/link4.mat');     ms_1.s4 = temp.modelstruct;
            temp = load('mesh/fer/visual/link5.mat');     ms_1.s5 = temp.modelstruct;
            temp = load('mesh/fer/visual/link6.mat');     ms_1.s6 = temp.modelstruct;
            temp = load('mesh/fer/visual/link7.mat');     ms_1.s7 = temp.modelstruct;
            temp = load('mesh/fer/visual/hand.mat');       ms_1.sh = temp.modelstruct;
        else
            temp = load('mesh/fer/collision/link0.mat');     ms_1.s0 = temp.modelstruct;
            temp = load('mesh/fer/collision/link1.mat');     ms_1.s1 = temp.modelstruct;
            temp = load('mesh/fer/collision/link2.mat');     ms_1.s2 = temp.modelstruct;
            temp = load('mesh/fer/collision/link3.mat');     ms_1.s3 = temp.modelstruct;
            temp = load('mesh/fer/collision/link4.mat');     ms_1.s4 = temp.modelstruct;
            temp = load('mesh/fer/collision/link5.mat');     ms_1.s5 = temp.modelstruct;
            temp = load('mesh/fer/collision/link6.mat');     ms_1.s6 = temp.modelstruct;
            temp = load('mesh/fer/collision/link7.mat');     ms_1.s7 = temp.modelstruct;
            temp = load('mesh/fer/collision/hand.mat');       ms_1.sh = temp.modelstruct;
        end
    end
    if app.coord_frame_on == 1, temp = load('mesh/coord_frame_thicc2.mat');  ms.sF = temp.modelstruct; end

end
