
clc; clear; close all;

% load the parameter of the DNN
sim_data

% Bounds of the UAV state
Xt_lb = [-30; -30; -10; -0.1; -0.1; -0.1; -0.1; -0.1; -0.1; -0.1; -0.1; -0.1] ;  % lower bound
Xt_ub = [30;  30; 10;  0.1;  0.1;  0.1;  0.1;  0.1;  0.1;  0.1;  0.1;  0.1] ;% upper bound


% Bounds of the target state
Tar_x_lb = [-20; -20; 0; -1; -1; 0] ; 
Tar_x_ub = [ 20;  20; 0;  1;  1; 0] ;

% Prediction horizon for MPC
Np = 20 ;

%% State and input of the target 
T_state = [0;0;0;0;0;0] ;
T_state_history = T_state' ;
T_input = zeros(1,2) ;

% Simulation time
Duration = 60 ;

% Commands
rd = 2 ; % desired rdius
rd_his = rd ; % history of the desired radius
vd = 1.0 ; % desired speed
vd_his = vd ; % history of the desired speed

for k = 1:(Duration/Ts)  % 目标静态
%     T_input = [0.2*sin(0.001*k),0.2*cos(0.004*k)] ; % moving target
    T_state(4:5) = T_input ;
    T_state(1:2) = T_state(1:2)+ T_input'*Ts ;
    T_state_history(k+1,:) = T_state ;  % history of the target state
end

tic;

total_case = 4 ; % four initial states for the UAV
 
A = zeros(Duration /Ts,12) ;
xHistory = repmat(A,1,1,total_case) ;

rangeHistory = zeros(total_case,Duration /Ts) ;

for case_num=1:total_case
    switch(case_num) % different initial states
    case 1
        x_ini = [5;5;0;0;0;0*pi/3;0;0;0;0;0;0] ; % initial state
    case 2
        x_ini = [-5;5;0;0;0;-1*pi/3;0;0;0;0;0;0] ; % initial state
    case 3
        x_ini = [-5;-5;0;0;0;1*pi/3;0;0;0;0;0;0] ; % initial state
    otherwise
        x_ini = [5;-5;0;0;0;-1*pi/3;0;0;0;0;0;0] ; % initial state
    end
    xHistory(1,:,case_num) = x_ini';
    
    x = x_ini' ;
    TargetPosition = [0 0 0] ;
    range = norm(x(1:2)-TargetPosition(1:2),2) ;
    rangeHistory(case_num,1) = range ;
    xrefHistory =x(1)* ones(1,Np)   ;
    yrefHistory =x(2)* ones(1,Np)   ;
    vHis = [];

    
    u = [0;0;0;0]  ; % input of the UAV
    uHistory = u' ; % histroy of input
    integ = 0 ; % integral of the range error
    
    for k = 1: (Duration /Ts) % Start!
        t = linspace(k*Ts, (k+Np-1)*Ts,Np);  % prediction horizon
        QuadPosition = xHistory(k,1:3,case_num); 
        TargetPosition=T_state_history(k,:);
        rangeHistory(case_num,k) = norm(QuadPosition(1:2)-TargetPosition(1:2),2) ; % history of the relative range
        vHis(k) =  norm(xHistory(k,7:8,case_num) - T_state_history(k,4:5),2) ; % history if the relative speed
        error = rangeHistory(case_num,k)-rd ; % range error
        error = sat_integral(error,0.2) ; % saturation
        integ = integ + Ts*error ; % integral

        rdk = rd - 0.2*integ  ;

        rd_his(k) = rdk ;
        
        ref = LyapunovVector(QuadPosition,TargetPosition,t, rdk, vd);  % reference trajectory
        xrefHistory(k,:) = ref(1,:) ;
        yrefHistory(k,:) = ref(2,:) ;
       
        xk = xHistory(k,:,case_num);

        in = [xk'; ref(1:3,1); ref(7:9,1)];

        out = DNN(in, full_pw1,full_pw2,full_pw3,full_pw4, ...
        full_pb1,full_pb2,full_pb3,full_pb4, ...
        full_Pt_max',full_Pt_min',full_p_max',full_p_min') ;

        uk = out ;   
        uHistory(k+1,:) = uk';
   
        ODEFUN = @(t,xk) QuadrotorStateFcn(xk,uk); % 执行控制量
        [TOUT,YOUT] = ode45(ODEFUN,[0 Ts], xHistory(k,:,case_num)');
        xHistory(k+1,:,case_num) = YOUT(end,:);
    end
end
toc


re = rangeHistory(end)- rd
ve = vHis(end)-vd
he = xHistory(end,3) - 5

%% Plotting
% figure
% plot(xHistory(:,1,1),xHistory(:,2,1),'r','LineWidth',1.2) ;
% hold on 
% plot(T_state_history(:,1),T_state_history(:,2),'b--') ;

tim = Ts :Ts : Duration;


figure
% Position of the target
plot(T_state_history(1,1),T_state_history(1,2),'ks','LineWidth',2.0) ;
% initial position of the UAV
hold on
plot(xHistory(1,1,1),xHistory(1,2,1),'r>','LineWidth',2.0) ;
hold on
plot(xHistory(1,1,2),xHistory(1,2,2),'r>','LineWidth',2.0) ;
hold on
plot(xHistory(1,1,3),xHistory(1,2,3),'r>','LineWidth',2.0) ;
hold on
plot(xHistory(1,1,4),xHistory(1,2,4),'r>','LineWidth',2.0) ;
% Trajectory of the UAV
hold on
plot(xHistory(:,1,1),xHistory(:,2,1),'b','LineWidth',1.2) ;
hold on
plot(xHistory(:,1,2),xHistory(:,2,2),'b','LineWidth',1.2) ;
hold on
plot(xHistory(:,1,3),xHistory(:,2,3),'b','LineWidth',1.2) ;
hold on
plot(xHistory(:,1,4),xHistory(:,2,4),'b','LineWidth',1.2) ;

h = legend('Position of the target','Initial position of the UAV') ;
xlabel('X-position (m)') ;
ylabel('Y-position (m)') ;

set(h,'fontsize',11);
set(gca,'fontsize',12);

axis([-7,10,-6,6])
axis equal
grid on


figure 
% Position of the target
plot3(T_state_history(1,1),T_state_history(1,2),T_state_history(1,3),'ks','LineWidth',5.0) ;
% Initial position of the UAV
hold on
plot3(xHistory(1,1,1),xHistory(1,2,1),xHistory(1,3,1),'r>','LineWidth',2.0) ;
hold on
plot3(xHistory(1,1,2),xHistory(1,2,2),xHistory(1,3,2),'r>','LineWidth',2.0) ;
hold on
plot3(xHistory(1,1,3),xHistory(1,2,3),xHistory(1,3,3),'r>','LineWidth',2.0) ;
hold on
plot3(xHistory(1,1,4),xHistory(1,2,4),xHistory(1,3,4),'r>','LineWidth',2.0) ;
% Trajectory of the UAV
hold on
plot3(xHistory(:,1,1),xHistory(:,2,1),xHistory(:,3,1),'b','LineWidth',1.2) ;
hold on
plot3(xHistory(:,1,2),xHistory(:,2,2),xHistory(:,3,2),'b','LineWidth',1.2) ;
hold on
plot3(xHistory(:,1,3),xHistory(:,2,3),xHistory(:,3,3),'b','LineWidth',1.2) ;
hold on
plot3(xHistory(:,1,4),xHistory(:,2,4),xHistory(:,3,4),'b','LineWidth',1.2) ;

% h = legend('Position of the target','Initial position of the UAV') ;
xlabel('X-position (m)') ;
ylabel('Y-position (m)') ;
zlabel('Z-position (m)') ;


% set(h,'fontsize',11);
set(gca,'fontsize',12);
grid on
axis([-7,7,-7,7,0,6])



function sat_integ = sat_integral(sat_in,a) % saturation function

    sat_integ = sat_in ;
    if sat_integ > a 
        sat_integ = a ;
    elseif sat_integ<-a
        sat_integ = -a;
    end

end

