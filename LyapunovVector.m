function [ xdesired ]  = LyapunovVector(QuadPosition,TargetPosition,t, rd, ud)
%UNTITLED3 此处显示有关此函数的摘要
%   reference的输出是期望的位置和速度！
% ud = 0.5 ;
% rd = 2 ;
Ts = 0.1 ;
p = size(t,2) ;
zd = 5 ;

x = zeros(1,length(t));
y = zeros(1,length(t));
z = zeros(1,length(t));
phi = zeros(1,length(t));
theta = zeros(1,length(t));
psi = zeros(1,length(t));
xdot = zeros(1,length(t));
ydot = zeros(1,length(t));
zdot = zeros(1,length(t));
phidot = zeros(1,length(t));
thetadot = zeros(1,length(t));
psidot = zeros(1,length(t));


% xdot =vx* ones(1,length(t));
% ydot = vy*ones(1,length(t));

Qx = QuadPosition(1); Qy = QuadPosition(2); Tx =  TargetPosition(1); Ty =  TargetPosition(2); 
tvx = TargetPosition(4); tvy = TargetPosition(5);
Qz = QuadPosition(3);
m = 3 ;
for k = 1: p
    r_x = Qx - Tx ;
    r_y = Qy - Ty ;
    r = sqrt(r_x^2 + r_y ^2) ;
    vx = tvx+(-ud / (r*(r^m+rd^m))) * (r_x*(r^m-rd^m)+r_y*2*sqrt(r^m*rd^m)) ;
    vy = tvy+(-ud / (r*(r^m+rd^m))) * (r_y*(r^m-rd^m)-r_x*2*sqrt(r^m*rd^m)) ;
%     vx = (-ud / (r*(r^2+rd^2))) * (r_x*(r^2-rd^2)+r_y*2*r*rd) ;
%     vy = (-ud / (r*(r^2+rd^2))) * (r_y*(r^2-rd^2)-r_x*2*r*rd) ;
    vz = - 1 * tanh(Qz - zd) ; 
    xdot(k) = vx ;
    ydot(k) = vy ;
    zdot(k) = vz ;
    Qx = Qx + vx*Ts ;
    Qy = Qy + vy*Ts ;
    Qz = Qz + vz * Ts ;
    Tx = Tx +tvx *Ts ; % 预测目标的位置
    Ty = Ty +tvy *Ts ; % 假设目标匀速直线运动
    x(k) = Qx ;
    y(k) = Qy ;
    z(k) = Qz ;
    
end

xdesired = [x;y;z;phi;theta;psi;xdot;ydot;zdot;phidot;thetadot;psidot];

end

