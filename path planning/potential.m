function [t,x] = potential()
clc, close all
 
time = [0 100];
x0=[1;0;1;0;-2;0;1;0;-2;0;0;0;-2;0;-2;0;-4;0;0;0];

Goal = [20;0];
Obs1 = [10;-1.5];
Obs2 = [10;1.5];
    
alpha = 30;
Ko = 30;
Kij = 50;
KLi = 50;
KF = 10;
rD = 1;

[t, x] = ode23(@vehicles,time,x0);
Goal = [20;0];
figure(1)
plot(x(:,1),x(:,3),'k', x(:,5),x(:,7),'r', x(:,9),x(:,11),'g',...
    x(:,13),x(:,15),'b',x(:,17),x(:,19),'b',Goal(1),Goal(2),'o',Obs1(1),Obs1(2),'ob',...
    Obs2(1),Obs2(2),'ob')

title('Robot Trajectory')
xlabel('x-axis (m)')
ylabel('y-axis (m)')
axis([-5 25 -10 10])

figure(2)
hold on
plot(t,[x(:,1),x(:,5),x(:,9),x(:,13),x(:,17)],'LineWidth',1)
legend('x','y')
xlabel('time step')
ylabel('x position')
grid on
hold off

figure(3)
hold on
plot(t,[x(:,2),x(:,6),x(:,10),x(:,14),x(:,18)],'LineWidth',1)
legend('show')
xlabel('time step')
ylabel('velocity (m/s)')
axis([0 100 -2 10])
grid on
hold off

figure(4)
title('Robot Trajectory')
xlabel('x-axis (m)')
ylabel('y-axis (m)')
plot(Goal(1),Goal(2),'o',Obs1(1),Obs1(2),'ob',...
    Obs2(1),Obs2(2),'ob')
axis([-10 25 -10 10])

grid on;

% aL = animatedline('Color', 'k');
% a1 = animatedline('Color', 'r');
% a2 = animatedline('Color', 'g');
% a3 = animatedline('Color', 'b');
% a4 = animatedline('Color', 'm');
aLL = animatedline('Color', 'k', 'marker', 'o', 'MaximumNumPoints', 1);
a11 = animatedline('Color', 'r', 'marker', 'o', 'MaximumNumPoints', 1);
a22 = animatedline('Color', 'g', 'marker', 'o', 'MaximumNumPoints', 1);
a33 = animatedline('Color', 'b', 'marker', 'o', 'MaximumNumPoints', 1); 
a44 = animatedline('Color', 'm', 'marker', 'o', 'MaximumNumPoints', 1); 

for i=1:length(x)
%     addpoints(aL,x(i,1),x(i,3));
%     addpoints(a1,x(i,5),x(i,7));
%     addpoints(a2,x(i,9),x(i,11));
%     addpoints(a3,x(i,13),x(i,15));
%     addpoints(a4,x(i,17),x(i,19));
 
    addpoints(aLL,x(i,1),x(i,3));
    addpoints(a11,x(i,5),x(i,7));
    addpoints(a22,x(i,9),x(i,11));
    addpoints(a33,x(i,13),x(i,15));
    addpoints(a44,x(i,17),x(i,19));
    drawnow
end 

function dx = vehicles(t,x)
 %% Leader
    % Potential function without cooperative control
    % mobile robot 1 and robot 2 will move but ignore the potential
    % on each other robot
    
    % Define for mobile robot 1 as the leader corresponds to goal
    rG = sqrt((Goal(1)-x(1))^2+(Goal(2)-x(3))^2);
    FGx = alpha*(Goal(1)-x(1));
    FGy = alpha*(Goal(2)-x(3));
%     FGx = KG*(Goal(1)-x(1))/max([rG 0.01]);
%     FGy = KG*(Goal(2)-x(3))/max([rG 0.01]);
    
    % Leader corresponds with Obstacle 1
    ro1 = sqrt((Obs1(1)-x(1))^2+(Obs1(2)-x(3))^2);
    Fo1x = Ko*(Obs1(1)-x(1))/ro1^2;
    Fo1y = Ko*(Obs1(2)-x(3))/ro1^2;

%     Fo1x = -Ko*(Obs1(1)-x(1))/ro1^3;
%     Fo1y = -Ko*(Obs1(2)-x(3))/ro1^3;
    
    % Leader corresponds with Obstacle 2
    ro2 = sqrt((Obs2(1)-x(1))^2+(Obs2(2)-x(3))^2);
    Fo2x = Ko*(Obs2(1)-x(1))/ro2^2;
    Fo2y = Ko*(Obs2(2)-x(3))/ro2^2;
%     Fo2x = -Ko*(Obs2(1)-x(1))/ro2^3;
%     Fo2y = -Ko*(Obs2(2)-x(3))/ro2^3;
     
    % Prevent from oscillating
    FrLx = -KF*x(2);
    FrLy = -KF*x(4);
    
    % Net force on leader
    Fx = (FGx+Fo1x+Fo2x+FrLx);
    Fy = (FGy+Fo1y+Fo2y+FrLy);
    
    % Leader dynamics
    xL = [0 1; 0 0]*[x(1); x(2)] + [0;1]*Fx;
    yL = [0 1; 0 0]*[x(3); x(4)] + [0;1]*Fy;
 
 %% Follower 1   
    % Follower 1 with obstacle 1
    r1o1 = sqrt((Obs1(1)-x(5))^2+(Obs1(2)-x(7))^2);
    F1o1x = Ko*(Obs1(1)-x(5))/r1o1^2;
    F1o1y = Ko*(Obs1(2)-x(7))/r1o1^2;
    
    % Follower 1 with obstacle 2
    r1o2 = sqrt((Obs2(1)-x(5))^2+(Obs2(2)-x(7))^2);
    F1o2x = Ko*(Obs2(1)-x(5))/r1o2^2;
    F1o2y = Ko*(Obs2(2)-x(7))/r1o2^2;
    
    % Follower 1 with leader
    rL1 = sqrt((x(1)-x(5))^2+(x(3)-x(7))^2);
    FL1x = KLi*(x(1)-x(5)-rD);
    FL1y = KLi*(x(3)-x(7)-rD);
%     FL1x = KLi*(rL1-rD)*(x(1)-x(5))/max([rL1 0.01]);
%     FL1y = KLi*(rL1-rD)*(x(3)-x(7))/max([rL1 0.01]);
    
    % Follower 1 and follower 2
    r21 = sqrt((x(9)-x(5))^2+(x(11)-x(7))^2);
    F21x = Kij*(x(9)-x(5)-rD);
    F21y = Kij*(x(11)-x(7)-rD);
%     F21x = -2*Kij*(x(9)-x(5))/max([r21^4 0.01]);
%     F21y = -2*Kij*(x(11)-x(7))/max([r21^4 0.01]);
    
    % Follower 1 and follower 3
    r31 = sqrt((x(13)-x(5))^2+(x(15)-x(7))^2);
    F31x = Kij*(x(13)-x(5)-rD);
    F31y = Kij*(x(15)-x(7)-rD);
%     F31x = -2*Kij*(x(13)-x(5))/max([r31^4 0.01]);
%     F31y = -2*Kij*(x(15)-x(7))/max([r31^4 0.01]);
    
    % Follower 1 and follower 4
    r41 = sqrt((x(17)-x(5))^2+(x(19)-x(7))^2);
    F41x = Kij*(x(17)-x(5)-rD);
    F41y = Kij*(x(19)-x(7)-rD);    
%     F41x = -2*Kij*(x(17)-x(5))/max([r41^4 0.01]);
%     F41y = -2*Kij*(x(19)-x(7))/max([r41^4 0.01]);
%     
    Fr1x = -KF*x(6);
    Fr1y = -KF*x(8);
    
    % Net force on follower 1
    F1x = (F1o1x+F1o2x+FL1x+F21x+Fr1x+F31x+F41x);
    F1y = (F1o1y+F1o2y+FL1y+F21y+Fr1y+F31y+F41y);
    
    % Dynamics of Follower 1
    x1 = [0 1; 0 0]*[x(5); x(6)] + [0;1]*F1x;
    y1 = [0 1; 0 0]*[x(7); x(8)] + [0;1]*F1y;
    
 %% Follower 2   
    % Follower 2 with obstacle 1
    r2o1 = sqrt((Obs1(1)-x(9))^2+(Obs1(2)-x(11))^2);
    F2o1x = Ko*(Obs1(1)-x(9)-rD);
    F2o1y = Ko*(Obs1(2)-x(11)-rD);    
%     F2o1x = -Ko*(Obs1(1)-x(9))/r2o1^3;
%     F2o1y = -Ko*(Obs1(2)-x(11))/r2o1^3;
    
    % Follower 2 with obstacle 2
    r2o2 = sqrt((Obs2(1)-x(9))^2+(Obs2(2)-x(11))^2);
    F2o2x = Ko*(Obs2(1)-x(9)-rD);
    F2o2y = Ko*(Obs2(2)-x(11)-rD);
%     F2o2x = -Ko*(Obs2(1)-x(9))/r2o2^3;
%     F2o2y = -Ko*(Obs2(2)-x(11))/r2o2^3;
    
    % Follower 2 with Leader
    rL2 = sqrt((x(1)-x(9))^2+(x(3)-x(11))^2);
    FL2x = KLi*(x(1)-x(9)-rD);
    FL2y = KLi*(x(3)-x(11)-rD);
%     FL2x = KLi*(rL2-rD)*(x(1)-x(9))/max([rL2 0.01]);
%     FL2y = KLi*(rL2-rD)*(x(3)-x(11))/max([rL2 0.01]);
%     
    % Follower 2 and follower 1
    r12 = sqrt((x(5)-x(9))^2+(x(7)-x(11))^2);
    F12x = Kij*(x(5)-x(9)-rD);
    F12y = Kij*(x(7)-x(11)-rD);
%     F12x = -2*Kij*(x(5)-x(9))/max([r12^4 0.01]);
%     F12y = -2*Kij*(x(7)-x(11))/max([r12^4 0.01]);
    
    % Follower 2 and follower 3
    r32 = sqrt((x(13)-x(9))^2+(x(15)-x(11))^2);
    F32x = Kij*(x(13)-x(9)-rD);
    F32y = Kij*(x(15)-x(11)-rD);
%     F32x = -2*Kij*(x(13)-x(9))/max([r32^4 0.01]);
%     F32y = -2*Kij*(x(15)-x(11))/max([r32^4 0.01]);
    
    % Follower 2 and follower 4
    r42 = sqrt((x(17)-x(9))^2+(x(19)-x(11))^2);
    F42x = Kij*(x(17)-x(9)-rD);
    F42y = Kij*(x(19)-x(11)-rD);
%     F42x = -2*Kij*(x(17)-x(9))/max([r42^4 0.01]);
%     F42y = -2*Kij*(x(19)-x(11))/max([r42^4 0.01]);
    
    Fr2x = -KF*x(10);
    Fr2y = -KF*x(12);
       
    % Net force on Follower 2
    F2x = (F2o1x+F2o2x+FL2x+F12x+Fr2x+F32x+F42x);
    F2y = (F2o1y+F2o2y+FL2y+F12y+Fr2y+F32y+F42y);
    
    % Dynamics of Follower 2
    x2 = [0 1; 0 0]*[x(9); x(10)] + [0;1]*F2x;
    y2 = [0 1; 0 0]*[x(11); x(12)] + [0;1]*F2y;
    
 %% Follower 3   
    
    % Follower 3 with obstacle 1
    r3o1 = sqrt((Obs1(1)-x(13))^2+(Obs1(2)-x(15))^2);
    F3o1x = -Ko*(Obs1(1)-x(13))/r3o1^3;
    F3o1y = -Ko*(Obs1(2)-x(15))/r3o1^3;
    
    % Follower 3 with obstacle 2
    r3o2 = sqrt((Obs2(1)-x(13))^2+(Obs2(2)-x(15))^2);
    F3o2x = -Ko*(Obs2(1)-x(13))/r3o2^3;
    F3o2y = -Ko*(Obs2(2)-x(15))/r3o2^3;
    
    % Follower 3 with Leader
    rL3 = sqrt((x(1)-x(13))^2+(x(3)-x(15))^2);
    FL3x = KLi*(rL3-rD)*(x(1)-x(13))/max([rL3 0.01]);
    FL3y = KLi*(rL3-rD)*(x(3)-x(15))/max([rL3 0.01]);
    
    % Follower 3 and follower 1
    r13 = sqrt((x(5)-x(13))^2+(x(7)-x(15))^2);
    F13x = -2*Kij*(x(5)-x(13))/max([r13^4 0.01]);
    F13y = -2*Kij*(x(7)-x(15))/max([r13^4 0.01]);
    
    % Follower 3 and follower 2
    r23 = sqrt((x(9)-x(13))^2+(x(11)-x(15))^2);
    F23x = -2*Kij*(x(9)-x(13))/max([r23^4 0.01]);
    F23y = -2*Kij*(x(11)-x(15))/max([r23^4 0.01]);
    
    % Follower 3 and follower 4
    r43 = sqrt((x(17)-x(13))^2+(x(19)-x(15))^2);
    F43x = -2*Kij*(x(17)-x(13))/max([r43^4 0.01]);
    F43y = -2*Kij*(x(19)-x(15))/max([r43^4 0.01]);

    Fr3x = -KF*x(14);
    Fr3y = -KF*x(16);
    
    % Net force on Follower 3
    F3x = (F3o1x+F3o2x+FL3x+F23x+Fr3x+F13x+F43x);
    F3y = (F3o1y+F3o2y+FL3y+F23y+Fr3y+F13y+F43y);
    
    % Dynamics of Follower 3
    x3 = [0 1; 0 0]*[x(13); x(14)] + [0;1]*F3x;
    y3 = [0 1; 0 0]*[x(15); x(16)] + [0;1]*F3y;
    
%% Follower 4

    % Follower 4 with Leader
    rL4 = sqrt((x(1)-x(17))^2+(x(3)-x(19))^2);
    FL4x = KLi*(rL4-rD)*(x(1)-x(17))/max([rL4 0.01]);
    FL4y = KLi*(rL4-rD)*(x(3)-x(19))/max([rL4 0.01]);
    
    % Follower 4 with obstacle 1
    r4o1 = sqrt((Obs1(1)-x(17))^2+(Obs1(2)-x(19))^2);
    F4o1x = -Ko*(Obs1(1)-x(17))/r4o1^3;
    F4o1y = -Ko*(Obs1(2)-x(19))/r4o1^3;
    
    % Follower 4 with obstacle 2
    r4o2 = sqrt((Obs2(1)-x(17))^2+(Obs2(2)-x(19))^2);
    F4o2x = -Ko*(Obs2(1)-x(17))/r4o2^3;
    F4o2y = -Ko*(Obs2(2)-x(19))/r4o2^3;
    
    % Follower 4 and follower 1
    r14 = sqrt((x(5)-x(17))^2+(x(7)-x(19))^2);
    F14x = -2*Kij*(x(5)-x(17))/max([r14^4 0.01]);
    F14y = -2*Kij*(x(7)-x(19))/max([r14^4 0.01]);
    
    % Follower 4 and follower 2
    r24 = sqrt((x(9)-x(17))^2+(x(11)-x(19))^2);
    F24x = -2*Kij*(x(9)-x(19))/max([r24^4 0.01]);
    F24y = -2*Kij*(x(11)-x(19))/max([r24^4 0.01]);
    
    % Follower 4 and follower 3
    r34 = sqrt((x(13)-x(17))^2+(x(15)-x(19))^2);
    F34x = -2*Kij*(x(13)-x(19))/max([r34^4 0.01]);
    F34y = -2*Kij*(x(15)-x(19))/max([r34^4 0.01]);
    
    Fr4x = -KF*x(18);
    Fr4y = -KF*x(20);
    
    % Net force on Follower 4
    F4x = (F4o1x+F4o2x+F14x+F24x+F34x+Fr4x+FL4x);
    F4y = (F4o1y+F4o2y+F14y+F24y+F34y+Fr4y+FL4y);
    
    % Dynamics of Follower 4
    x4 = [0 1; 0 0]*[x(17); x(18)] + [0;1]*F4x;
    y4 = [0 1; 0 0]*[x(19); x(20)] + [0;1]*F4y;
    
 %% Total calculation   
    dx = [xL(1);xL(2);yL(1);yL(2);x1(1);x1(2);y1(1);y1(2);...
        x2(1);x2(2);y2(1);y2(2);x3(1);x3(2);y3(1);y3(2);x4(1);x4(2);y4(1);y4(2)];
    
    if x(2)==0 && x(4)==0
        w = 0.01*randn(2,1);
        dx = dx+[0;w(1);0;w(2);0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0];
    end
    
    if r12==0 && r13 == 0 && r14 == 0 && r21 == 0 && r23 == 0 && r24 == 0 && r31 == 0 && r32 == 0 && r34 == 0 && r41 == 0 && r42 == 0 && r43 == 0
        w = 0.01*randn(8,1);
        dx = dx+[0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0];
    end      
end
end
