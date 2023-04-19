% TODO: Fix the H value to make the robots avoid from collision
% NOTE: This code is cbf with centralized safety control

clc, clear, close all;

% Define parameters
x0 = [-0.6 0.6 -0.2 0.4 0.2 0.5 0.6 0.6]'; %Initial position 
xg = [1.2 2.5 -0.4 2.5 0.4 2.5 -1.2 2.5]'; %goal position

Q = 2*eye(8);

Ts = 0.01;
time = 550;

x = zeros(8,time);
x(:,1) = x0;
u = zeros(8,time);

% Simulator  (Centralized)
for k = 2:time
    H = [-(x(1,k-1)-x(3,k-1)) -(x(2,k-1)-x(4,k-1)) x(1,k-1)-x(3,k-1) x(2,k-1)-x(4,k-1) 0 0 0 0;
        -(x(1,k-1)-x(5,k-1)) -(x(2,k-1)-x(6,k-1)) 0 0 x(1,k-1)-x(5,k-1) x(2,k-1)-x(6,k-1) 0 0;
        -(x(1,k-1)-x(7,k-1)) -(x(2,k-1)-x(8,k-1)) 0 0 0 0 x(1,k-1)-x(7,k-1) x(2,k-1)-x(8,k-1);
        x(3,k-1)-x(1,k-1) x(4,k-1)-x(2,k-1) -(x(3,k-1)-x(1,k-1)) -(x(4,k-1)-x(2,k-1)) 0 0 0 0;
        0 0 -(x(3,k-1)-x(5,k-1)) -(x(4,k-1)-x(6,k-1)) x(3,k-1)-x(5,k-1) x(4,k-1)-x(6,k-1) 0 0;
        0 0 -(x(3,k-1)-x(7,k-1)) -(x(4,k-1)-x(8,k-1)) 0 0 x(3,k-1)-x(7,k-1) x(4,k-1)-x(8,k-1);
        x(5,k-1)-x(1,k-1) x(6,k-1)-x(2,k-1) 0 0 -(x(5,k-1)-x(1,k-1)) -(x(6,k-1)-x(2,k-1)) 0 0;
        0 0 x(5,k-1)-x(3,k-1) x(6,k-1)-x(4,k-1) -(x(5,k-1)-x(3,k-1)) -(x(6,k-1)-x(4,k-1)) 0 0;    
        0 0 0 0 -(x(5,k-1)-x(7,k-1)) -(x(6,k-1)-x(8,k-1)) x(5,k-1)-x(7,k-1) x(6,k-1)-x(8,k-1);
        x(7,k-1)-x(1,k-1) x(8,k-1)-x(2,k-1) 0 0 0 0 -(x(7,k-1)-x(1,k-1)) -(x(8,k-1)-x(2,k-1));
        0 0 x(7,k-1)-x(3,k-1) x(8,k-1)-x(4,k-1) 0 0 -(x(7,k-1)-x(3,k-1)) -(x(8,k-1)-x(4,k-1));
        0 0 0 0 x(7,k-1)-x(5,k-1) x(8,k-1)-x(6,k-1) -(x(7,k-1)-x(5,k-1)) -(x(8,k-1)-x(6,k-1))];
    % Proportional control: u_gtg = xg - x
    c = -2*[xg(1)-x(1,k-1);xg(2)-x(2,k-1);xg(3)-x(3,k-1);xg(4)-x(4,k-1);
        xg(5)-x(5,k-1);xg(6)-x(6,k-1);xg(7)-x(7,k-1);xg(8)-x(8,k-1)];
    b = 10*(sum(H'.^2)'-0.2^2*ones(12,1)).^3;
    u(:,k-1) = quadprog(Q,c,H,b,[],[],[],[],[],optimset('Display','off'));
    x(:,k) = x(:,k-1) + Ts*u(:,k-1);  
end

for k=1:time-1
    clf; hold on; grid on;
    % Agent 1
    plot(x(1,:), x(2,:), 'b.-', 'MarkerSize', 2); % plot the entire trajectory up to the current point
    plot(x(1,k), x(2,k), 'bo', 'MarkerSize', 5); % plot the current point in red

    % Agent 2
    plot(x(3,:), x(4,:), 'r.-', 'MarkerSize', 2); % plot the entire trajectory up to the current point
    plot(x(3,k), x(4,k), 'ro', 'MarkerSize', 5); % plot the current point in red

    % Agent 3
    plot(x(5,:), x(6,:), 'g.-', 'MarkerSize', 2); % plot the entire trajectory up to the current point
    plot(x(5,k), x(6,k), 'go', 'MarkerSize', 5); % plot the current point in red

    % Agent 4
    plot(x(7,:), x(8,:), 'k.-', 'MarkerSize', 2); % plot the entire trajectory up to the current point
    plot(x(7,k), x(8,k), 'ko', 'MarkerSize', 5); % plot the current point in red
    
    plot(xg(1),xg(2),'p','LineWidth',2,'MarkerSize',6,...
    'MarkerEdgeColor','k','MarkerFaceColor','k'); %Black Star for goal pos
    plot(xg(3),xg(4),'p','LineWidth',2,'MarkerSize',6,...
    'MarkerEdgeColor','r','MarkerFaceColor','r'); %Red Star for goal pos
    plot(xg(5),xg(6),'p','LineWidth',2,'MarkerSize',6,...
    'MarkerEdgeColor','b','MarkerFaceColor','b'); %Red Star for goal pos
    plot(xg(7),xg(8),'p','LineWidth',2,'MarkerSize',6,...
    'MarkerEdgeColor','g','MarkerFaceColor','g'); %Red Star for goal pos
    hold off
    viscircles([x(1,k), x(2,k)], 0.2); % plot the moving object
    viscircles([x(3,k), x(4,k)], 0.2); % plot the moving object
    viscircles([x(5,k), x(6,k)], 0.2); % plot the moving object
    viscircles([x(7,k), x(8,k)], 0.2); % plot the moving object
    xlim([-2 2]); ylim([-2 5]);
    xlabel("pos x (m)"); ylabel("pos y (m)");
    title('Centralized Systems Control');
    drawnow;
    pause(0.01);
end

figure
plot(1:time,sqrt(sum(abs(x(1:2,:)-x(3:4,:)).^2)), 'LineWidth',1.5)
hold on
plot(1:time,sqrt(sum(abs(x(1:2,:)-x(5:6,:)).^2)), 'LineWidth',1.5)
plot(1:time,sqrt(sum(abs(x(1:2,:)-x(7:8,:)).^2)), 'LineWidth',1.5)
plot(1:time,sqrt(sum(abs(x(3:4,:)-x(5:6,:)).^2)), 'LineWidth',1.5)
plot(1:time,sqrt(sum(abs(x(3:4,:)-x(7:8,:)).^2)), 'LineWidth',1.5)
plot(1:time,sqrt(sum(abs(x(5:6,:)-x(7:8,:)).^2)), 'LineWidth',1.5)
ylim([0, 3]);
title("Time series of euclidean distance between all pairs of robots")
legend("(r1,r2)","(r1,r3)","(r1,r4)","(r2,r3)",...
    "(r2,r4)","(r3,r4)",Location="best")
xlabel("Time")
ylabel("Distance")

figure
plot(1:time,xg(1)-x(1,:),'b-', 'LineWidth', 1.5, 'DisplayName', 'u_{gtg1}')
hold on, grid on;
plot(1:time,xg(3)-x(3,:), 'r-', 'LineWidth', 1.5, 'DisplayName', 'u_{gtg2}')
plot(1:time,xg(5)-x(5,:), 'k-', 'LineWidth', 1.5, 'DisplayName', 'u_{gtg3}')
plot(1:time,xg(7)-x(7,:), 'y-', 'LineWidth', 1.5, 'DisplayName', 'u_{gtg4}')
plot(1:time,u(1,:), 'b--', 'LineWidth', 1.5, 'DisplayName', 'u_1')
plot(1:time,u(3,:), 'r--', 'LineWidth', 1.5, 'DisplayName', 'u_2')
plot(1:time,u(5,:), 'k--', 'LineWidth', 1.5, 'DisplayName', 'u_3')
plot(1:time,u(7,:), 'y--', 'LineWidth', 1.5, 'DisplayName', 'u_4')

legend({'$u_{gtg,1}$','$u_{gtg,2}$', '$u_{gtg,3}$','$$u_{gtg,4}$', '$u_1$','$u_2$', '$u_3$','$u_4$'},'Interpreter','latex')
title("Nominal vs optimal control input in x direction")
xlabel("Time")
ylabel("Control input")

figure
plot(1:time,xg(2)-x(2,:),'b-', 'LineWidth', 1.5, 'DisplayName', 'u_{gtg1}')
hold on, grid on;
plot(1:time,xg(4)-x(4,:), 'r-', 'LineWidth', 1.5, 'DisplayName', 'u_{gtg2}')
plot(1:time,xg(6)-x(6,:), 'k-', 'LineWidth', 1.5, 'DisplayName', 'u_{gtg3}')
plot(1:time,xg(8)-x(8,:), 'y-', 'LineWidth', 1.5, 'DisplayName', 'u_{gtg4}')
plot(1:time,u(2,:), 'b--', 'LineWidth', 1.5, 'DisplayName', 'u_1')
plot(1:time,u(4,:), 'r--', 'LineWidth', 1.5, 'DisplayName', 'u_2')
plot(1:time,u(6,:), 'k--', 'LineWidth', 1.5, 'DisplayName', 'u_3')
plot(1:time,u(8,:), 'y--', 'LineWidth', 1.5, 'DisplayName', 'u_4')

legend({'$u_{gtg,1}$','$u_{gtg,2}$', '$u_{gtg,3}$','$$u_{gtg,4}$', '$u_1$','$u_2$', '$u_3$','$u_4$'},'Interpreter','latex')
title("Nominal vs optimal control input in y direction")
xlabel("Time")
