clc, clear, close all;

% Define parameters
Ts = 0.01; % in seconds (time sampling)
Tmax = 8; % maximum simulation time
time_hist = 0:Ts:Tmax;
gamma = 10; % Gamma value for CBF
gamma2 = 1; % Gamma value for CBF
n = length(time_hist);

% Initial positions
px = 0; 
py = 0;
x_hist = zeros(n,2);
x_hist(1,:) = [0, 0];

x_hist2 = zeros(n,2);
x_hist2(1,:) = [0, 0];

x_hist3 = zeros(n,2);
x_hist3(1,:) = [0, 0];

% Radius of circle around moving object
r_robot = 0.2;

% Goal Position
xg = 3; 
yg = 3;
x_gtg = [xg,yg];

% Obstacle position
xo = 1.5;
yo = 1.8;
x_obs = [xo,yo];
r = 0.8; % radius of the obstacle

ustarx = zeros(n,1);
ustary = zeros(n,1);
cbf = zeros(n,1);
cbf2 = zeros(n,1);
cbf3 = zeros(n,1);
u_nomx = zeros(n,1);
u_nomy = zeros(n,1);
p_e = zeros(n,1);
R = 0.8;
dist = zeros(n,1);

% Define animation parameters
animation_delay = 0.1; % in seconds (delay between each frame)

for k = 1:n-1
    % Current state of robot
    x_k = x_hist(k, :);

    % control input 
    u = [1, 1];
    u_gtg = (x_gtg - x_k);

    % Control barrier function parameters
    hx = (x_k(1) - x_obs(1))^2 + (x_k(2) - x_obs(2))^2 - (R)^2; %CBF constraint
    cbf(k) = gamma * (hx);
    % Quadratic programming parameters 1
    Q = 2 * eye(2);
    c = -2 * u_gtg;    
    temp = [x_k(1)-x_obs(1) x_k(2)-x_obs(2)];
    H = -2 * temp;
    b = [cbf(k)];
    lb = [-1];
    ub = [1];
    options =  optimset('Display','off');
    sol = quadprog(Q, c, H, b, [], [], [], [], [], options);
    u = [sol(1) sol(2)];

    % Can comment if function below and add ub and lb in quadprog above
    if u(1) > ub
        u(1) = ub;
    end
    if u(1) < lb
        u(1) = lb;
    end

    % update the next state 10 h(x)
    x_hist(k+1, :) = x_k + Ts * u; 

    ustarx(k) = u(1); %ustarx is vx (only for plot)
    ustary(k) = u(2); %ustary is vy (only for plot)
    u_nomx(k) = u_gtg(1); %ustary is u_gtg_x (only for plot)
    u_nomy(k) = u_gtg(2); %ustary is u_gtg_y (only for plot)
    dist(k+1) = (u_gtg(1) - u(1))^2 + (u_gtg(2) - u(2))^2;
end

% Plot animation of trajectory
fig = figure;
set(fig,'Position',[100 100 800 600]);

for k=1:n
    clf; hold on; grid on;
    plot(x_hist(:,1), x_hist(:,2), 'b.-', 'MarkerSize', 2); % plot the entire trajectory up to the current point
    plot(x_hist(k,1), x_hist(k,2), 'ro', 'MarkerSize', 10); % plot the current point in red
    plot(x_hist(1),x_hist(2),'p','LineWidth',2,'MarkerSize',6,...
    'MarkerEdgeColor','r','MarkerFaceColor','r'); %Red Star for Init pos
    plot(x_gtg(1),x_gtg(2),'p','LineWidth',2,'MarkerSize',6,...
    'MarkerEdgeColor','k','MarkerFaceColor','r'); %Black Star for goal pos
    viscircles([x_obs(1),x_obs(2)],r); % plot the obstacle
    viscircles([x_hist(k,1), x_hist(k,2)], r_robot); % plot the moving object
    xlim([-1 5]); ylim([-1 5]);
    xlabel("pos x (m)"); ylabel("pos y (m)");
    title(sprintf('Time: %.2f s', (k-1)*Ts));
    drawnow;
    pause(0.01);
end

%Plotting u_nomx vs u_starx (linear velocity)
figure (3)
hold on; grid on; hold on;
plot (time_hist,ustarx,'LineWidth',2)
plot (time_hist,ustary,'LineWidth',2)
plot (time_hist,u_nomx,'LineWidth',2)
plot (time_hist,u_nomy,'LineWidth',2)
title ('Linear Velocity x')
xlabel ('time')
ylabel ('Control Input')
legend('vx', 'vy', 'u_gtg_x', 'u_gtg_y')

figure(4)
hold on; grid on;
plot (time_hist,cbf,'LineWidth',2)
hold on
% plot (time_hist,cbf2,'LineWidth',2)
xlabel('Time (s)');
ylabel('CBF value');
title('Time series of CBF constraint');

figure(5)
hold on; grid on;
plot(x_hist(:,1), x_hist(:,2), 'b.-', 'MarkerSize', 2); % plot the entire trajectory up to the current point
plot(x_hist(1),x_hist(2),'p','LineWidth',2,'MarkerSize',6,...
    'MarkerEdgeColor','r','MarkerFaceColor','r'); %Red Star for Init pos
plot(x_gtg(1),x_gtg(2),'p','LineWidth',2,'MarkerSize',6,...
    'MarkerEdgeColor','k','MarkerFaceColor','r'); %Black Star for goal pos
viscircles([x_obs(1),x_obs(2)],r); % plot the obstacle
viscircles([x_hist(k,1), x_hist(k,2)], r_robot); % plot the moving object
xlim([-1 5]); ylim([-1 5]);
xlabel("pos x (m)"); ylabel("pos y (m)");
title('Result in X-Y axis');
legend('\gamma(h(x) = 10h(x)')

figure(6)
hold on; grid on;
plot (time_hist,dist,'LineWidth',2)
xlabel('Time (s)');
ylabel('||u_{gtg} - u ||^2');
