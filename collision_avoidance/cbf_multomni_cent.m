% TODO: Fix the H value to make the robots avoid from collision
% NOTE: This code is cbf with centralized safety control

clc, clear, close all;

% Define parameters
Ts = 0.01; % in seconds (time sampling)
Tmax = 8; % maximum simulation time
time_hist = 0:Ts:Tmax;

P_gain = 0.4; % Gain parameter 
robot_num = 4; % Number of robots (updated to 4)
des_dist = 0.2; % Desired distance

% Initialize array to store information
n = length(time_hist);
x_hist = zeros(n,2*robot_num);

cbf = zeros(n,1); % Define cbf value
gamma = 10; % Gamma value for CBF

% Initialize value [p_1x, p_1y, ...]'
x_hist(1,:) = [-0.6, 0.6, -0.2, 0.4, 0.2, 0.5, 0.6, 0.6]; % Update to include initial positions for 4 robots

% Initialize value [u_goal_x, u_goal_y ...]'
x_goal = [1.2, 2.5, -0.4, 2.5, 0.4, 2.5, -1.2, 2.5]; % Update to include goal positions for 4 robots

% Simulation loop
for k = 1:n-1
    % Current state of the robots
    x_k = x_hist(k,:);
    
    % Compute control input (centralized)
    u = ones(2*robot_num, 1)'; % Update to include control input for 4 robots
    
    % Compute u nominal
    u_gtg = (x_goal - x_k);
    
    % Control barrier function parameters
    hx = norm(x_k(1:2) - x_k(3:4)).^2 - des_dist^2; %CBF constraint of 2 agents; % Initialize CBF constraint values for 4 robots
    count = 1;
    for i = 1:robot_num
        for j = i+1:robot_num
            hx(count) = norm(x_k((i-1)*2+1:i*2) - x_k((j-1)*2+1:j*2))^2 - des_dist^2; % Update to include CBF constraint for 4 robots
            count = count + 1;
        end
    end
    
    hx = reshape(hx, [], 1); % Reshape hx into a column vector
    cbf(k) = gamma * sum(hx); % Update to compute CBF value for 4 robots
    
    % Quadratic programming parameters
    Q = 2 * eye(2*robot_num); % Update to include quadratic cost for 4 robots
    c = -2 * u_gtg';  
    temp = [x_k(1)-x_k(3) x_k(2)-x_k(4) x_k(3)-x_k(1) x_k(4)-x_k(2)];
    temp1 = [x_k(1)-x_k(3) x_k(2)-x_k(4) x_k(3)-x_k(1) x_k(4)-x_k(2) x_k(5)-x_k(1) x_k(6)-x_k(2) x_k(7)-x_k(1) x_k(8)-x_k(2)];
    temp2 = [x_k(3)-x_k(1) x_k(4)-x_k(2) x_k(1)-x_k(3) x_k(2)-x_k(4) x_k(5)-x_k(3) x_k(6)-x_k(4) x_k(7)-x_k(3) x_k(8)-x_k(4)];
    temp3 = [x_k(5)-x_k(1) x_k(6)-x_k(2) x_k(1)-x_k(5) x_k(2)-x_k(6) x_k(3)-x_k(5) x_k(4)-x_k(6) x_k(7)-x_k(5) x_k(8)-x_k(6)];
    temp4 = [x_k(7)-x_k(1) x_k(8)-x_k(2) x_k(1)-x_k(7) x_k(2)-x_k(8) x_k(7)-x_k(3) x_k(8)-x_k(4) x_k(7)-x_k(5) x_k(8)-x_k(6)];
    temp5 = [x_k(1)-x_k(3) x_k(2)-x_k(4) x_k(3)-x_k(1) x_k(4)-x_k(2) x_k(1)-x_k(3) x_k(2)-x_k(4) x_k(3)-x_k(1) x_k(4)-x_k(2)];
    temp6 = [x_k(3)-x_k(1) x_k(4)-x_k(2) x_k(3)-x_k(5) x_k(4)-x_k(6) x_k(3)-x_k(1) x_k(4)-x_k(2) x_k(3)-x_k(5) x_k(4)-x_k(6)];
    temp_total = [temp1; temp2; temp3; temp4; temp5; temp6];
 
    H = -2* temp_total; % Update to include constraints for 4 robots

    b = 2*gamma*hx.^3; % Update to include constraints for 4 robots
    options = optimset('Display','off');
    sol = quadprog(Q, c, H, b, [], [], [], [], [], options); % Update to remove Aeq and beq as there are no equality constraints
    u = [sol(1) sol(2) sol(3) sol(4) sol(5) sol(6) sol(7) sol(8)]; % Update to extract control input for 4 robots
    
    % Update the next states
    x_hist(k+1,:) = x_k + Ts*u;

    % Extract time series data
    dist_hist = zeros(n, robot_num * (robot_num - 1) / 2);
    count = 1;
    for i = 1:robot_num
        for j = i+1:robot_num
            dist_hist(:,count) = vecnorm(x_hist(:,(i-1)*2+1:i*2) - x_hist(:,(j-1)*2+1:j*2), 2, 2);
            count = count + 1;
        end
    end
end


for k=1:n-1
    clf; hold on; grid on;
    % Agent 1
    plot(x_hist(:,1), x_hist(:,2), 'b.-', 'MarkerSize', 2); % plot the entire trajectory up to the current point
    plot(x_hist(k,1), x_hist(k,2), 'ro', 'MarkerSize', 5); % plot the current point in red

    % Agent 2
    plot(x_hist(:,3), x_hist(:,4), 'r.-', 'MarkerSize', 2); % plot the entire trajectory up to the current point
    plot(x_hist(k,3), x_hist(k,4), 'ro', 'MarkerSize', 5); % plot the current point in red

    % Agent 3
    plot(x_hist(:,5), x_hist(:,6), 'k.-', 'MarkerSize', 2); % plot the entire trajectory up to the current point
    plot(x_hist(k,5), x_hist(k,6), 'ko', 'MarkerSize', 5); % plot the current point in red

    % Agent 4
    plot(x_hist(:,7), x_hist(:,8), 'k.-', 'MarkerSize', 2); % plot the entire trajectory up to the current point
    plot(x_hist(k,7), x_hist(k,8), 'ko', 'MarkerSize', 5); % plot the current point in red
    
    plot(x_goal(1),x_goal(2),'p','LineWidth',2,'MarkerSize',6,...
    'MarkerEdgeColor','k','MarkerFaceColor','k'); %Black Star for goal pos
    plot(x_goal(3),x_goal(4),'p','LineWidth',2,'MarkerSize',6,...
    'MarkerEdgeColor','r','MarkerFaceColor','r'); %Red Star for goal pos
    plot(x_goal(5),x_goal(6),'p','LineWidth',2,'MarkerSize',6,...
    'MarkerEdgeColor','r','MarkerFaceColor','r'); %Red Star for goal pos
    plot(x_goal(7),x_goal(8),'p','LineWidth',2,'MarkerSize',6,...
    'MarkerEdgeColor','r','MarkerFaceColor','r'); %Red Star for goal pos
    
    viscircles([x_hist(k,1), x_hist(k,2)], 0.2); % plot the moving object
    viscircles([x_hist(k,3), x_hist(k,4)], 0.2); % plot the moving object
    viscircles([x_hist(k,5), x_hist(k,6)], 0.2); % plot the moving object
    viscircles([x_hist(k,7), x_hist(k,8)], 0.2); % plot the moving object
    xlim([-2 2]); ylim([-2 5]);
    xlabel("pos x (m)"); ylabel("pos y (m)");
    title(sprintf('Time: %.2f s', (k-1)*Ts));
    drawnow;
    pause(0.01);
end



% Plot time series of Euclidean distance
figure(2);
plot(time_hist, dist_hist, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Euclidean Distance');
title('Time Series of Euclidean Distance between Robot Pairs');
legend('Robot 1-2', 'Robot 1-3', 'Robot 1-4', 'Robot 2-3', 'Robot 2-4', 'Robot 3-4', 'Location', 'best');
grid on;

% Extract data for plotting
u_gtg_hist = zeros(n, 2*robot_num); % Array to store u_gtg for all robots
u_i_hist = zeros(n, 2*robot_num); % Array to store u_i for all robots
for k = 1:n
    x_k = x_hist(k,:);
    u_gtg = (x_goal - x_k);
    u_gtg_hist(k,:) = u_gtg;
    u_i = [sol(1) sol(2) sol(3) sol(4) sol(5) sol(6) sol(7) sol(8)];
    u_i_hist(k,:) = u_i;
end

% Plot u_gtg vs time for all robots
figure;
plot(time_hist, u_gtg_hist(:,1), 'b-', 'LineWidth', 1.5, 'DisplayName', 'u_{gtg1}');
hold on;
plot(time_hist, u_gtg_hist(:,2), 'g-', 'LineWidth', 1.5, 'DisplayName', 'u_{gtg2}');
plot(time_hist, u_gtg_hist(:,3), 'c-', 'LineWidth', 1.5, 'DisplayName', 'u_{gtg3}');
plot(time_hist, u_gtg_hist(:,4), 'y-', 'LineWidth', 1.5, 'DisplayName', 'u_{gtg4}');
plot(time_hist, u_i_hist(:,1), 'r', 'LineWidth', 1.5, 'DisplayName', 'u_{gtg4}');
plot(time_hist, u_i_hist(:,2), 'k', 'LineWidth', 1.5, 'DisplayName', 'u_{gtg4}');
plot(time_hist, u_i_hist(:,3), 'b', 'LineWidth', 1.5, 'DisplayName', 'u_{gtg4}');
plot(time_hist, u_i_hist(:,4), 'y', 'LineWidth', 1.5, 'DisplayName', 'u_{gtg4}');
xlabel('Time (s)');
ylabel('u_{gtg}');
title('u_{gtg} vs u_i');
legend('Location', 'best');
grid on;


