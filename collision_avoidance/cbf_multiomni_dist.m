% TODO: Finish the code
% NOTE: This code is cbf with centralized safety control

clc, clear, close all;

% Define parameters
Ts = 0.01; % in seconds (time sampling)
Tmax = 8; % maximum simulation time
time_hist = 0:Ts:Tmax;

robot_num = 4; % Number of robots (updated to 4)
des_dist = 0.2; % Desired distance

% Initialize array to store information
n = length(time_hist);
x_hist = zeros(n,2*robot_num);
u_hist = zeros(n, 2 * robot_num);
cbf = zeros(n,1); % Define cbf value
gamma = 10; % Gamma value for CBF

% Initialize value [p_1x, p_1y, ...]'
x_hist(1,:) = [-0.6, 0.6, -0.2, 0.4, 0.2, 0.5, 0.6, 0.6]; % Update to include initial positions for 4 robots

% Initialize value [u_goal_x, u_goal_y ...]'
x_goal = [1.2, 2.5, -0.4, 2.5, 0.4, 2.5, -1.2, 2.5]; % Update to include goal positions for 4 robots

% Simulation loop
for k = 1:n-1
    for i = 1:robot_num
        % Get array index to be sliced
        idx = 2*i-1;

        % Current state of the robot
        x_i_k = x_hist(k, idx:idx+1);

        % Compute control distributed
        u_goal = (x_goal(idx:idx+1) - x_i_k); % Unit vector towards goal

        % Compute control distributed
        u_i = [1,1];

        % Initialize CBF constraint values for 4 robots
        hx_vals = norm(x_i_k(1) - x_i_k(2)).^2 - des_dist^2;
        cbf(k) = gamma * hx_vals; 

        % Compute the qudprog
        Q = 2*eye(2);
%         c = -2 * u_gtg((i-1)*2+1:i*2)';

        % Update 
        x_hist(k+1, idx:idx+1) = x_i_k + Ts*u_i;
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
    title('Distributed Systems Control');
    drawnow;
    pause(0.01);
end

