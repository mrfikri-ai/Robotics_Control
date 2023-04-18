% This code works for CBF between two agent
% TODO: extend this code to be able controlling robot more than 2

clc, clear, close all;

% Define parameters
Ts = 0.01; % in seconds (time sampling)
Tmax = 8; % maximum simulation time
time_hist = 0:Ts:Tmax;

P_gain = 0.4; % Gain parameter 
robot_num = 2; % Number of robots (updated to 4)
des_dist = 0.2; % Desired distance

% Initialize array to store information
n = length(time_hist);
x_hist = zeros(n,2*robot_num);

cbf = zeros(n,1); % Define cbf value
gamma = 10; % Gamma value for CBF

% Initialize value [p_1x, p_1y, ...]'
x_hist(1,:) = [-0.6, 0.6, -0.2, 0.4]; % Update to include initial positions for 4 robots

% Initialize value [u_goal_x, u_goal_y ...]'
x_goal = [1.2, 2.5, -0.4, 2.5]; % Update to include goal positions for 4 robots

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
   
    cbf(k) = gamma * hx; % Update to compute CBF value for 4 robots
    
    % Quadratic programming parameters
    Q = 2 * eye(2*robot_num); % Update to include quadratic cost for 4 robots
    c = -2 * u_gtg';  
    temp = [x_k(1)-x_k(3) x_k(2)-x_k(4) x_k(3)-x_k(1) x_k(4)-x_k(2)];
    
    H = -2* temp; % Update to include constraints for 4 robots

    b = cbf(k); % Update to include constraints for 4 robots
    options = optimset('Display','off');
    sol = quadprog(Q, c, H, b, [], [], [], [], [], options); % Update to remove Aeq and beq as there are no equality constraints
    u = [sol(1) sol(2) sol(3) sol(4)]; % Update to extract control input for 4 robots
    
    % Update the next states
    x_hist(k+1,:) = x_k + Ts*u;
end


for k=1:n
    clf; hold on; grid on;
    % Agent 1
    plot(x_hist(:,1), x_hist(:,2), 'b.-', 'MarkerSize', 2); % plot the entire trajectory up to the current point
    plot(x_hist(k,1), x_hist(k,2), 'ro', 'MarkerSize', 5); % plot the current point in red

    % Agent 2
    plot(x_hist(:,3), x_hist(:,4), 'r.-', 'MarkerSize', 2); % plot the entire trajectory up to the current point
    plot(x_hist(k,3), x_hist(k,4), 'ro', 'MarkerSize', 5); % plot the current point in red

    
    plot(x_goal(1),x_goal(2),'p','LineWidth',2,'MarkerSize',6,...
    'MarkerEdgeColor','k','MarkerFaceColor','k'); %Black Star for goal pos
    plot(x_goal(3),x_goal(4),'p','LineWidth',2,'MarkerSize',6,...
    'MarkerEdgeColor','r','MarkerFaceColor','r'); %Red Star for goal pos
    
    viscircles([x_hist(k,1), x_hist(k,2)], 0.2); % plot the moving object
    viscircles([x_hist(k,3), x_hist(k,4)], 0.2); % plot the moving object
    xlim([-2 2]); ylim([-2 5]);
    xlabel("pos x (m)"); ylabel("pos y (m)");
    title(sprintf('Time: %.2f s', (k-1)*Ts));
    drawnow;
    pause(0.01);
end
