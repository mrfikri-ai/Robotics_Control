% CBF Using differential drive

clear
close all
clc

%parameters 
b=0.3/2; %length from center of body to wheel
eps = 0.15; %tolerance for h(x)
cbf_gamma = 10; %gamma for CBF
cbf_power = 3; %power for CBF

% Simulation length (in steps)
k_max = 1500;

% Initial positions
x_0 = 0; 
y_0 = 0;
theta_0 = 60;
plot(x_0,y_0,'p','LineWidth',2,'MarkerSize',6,...
    'MarkerEdgeColor','r','MarkerFaceColor','r') %Red Star for Init pos
hold on

% Goal Position
xg = 3; 
yg = 3;
plot(xg,yg,'p','LineWidth',2,'MarkerSize',6,...
    'MarkerEdgeColor','k','MarkerFaceColor','r') %Black Star for goal pos
hold on

% Obstacle position
xo = 1.5;
yo = 1.8;
r = 0.7; %radius of the obstacle
viscircles([xo,yo],r); %Red Circle for plot obstacle

% Reference vectors
x_r = xg*ones(k_max,1);
y_r = yg*ones(k_max,1);

% Preallocate memory
x = zeros(k_max,1); % position on x axis
y = zeros(k_max,1); % position on y axis
v = zeros(k_max,1); % velocity
theta = zeros(k_max,1); % angular position
omega = zeros(k_max,1); % angular velocity
x_e = zeros(k_max,1); % error in x
y_e = zeros(k_max,1); % error in y
theta_e = zeros(k_max,1); % error in theta
theta_d = zeros(k_max,1); % desired theta
p_e = zeros(k_max,1); % error in position
cbf = zeros(k_max,1); % CBF
u_nomx = zeros(k_max,1);
u_nomy = zeros(k_max,1);
ustarx = zeros(k_max,1);
ustary = zeros(k_max,1);

% Time step
T = 0.01; %change to 0.01 for better result but change k_max to 1500

% Proportional controller gains
k_v = 0.4; % velocity gain
k_omega = 1; % angular velocity gain

% Plot animation-----------------
% Initial robot
hold on
grid on
wls=[0 0;-0.5*b 0.5*b]; %car
wlsrot=[cos(theta_0) -sin(theta_0);sin(theta_0) cos(theta_0)]*wls; %orientation of car
h1=plot(wlsrot(1,1)+x_0,wlsrot(2,1)+y_0,'ro','LineWidth',2,'MarkerFaceColor','r'); %plot animation for left wheel
h2=plot(wlsrot(1,2)+x_0,wlsrot(2,2)+y_0,'ro','LineWidth',2,'MarkerFaceColor','r'); %plot animation for left wheel
h3=plot(x_0,y_0,'bo','MarkerSize',20); %plot animation for the body
axis equal

pause (5)
for t = 1:k_max
       
    % Calculate error in position
    p_e(t) = norm([xg;yg] - [x(t);y(t)],2);
    
    % Calculate control input 1 (velocity)
    v(t+1) = k_v*p_e(t);
    
    % Calculate desired theta using atan2
    theta_d(t) = atan2((y_r(t) - y(t)),(x_r(t) - x(t)));
  
    % Calculate error in theta
    theta_e(t) = theta_d(t) - theta(t);
    
    % Calculate control input 2 (angular velocity)
    omega(t+1) = k_omega*theta_e(t);

    % U Nominal
    u_nom = [v(t+1) omega(t+1)];
    u_nomx(t) = u_nom(1);
    u_nomy(t) = u_nom(2);

    % U using CBF
    P_mat = 2 * eye(2); %matrix [2 0;0 2]
    q_mat = -2 * u_nom; %matrix [-2*v(t+1)  -2*omega(t+1)]

    hx = (x(t) - xo)^2 + (y(t) - yo)^2 - (r+eps)^2; %CBF constraint
    lghx = (2*(x(t)-xo)*v(t)*cos(theta(t)) + 2*(y(t)-yo)*v(t)*sin(theta(t))); %derivative of hx
    cbf(t) = lghx + cbf_gamma * (hx); %CBF final form

    %if cbf(t)<0
    %    break
    %end
    
    temp_G = [x(t)-xo y(t)-yo]; %current position - obstacle position
    G_mat = -2*temp_G; %matrix of cur pos - obs pos
    h_mat = [cbf(t)]; %cbf
    options =  optimset('Display','off'); %verbose = off
    sol = quadprog(P_mat, q_mat, G_mat, h_mat, [], [], [], [], [], options); %QP to find v and omega
    u = [sol(1) sol(2)]; %u(1) is v, u(2) is omega
    
    ustarx(t) = u(1); %ustarx is v (only for plot)
    ustary(t) = u(2); %ustary is omega (only for plot)
 
    % Update states
    x(t+1) = x(t) + T*cos(theta(t))*u(1);
    y(t+1) = y(t) + T*sin(theta(t))*u(1);
    theta(t+1) = theta(t) + T*u(2);
    
    %Plot for moving vehicle
    wlsrot=[cos(theta(t)) -sin(theta(t));sin(theta(t)) cos(theta(t))]*wls;
    set(h1,'XData',wlsrot(1,1)+x(t));%plot animation for left wheel
    set(h1,'YData',wlsrot(2,1)+y(t));%plot animation for left wheel
    set(h2,'XData',wlsrot(1,2)+x(t));%plot animation for right wheel
    set(h2,'YData',wlsrot(2,2)+y(t));%plot animation for right wheel
    set(h3,'XData',x(t));%plot animation for the body
    set(h3,'YData',y(t));%plot animation for the body
    
    drawnow; %update plot
    
end

plot(x,y,'k-'); %plot animation for the track

%Plotting u_nomx vs u_starx (linear velocity)
z = 1:k_max;
figure (2)
plot (z,ustarx,'LineWidth',2)
hold on
plot (z,u_nomx,'LineWidth',2)
title ('Linear Velocity')
xlabel ('time')
legend('u_x','unom_x')

%Plotting u_nomy vs u_stary (angular velocity)
figure (3)
plot (z,ustary,'LineWidth',2)
hold on
plot (z,u_nomy,'LineWidth',2)
title ('Angular Velocity')
xlabel ('time')
legend('u_y','unom_y')
