%% Constants

% Link lengths
link_lengths = [0.4, 0.1];

% Stiffness coefficients
k_const = [1, 2, 0.5]*1e06;

% Stiffness matrix
K_t = diag(k_const);

% Number of experiments
N = 30;

% initialize thetas
theta = [0, 0, 0];

% Initialize the A matrices
At_1 = zeros(3,3);
At_2 = zeros(3,1);
%% Getting the True stiffness matrix
for n = 1:N
    % Random Wrench vector
    w = randn(6,1)*1000/4;

    % Random angle
    qr = randn(1,1)*pi/3;
    qt = rand(2,1);
    
    q = [qr, qt(1), qt(2)];

    % Compute the Jacobian
    [Jt, Jt1, Jt2, Jt3] = Jac_t_RPP(q, theta, link_lengths);
    
    % random noise
    eps = randn(6,1)*1e-05;
   
    % Calculate the deflection vector
    dt = (Jt / K_t * Jt')*w + eps;
    
    wt = w(1:3);
    
    % Calculate the A matrices
    A1 = Jt1(1:3) * Jt1(1:3)' * wt;
    A2 = Jt2(1:3) * Jt2(1:3)' * wt;
    A3 = Jt3(1:3) * Jt3(1:3)' * wt;
    
    A = [A1, A2, A3];
    
    At_1 = At_1 + A'*A;
    At_2 = At_2 + A'*dt(1:3);
end

%% Calculate the real compliance vector
Kc = At_1 \ At_2;

Ks = 1./Kc

%% Random force

F = [-500, -189, -2000, 0, 0, 0]';

%% Trajectory points
Trj_pts = zeros(3,60);

t = linspace(0,2*pi,60);

x = 16 * (sin(t)).^3 * 0.05;

y = 0.05*(13 * cos(t) - 5 * cos(2*t) - 2 * cos(3*t) - cos(4*t));

z = 0.75 * ones(1,60);

N = 60;

for n = 1:N
    Trj_pts(:,n) = IK_RPP([x(n), y(n), z(n)], link_lengths);
end

% Get the stiffness matrix
K_t = diag(Ks);

% Make an empty circle
circle = zeros(size(Trj_pts));

dt_stack = zeros(3,N);
for n = 1:N
    q = Trj_pts(:,n);
    [Jt, Jt1, Jt2, Jt3] = Jac_t_RPP(q, theta, link_lengths);
    eps = randn(6,1)*1e-05;
    dt = (Jt / K_t * Jt')*F + eps;
    circle(:,n) = dt(1:3) + [x(n); y(n); z(n)];
    dt_stack(:,n) = dt(1:3);
end

diff = - dt_stack;

compensation = [x; y; z] + diff;

%% Get the new angles 
for n = 1:N
    Trj_pts(:,n) = IK_RPP([compensation(1,n), compensation(2,n), compensation(3,n)], link_lengths);
end

% Make an empty circle
circle2 = zeros(size(Trj_pts));

for n = 1:N
    q= Trj_pts(:,n);
    [Jt, Jt1, Jt2, Jt3] = Jac_t_RPP(q, theta, link_lengths);
    eps = randn(6,1)*1e-05;
    dt = (Jt / K_t * Jt')*F + eps;
    circle2(:,n) = dt(1:3) + [compensation(1,n); compensation(2,n); compensation(3,n)];

end
figure(1)
scatter3(circle(1,:),circle(2,:),circle(3,:),20, [0 0 1])
hold on
scatter3(circle2(1,:),circle2(2,:),circle2(3,:),20, [0 0.75  0.5])
 
 
plot3(x,y,z, 'Color' , 'r')

legend('UnClaibrated','Calibrated','Desired Trajectory')

