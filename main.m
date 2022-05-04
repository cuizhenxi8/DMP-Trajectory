%%
clear
clc
close all

%% Parameter Setting
n = 4;                                %Number of agents
kv = 10;                             % Control gain kv
ka = 10;                             % Control gain ka

Adj = [0 1 1 1;
            1 0 1 1 ;
            1 1 0 1 ;
            1 1 1 0 ];
 
d = zeros(n,n);                       % initialize the desired distance
x_coor = [-1;1;1;-1]*0.15;      % x coordinate of framework F*(t)
y_coor = [-1;-1;1;1]*0.15;      % y coordinate of framework F*(t)
z_coor = [0.1;0.1;0.1;0.1]*2;   % z coordinate of framework F*(t)

for ii = 1:n
    for jj = 1:n
        d(ii,jj) = sqrt((x_coor(ii)-x_coor(jj))^2+...
            (y_coor(ii)-y_coor(jj))^2+...
            (z_coor(ii)-z_coor(jj))^2);
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tfinal =5;                          % Simulation ending time  
h = 1e-1;                          
para.tau=tfinal;
% Encapusulate the paremeters into a structure para
para.n = n;
para.kv = kv;
para.ka = ka;
para.Adj = Adj;
para.d = d;

load('semi_circle.mat')
para.fx=force_term(1:1573,:);

%%
ub = 0.02;                           % Upper bound for random ini. condition
lb = -0.02;                          % Lower bound for random ini. condition
q_0 = [x_coor y_coor z_coor]'+(lb*ones(3,n)+(ub-lb)*rand(3,n));
% v_0 = (lb*rand(3,n)+(ub-lb)*rand(3,n))*2;    % Initial velocity
v_0 =zeros(3,4);
qv_0 = [q_0 v_0];


%% goal
x_coor = x_coor'-0.4;
y_coor = y_coor'-0.4;
z_coor = z_coor';

g=[x_coor; y_coor; z_coor];
g = reshape(g,1,[])';
para.ini=reshape(q_0,1,[])';
para.goal=g;

qv_0_vec = reshape(qv_0,1,[]);      % reshape qv_0 into a vector
time_span = 0:h:tfinal;             % simulation time span

[t,qv] = ode45(@Coupled_DMP, time_span, qv_0_vec,[],para);    % ODE



%% results
xx = qv(:,3*(0:n-1)+1);
yy = qv(:,3*(0:n-1)+2);
zz = qv(:,3*(0:n-1)+3);
vx = qv(:,3*(0:n-1)+3*n+1);
vy = qv(:,3*(0:n-1)+3*n+2);
vz = qv(:,3*(0:n-1)+3*n+3);
plot3(xx(:,1:4),yy(:,1:4),zz(:,1:4));
view([30 34])

x_c_2 = 0;
y_c_2 = -0.4;
z_c_2 = 0.35;
a_2 = .1;
b_2 = .1;
c_2 = .1;
hold on
[x, y, z] = ellipsoid(x_c_2,y_c_2,z_c_2,a_2,b_2,c_2);
surf(x, y, z)
axis equal
plot3(xx(1:1:51,[1:4 1])', yy(1:1:51,[1:4 1])', zz(1:1:51,[1:4 1])','blue')
xlabel('x') 
ylabel('y') 
zlabel('z') 




