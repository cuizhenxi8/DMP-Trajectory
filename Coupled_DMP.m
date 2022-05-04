function [ dqv ] = Coupled_DMP( t,qv_vec,para )
%% Obtain parameters from structure para
n = para.n;
kv = para.kv;
Adj = para.Adj;
d = para.d;
ka = para.ka;
rr=zeros(12,1);
qv = reshape(qv_vec,3,[]);
q = qv(:,1:n);
v = qv(:,n+1:2*n);

%%
z = zeros(3*n-6,1);             % initialize z
R = zeros(3*n-6,3*n);           % initialize Rigidity Matrix
R_dot = zeros(3*n-6,3*n);       % Rigidity Matrix with v_ij as its term
e = zeros(n,n);                 % Distance error matrix
 
%% Construct R and R_dot, obtain e and z
ord = 1;
for i = 1:n-1
    for j = i+1:n
        e(i,j) = sqrt((q(:,i)-q(:,j))'*(q(:,i)-q(:,j)))-d(i,j);
        %         e(i,j) =e(i,j) *1;
        if Adj(i,j) == 1
            z(ord) = e(i,j)*(e(i,j)+2*d(i,j));
            R(ord,3*i-2:3*i) = (q(:,i)-q(:,j))';
            R(ord,3*j-2:3*j) = (q(:,j)-q(:,i))';
            R_dot(ord,3*i-2:3*i) = (v(:,i)-v(:,j))';
            R_dot(ord,3*j-2:3*j) = (v(:,j)-v(:,i))';
            ord = ord+1;
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
v = qv_vec(3*n+1:6*n);       % Change v from  2Xn matrix to a column vector
ua = -kv*R'*z;                       % Acquisition control for single integrator model
vf = ua;
vfdot = -kv*R_dot'*z-kv*R'*2*R*v; % time derivative for ua
s = v-vf;

%%%% define the DMP goal position
g = para.goal;
gt =linspace(0,para.tau,1573);
fx = interp1(gt',para.fx,t);
fx=kron(ones(4,1),fx')/1.*(para.ini-para.goal+[0 0 0.4 0 0 0.4 0 0 0.4 0 0 0.4 ]' );

rr=collision_avoid(qv_vec(1:3*n))*2;
rr=repmat(rr(4:6),[4 1]);
%%
if norm(e)<0.01
    u =(-ka*s+vfdot-R'*z)*0 +1*(fx+48*(12*(g-qv_vec(1:3*n))-para.tau*v)+rr)/para.tau^2;
else
    u =(-ka*s+vfdot-R'*z)*15+1*(fx+48*(12*(g-qv_vec(1:3*n))-para.tau*v)+rr)/para.tau^2;
end
dqv = [v;u];