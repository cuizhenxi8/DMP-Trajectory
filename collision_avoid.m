function y = collision_avoid(qv_vec)

temp=reshape(qv_vec,3,[]);
y=zeros(12,1);

for a=1:4
    u=temp(:,a);
    n_dim=3;
    n_obt=1;
    phi=zeros(n_dim,1);
    phi_tal=zeros(n_dim,1);
    
    %% possible obstacles
    x_c_1 = -0.1;
    y_c_1 = -0.5;
    z_c_1 = 0.35;
    a_1 = .1;
    b_1 = .1;
    c_1 = .1;
    
    x_c_2 = 0;
    y_c_2 = -0.4;
    z_c_2 = 0.35;
    a_2 = .1;
    b_2 = .1;
    c_2 = .1;
    
    x_c_3 = -3;
    y_c_3 = 0;
    z_c_3 = 3;
    a_3 = 1.1;
    b_3 = 1.1;
    c_3 =1.1;
    %% self second plot
    
    center=[x_c_1 y_c_1 z_c_1;
        x_c_2 y_c_2 z_c_2;
        x_c_3 y_c_3 z_c_3];
    axis=[a_1 b_1 c_1;
        a_2 b_2 c_2;
        a_3 b_3 c_3 ];
    center=center(2:3,:);
    axis=axis(2:3,:);
    
    %%%%
    for j=1:n_obt
        K = 0;
        for i = 1:n_dim
            K =K+( (u(i) - center(j,i)) / axis(j,i)) ^2;
        end
        K = K - 1;
        
        for i = 1:n_dim
            phi(i) = (u(i) - center(j,i)) / axis(j,i) ^2;
            
        end
        
        phi =phi * ( exp(-K)) * (1 / K + 1 / K / K) * 2;
        
        phi_tal=phi_tal+phi;
    end
    y(1+(a-1)*3:a*3 ) = phi_tal;
end