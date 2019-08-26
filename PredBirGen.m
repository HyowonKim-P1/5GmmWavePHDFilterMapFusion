function [x_hat,P] = PredBirGen(BS, state, measurement, j, m, R)
    % 5G mmWave Positioning and Mapping
    % (c) Hyowon Kim, 2019 (Ph.D. student at Hanyang Univerisy, Seoul, South Korea, emai: khw870511@hanyang.ac.kr)
    % Usage: this code generates birth map
    
    birth_error_figure = 0;
    C = inv(R); 
    z = measurement(j,:)';
    diff = 1e-3;
    alpha = .2;
    
    % Cubature update 
    n_D = 5; % number of dimensions
    X = zeros(3,2*n_D);
    xx = zeros(3,2*n_D);
    zeta = sqrt(n_D)*[eye(n_D,n_D) -eye(n_D,n_D)];
    
    % 1) factorize
    factorized_R = sqrtm(R);

    % 2) evaluate the cubature points (c_i = 1,2,...,2*n_D)
    for c_i = 1:2*n_D
        Z(:,c_i) = z + factorized_R*zeta(:,c_i);
    end
    
    % 3) evaluate the propgated cubature points (c_i = 1,2,...,2*n_D)
    for c_i = 1:2*n_D
        % initial point generation for each cubature point
        TOA = Z(1,c_i); AODaz = Z(2,c_i); AODel = Z(3,c_i); AOAaz = Z(4,c_i); AOAel = Z(5,c_i);
        if (m==2)
            Radius = TOA-state(7);
            R_xy = Radius*cos(AOAel);
            x_0 = [state(1) + R_xy*cos(AOAaz + state(4)) state(2) + R_xy*sin(AOAaz + state(4)) state(3) + Radius*sin(AOAel)]; 
        elseif (m==3)
            Radius = TOA-state(7);
            R_xy = Radius*cos(AOAel);
            VA_geo = [state(1) + R_xy*cos(AOAaz + state(4)) state(2) + R_xy*sin(AOAaz + state(4)) state(3) + Radius*sin(AOAel)]';
            u = (BS.pos-VA_geo)/norm(BS.pos-VA_geo);
            f = (BS.pos+VA_geo)/2;
            x_0 = (VA_geo + (f-VA_geo)'*u/( (state(1:3,1) - VA_geo)'*u )* (state(1:3,1)-VA_geo))';
        end
        xx(:,c_i) = x_0';
        % minimize_x G(x) = [H(x) - Z]^T R^(-1) [H(x) - Z] with iteration process [Iterative maximum likelihood estimation method]
        x_stack = [];
        x_stack(:,1) = x_0';
        [HX,HZ,DH] = F_JacobianDiff(BS,state,x_0,diff,Z(:,c_i),m);
        Gx(2,1) = (HX-HZ)'*C*(HX-HZ);
        Gx(1,1) = Gx(2,1) + 1;
        iter = 1;
        while (iter==1 || Gx(iter+1,1)<Gx(iter,1))
            iter = iter + 1;
            if alpha+.2 <= 1
                alpha = alpha + .2;
            end
            A = DH;
            K=inv(A'*C*A + A'*C'*A);
            B = -DH*x_0' + HX - HZ;
            BM_hat = K*(-A'*C*B-A'*C'*B);
            x_stack(:,iter) = (1-alpha)*x_0' + alpha*BM_hat;
            x_0 = x_stack(:,iter)';
            [HX,HZ,DH] = F_JacobianDiff(BS,state,x_0,diff,HZ,m);
            Gx(iter+1,1) = (HX-HZ)'*C*(HX-HZ);
        end
        X(:,c_i) = x_stack(:,iter-1);
    end
    
    XX = 0;
    for c_i = 1:2*n_D
        XX = XX + X(:,c_i)*X(:,c_i)';
    end
    x_hat = mean(X,2); % Initial birth mean (3 by 1)
    P = XX/(2*n_D) - x_hat*x_hat'; % Initial birth covariance (3 by 3)
    P = diag(diag(P));
    
end


