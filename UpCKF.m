function [P_up, z_hat, K, S] = UpCKF(P, x, BS, UEsample, R, m, para)
    % 5G mmWave Positioning and Mapping
    % (c) Hyowon Kim, 2019 (Ph.D. student at Hanyang Univerisy, Seoul, South Korea, emai: khw870511@hanyang.ac.kr)
    % Usage: this code generates the CKF compoenents
    
    
    % z_hat (5X5) 
    % Residual covariance S (5X5 = 5X3 3X3 3X5 + 5X5)
    % Kalman gain K (3X5 = 3X3 3X5 5X5)
    % Update covariance estimate P_up (3X3 = [3X3 - 3X5 5X3] 3X3)
    
    % parameter
    n_D = 3; % # of object state dimensions
    zeta = sqrt(n_D)*[eye(n_D,n_D) -eye(n_D,n_D)];
    
    % 1) factorize
    factorized_P = sqrtm(P);

    % 2) evaluate the cubature points (c_i = 1,2,...,2*n_D)
    for c_i = 1:2*n_D
        X(:,c_i) = x' + factorized_P*zeta(:,c_i);
    end

    % 3) evaluate the propgated cubature points (c_i = 1,2,...,2*n_D)
    for c_i = 1:2*n_D % considering BS/VA/SP cases
        if (m ==1)
            Z(:,c_i) = getChannelParameters(BS.pos,UEsample,X(:,c_i),'LOS');
        elseif (m==2)
            Z(:,c_i) = getChannelParameters(BS.pos,UEsample,X(:,c_i),'VA');
        elseif (m==3)
            Z(:,c_i) = getChannelParameters(BS.pos,UEsample,X(:,c_i),'SP'); 
        end
        Z(:,c_i) = F_MeaCali(Z(:,c_i));
        if c_i > 1
            Z(:,c_i) = F_InovCali(Z(:,c_i),Z(:,1));
        end
    end
    
    % 4) estimate the predicted measurement
    z_hat = mean(Z,2);
    
    % 5) estimate the innovation covariance matrix
    ZZ = 0;
    for c_i = 1:2*n_D
        ZZ = ZZ + Z(:,c_i)*Z(:,c_i)';
    end
    S = ZZ/(2*n_D) - z_hat*z_hat' + R;

    if m ~= 1
        S_inv = inv(S);
        % 6) estimate the corss-covariance matrix
        cross_XZ=0;
        for c_i = 1:2*n_D
            cross_XZ = cross_XZ + X(:,c_i)*Z(:,c_i)';
        end
        cross_XZ = cross_XZ/(2*n_D);
        P_cross = cross_XZ - x'*z_hat';

        % 7) estimate the Kalman gain
        K = P_cross*S_inv;

        % 8) updated state will be performed in the PHD update step
        
        % 9) estimate the corresponding error covariance
        P_up = P - K*S*K';
        if det(S_inv) <= 0
            sprintf('Residual covariance error')
        end
        if det(P_up) <= 0
            sprintf('Posterior covariance error')
        end
    end
    
    if m == 1
        K = zeros(3,5);
        P_up = para.BS_cov*eye(3,3);
    end
    
end
    