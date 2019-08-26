function log_likelihood = F_PredLogLikeli(z,z_hat,S)
    % 5G mmWave Positioning and Mapping
    % (c) Hyowon Kim, 2019 (Ph.D. student at Hanyang Univerisy, Seoul, South Korea, emai: khw870511@hanyang.ac.kr)
    % Usage: this code generates log-likelihood calulcation log p(z|h(x), S)
    % z: measurement
    % z_hat: predicted measurement
    % S: residual covariance
    
    z = F_MeaCali(z); z_hat = F_MeaCali(z_hat);
    z = F_InovCali(z,z_hat);
    residual = z-z_hat;
    log_likelihood = -0.5*log((2*pi)^5*det(S))-1/2*(residual)'*inv(S)*(residual);
    if abs(residual(2)) > pi || abs(residual(4)) > pi || abs(residual(3)) > pi || abs(residual(5)) > pi
        sprintf('residual error')
    end
end