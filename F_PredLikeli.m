function likelihood = F_PredLikeli(z,z_hat,S)
    % 5G mmWave Positioning and Mapping
    % (c) Hyowon Kim, 2019 (Ph.D. student at Hanyang Univerisy, Seoul, South Korea, emai: khw870511@hanyang.ac.kr)
    % Usage: this code generates likelihood calulcation p(z|h(x), S)
    % z: measurement
    % z_hat: predicted measurement
    % S: residual covariance
    
    z = F_MeaCali(z); z_hat = F_MeaCali(z_hat);
    z = F_InovCali(z, z_hat);
    likelihood = 1/sqrt((2*pi)^5*det(S))*exp(-1/2*(z-z_hat)'*S^-1*(z-z_hat));
end