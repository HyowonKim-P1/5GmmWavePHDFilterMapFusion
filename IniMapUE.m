function [UE, Map, ULMap, V_Est]  = IniMapUE(para, state, BS)
    % 5G mmWave Positioning and Mapping
    % (c) Hyowon Kim, 2019 (Ph.D. student at Hanyang Univerisy, Seoul, South Korea, emai: khw870511@hanyang.ac.kr)
    % Usage: this code generates initial map and vehicle particle

    
    % Prior particle (longitudial and rotational velocity are known)
    UE.PosteriorCovariance = para.UECovInitial;
    prior_noise_dist = sqrt(UE.PosteriorCovariance*ones(length(UE.PosteriorCovariance),1)).*randn(length(UE.PosteriorCovariance),para.N_p);
    prior_near = sqrt(UE.PosteriorCovariance*ones(length(UE.PosteriorCovariance),1)).*ones(length(UE.PosteriorCovariance),1);
    UE.state = state + [1;-1;0;1;0;0;-1].*prior_near + prior_noise_dist; % will be replaced with the prior
    UE.state(5,:) = state(5,1)*ones(1,para.N_p);
    UE.state(6,:) = state(6,1)*ones(1,para.N_p);
    UE.weight = 1/para.N_p*ones(1,para.N_p);
    V_Est = sum(UE.state.*UE.weight,2);
    
    % Initial map generation
    for i = 1:para.N_p
        Map.ST(1).P(i).J = 1; % # of BS Gaussians = 1
        Map.ST(1).P(i).OriginatedMea = 0;
        if para.onlyLOS ~= 1
            for m = 2:3
                Map.ST(m).P(i).J = 0; % # of VA/SP Gaussians
                Map.ST(m).P(i).OriginatedMea = [];
            end
        end
    end
    
    % Initial Gaussians setting (# Gaussians, mean, and covariance) for BS/VA/SP
    % In the initial time, there are no Gaussians in the VA/SP map
    for i = 1:para.N_p
        Map.ST(1).P(i).x = BS.pos';
        Map.ST(1).P(i).P = diag([para.BS_cov, para.BS_cov,1e-2]);
        Map.ST(1).P(i).weight = 1;
        Map.ST(1).P(i).m_k = 1;
    end
    
    % UP and DL map initialization
    ULMap.ST(1).P(1).x = BS.pos'; ULMap.ST(1).P(1).P = diag([para.BS_cov, para.BS_cov,1e-2]); ULMap.ST(1).P(1).J = 1; ULMap.ST(1).P(1).weight = 1; ULMap.ST(1).P(1).m_k = 1;
    if para.onlyLOS ~= 1
        for m = 2:3
            ULMap.ST(m).P(1).x = []; ULMap.ST(m).P(1).P = []; ULMap.ST(m).P(1).J = 0; ULMap.ST(m).P(1).weight = []; ULMap.ST(m).P(1).m_k = 0;
        end
    end
end