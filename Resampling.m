function up_UE = Resampling(up_UE, V_Est, state, para)
    % 5G mmWave Positioning and Mapping
    % (c) Hyowon Kim, 2019 (Ph.D. student at Hanyang Univerisy, Seoul, South Korea, emai: khw870511@hanyang.ac.kr)
    % Usage: this code generates the resampled particle
    
    if para.N_p > 1
        Dither.LH = .3;
        Dither.H = 0.6;
        Dither.B = .3;
        NumParticle = para.N_p;
        Dim = 7;
        ReParticle = F_ResampleFromEst(NumParticle,Dither,Dim,V_Est);
        up_UE.state = ReParticle;
        up_UE.weight = 1/para.N_p*ones(1,para.N_p);
        up_UE.state(5,:) = state(5,1);
        up_UE.state(6,:) = state(6,1);
        
        if para.BiasHeadingKnown == 1
            up_UE.state(4,:) = state(4,1);
            up_UE.state(7,:) = state(7,1);
        elseif para.HeadingKnown == 1
            up_UE.state(4,:) = state(4,1);
            up_UE.state(1,:) = state(1,1);
            up_UE.state(7,:) = state(7,1);
        end
        up_UE.PosteriorCovariance = cov(up_UE.state');
        
    elseif para.N_p == 1
        up_UE.PosteriorCovariance = diag([0.4; 0.4; 0; (3e-2)^2; 0; 0; 0.4^2]);
        up_UE.state = state + sqrt(up_UE.PosteriorCovariance)*randn(7,para.N_p);
        up_UE.weight = 1;
    end
end

