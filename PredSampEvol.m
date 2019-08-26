function UE_sample = PredSampEvol(UE,state,para)
    % 5G mmWave Positioning and Mapping
    % (c) Hyowon Kim, 2019 (Ph.D. student at Hanyang Univerisy, Seoul, South Korea, emai: khw870511@hanyang.ac.kr)
    % Usage: this code generates particle evolution of vehicle state

    if para.N_p > 1 % when # particle samples is more than 1, then vehicle state is unknown
        UEpredSamples = UE.state;
        processNoiseSamples=sqrt(para.ProcessNoiseCovariance).*randn(7,para.N_p);
        for k=1:para.N_p
            UE_sample(:,k) = UEpredSamples(:,k) + [UEpredSamples(5,k)/UEpredSamples(6,k) * (sin( UEpredSamples(4,k) + para.t_du*UEpredSamples(6,k) ) - sin( UEpredSamples(4,k)));...
                                                 UEpredSamples(5,k)/UEpredSamples(6,k) * (-cos( UEpredSamples(4,k) + para.t_du*UEpredSamples(6,k) ) + cos( UEpredSamples(4,k)));...
                                                 0; para.t_du*UEpredSamples(6,k);0;0;0] + processNoiseSamples(:,k);
        end
        if para.BiasHeadingKnown == 1
            UE_sample(4,:) = state(4,1);
            UE_sample(7,:) = state(7,1);
        elseif para.HeadingKnown == 1
            UE_sample(4,:) = state(4,1);
            UE_sample(7,:) = state(7,1);
            UE_sample(1,:) = state(1,1);
        end
        UE_sample(5,:) = state(5,1); % longitudinal velocity is known
        UE_sample(6,:) = state(6,1); % rotational velocity is known
        
    elseif para.N_p ==1 % when # particle samples is 1, then vehicle state is known (for mapping debugging)
        UE_sample = state;
    end
end
