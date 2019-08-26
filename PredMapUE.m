function [UE, Map, Birth]  = PredMapUE(previous_UE, previous_Map, para, state, Channel, Clutter, BS, measurement, R)
    % 5G mmWave Positioning and Mapping
    % (c) Hyowon Kim, 2019 (Ph.D. student at Hanyang Univerisy, Seoul, South Korea, emai: khw870511@hanyang.ac.kr)
    % Usage: this code generates predicted vehicle state and map
    
    %% UE prediction
    UE = previous_UE;
    UE.state = PredSampEvol(previous_UE,state,para);
    
    %% Map prediction
    N_m = sum(Channel) + Clutter.visible; % # births = # received measurements
    for i = 1:para.N_p
        Birth.ST(1).P(i).J = 0; % no brith in the BS map
        Map.ST(1).P(i).OriginatedMea = 0; % 0: no corresponding measurement
    end
    
    % birth generation in map prediction when all paths are used.
    if para.onlyLOS ~= 1
        for i = 1:para.N_p
            for m = 2:3
                Map.ST(m).P(i).J = N_m; % # of new features Gaussian = # measurements + # clutter
                Map.ST(m).P(i).OriginatedMea = zeros(N_m,1);
            end
        end

        % birth generation (# Gaussians, mean, and covariance) for VA/SP
        for i = 1:para.N_p
            for m = 2:3
                for j = 1:Map.ST(m).P(i).J
                    [Map.ST(m).P(i).x(j,:), Map.ST(m).P(i).P(:,:,j)] = PredBirGen(BS, UE.state(:,i), measurement, j, m, R);
                    Map.ST(m).P(i).weight(j,1) = para.birth_weight;
                    Map.ST(m).P(i).OriginatedMea(j,1) = j; % originated-measurement numbering
                end
                if m == 2 % for VA
                    Map.ST(m).P(i).J = numel(Map.ST(m).P(i).weight);
                end
                if m == 3 % for SP, if birth location is out of the FoV, the birth is not generated
                    SP_birth_dist = vecnorm(Map.ST(3).P(i).x' - UE.state(1:3,i))';
                    birth_id = find (SP_birth_dist <= para.SPVisibilityRadius + para.r_UC);
                    Map.ST(3).P(i).OriginatedMea = Map.ST(3).P(i).OriginatedMea(birth_id);
                    Map.ST(3).P(i).x = Map.ST(3).P(i).x(birth_id,:);
                    Map.ST(3).P(i).P = Map.ST(3).P(i).P(:,:,birth_id);
                    Map.ST(3).P(i).weight = Map.ST(3).P(i).weight(birth_id,:);
                    Map.ST(3).P(i).J = numel(Map.ST(3).P(i).weight);
                end
                Map.ST(m).P(i).m_k = sum(Map.ST(m).P(i).weight);
            end
        end
    end
    
    % predicted map = previous updated map + birth
    Map.ST(1).P = previous_Map.ST(1).P; % no birth in the BS map
    if para.onlyLOS ~= 1
        for i = 1:para.N_p
            for m = 2:3
                if (previous_Map.ST(m).P(i).J >0)
                    Birth.ST(m).P(i).J = Map.ST(m).P(i).J;
                    Map.ST(m).P(i).x = [previous_Map.ST(m).P(i).x; Map.ST(m).P(i).x];
                    Map.ST(m).P(i).P = cat(3, previous_Map.ST(m).P(i).P, Map.ST(m).P(i).P);
                    Map.ST(m).P(i).OriginatedMea = [previous_Map.ST(m).P(i).OriginatedMea; Map.ST(m).P(i).OriginatedMea];
                    Map.ST(m).P(i).weight = [previous_Map.ST(m).P(i).weight; Map.ST(m).P(i).weight];
                    Map.ST(m).P(i).m_k = sum(Map.ST(m).P(i).weight);
                    Map.ST(m).P(i).J = previous_Map.ST(m).P(i).J+ Map.ST(m).P(i).J;
                else
                    Birth.ST(m).P(i).J = Map.ST(m).P(i).J;
                    Map.ST(m).P(i).x = Map.ST(m).P(i).x;
                    Map.ST(m).P(i).P = Map.ST(m).P(i).P;
                    Map.ST(m).P(i).OriginatedMea = Map.ST(m).P(i).OriginatedMea;
                    Map.ST(m).P(i).weight = Map.ST(m).P(i).weight;
                    Map.ST(m).P(i).m_k = sum(Map.ST(m).P(i).weight);
                    Map.ST(m).P(i).J = Map.ST(m).P(i).J;
                end
            end
        end
    end

end