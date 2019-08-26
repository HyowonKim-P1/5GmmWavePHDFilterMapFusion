function [Output] = Map_PHD_calculation(para, Map, UE, Temp,mode)
    % 5G mmWave Positioning and Mapping
    % (c) Hyowon Kim, 2019 (Ph.D. student at Hanyang Univerisy, Seoul, South Korea, emai: khw870511@hanyang.ac.kr)
    % Usage: this code generates map PHD
    
    if para.onlyLOS == 1
        m_type = 1;
    else
        m_type = 1:3;
    end
    if mode == 2
        para.N_p = 1;
    end
        
    for m = m_type
        Temp(m).one_d = zeros(numel(para.X_grid),para.N_p);
        Temp(m).PHD_1d = zeros(numel(para.X_grid),para.N_p);
    end

    for m = m_type
        for i = 1:para.N_p
            Temp(m).PHD_1d_i = zeros(numel(para.X_grid),1);
            for  j = 1:Map.ST(m).P(i).J % BS/VA/SP
                temp_density = mvnpdf([para.X_grid para.Y_grid],Map.ST(m).P(i).x(j,1:2),Map.ST(m).P(i).P(1:2,1:2,j));
                if (sum(temp_density)~=0)
                    temp_density = temp_density/sum(temp_density);
                    Temp(m).PHD_1d_i = Temp(m).PHD_1d_i + Map.ST(m).P(i).weight(j)*temp_density;
                end
            end
            Temp(m).PHD_1d(:,i) = Temp(m).PHD_1d_i;
        end
        Temp(m).PHD = sum(Temp(m).PHD_1d.*UE.weight(1,:),2);
        Output(m).D = reshape(Temp(m).PHD,numel(para.yy),numel(para.xx));
    end
end