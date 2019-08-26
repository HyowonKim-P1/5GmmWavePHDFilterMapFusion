function [Map,Prior_map,update_map,map]  = ULMapFusion(fig_mode, Stack, PriorFusedMap, UE, ave_Map, para, state, BS, VA, SP, v, ti)
    % 5G mmWave Positioning and Mapping
    % (c) Hyowon Kim, 2019 (Ph.D. student at Hanyang Univerisy, Seoul, South Korea, emai: khw870511@hanyang.ac.kr)
    % Usage: this code generates the fused map at the BS
    
    para.beta_0 = .5;
    para.beta_i = 1 - para.beta_0;
    Map.ST(1).P(1) = PriorFusedMap.ST(1).P(1);
    for m = 2  % VA map // map fusion rule 1, 3
        if PriorFusedMap.ST(m).P(1).J > 0
            Map.ST(m).P(1).x = [PriorFusedMap.ST(m).P(1).x; ave_Map.ST(m).P(1).x];
            Map.ST(m).P(1).P = cat(3, PriorFusedMap.ST(m).P(1).P, ave_Map.ST(m).P(1).P);
            Map.ST(m).P(1).weight = [para.beta_0*PriorFusedMap.ST(m).P(1).weight; para.beta_i*ave_Map.ST(m).P(1).weight];
            Map.ST(m).P(1).m_k = sum(Map.ST(m).P(1).weight);
            Map.ST(m).P(1).J = PriorFusedMap.ST(m).P(1).J+ ave_Map.ST(m).P(1).J;
        else % fusion rule 2)
            Map.ST(m).P(1) = ave_Map.ST(m).P(1);
        end
    end
    for m = 3 % SP map
        % Calculating FoV (if j_p is within FoV (Ind_FoV = 1), or (Ind_FoV = 0))
        if PriorFusedMap.ST(m).P(1).J > 0
            Ind_FoV = ones(PriorFusedMap.ST(m).P(1).J,1);
            R_p = zeros(PriorFusedMap.ST(m).P(1).J,5);
            for j_p = 1:PriorFusedMap.ST(m).P(1).J
                l = 0;
                for k = ti-para.ULTD+1:ti
                    l = l+1;
                    R_p(j_p,l) = max(vecnorm(Stack(k).V_Est(1:3,v) - Stack(k).up_UE(v).state(1:3,:)));
                    if norm(PriorFusedMap.ST(m).P(1).x(j_p,:) - Stack(k).V_Est(1:3,v)') > para.SPVisibilityRadius + R_p(j_p,l)
                        Ind_FoV(j_p,1) = 0;
                    end
                end
            end
        end
        % Condition of up-link map fusion
        if PriorFusedMap.ST(m).P(1).J > 0 % map fusion rule 1, 3
            Map.ST(m).P(1).x = [PriorFusedMap.ST(m).P(1).x; ave_Map.ST(m).P(1).x];
            Map.ST(m).P(1).P = cat(3, PriorFusedMap.ST(m).P(1).P, ave_Map.ST(m).P(1).P);
            
            beta_0 = 10*ones(PriorFusedMap.ST(m).P(1).J,1); beta_i = 10*ones(ave_Map.ST(m).P(1).J,1); 
            Check_all_ja = 10*ones(ave_Map.ST(m).P(1).J,PriorFusedMap.ST(m).P(1).J);
            Check_all_jp = 10*ones(PriorFusedMap.ST(m).P(1).J,ave_Map.ST(m).P(1).J);
            for j_p = 1:PriorFusedMap.ST(m).P(1).J % j_p -> j_a
                for j_a = 1:ave_Map.ST(m).P(1).J
                    if (PriorFusedMap.ST(m).P(1).x(j_p,:)-ave_Map.ST(m).P(1).x(j_a,:))*inv(PriorFusedMap.ST(m).P(1).P(:,:,j_p))*(PriorFusedMap.ST(m).P(1).x(j_p,:)-ave_Map.ST(m).P(1).x(j_a,:))' <= para.pruning_U
                        % condition iii)
                        Check_all_ja(j_a,j_p) = 0;
                        beta_0(j_p,1) = 1/2; beta_i(j_a,1) = 1/2;
                    else
                        Check_all_ja(j_a,j_p) = 1;
                    end
                end
            end
            
            for j_a = 1:ave_Map.ST(m).P(1).J % j_a -> j_p
                for j_p = 1:PriorFusedMap.ST(m).P(1).J
                    if (PriorFusedMap.ST(m).P(1).x(j_p,:)-ave_Map.ST(m).P(1).x(j_a,:))*inv(ave_Map.ST(m).P(1).P(:,:,j_a))*(PriorFusedMap.ST(m).P(1).x(j_p,:)-ave_Map.ST(m).P(1).x(j_a,:))' <= para.pruning_U
                        % condition iv)
                        Check_all_jp(j_p,j_a) = 0;
                        beta_0(j_p,1) = 1/2; beta_i(j_a,1) = 1/2;
                    else
                        Check_all_jp(j_p,j_a) = 1;
                    end
                end
            end
            for j_p = 1:PriorFusedMap.ST(m).P(1).J % j_p -> j_a,  condition i) or v)  
                if sum(Check_all_ja(:,j_p)) == ave_Map.ST(m).P(1).J
                    if Ind_FoV(j_p,1) == 1 % condition v)  <j_p is within FoV>
                        beta_0(j_p,1) = 1/2;
                    else % condition i)   <j_p is out of FoV>
                        beta_0(j_p,1) = 1;
                    end
                end
            end
            for j_a = 1:ave_Map.ST(m).P(1).J % j_a -> j_p, condition ii)
                if sum(Check_all_jp(:,j_a)) == PriorFusedMap.ST(m).P(1).J
                    beta_i(j_a,1) = 1;
                end
            end
            
            Map.ST(m).P(1).weight = [beta_0.*PriorFusedMap.ST(m).P(1).weight; beta_i.*ave_Map.ST(m).P(1).weight];
            Map.ST(m).P(1).m_k = sum(Map.ST(m).P(1).weight);
            Map.ST(m).P(1).J = PriorFusedMap.ST(m).P(1).J+ ave_Map.ST(m).P(1).J;
        else % map fusion rule 2
            Map.ST(m).P(1) = ave_Map.ST(m).P(1);
        end
    end
    % Pruning and merging for the GM PHD
    for m = 2:3
        [Map.ST(m).P(1).x, Map.ST(m).P(1).P, Map.ST(m).P(1).weight, Map.ST(m).P(1).J] = ULMapPrun(Map.ST(m).P(1),para); % v1 normal pruning and merging
        Map.ST(m).P(1).OriginatedMea = zeros(Map.ST(m).P(1).J,1);
    end
    
    % Map PHD calculation
    Temp = []; % fused map PHD
    Map_PHD_mode = 2; % 1: testing, 2: Average map
    [map.PHD]  = Map_PHD_calculation(para, Map, UE, Temp,Map_PHD_mode);
    map.PHD(4).D = map.PHD(1).D + map.PHD(2).D + map.PHD(3).D;
    
    Temp = []; % updated map PHD
    Map_PHD_mode = 2; % 1: testing, 2: Average map
    [update_map.PHD]  = Map_PHD_calculation(para, ave_Map, UE, Temp,Map_PHD_mode);
    update_map.PHD(4).D = update_map.PHD(1).D + update_map.PHD(2).D + update_map.PHD(3).D;
    
    Temp = []; % prior map PHD
    Map_PHD_mode = 2; % 1: testing, 2: Average map
    [Prior_map.PHD]  = Map_PHD_calculation(para, PriorFusedMap, UE, Temp,Map_PHD_mode);
    Prior_map.PHD(4).D = Prior_map.PHD(1).D + Prior_map.PHD(2).D + Prior_map.PHD(3).D;
    
    %------------------------------- figure -------------------------------%
    if (fig_mode == 1)  
        figure
        hold on; box on; grid on;
        title(sprintf('Prior map PHD at time %d',ti)); xlabel('x-axis [m]'); ylabel('y-axis [m]');
        view(-32,63)
        mesh(para.X,para.Y,Prior_map.PHD(4).D)
        p1 = plot3(state(1,1),state(2,1),1,'ko');
        p2 = plot3(BS.pos(1),BS.pos(2),1,'ro');
        for s = 1:numel(VA)
            p3=plot3(VA(s).pos(1),VA(s).pos(2),1,'rs');
        end
        for s = 1:numel(SP)
            if norm(SP(s).pos-state(1:3)) > para.SPVisibilityRadius
                p4=plot3(SP(s).pos(1),SP(s).pos(2),1,'bs');
            else
                p4=plot3(SP(s).pos(1),SP(s).pos(2),1,'rs');
            end
        end
        legend([p1,p2,p3,p4],'Vehicle', 'BS', 'VA', 'SP')
        
        figure
        hold on; box on; grid on;
        title(sprintf('(Vehicle %d) updated map PHD at time %d',v,ti)); xlabel('x-axis [m]'); ylabel('y-axis [m]');
        view(-32,63)
        mesh(para.X,para.Y,update_map.PHD(4).D)
        p1 = plot3(state(1,1),state(2,1),1,'ko');
        p2 = plot3(BS.pos(1),BS.pos(2),1,'ro');
        for s = 1:numel(VA)
            p3=plot3(VA(s).pos(1),VA(s).pos(2),1,'rs');
        end
        for s = 1:numel(SP)
            if norm(SP(s).pos-state(1:3)) > para.SPVisibilityRadius
                p4=plot3(SP(s).pos(1),SP(s).pos(2),1,'bs');
            else
                p4=plot3(SP(s).pos(1),SP(s).pos(2),1,'rs');
            end
        end
        legend([p1,p2,p3,p4],'Vehicle', 'BS', 'VA', 'SP')
        
        figure
        hold on; box on; grid on;
        title(sprintf('Fused map PHD at time %d',ti)); xlabel('x-axis [m]'); ylabel('y-axis [m]');
        view(-32,63)
        mesh(para.X,para.Y,map.PHD(4).D)
        p1 = plot3(state(1,1),state(2,1),1,'ko');
        p2 = plot3(BS.pos(1),BS.pos(2),1,'ro');
        for s = 1:numel(VA)
            p3=plot3(VA(s).pos(1),VA(s).pos(2),1,'rs');
        end
        for s = 1:numel(SP)
            if norm(SP(s).pos-state(1:3)) > para.SPVisibilityRadius
                p4=plot3(SP(s).pos(1),SP(s).pos(2),1,'bs');
            else
                p4=plot3(SP(s).pos(1),SP(s).pos(2),1,'rs');
            end
        end
        legend([p1,p2,p3,p4],'Vehicle', 'BS', 'VA', 'SP')
    end
    close all;
end