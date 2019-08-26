function [ave_Map, map]  = AveMap(fig_mode, UE, previous_Map, para, state, V_Est, BS, VA, SP, v, ti)
    % 5G mmWave Positioning and Mapping
    % (c) Hyowon Kim, 2019 (Ph.D. student at Hanyang Univerisy, Seoul, South Korea, emai: khw870511@hanyang.ac.kr)
    % Usage: this code generates the averaged map
    
    if para.onlyLOS == 1
        m_type = 1;
    else
        m_type = 1:3;
    end
    %% Merging the map for the GM PHD
    map.PHD = [];
    Map.ST(1).P(1) = previous_Map.ST(1).P(1);
    if para.onlyLOS ~= 1
        for m = 2:3
            [Map.ST(m).P(1).x, Map.ST(m).P(1).P, Map.ST(m).P(1).weight, Map.ST(m).P(1).J] = AveMapPrun(previous_Map.ST(m),UE,para); % v1 normal pruning and merging
            Map.ST(m).P(1).m_k = sum(Map.ST(m).P(1).weight);
            Map.ST(m).P(1).OriginatedMea = zeros(numel(Map.ST(m).P(1).weight),1);
        end
%         % i) In the VA map, the intensity near the BS could be eliminated, because we know there are no VA near the BS.
%         index_j = [];
%         for j =  1:Map.ST(2).P(1).J
%             if norm(Map.ST(2).P(1).x(j,:)' - BS.pos) <= 10
%                 index_j = [index_j; j];
%             end
%         end
%         if numel(index_j) > 0
%             Map.ST(2).P(1).x(index_j,:) = [];
%             Map.ST(2).P(1).P(:,:,index_j) = [];
%             Map.ST(2).P(1).weight(index_j) = [];
%             Map.ST(2).P(1).J = numel(Map.ST(2).P(1).weight);
%             Map.ST(2).P(1).m_k = sum(Map.ST(2).P(1).weight);
%         end
    end
    
    for m = m_type
        ave_Map.ST(m).P(1:para.N_p) = Map.ST(m).P(1);
    end
    
    %% Map PHD calculation
    Temp = [];
    Map_PHD_mode = 2; % 1: testing, 2: Average map
    [map.PHD]  = Map_PHD_calculation(para, Map, UE, Temp, Map_PHD_mode);
    if para.onlyLOS == 1
        map.PHD(4).D = map.PHD(1).D;
    else
        map.PHD(4).D = map.PHD(1).D + map.PHD(2).D + map.PHD(3).D;
    end
    
    %------------------------------- figure -------------------------------%
    %% Figure
    if (fig_mode == 1)
        figure
        hold on; box on; grid on;
        title(sprintf('(Vehicle %d) update of the map PHD (VA) at time %d',v,ti)); xlabel('x-axis [m]'); ylabel('y-axis [m]');
        view(-45,60)
        mesh(para.X,para.Y,map.PHD(2).D)
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
        p5 = plot3(V_Est(1), V_Est(2), 1, 'kx');
        legend([p1,p2,p3,p4,p5],'Vehicle', 'BS', 'VA', 'SP','Vehicle estimation')
        
        figure
        hold on; box on; grid on;
        title(sprintf('(Vehicle %d) update of the map PHD (SP) at time %d',v,ti)); xlabel('x-axis [m]'); ylabel('y-axis [m]');
        view(-45,60)
        mesh(para.X,para.Y,map.PHD(3).D)
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
        p5 = plot3(V_Est(1), V_Est(2), 1, 'kx');
        legend([p1,p2,p3,p4,p5],'Vehicle', 'BS', 'VA', 'SP','Vehicle estimation')
    end
    close all;
end