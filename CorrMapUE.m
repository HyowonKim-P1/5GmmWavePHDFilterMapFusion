function [Map, UE, V_Est]  = CorrMapUE(fig_mode,prediction_UE,prediction_Map,Birth,para,state,Channel,BS,VA,SP,measurement,Clutter,R,v,ti,MapFusionMode, mc)
    % 5G mmWave Positioning and Mapping
    % (c) Hyowon Kim, 2019 (Ph.D. student at Hanyang Univerisy, Seoul, South Korea, emai: khw870511@hanyang.ac.kr)
    % Usage: this code generates the updated vehicle state and map
    
    UE = prediction_UE;
    if para.onlyLOS == 1
        m_type = 1;
        N_Z = 1;
    else
        m_type = 1:3;
        N_Z = sum(Channel) + Clutter.visible;
    end
    
    %% Detection probability calculation
    for i = 1:para.N_p
        P(i).ST = DetectProb(prediction_Map, Birth, UE.state(:,i), i, para);
    end
    
    %% Map Correction
    KF(1).P(1).temp=[];
    for i = 1:para.N_p
        % Step1: Compute PHD update components by CKF
        UE_147 = UE.state([1:4 7],i);
        for m = m_type
            Nu(m).P(i).likelihood = zeros(N_Z,prediction_Map.ST(m).P(i).J);
            Nu(m).P(i).tau = zeros(N_Z,prediction_Map.ST(m).P(i).J);
            Nu(m).P(i).sum = 0;
            for j = 1:prediction_Map.ST(m).P(i).J
                [KF(m).P(i).P(:,:,j), KF(m).P(i).z_hat(:,j), KF(m).P(i).K(:,:,j), KF(m).P(i).S(:,:,j)] =  UpCKF(prediction_Map.ST(m).P(i).P(:,:,j), prediction_Map.ST(m).P(i).x(j,:), BS, UE_147, R, m, para);
            end
            if para.onlyLOS ~=1
                l = 0;
                for z = 1:N_Z
                    l = l+1;
                    for j = 1:prediction_Map.ST(m).P(i).J
                        if j >= prediction_Map.ST(m).P(i).J-Birth.ST(m).P(i).J+1 && prediction_Map.ST(m).P(i).OriginatedMea(j,1) == z % if a birth with corresponding measurement
                            Nu(m).P(i).tau(z,j) = prediction_Map.ST(m).P(i).weight(j,1);
                        else % i) if a birth, but a different measurement or persistent or ii) persistent
                            measurement(z,:) = F_MeaCali(measurement(z,:));
                            measurement(z,:) = F_InovCali(measurement(z,:),KF(m).P(i).z_hat(:,j));
                            Nu(m).P(i).likelihood(z,j) = F_PredLikeli(measurement(z,:)', KF(m).P(i).z_hat(:,j), KF(m).P(i).S(:,:,j));
                            Nu(m).P(i).tau(z,j) = P(i).ST(m).detection(j,1)*prediction_Map.ST(m).P(i).weight(j,1)*Nu(m).P(i).likelihood(z,j);
                        end
                    end
                    if prediction_Map.ST(m).P(i).J > 0
                        Nu(m).P(i).sum(z) = sum(Nu(m).P(i).tau(z,:));
                    else
                        Nu(m).P(i).sum(z) = 0;
                    end
                end
            end
        end
        if para.onlyLOS ~= 1
        % Step2: PHD update
            for m = 2:3
                for j = 1:prediction_Map.ST(m).P(i).J % missed detections
                    Map.ST(m).P(i).weight(j,1) = (1-P(i).ST(m).detection(j,1))*prediction_Map.ST(m).P(i).weight(j,1);
                    Map.ST(m).P(i).x(j,:) = prediction_Map.ST(m).P(i).x(j,:);
                    Map.ST(m).P(i).P(:,:,j) = prediction_Map.ST(m).P(i).P(:,:,j);
                end
                l = 0;
                for z = 1:N_Z % detections
                    l = l+1;
                    for j = 1:prediction_Map.ST(m).P(i).J
                        if j >= prediction_Map.ST(m).P(i).J-Birth.ST(m).P(i).J+1 && prediction_Map.ST(m).P(i).OriginatedMea(j,1) == z % if a birth with corresponding measurement
                            Map.ST(m).P(i).x(l*prediction_Map.ST(m).P(i).J+j,:) = prediction_Map.ST(m).P(i).x(j,:);
                            Map.ST(m).P(i).P(:,:,l*prediction_Map.ST(m).P(i).J+j) = prediction_Map.ST(m).P(i).P(:,:,j);
                        else % i) if a birth, but a different measurement or persistent or ii) persistent
                            measurement(z,:) = F_MeaCali(measurement(z,:));
                            measurement(z,:) = F_InovCali(measurement(z,:),KF(m).P(i).z_hat(:,j));
                            Map.ST(m).P(i).x(l*prediction_Map.ST(m).P(i).J+j,:) = Map.ST(m).P(i).x(j,:) + (KF(m).P(i).K(:,:,j)*(measurement(z,:)' - KF(m).P(i).z_hat(:,j)))';
                            Map.ST(m).P(i).P(:,:,l*prediction_Map.ST(m).P(i).J+j) = KF(m).P(i).P(:,:,j);
                        end
                        Map.ST(m).P(i).weight(l*prediction_Map.ST(m).P(i).J+j,1) = Nu(m).P(i).tau(z,j)/(para.c_z + Nu(1).P(i).sum(z) + Nu(2).P(i).sum(z)+ Nu(3).P(i).sum(z));
                    end
                end
                Map.ST(m).P(i).J= (1+l)*prediction_Map.ST(m).P(i).J;
                Map.ST(m).P(i).m_k = sum(Map.ST(m).P(i).weight);
            end
        end
        Map.ST(1).P(i).weight = prediction_Map.ST(1).P(i).weight;
        Map.ST(1).P(i).x = prediction_Map.ST(1).P(i).x;
        Map.ST(1).P(i).P = prediction_Map.ST(1).P(i).P;
        Map.ST(1).P(i).J = prediction_Map.ST(1).P(i).J;
        Map.ST(1).P(i).m_k = prediction_Map.ST(1).P(i).m_k;
    end
    Before = Map;
    
    %% Vehicle state correction (weight calculation with log weight)
    Size_Z = N_Z;
    Log_likelihood = zeros(Size_Z,para.N_p);
    for i = 1:para.N_p
        for m = m_type
            Nu(m).log_likelihood = zeros(Size_Z,prediction_Map.ST(m).P(i).J);
        end
        for z = 1:N_Z
            %%    log ( c(z) + [sum_{for # Gaussians and source type} {p_d * eta_k|k-1 * p(z|h(x),S)}] ) -> log_liklihood = log ( q^1 + sum_{for # Gaussians and source type} q^f )>, where q^1> ... q^F = log ( q^1 ) + log [ 1+ log {1 + sum_f=2^F exp( log(q^f) - log(q^1) ) } ] -> log_sum_w
            likelihood(i).Z(z).log = log(para.c_z);
            for m = m_type
                for j = 1:prediction_Map.ST(m).P(i).J
                    if j >= prediction_Map.ST(m).P(i).J-Birth.ST(m).P(i).J+1 && prediction_Map.ST(m).P(i).OriginatedMea(j,1) == z % if a birth with corresponding measurement
                        likelihood(i).Z(z).log = [likelihood(i).Z(z).log; log(prediction_Map.ST(m).P(i).weight(j,1))];
                    else % i) if a birth, but a different measurement or persistent or ii) persistent
                        measurement(z,:) = F_MeaCali(measurement(z,:));
                        measurement(z,:) = F_InovCali(measurement(z,:),KF(m).P(i).z_hat(:,j));
                        temp_inov = abs(measurement(z,:)' - KF(m).P(i).z_hat(:,j));
                        if temp_inov(2) > pi || temp_inov(3) > pi || temp_inov(4) > pi || temp_inov(5) > pi
                            sprintf('inoovation error')
                            temp_inov;
                        end
                        Nu(m).log_likelihood(z,j) = F_PredLogLikeli(measurement(z,:)', KF(m).P(i).z_hat(:,j), KF(m).P(i).S(:,:,j));
                        likelihood(i).Z(z).log = [likelihood(i).Z(z).log; log(P(i).ST(m).detection(j,1)) + log(prediction_Map.ST(m).P(i).weight(j,1)) + Nu(m).log_likelihood(z,j)'];
                    end
                end
            end
            log_likeli = maxstar(likelihood(i).Z(z).log,[],1); % log(sum_{i=1}^I x_i) can be calculated by a maxstar function as a input: log(x_i) for all i
            Log_likelihood(z,i) = log_likeli;
        end
        Log_weight(i,1) = log(prediction_UE.weight(1,i)) + sum(Log_likelihood(:,i)); % log(w_k|k) = log(w_k|k-1) + sum_z=1^Z log( c(z) + A(z) )
    end
    LogSumW = maxstar(Log_weight,[],1);
    NormalizedLogWeight = Log_weight - LogSumW; %normalized log weight
    UE.weight = (exp(NormalizedLogWeight))'; % w_k|k
    [maxV, maxWeightIndex] = max(UE.weight);
    upperHeadingIndex = find(UE.state(4,maxWeightIndex) + pi < UE.state(4,:));
    lowerHeadingIndex = find(UE.state(4,maxWeightIndex) - pi >UE.state(4,:));
    UE.state(4,upperHeadingIndex) = UE.state(4,upperHeadingIndex) - 2*pi;
    UE.state(4,lowerHeadingIndex) = UE.state(4,lowerHeadingIndex) + 2*pi;
    V_Est = sum(UE.state.*UE.weight,2);
    V_Est(4) = mod(V_Est(4),2*pi);
    if abs(V_Est(4)-state(4)) >= 1.5*pi
        V_Est(4) = abs(V_Est(4) - 2*pi);
    end
    if para.error_check == 1
        V_Est-state
        if abs(V_Est(4)-state(4)) >= 1.5*pi
            sprintf('heading period error')
        end
    end
    
    %% Pruning and merging for each particle (is added for reducing the complexity in determining average PHD, but trucating is performed in this function)
    if para.onlyLOS ~= 1
        for i = 1:para.N_p
            for m = 2:3
                [Map.ST(m).P(i).x, Map.ST(m).P(i).P, Map.ST(m).P(i).weight, Map.ST(m).P(i).J] = CorrMapPrun(Map.ST(m).P(i),para); % normal pruning and merging
                Map.ST(m).P(i).OriginatedMea = zeros(Map.ST(m).P(i).J,1);
            end
        end
    end
    
    %------------------------------- figure -------------------------------%
    if para.particle_PDF == 1
        if MapFusionMode == 1
            FolderN = 'ParticleFigure/NoFusion';
        elseif MapFusionMode == 2
            FolderN = 'ParticleFigure/UplinkFusion';
        elseif MapFusionMode == 3
            FolderN = 'ParticleFigure/UpDownlinkFusion';
        end
        
        figure
        hold on; box on; grid on;
        hist(Log_likelihood')
        xlabel('Log-likelihood')
        FN = ['Hist_P_' num2str(para.N_p) '_T' num2str(ti) '_V' num2str(v) '_FM' num2str(MapFusionMode) '_MC' num2str(mc)];
        FigureSave(FolderN,FN)
        close all;
        
        K_weight = 1/sum(UE.weight.^2);
        figure
        hold on; box on; grid on;
        plot(0,K_weight, 'bo')
        plot(0,para.N_p/10, 'ro')
        FN = ['EP_P_' num2str(para.N_p) '_T' num2str(ti) '_V' num2str(v) '_FM' num2str(MapFusionMode) '_MC' num2str(mc)];
        FigureSave(FolderN,FN)
        if K_weight < 150
            sprintf('Particle effectiveness error occur')
            K_weight;
        end
        close all;
        
        figure
        hold on; box on; grid on;
        plot(UE.state(7,:),UE.weight(:),'kx')
        plot(state(7), 0, 'bo')
        plot(V_Est(7), 0 ,'rx')
        xlabel('Bias')
        ylabel('Weight')
        FN = ['Bias_P_' num2str(para.N_p) '_T' num2str(ti) '_V' num2str(v) '_FM' num2str(MapFusionMode) '_MC' num2str(mc)];
        FigureSave(FolderN,FN)
        close all;
        
        figure
        hold on; box on; grid on;
        plot(UE.state(4,:),UE.weight(:),'kx')
        plot(state(4), 0 ,'bo')
        plot(V_Est(4), 0 ,'rx')
        xlabel('Heading')
        ylabel('Weight')
        FN = ['Heading_P_' num2str(para.N_p) '_T' num2str(ti) '_V' num2str(v) '_FM' num2str(MapFusionMode) '_MC' num2str(mc)];
        FigureSave(FolderN,FN)
        close all;
        
        figure
        hold on; box on; grid on;
        plot(UE.state(1,:),UE.weight(:),'kx')
        plot(state(1), 0 ,'bo')
        plot(V_Est(1), 0 ,'rx')
        xlabel('x-location')
        ylabel('Weight')
        FN = ['X_P_' num2str(para.N_p) '_T' num2str(ti) '_V' num2str(v) '_FM' num2str(MapFusionMode) '_MC' num2str(mc)];
        FigureSave(FolderN,FN)
        close all;
        
        figure
        hold on; box on; grid on;
        plot(UE.state(2,:),UE.weight(:),'kx')
        plot(state(2), 0 ,'bo')
        plot(V_Est(2), 0 ,'rx')
        xlabel('y-location')
        ylabel('Weight')
        FN = ['Y_P_' num2str(para.N_p) '_T' num2str(ti) '_V' num2str(v) '_FM' num2str(MapFusionMode) '_MC' num2str(mc)];
        FigureSave(FolderN,FN)
        close all;
        
        figure
        hold on; box on; grid on;
        pointsize = 2;
        scatter(UE.state(1,:),UE.state(2,:),pointsize,UE.weight,'filled')
        colormap('jet');
        p0 = plot(BS.pos(1), BS.pos(2), 'k^');
        p1 = plot(state(1),state(2),'ko');
        p2 = plot(V_Est(1),V_Est(2),'rx');
        for va = 1:para.N_VA
            p3 = plot(VA(va).pos(1), VA(va).pos(2), 'r^');
        end
        for sp = 1:para.N_SP
            p4 = plot(SP(sp).pos(1), SP(sp).pos(2), 'rs');
        end
        legend([p0,p1,p2,p3,p4],'BS','Vehicle','Estimated vehicle','VA','SP')
        FN = ['VLoc_P_' num2str(para.N_p) '_T' num2str(ti) '_V' num2str(v) '_FM' num2str(MapFusionMode) '_MC' num2str(mc)];
        FigureSave(FolderN,FN)
        close all;
    end
    
    % Figure
    if (fig_mode == 1)
        % Map PHD calculation
        Map_PHD_mode = 1; % 1: testing, 2: Average map
        [map.PHD]  = Map_PHD_calculation(para, Map, UE, Nu,Map_PHD_mode);
        map.PHD(4).D = map.PHD(1).D + map.PHD(2).D + map.PHD(3).D;

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
        legend([p1,p2,p3,p4],'Vehicle', 'BS', 'VA', 'SP')
        
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
        legend([p1,p2,p3,p4],'Vehicle', 'BS', 'VA', 'SP')    
    end
    close all;
end