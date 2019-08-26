function PerformMC(para, MapFusionMode, Perform)
    % 5G mmWave Positioning and Mapping
    % (c) Hyowon Kim, 2019 (Ph.D. student at Hanyang Univerisy, Seoul, South Korea, emai: khw870511@hanyang.ac.kr)
    % Usage: this code generates averaged performances over Monte Carlo run

    
    if MapFusionMode == 1
        FolderN = 'Performance/NoFusion';
    elseif MapFusionMode ==2
        FolderN = 'Performance/UplinkFusion';
    elseif MapFusionMode == 3
        FolderN = 'Performance/UpDownlinkFusion';
    elseif MapFusionMode == 0
        FolderN = 'Performance/NoUpdate';
    end

    %% vehicle location
    VL_error = zeros(para.TIME,para.MC,para.N_vehicle);
    VL_max = zeros(para.TIME,para.N_vehicle);
    VL_min = zeros(para.TIME,para.N_vehicle);
    VL_mean = zeros(para.TIME,para.N_vehicle);
    VL_std = zeros(para.TIME,para.N_vehicle);
    for mc = 1:para.MC
        for v = 1:para.N_vehicle
            VL_error(:,mc,v) = Perform(mc).VecLoc(:,v);
        end
    end
    for ti = 1:para.TIME
        for v = 1:para.N_vehicle
            VL_max(ti,v) = max(VL_error(ti,:,v));
            VL_min(ti,v) = min(VL_error(ti,:,v));
            VL_mean(ti,v) = mean(VL_error(ti,:,v));
            VL_std(ti,v) = std(VL_error(ti,:,v));
        end
    end
    figure
    hold on; grid on; box on;
    if para.N_vehicle == 1
        p1 = errorbar(1:para.TIME, (VL_max(:,1)' + VL_min(:,1)')/2, (VL_max(:,1)' - VL_min(:,1)')/2,'bx-','LineWidth',1);
        leg1 = legend(p1,'Vechiel 1');
    elseif para.N_vehicle == 2
        p1 = errorbar(1:para.TIME, (VL_max(:,1)' + VL_min(:,1)')/2, (VL_max(:,1)' - VL_min(:,1)')/2,'bx-','LineWidth',1);
        p2 = errorbar(1:para.TIME, (VL_max(:,2)' + VL_min(:,2)')/2, (VL_max(:,2)' - VL_min(:,2)')/2,'ro-','LineWidth',1);
        leg1 = legend([p1,p2],'Vechiel 1', 'Vehicle 2');
    end

    xlabel('time $k$','Interpreter','latex');
    ylabel('vehicle localization (RMSE) [m]', 'Interpreter','latex');
    set(leg1, 'Interpreter','latex');

    FN = ['AVE_VLoc_P_' num2str(para.N_p) '_T' num2str(para.TIME) '_FM' num2str(MapFusionMode) '_MC' num2str(para.MC)];
    FigureSave(FolderN,FN)

    % bias
    Bias_error = zeros(para.TIME,para.MC,para.N_vehicle);
    Bias_max = zeros(para.TIME,para.N_vehicle);
    Bias_min = zeros(para.TIME,para.N_vehicle);
    Bias_mean = zeros(para.TIME,para.N_vehicle);
    Bias_std = zeros(para.TIME,para.N_vehicle);
    for mc = 1:para.MC
        for v = 1:para.N_vehicle
            Bias_error(:,mc,v) = Perform(mc).Bias(:,v);
        end
    end
    for ti = 1:para.TIME
        for v = 1:para.N_vehicle
            Bias_max(ti,v) = max(Bias_error(ti,:,v));
            Bias_min(ti,v) = min(Bias_error(ti,:,v));
            Bias_mean(ti,v) = mean(Bias_error(ti,:,v));
            Bias_std(ti,v) = std(Bias_error(ti,:,v));
        end
    end
    
    figure
    hold on; grid on; box on;
    if para.N_vehicle == 1
        p1 = errorbar(1:para.TIME, (Bias_max(:,1)' + Bias_min(:,1)')/2, (Bias_max(:,1)' - Bias_min(:,1)')/2,'bx-','LineWidth',1);
        leg1 = legend(p1,'Vechiel 1');
    elseif para.N_vehicle == 2
        p1 = errorbar(1:para.TIME, (Bias_max(:,1)' + Bias_min(:,1)')/2, (Bias_max(:,1)' - Bias_min(:,1)')/2,'bx-','LineWidth',1);
        p2 = errorbar(1:para.TIME, (Bias_max(:,2)' + Bias_min(:,2)')/2, (Bias_max(:,2)' - Bias_min(:,2)')/2,'ro-','LineWidth',1);
        leg1 = legend([p1,p2],'Vechiel 1', 'Vehicle 2');
    end
    xlabel('time $k$' ,'Interpreter','latex');
    ylabel('bias (RMSE) [m]' ,'Interpreter','latex');
    set(leg1, 'Interpreter','latex');

    FN = ['AVE_Bias_P_' num2str(para.N_p) '_T' num2str(para.TIME) '_FM' num2str(MapFusionMode) '_MC' num2str(para.MC)];
    FigureSave(FolderN,FN)

    % heading
    Heading_error = zeros(para.TIME,para.MC,para.N_vehicle);
    Heading_max = zeros(para.TIME,para.N_vehicle);
    Heading_min = zeros(para.TIME,para.N_vehicle);
    Heading_mean = zeros(para.TIME,para.N_vehicle);
    Heading_std = zeros(para.TIME,para.N_vehicle);
    for mc = 1:para.MC
        for v = 1:para.N_vehicle
            Heading_error(:,mc,v) = Perform(mc).Heading(:,v);
        end
    end
    for ti = 1:para.TIME
        for v = 1:para.N_vehicle
            Heading_max(ti,v) = max(Heading_error(ti,:,v));
            Heading_min(ti,v) = min(Heading_error(ti,:,v));
            Heading_mean(ti,v) = mean(Heading_error(ti,:,v));
            Heading_std(ti,v) = std(Heading_error(ti,:,v));
        end
    end
    
    figure
    hold on; grid on; box on;
    if para.N_vehicle == 1
        p1 = errorbar(1:para.TIME, (Heading_max(:,1)' + Heading_min(:,1)')/2, (Heading_max(:,1)' - Heading_min(:,1)')/2,'bx-','LineWidth',1);
        leg1 = legend(p1,'Vechiel 1');
    elseif para.N_vehicle == 2
        p1 = errorbar(1:para.TIME, (Heading_max(:,1)' + Heading_min(:,1)')/2, (Heading_max(:,1)' - Heading_min(:,1)')/2,'bx-','LineWidth',1);
        p2 = errorbar(1:para.TIME, (Heading_max(:,2)' + Heading_min(:,2)')/2, (Heading_max(:,2)' - Heading_min(:,2)')/2,'ro-','LineWidth',1);
        leg1 = legend([p1,p2],'Vechiel 1', 'Vehicle 2');
    end
    xlabel('time $k$' ,'Interpreter','latex');
    ylabel('heading (RMSE)' ,'Interpreter','latex');
    set(leg1,'Interpreter','latex');

    FN = ['AVE_Heading_P_' num2str(para.N_p) '_T' num2str(para.TIME) '_FM' num2str(MapFusionMode) '_MC' num2str(para.MC)];
    FigureSave(FolderN,FN)

    if para.onlyLOS ~= 1
        %% Mapping
        % Vehicle map
            % VA
            GOSPA_VA_evaluate = zeros(para.TIME,para.N_vehicle);
            for mc = 1:para.MC
                GOSPA_VA_evaluate = GOSPA_VA_evaluate + Perform(mc).GOSPA_VA;
            end
            GOSPA_VA_evaluate = GOSPA_VA_evaluate/para.MC;

            % SP
            GOSPA_SP_evaluate = zeros(para.TIME,para.N_vehicle);
            for mc = 1:para.MC
                GOSPA_SP_evaluate = GOSPA_SP_evaluate + Perform(mc).GOSPA_SP;
            end
            GOSPA_SP_evaluate = GOSPA_SP_evaluate/para.MC;


        if MapFusionMode == 2 || MapFusionMode == 3
        % Up-link map
            % VA
            UL_GOSPA_VA_evaluate = zeros(para.TIME,para.N_vehicle);
            for mc = 1:para.MC
                UL_GOSPA_VA_evaluate = UL_GOSPA_VA_evaluate + Perform(mc).UL_GOSPA_VA;
            end
            UL_GOSPA_VA_evaluate = UL_GOSPA_VA_evaluate/para.MC;

            UL_GOSPA_SP_evaluate = zeros(para.TIME,para.N_vehicle);
            for mc = 1:para.MC
                UL_GOSPA_SP_evaluate = UL_GOSPA_SP_evaluate + Perform(mc).UL_GOSPA_SP;
            end
            UL_GOSPA_SP_evaluate = UL_GOSPA_SP_evaluate/para.MC;
        end

        figure
        hold on; grid on; box on;
        if para.N_vehicle == 1
            p1 = plot(1:para.TIME,GOSPA_VA_evaluate(:,1),'bx-');
        else
            p1 = plot(1:para.TIME,GOSPA_VA_evaluate(:,1),'bx-');
            p2 = plot(1:para.TIME,GOSPA_VA_evaluate(:,2),'ro-');
        end
        if MapFusionMode == 2 || MapFusionMode == 3
            p3 = plot(10:2:para.TIME,UL_GOSPA_VA_evaluate(10:2:para.TIME,1),'k^-');
            leg1 = legend([p1,p2,p3],'Vechiel 1', 'Vehicle 2', 'BS');
        else
            if para.N_vehicle == 1
                leg1 = legend(p1,'Vechiel 1');
            else
                leg1 = legend([p1,p2],'Vechiel 1', 'Vehicle 2');
            end
        end
        xlabel('time $k$' ,'Interpreter','latex');
        ylabel('average GOSPA of the VA' ,'Interpreter','latex');
        set(leg1,'Interpreter','latex');

        FN = ['AVE_VA_P_' num2str(para.N_p) '_T' num2str(para.TIME) '_FM' num2str(MapFusionMode) '_MC' num2str(para.MC)];
        FigureSave(FolderN,FN)

        figure
        hold on; grid on; box on;
        if para.N_vehicle == 1
            p1 = plot(1:para.TIME,GOSPA_SP_evaluate(:,1),'bx-');
        else
            p1 = plot(1:para.TIME,GOSPA_SP_evaluate(:,1),'bx-');
            p2 = plot(1:para.TIME,GOSPA_SP_evaluate(:,2),'ro-');
        end
        if MapFusionMode == 2 || MapFusionMode == 3
            p3 = plot(10:2:para.TIME,UL_GOSPA_SP_evaluate(10:2:para.TIME,1),'k^-');
            leg1 = legend([p1,p2,p3],'Vechiel 1', 'Vehicle 2', 'BS');
        else
            if para.N_vehicle == 1
                leg1 = legend(p1,'Vechiel 1');
            else
                leg1 = legend([p1,p2],'Vechiel 1', 'Vehicle 2');
            end
        end
        xlabel('time $k$' ,'Interpreter','latex');
        ylabel('average GOSPA of the SP' ,'Interpreter','latex');
        set(leg1,'Interpreter','latex');

        FN = ['AVE_SP_P_' num2str(para.N_p) '_T' num2str(para.TIME) '_FM' num2str(MapFusionMode) '_MC' num2str(para.MC)];
        FigureSave(FolderN,FN)
    end

    % Effectiveness of the number of particles
    Eff = zeros(para.TIME,para.N_vehicle);
    for mc = 1:para.MC
        Eff = Eff + Perform(mc).Eff;
    end
    Eff = Eff/para.MC;
    
    figure
    hold on; grid on; box on;
    if para.N_vehicle == 1
        p1 = plot(1:para.TIME,Eff(:,1),'b-o');
        p3 = plot(1:para.TIME,para.N_p/10*ones(para.TIME,1),'k-^');
        leg1 = legend([p1,p3],'$N_\mathrm{eff}= 1/\sum_{i=1}^N\omega_i^2$','$N/10$');
    else
        p1 = plot(1:para.TIME,Eff(:,1),'b-o');
        p2 = plot(1:para.TIME,Eff(:,2),'r-x');
        p3 = plot(1:para.TIME,para.N_p/10*ones(para.TIME,1),'k-^');
        leg1 = legend([p1,p2,p3],'$N_\mathrm{eff}= 1/\sum_{i=1}^N\omega_i^2$','$N_\mathrm{eff}= 1/\sum_{i=1}^N\omega_i^2$','$N/10$');
    end
    set(leg1,'Interpreter','latex');
    xlabel('time $k$' ,'Interpreter','latex');
    ylabel('Effective number of particles' ,'Interpreter','latex');
    FN = ['AVE_Effectiveness_P_' num2str(para.N_p) '_T' num2str(para.TIME) '_FM' num2str(MapFusionMode) '_MC' num2str(para.MC)];
    FigureSave(FolderN,FN)
end