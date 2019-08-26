function Perform = PerformTime(VA, SP, para, MapFusionMode, state, Stack, mc)
    % 5G mmWave Positioning and Mapping
    % (c) Hyowon Kim, 2019 (Ph.D. student at Hanyang Univerisy, Seoul, South Korea, emai: khw870511@hanyang.ac.kr)
    % Usage: this code generates performances over time
    
    close all;
    if para.onlyLOS ~= 1
        for ti = 2:para.TIME
            for v = 1:para.N_vehicle
                for m = 2:3
                    for j = 1:numel(Stack(ti).ave_Map(v).ST(m).P(1).weight)
                        Stack(ti).ave_Map(v).ST(m).P(1).Tr(j) = trace(Stack(ti).ave_Map(v).ST(m).P(1).P(:,:,j));
                    end
                end
            end
        end
    end
    if MapFusionMode ~=1
        for ti = 10:2:para.TIME
            for m = 2:3
                for j = 1:numel(Stack(ti).UL_Map.ST(m).P.weight)
                    Stack(ti).UL_Map.ST(m).P.Tr(j) = trace(Stack(ti).UL_Map.ST(m).P.P(:,:,j));
                end
            end
        end
    end
    if MapFusionMode == 1
        FolderN = 'Performance/NoFusion';
    elseif MapFusionMode ==2
        FolderN = 'Performance/UplinkFusion';
    elseif MapFusionMode == 3
        FolderN = 'Performance/UpDownlinkFusion';
    end
    
    %% Vehicle state
    % vehicle location
    error_location = zeros(para.TIME,para.N_vehicle);
    for ti = 1:para.TIME
        for i = 1:para.N_vehicle
            error_location(ti,i) = norm(state(1:3,ti,i) -Stack(ti).V_Est(1:3,i));
        end
    end
    Perform.VecLoc = error_location;
    figure
    hold on; grid on; box on;
    if para.N_vehicle == 1
        p1 = plot(1:para.TIME,error_location(1:para.TIME,1),'bx-');
        leg1 = legend(p1,'Vechiel 1');
    elseif para.N_vehicle == 2
        p1 = plot(1:para.TIME,error_location(1:para.TIME,1),'bx-');
        p2 = plot(1:para.TIME,error_location(1:para.TIME,2),'ro-');
        leg1 = legend([p1,p2],'Vechiel 1', 'Vehicle 2');
    end

    xlabel('time $k$','Interpreter','latex');
    ylabel('vehicle localization (RMSE) [m]', 'Interpreter','latex');
    set(leg1, 'Interpreter','latex');

    FN = ['VLoc_P_' num2str(para.N_p) '_T' num2str(para.TIME) '_FM' num2str(MapFusionMode) '_mc' num2str(mc)];
    FigureSave(FolderN,FN)

    % clock bias
    error_bias = zeros(para.TIME,para.N_vehicle);
    for ti = 1:para.TIME
        for i = 1:para.N_vehicle
            error_bias(ti,i) = norm(state(7,ti,i) -Stack(ti).V_Est(7,i));
        end
    end
    Perform.Bias = error_bias;
    figure
    hold on; grid on; box on;
    if para.N_vehicle == 1
        p1 = plot(1:para.TIME,error_bias(1:para.TIME,1),'bx-');
        leg1 = legend(p1,'Vechiel 1');
    elseif para.N_vehicle == 2
        p1 = plot(1:para.TIME,error_bias(1:para.TIME,1),'bx-');
        p2 = plot(1:para.TIME,error_bias(1:para.TIME,2),'ro-');
        leg1 = legend([p1,p2],'Vechiel 1', 'Vehicle 2');
    end
    xlabel('time $k$' ,'Interpreter','latex');
    ylabel('bias (RMSE) [m]' ,'Interpreter','latex');
    set(leg1, 'Interpreter','latex');

    FN = ['Bias_P_' num2str(para.N_p) '_T' num2str(para.TIME) '_FM' num2str(MapFusionMode) '_mc' num2str(mc)];
    FigureSave(FolderN,FN)

    % vehicle heading
    error_heading = zeros(para.TIME,para.N_vehicle);
    for ti = 1:para.TIME
        for i = 1:para.N_vehicle
            heading_calibration = abs(state(4,ti,i)-Stack(ti).V_Est(4,i));
            if heading_calibration > pi
                heading_calibration = abs(heading_calibration - 2*pi);
            end
            error_heading(ti,i) = heading_calibration;
        end
    end
    Perform.Heading = error_heading;
    figure
    hold on; grid on; box on;
    if para.N_vehicle == 1
        p1 = plot(1:para.TIME,error_heading(1:para.TIME,1),'bx-');
        leg1 = legend(p1,'Vechiel 1');
    elseif para.N_vehicle == 2
        p1 = plot(1:para.TIME,error_heading(1:para.TIME,1),'bx-');
        p2 = plot(1:para.TIME,error_heading(1:para.TIME,2),'ro-');
        leg1 = legend([p1,p2],'Vechiel 1', 'Vehicle 2');
    end
    xlabel('time $k$' ,'Interpreter','latex');
    ylabel('heading (RMSE)' ,'Interpreter','latex');
    set(leg1,'Interpreter','latex');

    FN = ['Heading_P_' num2str(para.N_p) '_T' num2str(para.TIME) '_FM' num2str(MapFusionMode) '_mc' num2str(mc)];
    FigureSave(FolderN,FN)

    if para.onlyLOS ~= 1
        %% Mapping
        % with GOSPA function [d_gospa, x_to_y_assignment, decomposed_cost] = GOSPA(x_mat, y_mat, p, c, alpha);
        % Localization error + c/2(#missing + #flase)
        % x_mat: estimated target set (dimension of location X # estimated target)
        % y_mat: true target set (dimension of location X # true target)
        GOSPA_p = 2;
        GOSPA_c = 20;
        GOSPA_alpha = 2;

        % Vehicle map
            % VA
            x_mat_VA = []; y_mat_VA = []; d_gospa_VA = []; x_to_y_assignment_VA = []; decomposed_cost_VA = [];
            for va = 1:size(VA,2)
                y_mat_VA(:,va) = VA(va).pos;
            end

            for i = 1:para.N_vehicle
                for ti = 1:para.TIME
                    if ti == 1
                        x_mat_VA(ti,i).x=double.empty(3,0);
                    elseif ti >=2
                        Ind_a = find(Stack(ti).ave_Map(i).ST(2).P(1).weight > para.TargetDetectionThr_VA);
                        Est_index = Ind_a;
                        if numel(Est_index) > 0
                            for E_ind = 1:numel(Est_index)
                                x_mat_VA(ti,i).x(:,E_ind) = Stack(ti).ave_Map(i).ST(2).P(1).x(Est_index(E_ind),:)';
                            end
                        else
                            x_mat_VA(ti,i).x=double.empty(3,0);
                        end
                    end
                    [d_gospa, x_to_y_assignment, decomposed_cost] = GOSPA(x_mat_VA(ti,i).x, y_mat_VA, GOSPA_p, GOSPA_c, GOSPA_alpha);
                    GOSPA_VA(ti,i).GOSPA = d_gospa;
                    GOSPA_VA_evaluate(ti,i) = d_gospa;
                    GOSPA_VA(ti,i).Assign = x_to_y_assignment;
                    GOSPA_VA(ti,i).Cost= decomposed_cost;
                end
            end
            Perform.GOSPA_VA = GOSPA_VA_evaluate;

            % SP
            x_mat_SP = []; y_mat_SP = []; d_gospa_SP = []; x_to_y_assignment_SP = []; decomposed_cost_SP = [];
            for sp = 1:size(SP,2)
                y_mat_SP(:,sp) = SP(sp).pos;
            end

            for i = 1:para.N_vehicle
                for ti = 1:para.TIME
                    if ti == 1
                        x_mat_SP(ti,i).x=double.empty(3,0);
                    elseif ti >=2
                        Ind_a = find(Stack(ti).ave_Map(i).ST(3).P(1).weight > para.TargetDetectionThr_SP);
                        Est_index = Ind_a;
                        if numel(Est_index) > 0
                            for E_ind = 1:numel(Est_index)
                                x_mat_SP(ti,i).x(:,E_ind) = Stack(ti).ave_Map(i).ST(3).P(1).x(Est_index(E_ind),:)';
                            end
                        else
                            x_mat_SP(ti,i).x=double.empty(3,0);
                        end
                    end
                        [d_gospa, x_to_y_assignment, decomposed_cost] = GOSPA(x_mat_SP(ti,i).x, y_mat_SP, GOSPA_p, GOSPA_c, GOSPA_alpha);
                        GOSPA_SP(ti,i).GOSPA = d_gospa;
                        GOSPA_SP_evaluate(ti,i) = d_gospa;
                        GOSPA_SP(ti,i).Assign = x_to_y_assignment;
                        GOSPA_SP(ti,i).Cost= decomposed_cost;   
                end
            end
            Perform.GOSPA_SP = GOSPA_SP_evaluate;

        if MapFusionMode == 2 || MapFusionMode == 3
            i=1;
        % fused map at the BS
            % VA
            for ti = 10:2:para.TIME
                if ti == 1
                    UL_x_mat_VA(ti,i).x=double.empty(3,0);
                elseif ti >=2
                    Ind_a = find(Stack(ti).UL_Map.ST(2).P.weight > para.TargetDetectionThr_VA);
                    Est_index = Ind_a;
                    if numel(Est_index) > 0
                        for E_ind = 1:numel(Est_index)
                            UL_x_mat_VA(ti,i).x(:,E_ind) = Stack(ti).UL_Map.ST(2).P.x(Est_index(E_ind),:)';
                        end
                    else
                        UL_x_mat_VA(ti,i).x=double.empty(3,0);
                    end
                end
                    [d_gospa, x_to_y_assignment, decomposed_cost] = GOSPA(UL_x_mat_VA(ti,i).x, y_mat_VA, GOSPA_p, GOSPA_c, GOSPA_alpha);
                    UL_GOSPA_VA(ti,i).GOSPA = d_gospa;
                    UL_GOSPA_VA_evaluate(ti,i) = d_gospa;
                    UL_GOSPA_VA(ti,i).Assign = x_to_y_assignment;
                    UL_GOSPA_VA(ti,i).Cost= decomposed_cost;   
            end
            Perform.UL_GOSPA_VA = UL_GOSPA_VA_evaluate;

            %SP
            for ti = 10:2:para.TIME
                if ti == 1
                    UL_x_mat_SP(ti,i).x=double.empty(3,0);
                elseif ti >=2
                    Ind_a = find(Stack(ti).UL_Map.ST(3).P.weight > para.TargetDetectionThr_SP);
                    Est_index = Ind_a;
                    if numel(Est_index) > 0
                        for E_ind = 1:numel(Est_index)
                            UL_x_mat_SP(ti,i).x(:,E_ind) = Stack(ti).UL_Map.ST(3).P.x(Est_index(E_ind),:)';
                        end
                    else
                        UL_x_mat_SP(ti,i).x=double.empty(3,0);
                    end
                end
                    [d_gospa, x_to_y_assignment, decomposed_cost] = GOSPA(UL_x_mat_SP(ti,i).x, y_mat_SP, GOSPA_p, GOSPA_c, GOSPA_alpha);
                    UL_GOSPA_SP(ti,i).GOSPA = d_gospa;
                    UL_GOSPA_SP_evaluate(ti,i) = d_gospa;
                    UL_GOSPA_SP(ti,i).Assign = x_to_y_assignment;
                    UL_GOSPA_SP(ti,i).Cost= decomposed_cost;   
            end
            Perform.UL_GOSPA_SP = UL_GOSPA_SP_evaluate;
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
            leg1 = legend([p1,p2,p3],'Vechiel 1', 'Vehicle 2', 'Uplink-map');
        else
            if para.N_vehicle == 1
                leg1 = legend(p1,'Vechiel 1');
            else
                leg1 = legend([p1,p2],'Vechiel 1', 'Vehicle 2');
            end
        end
        xlabel('time $k$' ,'Interpreter','latex');
        ylabel('GOSPA of the VA' ,'Interpreter','latex');
        set(leg1,'Interpreter','latex');

        FN = ['VA_P_' num2str(para.N_p) '_T' num2str(para.TIME) '_FM' num2str(MapFusionMode) '_mc' num2str(mc)];
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
            leg1 = legend([p1,p2,p3],'Vechiel 1', 'Vehicle 2', 'Uplink-map');
        else
            if para.N_vehicle == 1
                leg1 = legend(p1,'Vechiel 1');
            else
                leg1 = legend([p1,p2],'Vechiel 1', 'Vehicle 2');
            end
        end
        xlabel('time $k$' ,'Interpreter','latex');
        ylabel('GOSPA of the SP' ,'Interpreter','latex');
        set(leg1,'Interpreter','latex');

        FN = ['SP_P_' num2str(para.N_p) '_T' num2str(para.TIME) '_FM' num2str(MapFusionMode) '_mc' num2str(mc)];
        FigureSave(FolderN,FN)
    end

    % Effectiveness of the number of particles
    Eff = zeros(para.TIME,para.N_vehicle);
    for ti = 1:para.TIME
        for i = 1:para.N_vehicle
            const_eff = 1/sum(Stack(ti).up_UE(i).weight.^2);
            Eff(ti,i) = const_eff; 
        end
    end
    Perform.Eff = Eff;
    
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
    FN = ['Effectiveness_P_' num2str(para.N_p) '_T' num2str(para.TIME) '_FM' num2str(MapFusionMode) '_mc' num2str(mc)];
    FigureSave(FolderN,FN)
end