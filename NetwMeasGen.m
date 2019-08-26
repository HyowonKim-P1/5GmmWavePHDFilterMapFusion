close all; clear all;
% 5G mmWave Positioning and Mapping
% (c) Hyowon Kim, 2019 (Ph.D. student at Hanyang Univerisy, Seoul, South Korea, emai: khw870511@hanyang.ac.kr)
% Usage: this code generates vehicular networks

%% Parameters
f_mode = 0; % 1) figure on;
para.save = 1; % 1) save;
para.MC = 10; % # Monte Carlo run
para.TIME = 40; % # time evolution (para.TIME = 2*pi/Vel_rot.ini/para.t_du)
para.t_du = .5; % time duration
para.N_vehicle = 2; % number of vehicles
para.N_VA = 4; % # number of VAs
para.N_SP = 4;  % # number of SPs
para.SPVisibilityRadius=50; % FoV for SP, [m]
para.lambda = 1; % expectation of the Poisson distribution
para.Rmax = 200; % maximum distance range, [m]

sigma.DOA_az = 1e-2; sigma.DOA_el = 1e-2; sigma.DOD_az = 1e-2; sigma.DOD_el = 1e-2; sigma.TOA = 1e-1; % DOA azimuth [radian], DOA elevation [radian], DOD azimuth [radian], DOD elevation [radian], TOA [m](1/3e-9 [s])
measurementCovariance = diag([sigma.TOA^2,sigma.DOD_az^2,sigma.DOD_el^2,sigma.DOA_az^2,sigma.DOA_el^2]);
para.ProcessNoiseCovariance=[(.2)^2; (.2)^2; 0; (1e-3)^2; 0; 0; (.2)^2]; % units: [m; m; m; rad; m/s; rad/s; m]^2 

%% Vehicular networks
% initial vehicle state
para.InitialState = zeros(para.N_vehicle,7);
para.InitialState(1,:)=[0 0 0 pi/2 22.22 pi/10 300]; % vehicle 1 initial state
para.InitialState(2,:)=[0 0 0 pi/2 -22.22 pi/10 300]; % vehicle 2initial state
for i = 1:para.N_vehicle
    para.InitialState(i,1)=para.InitialState(i,5)/para.InitialState(i,6); % put the UE in a good location to start (make a circle around the BS)
end

% BS/VA/SP location
VA=[]; SP=[];
BS.pos = [0; 0; 40];
VA(1).pos=[para.Rmax; 0; BS.pos(3)];
VA(2).pos=[0; para.Rmax; BS.pos(3)];
VA(3).pos=[-para.Rmax; 0; BS.pos(3)];
VA(4).pos=[0; -para.Rmax; BS.pos(3)];

SP(1).pos=[65; 65; rand(1)*40];
SP(2).pos=[65; -65; rand(1)*40];
SP(3).pos=[-65; -65; rand(1)*40];
SP(4).pos=[-65; 65; rand(1)*40];
    
for mc = 1:para.MC
    state = zeros(7,para.TIME,para.N_vehicle);  % vehicle state across time

    % data structure to generate the real channel parameters
    Channel.Values = zeros(5,para.N_VA+para.N_SP+1,para.TIME,para.N_vehicle); % ordered: TOA, DOD(az,el) DOA(az,el)
    Channel.Labels=['TOA ' ' DODaz' ' DODel' ' DOAaz' ' DOAel'];
    Channel.Visible.TOT= ones(para.N_VA+para.N_SP+1,para.TIME,para.N_vehicle);  % 0 or 1
    Channel.Visible.LOS= ones(1,para.TIME,para.N_vehicle);  % 0 or 1
    Channel.Visible.VA= ones(para.N_VA,para.TIME,para.N_vehicle);  % 0 or 1
    Channel.Visible.SP= ones(para.N_SP,para.TIME,para.N_vehicle);  % 0 or 1

    % measurement and vehicle state @ time 1
    for i = 1:para.N_vehicle
        ti = 1;
        state(:,1,i) = para.InitialState(i,:)'; % [x; y; z; orientation; longitudinal velocity; rotational velocity; bias]     
        Channel.Values(:,1,1,i)=getChannelParameters(BS.pos,state([1:4 7],1,i),BS.pos,'LOS');
        for k=1:para.N_VA
           Channel.Values(:,1+k,1,i)=getChannelParameters(BS.pos,state([1:4 7],1,i),VA(k).pos,'VA'); 
        end
        for k=1:para.N_SP
            Channel.Values(:,1+para.N_VA+k,1,i)=getChannelParameters(BS.pos,state([1:4 7],1,i),SP(k).pos,'SP'); 
            if (norm(state(1:3,1,i)-SP(k).pos)>para.SPVisibilityRadius)
               Channel.Visible.TOT(1+para.N_VA+k,1,i)= 0;
               Channel.Visible.SP(k,1,i)=0;
            end
        end
        N_clutter = poissrnd(para.lambda,1); 
        Channel.Clutter(ti).vehicle(i).visible = N_clutter;
        if N_clutter > 0
            for m = 1:N_clutter
                Channel.Clutter(ti).vehicle(i).measurement(:,m) = [para.Rmax*rand; 2*pi*rand; -pi/2+pi*rand; 2*pi*rand; -pi/2+pi*rand];
            end
        end
        
        % vehicle evolution and its measurement
        for ti = 2:para.TIME
            state(:,ti,i) = state(:,ti-1,i) + [state(5,ti-1,i)/state(6,ti-1,i) * (sin( state(4,ti-1,i) + para.t_du*state(6,ti-1,i) ) - sin( state(4,ti-1,i)));...
                                                 state(5,ti-1,i)/state(6,ti-1,i) * (-cos( state(4,ti-1,i) + para.t_du*state(6,ti-1,i) ) + cos( state(4,ti-1,i)));...
                                                 0; para.t_du*state(6,ti-1,i);0;0;0] + sqrt(para.ProcessNoiseCovariance).*randn(7,1);
            state(4,ti,i) = mod(state(4,ti,i),2*pi);
            state(6,ti,i) = mod(state(6,ti,i),2*pi);                            
            Channel.Values(:,1,ti,i)=getChannelParameters(BS.pos,state([1:4 7],ti,i),BS.pos,'LOS');        
            for k=1:para.N_VA
               Channel.Values(:,1+k,ti,i)=getChannelParameters(BS.pos,state([1:4 7],ti,i),VA(k).pos,'VA'); 
            end
            for k=1:para.N_SP
                Channel.Values(:,1+para.N_VA+k,ti,i)=getChannelParameters(BS.pos,state([1:4 7],ti,i),SP(k).pos,'SP'); 
                if (norm(state(1:3,ti,i)-SP(k).pos)>para.SPVisibilityRadius)
                   Channel.Visible.TOT(1+para.N_VA+k,ti,i)= 0;
                   Channel.Visible.SP(k,ti,i)=0;
                end
            end
            N_clutter = poissrnd(para.lambda,1);
            Channel.Clutter(ti).vehicle(i).visible = N_clutter;
            if N_clutter > 0
                for m = 1:N_clutter
                    Channel.Clutter(ti).vehicle(i).measurement(:,m) = [para.Rmax*rand; 2*pi*rand; -pi/2+pi*rand; 2*pi*rand; -pi/2+pi*rand];
                end
            end
            % Generating measurement
            map_temp=getMeasurement(Channel.Values(:,:,ti,i),Channel.Visible.TOT(:,ti,i),Channel.Clutter(ti).vehicle(i),measurementCovariance);
            v(i).Time(ti).measurement = map_temp;
        end
    end
    
%     for i = 1:para.N_VA
%         ED(i).pos = zeros(3,para.TIME,para.N_vehicle);
%     end
%     for i = 1:para.N_SP
%          EP(i).pos = zeros(3,para.TIME,para.N_vehicle);
%     end
%     for v = 1:para.N_vehicle
%         % ED (reflection point)
%         for ti = 1:para.TIME
%             for i = 1:para.N_VA
%                 ED(i).pos(:,ti,v) = VA(i).pos + ( ( (BS.pos + VA(i).pos)/2 - VA(i).pos)'*( (BS.pos-VA(i).pos)/norm(BS.pos-VA(i).pos) ) ) / ( (state(1:3,ti,v)-VA(i).pos)'*( (BS.pos-VA(i).pos)/norm(BS.pos-VA(i).pos) ) )*(state(1:3,ti,v) - VA(i).pos);
%             end
%         end
% 
%         % EP (extended point)
%         for ti = 1:para.TIME
%             for i = 1:para.N_SP
%                 EP(i).pos(:,ti,v) = state(1:3,ti,v) + (norm(state(1:3,ti,v) - SP(i).pos) + norm(BS.pos - SP(i).pos))*(SP(i).pos - state(1:3,ti,v))/norm(SP(i).pos - state(1:3,ti,v));
%             end
%         end
%     end
%------------------------------- figure -------------------------------%
    if para.save ==1
        Sp = ['measurement/measurement_' num2str(mc) '_' num2str(para.TIME)];
        filepath = Sp;
        save(filepath, 'state', 'para', 'BS', 'VA', 'SP', 'Channel', 'v', 'measurementCovariance')
    end
    if (f_mode == 1)
        %% Figure - vehicular networks
        % just some plotting
        figure(1)
        box on; grid on; hold on;
        ti = 2:para.TIME;
        p1 = plot3(state(1,ti,1), state(2,ti,1), state(3,ti,1), 'k-', 'Linewidth', 2, 'Markersize', 8);
%         p5 = plot3(state(1,ti,2), state(2,ti,2), state(3,ti,2), 'g--', 'Linewidth', 3);
        plot3([BS.pos(1) BS.pos(1)], [BS.pos(2) BS.pos(2)], [0 BS.pos(3)], 'r-', 'Linewidth', 2, 'Markersize', 8);
        p2 = plot3(BS.pos(1), BS.pos(2), BS.pos(3), 'r^', 'Linewidth', 2, 'Markersize', 8);
        for i = 1:para.N_VA
            p3 = plot3(VA(i).pos(1), VA(i).pos(2), VA(i).pos(3), 'md', 'Linewidth', 2, 'Markersize', 8);
            plot3([VA(i).pos(1) VA(i).pos(1)], [VA(i).pos(2) VA(i).pos(2)], [0 VA(i).pos(3)], 'm-', 'Linewidth', 2, 'Markersize', 8);
        end
        p4=[];
        for i = 1:para.N_SP
            p4 = plot3(SP(i).pos(1), SP(i).pos(2), SP(i).pos(3), 'bs', 'Linewidth', 2, 'Markersize', 8);
            plot3([SP(i).pos(1) SP(i).pos(1)], [SP(i).pos(2) SP(i).pos(2)], [0 SP(i).pos(3)], 'b-', 'Linewidth', 2, 'Markersize', 8);
        end
        p6 = plot3(state(1,1,1), state(2,1,1), state(3,1,1), 'rx', 'Linewidth', 2, 'Markersize', 8);
        p7 = plot3(state(1,2,2), state(2,2,2), state(3,2,2), 'go', 'Linewidth', 2, 'Markersize', 8);
        axis([-200 200 -200 200 0 40])
        xlabel('x-axis [m]','Interpreter','latex');
        ylabel('y-axis [m]','Interpreter','latex');
        zlabel('z-axis [m]','Interpreter','latex');
        leg1 = legend([p1,p6, p7, p2, p3, p4], 'Circular road' , 'Initial location of vehicle 1', 'Initial location of vehicle 2', 'BS', 'VA', 'SP');
        set(leg1,'Interpreter','latex');
        
        ti = 1:para.TIME;
        figure(2)
        subplot(4,1,1)
        hold on;
        plot(ti*para.t_du,state(1,ti,1),ti*para.t_du,state(2,ti,1))
        plot(ti*para.t_du,state(1,ti,2),ti*para.t_du,state(2,ti,2))
        grid
        legend('V1 x(t)','V1 y(t)','V2 x(t)','V2 y(t)');
        xlabel('time [s]')
        ylabel('[m]')
        subplot(4,1,2)
        hold on;
        plot(ti*para.t_du,state(4,ti,1))
        plot(ti*para.t_du,state(4,ti,2))
        grid
        legend('V1 heading(t)','V2 heading(t)');
        xlabel('time [s]')
        ylabel('[rad]')
        subplot(4,1,3)
        hold on;
        plot(ti*para.t_du,state(7,ti,1))
        plot(ti*para.t_du,state(7,ti,2))
        grid
        legend('V1 bias(t)','V2 bias(t)');
        xlabel('time [s]')
        ylabel('[m]')
        subplot(4,1,4)
        hold on;
        plot(ti*para.t_du,sum(Channel.Visible.TOT(:,:,1)))
        plot(ti*para.t_du,sum(Channel.Visible.TOT(:,:,2)))
        grid
        axis([ti(1)*para.t_du ti(end)*para.t_du 0 para.N_VA+para.N_SP+1])
        xlabel('time [s]')
        ylabel('visible paths')
        legend('V1 visible paths(t)','V2 visible paths(t)')
    end
end
