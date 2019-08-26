function VideoFusionPHD(Stack,BS,VA,SP,state,para,MapFusionMode,mc)
    % 5G mmWave Positioning and Mapping
    % (c) Hyowon Kim, 2019 (Ph.D. student at Hanyang Univerisy, Seoul, South Korea, emai: khw870511@hanyang.ac.kr)
    % Usage: this code generates video of fued map PHD
    
    m=4;
    if MapFusionMode == 1
        FolderN = 'video/NoFusion';
    elseif MapFusionMode ==2
        FolderN = 'video/UplinkFusion';
    elseif MapFusionMode == 3
        FolderN = 'video/UpDownlinkFusion';
    end
    for p = 1:3
        if (p ==1)
            FN = ['Downlink Map_P' num2str(para.N_p) '_T' num2str(para.TIME) '_FM' num2str(MapFusionMode) '_mc' num2str(mc)];
        elseif (p ==2)
            FN = ['Updated Map_P' num2str(para.N_p) '_T' num2str(para.TIME) '_FM' num2str(MapFusionMode) '_mc' num2str(mc)];
        elseif (p ==3)
            FN = ['Fusion Map_P' num2str(para.N_p) '_T' num2str(para.TIME) '_FM' num2str(MapFusionMode) '_mc' num2str(mc)];
        end
        filename = sprintf('%s/%s',FolderN,FN);
        vidObj = VideoWriter(filename);
        vidObj.FrameRate = 5;
        vidObj.Quality= 90;
        open(vidObj);
        for ti = 5:5:para.TIME
            if p == 1
                PHD = Stack(ti).PriorPHD.PHD(m).D;
            elseif p == 2
                PHD = Stack(ti).UpdatePHD.PHD(m).D;
            elseif p == 3
                PHD = Stack(ti).FusionPHD.PHD(m).D;
            end
            
            if rem(ti,10) == 5
                v = 1;
            elseif rem(ti,10) == 0
                v= 2 ;
            end

            figure(1)
            hold on; box on; grid on;
            axis([-220 220 -220 220 0 1.5])
            if p == 1
                title(sprintf('Downlink map PHD (total) at time %d',ti)); 
            elseif p == 2
                title(sprintf('Update map PHD (total) at time %d from vehicle %d',ti,v)); 
            elseif p == 3
                title(sprintf('Fusion map PHD (total) at time %d with vehicle %d',ti,v)); 
            end
            xlabel('x-axis [m]'); ylabel('y-axis [m]'); zlabel('PHD') 
%                 colorbar
%                 caxis([-1 1]);
            view(-60,75)
            mesh(para.X,para.Y,PHD)
            if p == 1
                
            else
                p1 = plot3(state(1,ti,v),state(2,ti,v),.1,'ko');
            end
            p2 = plot3(BS.pos(1),BS.pos(2),1,'k^');
            if numel(VA)>0
                for s = 1:numel(VA)
                    p3=plot3(VA(s).pos(1),VA(s).pos(2),1,'ms');
                end
            end
            if numel(SP)>0
                for s = 1:numel(SP)
                    if norm(SP(s).pos-state(1:3,ti,v)) > para.SPVisibilityRadius
                        p4=plot3(SP(s).pos(1),SP(s).pos(2),1,'bs');
                    else
                        p4=plot3(SP(s).pos(1),SP(s).pos(2),1,'rs');
                    end
                end
            end
            p6 = plot3(Stack(ti).V_Est(1,v),Stack(ti).V_Est(2,v),Stack(ti).V_Est(3,v), 'kx');
            if p == 1
                if numel(VA)>0 && numel(SP)>0
                    legend([p2,p3,p4],'BS', 'VA', 'SP')
                elseif  numel(VA)>0 && numel(SP)==0
                    legend([p2,p3],'BS', 'VA')
                elseif numel(SP)>0 && numel(VA)==0
                    legend([p2,p4],'BS', 'SP')
                end
            else                
                if numel(VA)>0 && numel(SP)>0
                    legend([p1,p2,p3,p4,p6],'Vehicle', 'BS', 'VA', 'SP','Vehicle estimation')
                elseif  numel(VA)>0 && numel(SP)==0
                    legend([p1,p2,p3,p6],'Vehicle', 'BS', 'VA','Vehicle estimation')
                elseif numel(SP)>0 && numel(VA)==0
                    legend([p1,p2,p4,p6],'Vehicle', 'BS', 'SP','Vehicle estimation')
                end
            end

            pause(0.01)
            currFrame = getframe(gcf);%,[-210 210 -210 +210 -40 40]);
            writeVideo(vidObj,currFrame);
            delete(gca)
        end
        close(vidObj);
    end
end