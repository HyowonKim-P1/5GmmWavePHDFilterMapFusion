function [HX,Z,DH] = F_JacobianDiff(BS,state,x_0,diff,Z,m)
    % 5G mmWave Positioning and Mapping
    % (c) Hyowon Kim, 2019 (Ph.D. student at Hanyang Univerisy, Seoul, South Korea, emai: khw870511@hanyang.ac.kr)
    % Usage: this code generates Jacobian matrix
    % The Jacobian matrix is hard to be derived as the closed form, the semi-finite difference mtehod could be utilized.
    
    if m == 1
        HX = getChannelParameters(BS.pos,state([1:4 7]),x_0','LOS');
        ax = getChannelParameters(BS.pos,state([1:4 7]),x_0'+[diff;0;0],'LOS');
        ay = getChannelParameters(BS.pos,state([1:4 7]),x_0'+[0;diff;0],'LOS');
        az = getChannelParameters(BS.pos,state([1:4 7]),x_0'+[0;0;diff],'LOS');
    elseif m == 2
        HX = getChannelParameters(BS.pos,state([1:4 7]),x_0','VA');
        ax = getChannelParameters(BS.pos,state([1:4 7]),x_0'+[diff;0;0],'VA');
        ay = getChannelParameters(BS.pos,state([1:4 7]),x_0'+[0;diff;0],'VA');
        az = getChannelParameters(BS.pos,state([1:4 7]),x_0'+[0;0;diff],'VA');
    elseif m ==3
        HX = getChannelParameters(BS.pos,state([1:4 7]),x_0','SP');
        ax = getChannelParameters(BS.pos,state([1:4 7]),x_0'+[diff;0;0],'SP');
        ay = getChannelParameters(BS.pos,state([1:4 7]),x_0'+[0;diff;0],'SP');
        az = getChannelParameters(BS.pos,state([1:4 7]),x_0'+[0;0;diff],'SP');
    end
    HX = F_MeaCali(HX); ax = F_MeaCali(ax); ay = F_MeaCali(ay); az = F_MeaCali(az); Z = F_MeaCali(Z);
    ax = F_InovCali(ax,HX); ay = F_InovCali(ay,HX); az = F_InovCali(az,HX);
    DH(:,1) = (ax-HX)/diff;  
    DH(:,2) = (ay-HX)/diff;
    DH(:,3) = (az-HX)/diff;
    HX = F_InovCali(HX,Z);
end