function out = F_MeaCali(in)
    % 5G mmWave Positioning and Mapping
    % (c) Hyowon Kim, 2019 (Ph.D. student at Hanyang Univerisy, Seoul, South Korea, emai: khw870511@hanyang.ac.kr)
    % Usage: this code generates calibrated measurement
    % The measurement of the azimuth should consider its period 2pi
    % in: before calibration, measurement
    % out: after cliabration, measurement
    
    while(in(2) < 0)
        in(2) = in(2) + 2*pi;
    end
    while (in(4) < 0)
            in(4) = in(4) + 2*pi;
    end
    out = in;
end