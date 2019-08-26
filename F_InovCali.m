function output = F_InovCali(output,input)
    % 5G mmWave Positioning and Mapping
    % (c) Hyowon Kim, 2019 (Ph.D. student at Hanyang Univerisy, Seoul, South Korea, emai: khw870511@hanyang.ac.kr)
    % Usage: this code generates calibrated measurement for the cubature points
    % in: before calibration, measurement
    % out: after cliabration, measurement
    
    while (output(2) - input(2) > pi)
        output(2) = output(2) - 2*pi;
    end
    while (output(2) - input(2) < -pi)
        output(2) = output(2) + 2*pi;
    end
    while (output(4) - input(4) > pi)
        output(4) = output(4) - 2*pi;
    end
    while (output(4) - input(4) < -pi)
        output(4) = output(4) + 2*pi;
    end
    if (abs(output(2) - input(2)) > pi ||abs(output(4) - input(4)) >pi)
        sprintf('angle domain error')
        output;
    end
end
