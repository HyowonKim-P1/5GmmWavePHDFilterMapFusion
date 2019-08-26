function ReParticle = F_ResampleFromEst(NumParticle,Dither,Dim,V_Est)
    % 5G mmWave Positioning and Mapping
    % (c) Hyowon Kim, 2019 (Ph.D. student at Hanyang Univerisy, Seoul, South Korea, emai: khw870511@hanyang.ac.kr)
    % Usage: this code generates the artificailly resampled particle
    % When the effective number of particle samples is too small (1~5), this function is useful.

    ReParticle =V_Est + [Dither.LH; Dither.LH; 0; Dither.H*pi/180; 0; 0; Dither.B].*randn(Dim,NumParticle);
end