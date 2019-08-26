function up_Map = DLMapCopy(UL_Map, para)
    % 5G mmWave Positioning and Mapping
    % (c) Hyowon Kim, 2019 (Ph.D. student at Hanyang Univerisy, Seoul, South Korea, emai: khw870511@hanyang.ac.kr)
    % Usage: this code bring the map from the BS (downlink transmission)
    % Bring the up-link updated map, and just copy the map to all prticles
    for m = 1:3
        up_Map.ST(m).P(1:para.N_p) = UL_Map.ST(m).P(1);
    end
end