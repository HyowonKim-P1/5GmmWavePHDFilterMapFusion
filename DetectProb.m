function p = DetectProb(prediction_Map, Birth, state, i, para)
    % (c) Hyowon Kim, 2019
    % (c) Hyowon Kim, 2019 (Ph.D. student at Hanyang Univerisy, Seoul, South Korea, emai: khw870511@hanyang.ac.kr)
    % Usage: this code generates detection probability

    % For BS
    p(1).detection = para.P_D*ones(prediction_Map.ST(1).P(i).J,1); % No birth for BS
    if para.onlyLOS ~= 1
        % For VA
        if prediction_Map.ST(2).P(i).J-Birth.ST(2).P(i).J > 0 % if persistent target exists
            for j = 1:prediction_Map.ST(2).P(i).J-Birth.ST(2).P(i).J % for persistent
                p(2).detection(j,1) = para.P_D;
            end
            for j = prediction_Map.ST(2).P(i).J-Birth.ST(2).P(i).J+1:prediction_Map.ST(2).P(i).J % for birth
                p(2).detection(j,1) = 1;
            end
        else % only birth
            p(2).detection = ones(prediction_Map.ST(2).P(i).J,1);
        end
        
        % For SP
        if prediction_Map.ST(3).P(i).J-Birth.ST(3).P(i).J > 0 % if peresistent target exists
            for j = 1:prediction_Map.ST(3).P(i).J-Birth.ST(3).P(i).J  % for persistent
                X_1 = prediction_Map.ST(3).P(i).x(j,:);
                X_0 = state(1:3);
                dist_VandT = norm(X_1'-X_0);
%                 if dist_VandT < para.SPVisibilityRadius - para.r_UC
%                     p(3).detection(j,1) = para.P_D;
%                 elseif dist_VandT >= para.SPVisibilityRadius - para.r_UC && dist_VandT <= para.SPVisibilityRadius + para.r_UC
% %                     K = (dist_VandT - (para.SPVisibilityRadius - para.r_UC))^2/2;
%                     K = (dist_VandT - (para.SPVisibilityRadius - para.r_UC))/2;
%                     p(3).detection(j,1) = para.P_D*exp(-K);
%                 elseif dist_VandT > para.SPVisibilityRadius + para.r_UC
%                     p(3).detection(j,1) = 0;
%                 end
                if dist_VandT + 3*sqrt(sum(diag(prediction_Map.ST(3).P(i).P(:,:,j)))) < para.SPVisibilityRadius
                    p(3).detection(j,1) = para.P_D;
                else
                    p(3).detection(j,1) = 0;
                end
            end
            for j = prediction_Map.ST(3).P(i).J-Birth.ST(3).P(i).J +1:prediction_Map.ST(3).P(i).J % for birth
                p(3).detection(j,1) = 1;
            end
        else % if only birth
            p(3).detection = ones(prediction_Map.ST(3).P(i).J,1);
        end
    end
    
end