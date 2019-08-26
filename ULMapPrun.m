function [x, P, weight, J] = ULMapPrun(map, para)
    % 5G mmWave Positioning and Mapping
    % (c) Hyowon Kim, 2019 (Ph.D. student at Hanyang Univerisy, Seoul, South Korea, emai: khw870511@hanyang.ac.kr)
    % Usage: this code generates the merged map for the fused map
    % After the merging step are performed, too small weight is ignored again for reducing the complexity
    
    p_ind = 0;
    J_set = find(map.weight > para.pruning_T);
    if numel(J_set)>0
        while (numel(J_set) ~= 0)
            p_ind = p_ind+1;
            pruning_P(:,:,p_ind) = zeros(3,3,1);
            j_ind = find( map.weight == max(map.weight(J_set)));
            if (numel(j_ind) > 1)
                j_intersect = intersect(j_ind,J_set);
                j_ind = j_intersect(1);
            end
            L_ind = []; temp_ind = [];
            for i_ind = 1:numel(J_set)
                MergingCondition = ((map.x(J_set(i_ind),:) - map.x(j_ind,:) )*inv(map.P(:,:,J_set(i_ind)))*(map.x(J_set(i_ind),:) - map.x(j_ind,:) )' <=para.pruning_U);
                if ( MergingCondition)
                    temp_ind = [temp_ind i_ind];
                end
            end
            L_ind = J_set(temp_ind);
            L_ind = J_set(temp_ind);
            pruning_weight(p_ind,1) = sum(map.weight(L_ind));
            if numel(L_ind) > 1
                pruning_x(p_ind,:) =sum( map.x(L_ind(1:numel(L_ind)),:).*map.weight(L_ind(1:numel(L_ind)),1) )/pruning_weight(p_ind,1);
            elseif numel(L_ind) == 1
                pruning_x(p_ind,:) =map.x(L_ind(1:numel(L_ind)),:)*map.weight(L_ind(1:numel(L_ind)),1) /pruning_weight(p_ind,1);
            end
            for i_ind = 1:numel(L_ind)
                pruning_P(:,:,p_ind) = pruning_P(:,:,p_ind) + map.weight(L_ind(i_ind))*(map.P(:,:,L_ind(i_ind)) + (pruning_x(p_ind,:) - map.x(L_ind(i_ind),:))'*(pruning_x(p_ind,:) - map.x(L_ind(i_ind),:)));
            end
            pruning_P(:,:,p_ind) = pruning_P(:,:,p_ind)/pruning_weight(p_ind);
            J_set = setdiff(J_set,L_ind);
        end
        
        M_set = find(pruning_weight > para.pruning_T); %% Too small weight is ignored for reducing the complexity
        if numel(M_set) > 0
            pruning_x = pruning_x(M_set,:);
            pruning_weight = pruning_weight(M_set,:);
            pruning_P = pruning_P(:,:,M_set);
            
            [sort_value, sort_index] = sort(pruning_weight, 'descend');
            pruning_weight = sort_value;
            pruning_x = pruning_x(sort_index,:);
            pruning_P = pruning_P(:,:,sort_index);
        end
        if (numel(pruning_weight) > para.pruning_J) % # the case that grouped GMs is more than J_max
            pruning_weight = pruning_weight(1:para.pruning_J);
            pruning_x = pruning_x(1:para.pruning_J,:);
            pruning_P = pruning_P(:,:,1:para.pruning_J);
        end
        x = pruning_x;
        P = pruning_P;
        weight = pruning_weight;
        J = numel(weight);
    else
        x=[];
        P=[];
        weight=[];
        J=[];
    end
end