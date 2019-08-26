function [log_w, log_sum_w]= normalizeLogWeights(log_w)

%
% Input:
% log_w - 'log weights', e.g., log likelihoods
%
% Output:
% log_w - 'log of the normalized weights'
% log_sum_w - 'log of the sum of the non-normalized weights'
%

if length(log_w)==1
    % Log of sum of weights times prior probabilities
    log_sum_w = log_w;
    % Normalize
    log_w = log_w-log_sum_w;
elseif length(log_w)>1
    % Log of sum of weights times prior probabilities
    log_w_aux = sort(log_w,'descend');
    log_sum_w = log_w_aux(1)+log(1+sum(exp(log_w_aux(2:end)-log_w_aux(1))));
    % Normalize
    log_w = log_w-log_sum_w;
end