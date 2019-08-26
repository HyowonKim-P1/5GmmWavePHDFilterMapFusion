function [measurement]=getMeasurement(ChannelValues,ChannelVisible,clutter,measurementCovariance)
% this function generates a TOA, AOD, AOA measurement, given the true
% value, which paths are currently visible, and the measurement Covariance

    N=sum(numel(ChannelVisible));
    measurement=zeros(sum(ChannelVisible)+clutter.visible,5);
    k=0;
    for n=1:N
        if (ChannelVisible(n))
            k=k+1;
            trueval=ChannelValues(:,n);
%             sb = sigma_bound(p);
%             estimatedval=trueval+sqrt(measurementCovariance)*sb;
            estimatedval=trueval+diag(sqrt(measurementCovariance)).*randn(5,1);
%             noiseless_mea(n,:)=trueval';
            measurement(k,:)=estimatedval';
        end
    end
    if clutter.visible>0
        for m = 1:clutter.visible
            k=k+1;
            estimatedval = clutter.measurement(:,m) + diag(sqrt(measurementCovariance)).*randn(5,1);
            measurement(k,:) = estimatedval';
        end
    end
end
    
    
