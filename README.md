# 5G mmWave Cooperative Positioning and Mappingusing Multi-Model PHD Filter and Map Fusion
This code computes the performances of the vehicle state estimation and the mapping of the environment in 5G mmWave vehicular networks.

## Summary
The NetwMeasGen.m generates vehicle trajectories, three-types of objects (base station, virtual anchors, scattering points), and clutter.

The main.m is used to estimate the vehicle state and the objects' state, and compute the RMSE of the vehicle sate and GOSPA of the objects.

## Main parameters
Both codes have the same parameters
```
para.MC = 10; % # Monte Carlo run
para.TIME = 40; % # time evolution (para.TIME = 2*pi/Vel_rot.ini/para.t_du)
para.UECovInitial = diag([.3 .3 0 0.3*pi/180 0 0 .3].^2); % vehicle state prior covariance
para.N_p = 2000; % # of particlces
measurementCovariance = 9*measurementCovariance; % tunning parameter for map update
para.BS_cov = 1e-2; %initial cov of BS position diag([cov cov])
para.birth_weight = 1.5*1e-5; % birth weight; considering clutter intensity; the sum of birth PHDs is not necessary to be 1.
para.P_D = 0.9; % Detection probability
para.c_z = para.lambda*(1/200)*(1/2/pi)^2*(1/pi)^2; % clutter intensity; 1.2832e-05  auumed to be constant (1/R_max, 1/R, 1/2pi)
para.pruning_T = 1e-4; % truncating threshold T, which should be bigger than (1-P_d)/(3#sources) (ref. Ba-Ngu et al., ``GM PHD filter,'' IEEE TSP, 2006.)
para.pruning_U = 7^2; % terging threshold U, Mahalnobis dist = sqrt(U) (ref. Ba-Ngu et al., ``GM PHD filter,'' IEEE TSP, 2006.)
para.pruning_J = 50; % maximum allowable number of Gaussians J_max (ref. Ba-Ngu et al., ``GM PHD filter,'' IEEE TSP, 2006.)
para.pruning_COV = 50; % if the unceratinty of a map is larger than 50 [m^2], then the map is ignored.
para.ULTD = 4; % map fusion duration
Fusion_v1 = 10:para.ULTD:para.TIME; % map fusion time index for the vehicle 1
Fusion_v2 = 12:para.ULTD:para.TIME; % map fusion time index for the vehicle 2
para.TargetDetectionThr_VA = 0.7; % weight threshold for VA detection
para.TargetDetectionThr_SP = 0.55; % weight threshold for SP detection
```

## Authors
The code was developed by [Hyowon Kim], Ph.D. student at Hanyang University, Seoul, South Korea.

For more information, please see **[here](https://arxiv.org/abs/xxx)**
```
will be added
```


## License
This project is licensed under the joint project of Chalmers University of Technology and Hanyang University - see the [LICENSE.md](LICENSE.md) file for details.
