%This script defines dynamic parameters for the simulink and matlab
%simulation
%
%Defines: KD, D matrices

%% Bushing joint Paramters
KD = zeros(5,2);
KD(1,:) = [15,30]; % Kx, Dx
KD(2,:) = [15,30]; % Ky, Dy
KD(3,:) = [10,20]; % Kz, Dz

KD(4,:) = [5,20]; % Krx, Drx
KD(5,:) = [5,25]; % Krx, Drx


KD = [100 200; 100 200; 100 200 ; 50 100; 50 100]*100;

%% Joint Damping Paramters
D = 5e-3*ones(5,1); %good value