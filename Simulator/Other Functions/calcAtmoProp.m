function [rho,Tau,a] = calcAtmoProp(h)
%Calculates atmospheric density, temperature, and speed of sound at given altitude
%
% Inputs
%
%   h = altitude [ft]
%
% Outputs
%
% rho = density [slugs/ft^3]
%   a = speed of sound [ft/s]
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Calculate temperature and density at the desired altitude
rho = zeros(length(h),1);
Tau = rho;
a = rho;
h1 = 36089;
h2 = 65617;
h3 = 104990;
R = 1716.5; %[ft^2 / s^2 oR]
gam = 1.4;
    for i = 1:length(h)
        if h(i) < h1
            %Troposphere
            Tau(i) = 518.69-(3.5662*10^(-3))*h(i);
            rho(i) = (6.6277*10^(-15))*Tau(i)^(4.2560);
        elseif h > h2
            %Stratosphere2
            Tau(i) = 389.99+(5.4864*10^(-4))*(h(i)-65617);
            rho(i) = (2.2099*10^(87))*Tau(i)^(-35.164);
        elseif h > h3
            disp('Error! Above Stratosphere')
            rho(i) = 0;
            Tau(i) = 0;
        else 
            %Stratosphere1 
            Tau(i) = 389.99;
            P = (2678.4)*exp(-h(i)*(4.8063*10^(-5)));
            rho(i) = (1.4939*10^(-6))*P;
        end
    %Calculate speed of sound
    a(i) = sqrt(gam*R*Tau(i));
    end
end