function [z_fil] = iir(z,a)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    z,
    a % alpha
end
arguments (Output)
    z_fil
end

    fil = 0.0;
    z_fil = zeros(size(z,1),1);
    for i = 1:size(z,1)
        z_fil(i,1) = (a)*z(i,1) + (1-a)*fil;
        fil = z_fil(i,1);
    end

end