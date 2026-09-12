function [fz_estimate] = estimate_lattice_force(z,t,t_inv,k,r1,r2,kr)
%estimate_lattice_force given a z displacement array, returns an estimated force 
%   Z is sampled at 100hz, starting window is atleast 0.1 seconds into the
%   measurement
%   characteristic data k,r1,r2,t_inv and kr should be loaded

arguments (Input)
    z
    t
    t_inv
    k
    r1
    r2
    kr
end

arguments (Output)
    fz_estimate
end

length = size(z,1);

frequency = 100; % hz
starting_window = 0.5; %seconds

last_time = 0.0;
vel_threshold = 5e-4;

fz_estimate = [];
dz = [];
signTerm = 1;

starting_index = starting_window*frequency;

for i=1:length
    if i>=starting_index
        dz(i-starting_index+1) = (z(i) - z(i - (t_inv*frequency)))/t_inv; % avg vel over back track
        if abs(dz(i-starting_index+1)) > vel_threshold
            last_time = t(i);
            signTerm = dz(i-starting_index+1)/abs(dz(i-starting_index+1));
        end
        time_elapsed = t(i) - last_time;
        fz_estimate(i-starting_index+1) = -(k*z(i) + signTerm*(r1*z(i)*(exp(-time_elapsed*kr)) + r2*z(i)));
    end
end

fz_prepend = ones(starting_index-1,1,"double");
fz_prepend = fz_estimate(1)*fz_prepend;
fz_estimate = [fz_prepend;fz_estimate'];

end