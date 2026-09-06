function [fz_estimate] = estimate_lattice_force(z,t,back_track,k,r1,r2,time_window)
%estimate_lattice_force given a z displacement array, returns an estimated force 
%   Z is sampled at 100hz, starting window is atleast 0.1 seconds into the
%   measurement
%   characteristic data k,r1,r2,back_track and time_window should be loaded

arguments (Input)
    z
    t
    back_track
    k
    r1
    r2
    time_window
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
        dz(i-starting_index+1) = (z(i) - z(i - (back_track*frequency)))/back_track; % avg vel over back track
        if abs(dz(i-starting_index+1)) > vel_threshold
            last_time = t(i);
            signTerm = dz(i-starting_index+1)/abs(dz(i-starting_index+1));
        end
        time_elapsed = t(i) - last_time;
        fz_estimate(i-starting_index+1) = -(-k*z(i) + signTerm*(r1*z(i)*(exp(-time_elapsed/time_window)) + r2*z(i)));
    end
end

fz_prepend = ones(starting_index-1,1,"double");
fz_prepend = fz_estimate(1)*fz_prepend;
fz_estimate = [fz_prepend;fz_estimate'];

end