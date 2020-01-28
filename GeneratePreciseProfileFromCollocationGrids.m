function [timeline, profile] = GeneratePreciseProfileFromCollocationGrids(grids, Nfe, tf, precision_level, is_control_profile)
timeline = [];
profile = [];
hi = tf / Nfe;
if (is_control_profile == 1)
    for ii = 1 : Nfe
        lb = hi * (ii-1);
        ub = hi * ii;
        abstracted_data = grids((3*ii-2):(3*ii));
        abstracted_time = lb + (ub - lb) .* [0.1550510257216822, 0.6449489742783178, 1];
        detailed_time = [lb : precision_level : ub];
        detailed_data = ResampleLagrangeFunction(abstracted_time, abstracted_data, detailed_time);
        timeline = [timeline, detailed_time];
        profile = [profile, detailed_data];
    end
else
    for ii = 1 : Nfe
        lb = hi * (ii-1);
        ub = hi * ii;
        abstracted_data = grids((4*ii-3):(4*ii));
        abstracted_time = lb + (ub - lb) .* [0, 0.1550510257216822, 0.6449489742783178, 1];
        detailed_time = [lb : precision_level : ub];
        detailed_data = ResampleLagrangeFunction(abstracted_time, abstracted_data, detailed_time);
        timeline = [timeline, detailed_time];
        profile = [profile, detailed_data];
    end
end
end

function y = ResampleLagrangeFunction(x0, y0, x)
n = length(x0);
m = length(x);
y = zeros(1,m);
for ii = 1 : m
    z = x(ii);
    s = 0;
    for kk = 1 : n
        p = 1;
        for jj = 1 : n
            if jj ~= kk
                p = p * (z - x0(jj)) / (x0(kk) - x0(jj));
            end
        end
        s = p * y0(kk) + s;
    end
    y(ii) = s;
end
end