function [ mu, dmu ] = burckhardt( lambda, road_condition )
% Burckhardt Friction model.
%
% Arguments
%  - lambda         -> longitudinal slip.
%  - road_condition -> road condition:
%                       - 1 -> Dry Asphalt;
%                       - 2 -> Wet Asphalt;
%                       - 3 -> Cobblestone;
%                       - 4 -> Snow.
%
% Outputs
%  - mu  -> friction coefficient.
%  - dmu -> sloped of the friction coefficient.
%

switch road_condition
    case 1 % Dry Asphalt
        theta = [ 1.28, 23.99, 0.52 ];
    case 2 % Wet Asphalt
        theta = [ 0.86, 33.82, 0.35 ];
    case 3 % Cobblestone
        theta = [ 1.37,  6.46, 0.67 ];
    case 4 % Snow
        theta = [ 0.19, 94.13, 0.06 ];
    otherwise % Dry Asphalt
        theta = [ 1.28, 23.99, 0.52 ];
end

   mu  = theta(1) * (1 - exp( -lambda * theta(2)) ) - lambda * theta(3);
   dmu = theta(1) * theta(2) * exp( -lambda * theta(2) ) - theta(3);

end