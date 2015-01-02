% Parking Assistant Using Web Cameras
% Martin Kersner's Master Thesis
%
% m.kersner@gmail.com
% 01/03/2015
%
% 2D matrix normalization 
% -- Function: Y = normalize2 (X)
%    Return the 2-D normalized matrix within range [0, 1].

function Y = normalize2(X)

    Xmin = min(min(X));
    Xmax = max(max(X));

    Y = (X .- Xmin) ./ (Xmax - Xmin);

endfunction
