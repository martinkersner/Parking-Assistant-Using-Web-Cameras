% Parking Assistant Using Web Cameras
% Martin Kersner's Master Thesis
%
% m.kersner@gmail.com
% 01/02/2015
%
% Sobel Edge Detection 
% -- Function: Y = sobel (IMG)
%    Return the 2-D matrix with enhanced edges found in given image IMG.

function Y = sobel(IMG)

    % convolutional masks for each axis
    Sx = [ -1 -2 -1; 
            0  0  0;
            1  2  1 ];

    Sy = Sx';

    % convolve 
    Dx = conv2(IMG, Sx, 'same');
    Dy = conv2(IMG, Sy, 'same');

    % magnitude
    M = abs(Dx) + abs(Dy);

    % normalize magnitude
    Y = normalize2(M);

endfunction
