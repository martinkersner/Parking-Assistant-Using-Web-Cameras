% Parking Assistant Using Web Cameras
% Martin Kersner's Master Thesis
%
% m.kersner@gmail.com
% 01/02/2015
%
% Kirsch Edge Detection 
% -- Function: Y = kirsch (IMG)
%    Return the 2-D matrix with enhanced edges found in given image IMG.

function Y = kirsch(IMG)
    
    % convolutional masks
    K0 = [ -3  -3  5; 
           -3   0  5;
           -3  -3  5 ];

    K1 = [ -3   5   5; 
           -3   0   5;
           -3  -3  -3 ];

    K2 = [  5   5   5; 
           -3   0  -3;
           -3  -3  -3 ];

    K3 = [  5   5  -3; 
            5   0  -3;
           -3  -3  -3 ];

    K4 = [  5  -3  -3; 
            5   0  -3;
            5  -3  -3 ];

    K5 = [ -3  -3  -3; 
            5   0  -3;
            5   5  -3 ];

    K6 = [ -3  -3  -3; 
           -3   0  -3;
            5   5   5 ];

    K7 = [ -3  -3  -3; 
           -3   0   5;
           -3   5   5 ];

    % convolve 
    D = zeros(size(IMG, 1), size(IMG, 2), 8);
    D(:,:,1) = conv2(IMG, K0, 'same');
    D(:,:,2) = conv2(IMG, K1, 'same');
    D(:,:,3) = conv2(IMG, K2, 'same');
    D(:,:,4) = conv2(IMG, K3, 'same');
    D(:,:,5) = conv2(IMG, K4, 'same');
    D(:,:,6) = conv2(IMG, K5, 'same');
    D(:,:,7) = conv2(IMG, K6, 'same');
    D(:,:,8) = conv2(IMG, K7, 'same');

    % normalize max of responses
    Y = normalize2(max(D, [], 3));

endfunction
