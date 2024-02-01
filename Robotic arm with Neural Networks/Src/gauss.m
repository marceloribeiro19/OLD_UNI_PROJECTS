function g = gauss(nx,x0,sigma,A,K)

% GAUSS
%  function g = gauss(nx,x0,sigma,A,K)
%
% Generates a gaussian like function :
%
% 		g = A*exp[-(( x-xo).^2)./(2*sigma^2) ] + K
%
%	nx - array of values over which gaussian fuction is computed
%	x0 - center of peak.
%
%	A - Peak amplitude (from minimum to maximum values).
%	
%	K - shift up or down gaussian function depeding on sign.
%  


g = A*exp(-(( nx-x0).^2)./(2*sigma^2) ) + K;
