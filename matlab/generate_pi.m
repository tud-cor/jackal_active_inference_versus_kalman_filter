function PI = generate_pi(k, SIGMA, s)
% GENERATE_PI  Generate the precision matrix
%
%   Derived from:
%   I. Hijne, “Computation of the generalized precision matrix,” 2019, unpublished.
%
%   Author: Dennis Benders, TU Delft
%   Last edited: 29.12.2019
%
%   Input:  k:      amount of elements of the generalized state vector
%                   (equal to the embedding order plus 1: p+1)
%           SIGMA:  covariance matrix of the noise used to construct the
%                   precision matrix
%           s:      kernel width of Gaussian filter used to create coloured
%                   noise (s = 0 in case of white noise)
%
%   Output: PI:     precision matrix
%
%   For usage, see ai_control.m.

% Limitation: cannot be calculated when sigma is unequal to zero
if max(max(SIGMA)) == 0
    error("PI cannot be generated if sigma is 0 or negative");
end

n = size(SIGMA, 1); %amount of states/outputs

if s ~= 0
    l = 0:2:2*k-1; % order of the required autocorrelation derivatives
    rho(1+l) = cumprod(1-l)./((sqrt(2).*s).^l); % rho^(l)(0)

    V = zeros(k,k); % preallocation of the temporal variance matrix
    for r = 1:k
        V(r,:) = rho(r:r+k-1); % assembly of the temporal variance matrix
        rho = -rho;
    end

    SIGMA_tilde = kron(V, SIGMA); % the generalised covariance matrix
    PI = inv(SIGMA_tilde);

else
    PI = zeros(k*n, k*n);
    PI(1:n, 1:n) = inv(SIGMA);

end