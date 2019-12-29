function [mu_new, u_new] = ai(mu_old, u_old, y, delta_t, Der, alpha_mu, alpha_u, A_tilde, C_tilde, PI_w, PI_z, xi, G)
% AI  Execute AI algorithm
%
%   Author: Dennis Benders, TU Delft
%   Last edited: 29.12.2019
%
%   Input:  mu_old:     previous generalized state estimate
%           u_old:      previous control input (not used yet!)
%           y:          current system output
%           delta_t:    time interval between previous and current system
%                       output
%           Der:        derivative matrix for shifting the generalized
%                       state estimate
%           alpha_mu:   learning rate used to update mu
%           alpha_u:    learning rate used to update u (not used yet!)
%           A_tilde:    generalized state matrix
%           C_tilde:    generalized output matrix
%           PI_w:       precision matrix used to weight the process error
%           PI_z:       precision matrix used to weight the output error
%           xi:         prior
%           G:          forward model used to update u (not used yet!)
%
%   Output: mu_new:     current generalized state estimate
%           u_new:      next control input to provide to the system (not
%                       implemented yet!)
%
%   For usage, see ai_control.m.

% mu update rule
process_error = (Der - A_tilde)'*PI_w*(Der*mu_old - A_tilde*mu_old - xi);
output_error = C_tilde'*PI_z*(y - C_tilde*mu_old);
mu_dot = Der*mu_old - alpha_mu*(process_error - output_error);
mu_new = mu_old + mu_dot*delta_t;

% u update rule
% u_dot = -alpha_u*G'*PI_z*(y - C_tilde*mu_old);
% u_new = u_old + u_dot*delta_t;
u_new = [0;0];
end