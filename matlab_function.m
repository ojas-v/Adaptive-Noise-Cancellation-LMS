function W_out = custom_nlms(x_prime_buf, e, W_in, mu_W, epsilon_W)
% Calculate adaptive step size
mu_adaptive = mu_W / (epsilon_W + norm(x_prime_buf)^2);

% Use .* for strict element-wise multiplication against the buffer
W_out = W_in + (mu_adaptive * e) .* x_prime_buf;
end