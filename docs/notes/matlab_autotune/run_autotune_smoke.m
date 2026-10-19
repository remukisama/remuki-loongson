% MATLAB R2016a-compatible smoke test for the Loongson autotune bridge.
% It does not touch car_config.hpp or the board; it verifies parser/scoring
% code inside MATLAB.

result = lq_autotune('mode', 'smoke');
disp(result);
