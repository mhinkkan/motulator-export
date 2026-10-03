function y = run_control(lib, name, widths, u, T_s)
%RUN_CONTROL Run the control system of a library with given inputs.
%   Y = RUN_CONTROL(LIB, NAME, WIDTHS, U, T_S) runs the block NAME of the library
%   LIB (built by build_control) alone in a temporary model with the fixed step T_S.
%   The rows of U are the inputs of the block at the sampling instants, the columns
%   being the signals of the input ports with the widths WIDTHS. The rows of Y are
%   the duty ratios and the monitored signals at the sampling instants.

load_system(lib);
sys = [lib '_run'];
if bdIsLoaded(sys)
    close_system(sys, 0);
end
new_system(sys);
closer = onCleanup(@() close_system(sys, 0));
add_block([lib '/' name], [sys '/Control']);
for k = 1:numel(widths)
    port = sprintf('u%d', k);
    add_block('built-in/Inport', [sys '/' port], ...
        'PortDimensions', num2str(widths(k)), 'SampleTime', blocks.num(T_s), ...
        'Interpolate', 'off');
    add_line(sys, [port '/1'], sprintf('Control/%d', k));
end
for k = 1:2
    port = sprintf('y%d', k);
    add_block('built-in/Outport', [sys '/' port]);
    add_line(sys, sprintf('Control/%d', k), [port '/1']);
end
t = (0:size(u, 1) - 1)'*T_s;
out = sim(sys, ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', blocks.num(T_s), ...
    'StopTime', blocks.num(t(end) + 0.5*T_s), ...
    'LoadExternalInput', 'on', ...
    'ExternalInput', '[t, u]', ...
    'SrcWorkspace', 'current', ...
    'SaveOutput', 'on', ...
    'SaveFormat', 'Array', ...
    'LimitDataPoints', 'off', ...
    'ReturnWorkspaceOutputs', 'on');
y = out.yout;
end
