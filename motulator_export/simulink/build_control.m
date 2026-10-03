function build_control(s)
%BUILD_CONTROL Build a Simulink library containing a control system.
%   BUILD_CONTROL(S) compiles the S-function of the control system, builds the
%   library S.name, and saves it in the folder S.folder. The struct S is written by
%   motulator_export.simulink, with the fields
%
%     name      Library name
%     folder    Folder of the library, the S-function, and its compiled version
%     control   Control system, see blocks.add_control_system
%
%   The library contains only the masked subsystem of the control system, to be used
%   in other models, e.g., with the I/O blocks of a real-time target. The source of
%   the S-function contains the C port, so the folder has all the files needed.

blocks.compile(s.control.sfunction, s.folder);
name = s.name;
if bdIsLoaded(name)
    close_system(name, 0);
end
slx = fullfile(s.folder, [name '.slx']);
if isfile(slx)
    delete(slx);
end
addpath(s.folder);
new_system(name, 'Library');
blocks.add_control_system([name '/' s.control.name], s.control, [160 100 300 300]);
save_system(name, slx);
close_system(name);
fprintf('Saved %s\n', slx);
end
