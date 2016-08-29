function R = rot(axis, angle)
% ROT Returns a rotation matrix for given axis and angle
% intrinsic rotations x-y'-z'' by angles a, b, c are equivalent
% to the extrinsic rotations z-y-x by angles c, b, a. Both are
% represented by a matrix rot('x',a)*rot('y',b)*rot('z',c)
% Ref: https://en.wikipedia.org/wiki/Euler_angles
%
% Example: R = rot('x', pi/4) * rot('y', pi/3)
%

c = cos(angle); s = sin(angle);
switch lower(axis)
    case 'x'
        R = [1 0 0; 0 c -s; 0 s c];
    case 'y'
        R = [c 0 s; 0 1 0; -s 0 c];
    case 'z'
        R = [c -s 0; s c 0; 0 0 1];
    otherwise
        error('Invalid input to function')
end

end
