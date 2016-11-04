function [depth, mask] = read_depth(depth_file, sizes)
    fid = fopen(depth_file, 'rb');
    depth = fread(fid, 'float');
    fclose(fid);
    depth = reshape(depth, sizes(2), sizes(1));
    depth = depth';
    mask = (depth > 0);
end