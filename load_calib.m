function [P, Tr] = load_calib(calibfile)
    fid = fopen(calibfile);
    p0_line = fgets(fid);
    p1_line = fgets(fid);
    p2_line = fgets(fid);
    p3_line = fgets(fid);
    tr_line = fgets(fid);
    p2 = str2num(p2_line(5:end));
    Tr = str2num(tr_line(5:end));
    P = reshape(p2, [4, 3]);
    P = P';
    Tr = reshape(Tr, [4, 3]);
    Tr = Tr';
end