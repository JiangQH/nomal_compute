function params = readParamsFromAct(act_file)
    fid = fopen(act_file, 'r');
    tline = fgetl(fid);
    while ischar(tline)
        if strfind(tline, '<intrinsic parameter>')
            params = fscanf(fid, '%f %f %f %f %f %f');
            break;
        end
        tline = fgetl(fid);
    end
    fclose(fid);
    params = params(1:4);
    params = params';
end