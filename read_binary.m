file = '389.txt';
w = 1241;
h = 376;

fid = fopen(file, 'rb');
A = fread(fid, 'float');
fclose(fid);

count = 1;
data = zeros(h, w, 3);
for cc = 1 : 3
    for ww = 1 : w
        for hh = 1 : h
            data(hh, ww, cc) = A(count);
            count = count + 1;
        end
    end
end
imshow(uint8((data/2+0.5)*255));