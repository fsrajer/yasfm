function outList = readFilenames(file)

outList = [];
fid = fopen(file, 'r');
line = fgetl(fid);

while ischar(line)
    outList{end+1} = line;
    line = fgetl(fid);
end
fclose(fid);

end