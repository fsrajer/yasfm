function pt = isectLines(l1, l2)

temp = cross(l1, l2);
pt = [temp(1); temp(2)]/temp(3);

end