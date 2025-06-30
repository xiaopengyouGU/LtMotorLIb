max_val = 12;
type = 1;
foc = lt_foc(max_val,type);
fd = 1;
fq = 6;
dir = 0;
foc.process(fd,fq,dir);
foc.plot();