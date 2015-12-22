# memlib

mm.c - simple light-weight memory allocation package. implementation based on first fit
search of a segregated fits table. seg_lists (segregated fits table header) is an array
of ptrs to linkedlists of free blocks for each size class. each ptr can be a null ref or a
memory address of the next n+1 free block's payload. storing addresses in payload requires a
16 byte block size (including 4 byte header and 4 byte footer). table lookup is based on naive
hash to find ideal class size. from this point classes are traversed until first fit is found.
this strategy combined with block splitting leads to high throughput and utilization performance
of a best bit strategy.
