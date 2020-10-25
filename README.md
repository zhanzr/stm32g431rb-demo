readelf -a -W stm32g431_t1.axf > /d/tmp_elf_out.txt

objdump stm32g431_t1.axf -x -D -S -t > /d/tmp_objdump.txt
