import os,sys,math

INT32_MAX = (1<<31)-1
UINT32_MAX = (1<<32)-1

ARRAY_SIZE = 64
TEST_CSV_FILE = "test_q.csv"

test_q_angle_cordic = [0x00000000, 0x04000000, 0x08000000, 0x0C000000,
  0x10000000, 0x14000000, 0x18000000, 0x1C000000,
  0x20000000, 0x24000000, 0x28000000, 0x2C000000,
  0x30000000, 0x34000000, 0x38000000, 0x3C000000,
  0x40000000, 0x44000000, 0x48000000, 0x4C000000,
  0x50000000, 0x54000000, 0x58000000, 0x5C000000,
  0x60000000, 0x64000000, 0x68000000, 0x6C000000,
  0x70000000, 0x74000000, 0x78000000, 0x7C000000,
  0x80000000, 0x84000000, 0x88000000, 0x8C000000,
  0x90000000, 0x94000000, 0x98000000, 0x9C000000,
  0xA0000000, 0xA4000000, 0xA8000000, 0xAC000000,
  0xB0000000, 0xB4000000, 0xB8000000, 0xBC000000,
  0xC0000000, 0xC4000000, 0xC8000000, 0xCC000000,
  0xD0000000, 0xD4000000, 0xD8000000, 0xDC000000,
  0xE0000000, 0xE4000000, 0xE8000000, 0xEC000000,
  0xF0000000, 0xF4000000, 0xF8000000, 0xFC000000]

test_q_angle_cmsis_dsp = [  0x00000000, 0x02000000, 0x04000000, 0x06000000,
  0x08000000, 0x0A000000, 0x0C000000, 0x0E000000,
  0x10000000, 0x12000000, 0x14000000, 0x16000000,
  0x18000000, 0x1A000000, 0x1C000000, 0x1E000000,
  0x20000000, 0x22000000, 0x24000000, 0x26000000,
  0x28000000, 0x2A000000, 0x2C000000, 0x2E000000,
  0x30000000, 0x32000000, 0x34000000, 0x36000000,
  0x38000000, 0x3A000000, 0x3C000000, 0x3E000000,
  0x40000000, 0x42000000, 0x44000000, 0x46000000,
  0x48000000, 0x4A000000, 0x4C000000, 0x4E000000,
  0x50000000, 0x52000000, 0x54000000, 0x56000000,
  0x58000000, 0x5A000000, 0x5C000000, 0x5E000000,
  0x60000000, 0x62000000, 0x64000000, 0x66000000,
  0x68000000, 0x6A000000, 0x6C000000, 0x6E000000,
  0x70000000, 0x72000000, 0x74000000, 0x76000000,
  0x78000000, 0x7A000000, 0x7C000000, 0x7E000000]

test_q_ref_sin = [  0x00000000, 0x0C8BD35E, 0x18F8B83C, 0x25280C5D,
  0x30FBC54D, 0x3C56BA70, 0x471CECE6, 0x5133CC94,
  0x5A827999, 0x62F201AC, 0x6A6D98A4, 0x70E2CBC6,
  0x7641AF3C, 0x7A7D055B, 0x7D8A5F3F, 0x7F62368F,
  0x80000000, 0x7F62368F, 0x7D8A5F3F, 0x7A7D055B,
  0x7641AF3C, 0x70E2CBC6, 0x6A6D98A4, 0x62F201AC,
  0x5A827999, 0x5133CC94, 0x471CECE6, 0x3C56BA70,
  0x30FBC54D, 0x25280C5D, 0x18F8B83C, 0x0C8BD35E,
  0x00000000, 0xF3742CA2, 0xE70747C4, 0xDAD7F3A3,
  0xCF043AB3, 0xC3A94590, 0xB8E3131A, 0xAECC336C,
  0xA57D8667, 0x9D0DFE54, 0x9592675C, 0x8F1D343A,
  0x89BE50C4, 0x8582FAA5, 0x8275A0C1, 0x809DC971,
  0x80000000, 0x809DC971, 0x8275A0C1, 0x8582FAA5,
  0x89BE50C4, 0x8F1D343A, 0x9592675C, 0x9D0DFE54,
  0xA57D8667, 0xAECC336C, 0xB8E3131A, 0xC3A94590,
  0xCF043AB3, 0xDAD7F3A3, 0xE70747C4, 0xF3742CA2]

test_q_ref_cos = [  0x80000000, 0x7F62368F, 0x7D8A5F3F, 0x7A7D055B,
  0x7641AF3C, 0x70E2CBC6, 0x6A6D98A4, 0x62F201AC,
  0x5A827999, 0x5133CC94, 0x471CECE6, 0x3C56BA70,
  0x30FBC54D, 0x25280C5D, 0x18F8B83C, 0x0C8BD35E,
  0x00000000, 0xF3742CA2, 0xE70747C4, 0xDAD7F3A3,
  0xCF043AB3, 0xC3A94590, 0xB8E3131A, 0xAECC336C,
  0xA57D8667, 0x9D0DFE54, 0x9592675C, 0x8F1D343A,
  0x89BE50C4, 0x8582FAA5, 0x8275A0C1, 0x809DC971,
  0x80000000, 0x809DC971, 0x8275A0C1, 0x8582FAA5,
  0x89BE50C4, 0x8F1D343A, 0x9592675C, 0x9D0DFE54,
  0xA57D8667, 0xAECC336C, 0xB8E3131A, 0xC3A94590,
  0xCF043AB3, 0xDAD7F3A3, 0xE70747C4, 0xF3742CA2,
  0x00000000, 0x0C8BD35E, 0x18F8B83C, 0x25280C5D,
  0x30FBC54D, 0x3C56BA70, 0x471CECE6, 0x5133CC94,
  0x5A827999, 0x62F201AC, 0x6A6D98A4, 0x70E2CBC6,
  0x7641AF3C, 0x7A7D055B, 0x7D8A5F3F, 0x7F62368F]

test_q_cordic_result = [0x00000600, 0x7FFFFE00,
0x0C8BD600, 0x7F623100,
0x18F8B700, 0x7D8A5A00,
0x25280600, 0x7A7D0700,
0x30FBC900, 0x7641A800,
0x3C56B400, 0x70E2CA00,
0x471CEC00, 0x6A6D9900,
0x5133C500, 0x62F20400,
0x5A827600, 0x5A827900,
0x62F20600, 0x5133C300,
0x6A6D9900, 0x471CEC00,
0x70E2CA00, 0x3C56B400,
0x7641AC00, 0x30FBC300,
0x7A7D0700, 0x25280600,
0x7D8A5A00, 0x18F8B500,
0x7F623100, 0x0C8BD400,
0x7FFFFF00, 0xFFFFFF00,
0x7F623200, 0xF3742900,
0x7D8A5B00, 0xE7074A00,
0x7A7D0800, 0xDAD7F100,
0x7641A800, 0xCF043C00,
0x70E2CB00, 0xC3A94B00,
0x6A6D9900, 0xB8E31100,
0x62F20400, 0xAECC3200,
0x5A827A00, 0xA57D8500,
0x5133C400, 0x9D0E0300,
0x471CED00, 0x95926A00,
0x3C56B500, 0x8F1D3700,
0x30FBC400, 0x89BE5500,
0x25280500, 0x85830100,
0x18F8B400, 0x8275A400,
0x0C8BD300, 0x809DCF00,
0xFFFFFA00, 0x80000200,
0xF3742A00, 0x809DCF00,
0xE7074900, 0x8275A600,
0xDAD7FA00, 0x8582F900,
0xCF043700, 0x89BE5800,
0xC3A94C00, 0x8F1D3600,
0xB8E31400, 0x95926700,
0xAECC3B00, 0x9D0DFC00,
0xA57D8A00, 0xA57D8700,
0x9D0DFA00, 0xAECC3D00,
0x95926700, 0xB8E31400,
0x8F1D3600, 0xC3A94C00,
0x89BE5400, 0xCF043D00,
0x8582F900, 0xDAD7FA00,
0x8275A600, 0xE7074B00,
0x809DCF00, 0xF3742C00,
0x80000100, 0x00000100,
0x809DCE00, 0x0C8BD700,
0x8275A500, 0x18F8B600,
0x8582F800, 0x25280F00,
0x89BE5800, 0x30FBC400,
0x8F1D3500, 0x3C56B500,
0x95926700, 0x471CEF00,
0x9D0DFC00, 0x5133CE00,
0xA57D8600, 0x5A827B00,
0xAECC3C00, 0x62F1FD00,
0xB8E31300, 0x6A6D9600,
0xC3A94B00, 0x70E2C900,
0xCF043C00, 0x7641AB00,
0xDAD7FB00, 0x7A7CFF00,
0xE7074C00, 0x7D8A5C00,
0xF3742D00, 0x7F623100,]

test_q_cmsis_result = [0x00000000, 0x7FFFFFFE,
0x0C8BD35E, 0x7F62368E,
0x18F8B83C, 0x7D8A5F40,
0x25280C5E, 0x7A7D055A,
0x30FBC54C, 0x7641AF3C,
0x3C56BA70, 0x70E2CBC6,
0x471CECE6, 0x6A6D98A4,
0x5133CC94, 0x62F201AC,
0x5A82799A, 0x5A82799A,
0x62F201AC, 0x5133CC94,
0x6A6D98A4, 0x471CECE6,
0x70E2CBC6, 0x3C56BA70,
0x7641AF3C, 0x30FBC54C,
0x7A7D055A, 0x25280C5E,
0x7D8A5F40, 0x18F8B83C,
0x7F62368E, 0x0C8BD35E,
0x7FFFFFFE, 0x00000000,
0x7F62368E, 0xF3742CA2,
0x7D8A5F40, 0xE70747C4,
0x7A7D055A, 0xDAD7F3A2,
0x7641AF3C, 0xCF043AB2,
0x70E2CBC6, 0xC3A94590,
0x6A6D98A4, 0xB8E31318,
0x62F201AC, 0xAECC336C,
0x5A82799A, 0xA57D8666,
0x5133CC94, 0x9D0DFE54,
0x471CECE6, 0x9592675C,
0x3C56BA70, 0x8F1D343A,
0x30FBC54C, 0x89BE50C2,
0x25280C5E, 0x8582FAA4,
0x18F8B83C, 0x8275A0C0,
0x0C8BD35E, 0x809DC970,
0x00000000, 0x80000000,
0xF3742CA2, 0x809DC970,
0xE70747C4, 0x8275A0C0,
0xDAD7F3A2, 0x8582FAA4,
0xCF043AB2, 0x89BE50C2,
0xC3A94590, 0x8F1D343A,
0xB8E31318, 0x9592675C,
0xAECC336C, 0x9D0DFE54,
0xA57D8666, 0xA57D8666,
0x9D0DFE54, 0xAECC336C,
0x9592675C, 0xB8E31318,
0x8F1D343A, 0xC3A94590,
0x89BE50C2, 0xCF043AB2,
0x8582FAA4, 0xDAD7F3A2,
0x8275A0C0, 0xE70747C4,
0x809DC970, 0xF3742CA2,
0x80000000, 0x00000000,
0x809DC970, 0x0C8BD35E,
0x8275A0C0, 0x18F8B83C,
0x8582FAA4, 0x25280C5E,
0x89BE50C2, 0x30FBC54C,
0x8F1D343A, 0x3C56BA70,
0x9592675C, 0x471CECE6,
0x9D0DFE54, 0x5133CC94,
0xA57D8666, 0x5A82799A,
0xAECC336C, 0x62F201AC,
0xB8E31318, 0x6A6D98A4,
0xC3A94590, 0x70E2CBC6,
0xCF043AB2, 0x7641AF3C,
0xDAD7F3A2, 0x7A7D055A,
0xE70747C4, 0x7D8A5F40,
0xF3742CA2, 0x7F62368E,]

def main():
    f_output = open(TEST_CSV_FILE, mode='wb')

    # The CORDIC Q Format input
    tmp_str = ("The CORDIC Q Format input\n")
    f_output.write(bytes(tmp_str, encoding="ansi"))
    
    for q in test_q_angle_cordic:
        if(q > INT32_MAX):
            q = q - (1<<32)
        f = q/(1<<31)
        tmp_str = ("%08X , %f\n" % (q, f))
        f_output.write(bytes(tmp_str, encoding="ansi"))            
        pass
    
    tmp_str = ("\n")
    f_output.write(bytes(tmp_str, encoding="ansi"))

    # The CMSIS DSP Q Format input
    tmp_str = ("The CMSIS DSP Q Format input\n")
    f_output.write(bytes(tmp_str, encoding="ansi"))    
    for q in test_q_angle_cmsis_dsp:
        if(q > INT32_MAX):
            q = q - (1<<32)        
        f = q/(1<<31)
        tmp_str = ("%08X , %f\n" % (q, f))
        f_output.write(bytes(tmp_str, encoding="ansi"))
        pass    
    pass

    tmp_str = ("\n")
    f_output.write(bytes(tmp_str, encoding="ansi"))

    # The Ref Sine Q Format input
    tmp_str = ("The Ref Sine Q Format input\n")
    f_output.write(bytes(tmp_str, encoding="ansi"))     
    for q in test_q_ref_sin:
        if(q > INT32_MAX):
            q = q - (1<<32)        
        f = q/(1<<31)
        tmp_str = ("%08X , %f\n" % (q, f))
        f_output.write(bytes(tmp_str, encoding="ansi"))
        pass    
    pass
    tmp_str = ("\n")
    f_output.write(bytes(tmp_str, encoding="ansi"))

    # The Ref Cos Q Format input
    tmp_str = ("The Ref Cos Q Format input\n")
    f_output.write(bytes(tmp_str, encoding="ansi"))    
    for q in test_q_ref_cos:
        if(q > INT32_MAX):
            q = q - (1<<32)        
        f = q/(1<<31)
        tmp_str = ("%08X , %f\n" % (q, f))
        f_output.write(bytes(tmp_str, encoding="ansi"))
        pass    
    pass
    tmp_str = ("\n")
    f_output.write(bytes(tmp_str, encoding="ansi"))

    # The Result of Cordic Calculation    
    tmp_str = ("The Result of Cordic Calculation\n")
    f_output.write(bytes(tmp_str, encoding="ansi"))
    res_len = len(test_q_cordic_result)
    for i in range(res_len//2):
        q = test_q_cordic_result[2*i]
        r = test_q_cordic_result[1 + (2*i)]
        
        if(q > INT32_MAX):
            q = q - (1<<32)        
        f = q/(1<<31)
        if(r > INT32_MAX):
            r = r - (1<<32)        
        g = r/(1<<31)
        
        tmp_str = ("%08X , %f, %08X , %f\n" % (q, f, r, g))
        f_output.write(bytes(tmp_str, encoding="ansi"))
        pass    
    pass
    tmp_str = ("\n")
    f_output.write(bytes(tmp_str, encoding="ansi"))

    # The Result of CMSIS DSP Calculation    
    tmp_str = ("The Result of CMSIS DSP Calculation\n")
    f_output.write(bytes(tmp_str, encoding="ansi"))
    res_len = len(test_q_cmsis_result)
    for i in range(res_len//2):
        q = test_q_cordic_result[2*i]
        r = test_q_cordic_result[1 + (2*i)]
        
        if(q > INT32_MAX):
            q = q - (1<<32)        
        f = q/(1<<31)
        if(r > INT32_MAX):
            r = r - (1<<32)        
        g = r/(1<<31)
        
        tmp_str = ("%08X , %f, %08X , %f\n" % (q, f, r, g))
        f_output.write(bytes(tmp_str, encoding="ansi"))
        pass    
    pass
    tmp_str = ("\n")
    f_output.write(bytes(tmp_str, encoding="ansi"))

    #Generate Ref Sine output
    print("The generated sine result")
    for i in range(ARRAY_SIZE):
        ang = (i * math.pi * 2)/ARRAY_SIZE

        d_sin_res = math.sin(ang)
        q_1_31_t = int(d_sin_res * (UINT32_MAX) / 2)
        if(q_1_31_t < 0):
            q_1_31_t = q_1_31_t + (1<<32)
        #print("0x%08X, %f" % (q_1_31_t, d_sin_res))
        print("0x%08X, " % q_1_31_t, end='')
        if(0 == (i+1)%4):
            print()
        pass
    print()
    
    print("The generated cos result")
    for i in range(ARRAY_SIZE):
        ang = (i * math.pi * 2)/ARRAY_SIZE
        
        d_cos_res = math.cos(ang)
        q_1_31_t = int(d_cos_res * (UINT32_MAX) / 2)
        if(q_1_31_t < 0):
            q_1_31_t = q_1_31_t + (1<<32)
        #print("0x%08X, %f" % (q_1_31_t, d_cos_res))
        print("0x%08X, " % q_1_31_t, end='')
        if(0 == (i+1)%4):
            print()
        pass
    print()
     
if __name__ == '__main__':
    main()
