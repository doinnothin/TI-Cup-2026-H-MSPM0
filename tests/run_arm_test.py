"""在仿真 ARM Cortex-M 上执行已编译的 C 控制测试，无需硬件。"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'build/testdeps'))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_PC
uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
uc.mem_map(0, 0x200000)
uc.mem_map(0x20000000, 0x20000)
with open(sys.argv[1], 'rb') as source:
    elf = ELFFile(source)
    for segment in elf.iter_segments():
        if segment['p_type'] == 'PT_LOAD':
            uc.mem_write(segment['p_vaddr'], segment.data())
    entry = elf.header['e_entry']
uc.reg_write(UC_ARM_REG_SP, 0x2001F000)
uc.reg_write(UC_ARM_REG_LR, 0x100001)
uc.emu_start(entry | 1, 0x100000, count=30000000)
assert uc.reg_read(UC_ARM_REG_PC) == 0x100000, 'test did not return (instruction budget exhausted)'
result = uc.reg_read(UC_ARM_REG_R0)
print('C control regression failures:', result)
sys.exit(1 if result else 0)
