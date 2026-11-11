"""在仿真 ARM 上执行 Q5 固件 C 回归测试；绝不接触硬件。"""
import argparse
import sys
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('elf')
parser.add_argument('--deps', default=str(Path(__file__).resolve().parents[1] / 'build/testdeps'))
args = parser.parse_args()
sys.path.insert(0, args.deps)
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_PC

uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
uc.mem_map(0, 0x200000)
uc.mem_map(0x20000000, 0x20000)
with open(args.elf, 'rb') as stream:
    elf = ELFFile(stream)
    for segment in elf.iter_segments():
        if segment['p_type'] == 'PT_LOAD':
            uc.mem_write(segment['p_vaddr'], segment.data())
    entry = elf.header['e_entry']
    symbols = {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
uc.reg_write(UC_ARM_REG_SP, 0x2001F000)
uc.reg_write(UC_ARM_REG_LR, 0x100001)
uc.emu_start(entry | 1, 0x100000, count=180000000)
counts = {name: int.from_bytes(uc.mem_read(symbols[name], 4), 'little')
          for name in ('checks', 'failures', 'first_failure_line')}
n_lines = int.from_bytes(uc.mem_read(symbols['failure_lines_count'], 4), 'little')
lines = [int.from_bytes(uc.mem_read(symbols['failure_lines'] + 4*i, 4), 'little')
         for i in range(min(n_lines, 32))]
counts['failure_lines'] = lines
print(counts)
if uc.reg_read(UC_ARM_REG_PC) != 0x100000:
    raise SystemExit(f'test did not return; PC={uc.reg_read(UC_ARM_REG_PC):#x}')
raise SystemExit(1 if uc.reg_read(UC_ARM_REG_R0) else 0)
