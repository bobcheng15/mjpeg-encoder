// This is free and unencumbered software released into the public domain.
//
// Anyone is free to copy, modify, publish, use, compile, sell, or
// distribute this software, either in source code form or as a compiled
// binary, for any purpose, commercial or non-commercial, and by any
// means.

#include "firmware.h"

//pseudo random number generator
static uint32_t xorshift32(void) {
	static uint32_t x = 314159265;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	return x;
}

void multest(void)
{
	for(int i = 0; i < 3; i++){
	uint32_t a = xorshift32();
	uint32_t b = xorshift32();

	uint64_t au = a, bu = b;
	int64_t as = (int32_t)a, bs = (int32_t)b;
	uint32_t start;
	uint32_t latency[8];
       
	print_str("input [");
	print_hex(as >> 32, 8);
	print_str("] ");
	print_hex(a, 8);
	print_str(" [");
	print_hex(bs >> 32, 8);
	print_str("] ");
	print_hex(b, 8);
	print_chr('\n');
	
	print_str("            mul      mulh    mulhsu     mulhu\n");

	uint32_t h_mul, h_mulh, h_mulhsu, h_mulhu;
		
	print_str("hard   ");
		
	start = tick();
	h_mul = hard_mul(a, b);
	latency[0] = tick() - start;
	print_hex(h_mul, 8);
	print_str("  ");

	start = tick();
	h_mulh = hard_mulh(a, b);
	latency[1] = tick() - start;
	print_hex(h_mulh, 8);
	print_str("  ");

	start = tick();
	h_mulhsu = hard_mulhsu(a, b);
	latency[2] = tick() - start;
	print_hex(h_mulhsu, 8);
	print_str("  ");

	start = tick();
	h_mulhu = hard_mulhu(a, b);
	latency[3] = tick() - start;
	print_hex(h_mulhu, 8);
		
	print_str("\ntime        ");
	for(int j = 0; j < 4; j++){
		print_dec(latency[j]);
		print_str("       ");
	}
	print_chr('\n');
		
	uint32_t s_mul, s_mulh, s_mulhsu, s_mulhu;
	print_str("soft   ");

	start = tick();
	s_mul = a * b;
	latency[4] = tick() - start;
	print_hex(s_mul, 8);
	print_str("  ");

	start = tick();
	s_mulh = (as * bs) >> 32;
	latency[5] = tick() - start;
	print_hex(s_mulh, 8);
	print_str("  ");

	start = tick();
	s_mulhsu = (as * bu) >> 32;
	latency[6] = tick() - start;
	print_hex(s_mulhsu, 8);
	print_str("  ");

	start = tick();
	s_mulhu = (au * bu) >> 32;
	latency[7] = tick() - start;
	print_hex(s_mulhu, 8);
	print_str("  ");
	
	print_str("\ntime        ");
	for(int j = 4; j < 8; j++){
		print_dec(latency[j]);
		print_str("      ");
	}
	print_str("\n\n");
		
	if (s_mul != h_mul || s_mulh != h_mulh || s_mulhsu != h_mulhsu || s_mulhu != h_mulhu) {
		print_str("ERROR!\n");
		__asm__ volatile ("ebreak");
		return;
	}
}

	print_str(" OK\n");

}

