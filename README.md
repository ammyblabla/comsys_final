อ่าน design2.sv

PC flipflop เก็บ counter ไว้ส่งให้ rom
ROM - Instruction memory / เรียกที่ testbench ทีเดียว
	ส่ง opcode1, 2 ออกมา //opcode2 = rom[PC+1]
ต่อ opcode แบบมัดมือชก
	controller opcode1[7:4]
	branch-jump control opcode1[3:0], opcode2
ALU func=opcode[6:4] 
	อาจจะมีโมดูลเรียกเลขจาก register หรือไม่ก็เรียกใน ALU เลย
	reg1=opcode1[3:0] reg2=opcode2[7:4] reg3=opcode[3:0]
	aluOp = (opcode1[7:6] == 2'b10) ? 1:0;
RAM
module RAM256x8 (inout logic[7:0] ram_data, input logic[7:0] ram_address, input logic n_cs, n_oe, n_we, clk);

	go to controller
	n_cs, n_oe, n_we
	n_cs = not chip select
		0 = store-load 
		1 = alu, jump
	n_oe = not read
		0 = load
		1 = store, alu, jump
	n_we = not write
		0 = store
		1 = , alu, store
REG
	regWrite
		0 = store, jump
		1 = alu, load
	reg address
		reg_read_addr1 = opcode1[3:0]
		reg_read_addr2 = opcode2[7:4]
		reg_write_addr 
			regdest
			1 = load opcode1[3:0]
			0 = alu opcode2[3:0]
	reg data
		reg_d_out1 = reg[opcode1[3:0]]
		reg_d_out2 = reg[opcode2[7:4]]
		reg_d_in 
			memToReg
			00 = load immediate opcode2
			01 = load ram[opcode2]
			10 = ALU output
JUMP
	jumpcond = (opcode1[7:4] == 2'b0100)?1:0;
	go to jump module
	0 = PC 1 = opcode2
ADDER PC = PC+1
MUX2to1 4 bit
MUX4to1 8 bit