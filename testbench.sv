module testbench(output logic reset);    
    
	initial begin
		   reset = 1; 
		#2 reset = 0;
	end

endmodule

module top;
	logic[7:0] opcode1, opcode2;
	logic clk, reset, pc_clk;
	logic[7:0] rom_address;
	initial begin
		$dumpfile("dump.vcd");
		$dumpvars(1);
		#50 $finish;
	end
	initial begin
		clk = 1;
		forever #1 clk = ~clk;
	end
  
  	initial begin
		pc_clk = 1;
		forever #2 pc_clk = ~pc_clk;
	end


	testbench t (.reset(reset));
	CPU cpu (.opcode1(opcode1), .opcode2(opcode2), .clk(clk), .reset(reset), .rom_address(rom_address),.pc_clk(pc_clk));
	ROM rom (.rom_address(rom_address), .rom_data1(opcode1), .rom_data2(opcode2));
endmodule


// // opcode 1+2
module ROM (input logic [7:0] rom_address, output logic [7:0] rom_data1, rom_data2);
	logic[7:0] rom [0:255];

	initial begin
      $monitor("ROM time=%d rom_address=%b", $time, rom_address);
	end

	// assign rom
	initial begin
		// load reg 0000 with imm 0000_0011 // multiplicant
        rom[0] = 8'b0001_0000;	rom[1] = 8'b0000_0011;

		// load reg 0001 with imm 0000_0010 // multiplier
        rom[2] = 8'b0001_0001;	rom[3] = 8'b0000_0010;

		// load reg 0010 with imm 0000_0000 // result
        rom[4] = 8'b0001_0010;	rom[5] = 8'b0000_0000;

		// load reg 0011 with imm 0000_0001 // 1
        rom[6] = 8'b0001_0011;	rom[7] = 8'b0000_0001;

		// reg 0000 + 0010 = reg 0010
		// multiplicant + result = result
        rom[8] = 8'b1000_0000;	rom[9] = 8'b0010_0010;
		
		// reg 0001 - reg 0011 = reg 0001
		// multiplier - 1 = multiplier
		rom[10] = 8'b1001_0001;	rom[11] = 8'b0011_0001;
		
		// jump to 8 if mutiplier not zero
		rom[12] = 8'b0100_0110; rom[13] = 8'b0000_1000;

		// store reg 0010 to mem 0000_0000
        rom[14] = 8'b0001_0010;	rom[15] = 8'b0000_0000;

	end

	always_comb begin
		rom_data1 = rom[rom_address];
		rom_data2 = (rom_address > 254) ? 8'b0000_0000:rom[rom_address+1];
	end
endmodule