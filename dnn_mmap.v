module DNN_MMAP (
    input clk,
    input en,
    input rst_n,
    input [31:0] addr,
    input valid,
    input [3:0] wstrb,
    output reg ready,
    input [31:0] wdata,
    output reg [31:0] rdata,
    
    // Memory interface
    input mem_ready_0,            //
    output mem_valid_0,           //
    output [31:0] mem_addr_0,     // input_addr
    input [31:0] mem_rdata_0,     // input_rdata
    output mem_write_0,           // 
    output [31:0] mem_wdata_0,    // 

    input mem_ready_1,            //
    output mem_valid_1,           //
    output [31:0] mem_addr_1,     // weight_addr
    input [31:0] mem_rdata_1,     // weight_rdata
    output mem_write_1,           //
    output [31:0] mem_wdata_1,    //

    input mem_ready_2,            //
    output mem_valid_2,           //
    output [31:0] mem_addr_2,     // output_addr
    input [31:0] mem_rdata_2,     // output_rdata
    output mem_write_2,           // output_wen
    output [31:0] mem_wdata_2     // output_wdata
);

// DNN_MMAP's address mapping
// TODO 0
// Assign your own memory mapping here
// I have defined some addresses which you may use for your accelerator.
// You can add additional addresses if necessary.

// Keep your memory address alligned to 4 bytes, so you won't run into any trouble.
// Each memory address represents 1 byte, so each MMAP mapping jumps at least 4 bytes = 32bits
// e.g. 0x0, 0x4, 0x8, 0xc
parameter DNN_MMAP_BASE = 32'h4000_0000;
parameter DNN_MMAP_RANG = 32'h0000_ffff;
parameter DNN_MMAP_READ_FINISH = 32'h0000_0000;
parameter DNN_MMAP_WRITE_CONV = 32'h0000_0004; // jump 4bytes = 32bits
parameter DNN_MMAP_WRITE_N  =   32'h0000_0008;
parameter DNN_MMAP_WRITE_C  =   32'h0000_000c;
parameter DNN_MMAP_WRITE_H  =   32'h0000_0010;
parameter DNN_MMAP_WRITE_W  =   32'h0000_0014;
parameter DNN_MMAP_WRITE_R  =   32'h0000_0018;
parameter DNN_MMAP_WRITE_S  =   32'h0000_001c;
parameter DNN_MMAP_WRITE_M  =   32'h0000_0020;
parameter DNN_MMAP_WRITE_P  =   32'h0000_0024;
parameter DNN_MMAP_WRITE_Q  =   32'h0000_0028;
parameter DNN_MMAP_INPUT_OFFSET	 = 32'h0000_002c;
parameter DNN_MMAP_WEIGHT_OFFSET = 32'h0000_0030;
parameter DNN_MMAP_OUTPUT_OFFSET = 32'h0000_0034;
parameter DNN_MMAP_WRITE_START   = 32'h0000_0038;

// Accelerator register
reg start, conv, finish;
reg [31:0] N, C, H, W, R, S, M, P, Q;
reg [31:0] input_offset, weight_offset, output_offset;
wire Ready;

// Internal masked address
wire [31:0] dnn_addr;
assign dnn_addr = (addr) & DNN_MMAP_RANG;

// Accelerator memory port
wire [15:0] input_addr, weight_addr, output_addr;
wire [31:0] output_rdata, output_wdata;
wire [31:0] input_rdata, weight_rdata;
wire output_wen;

// Memory port 1
// TODO 1
// This is for input data in memory.
// The following assignments are for demonstration.
// You need to assign the correct values.
assign mem_valid_0 = 1;
assign mem_addr_0 = 0;
assign mem_write_0 = 0;
assign mem_wdata_0 = 0;
assign input_rdata = 0;

// Memory port 2
// TODO 2
// This is for weight data in memory.
// The following assignments are for demonstration.
// You need to assign the correct values.
assign mem_valid_1 = 1;
assign mem_addr_1 = 0;
assign mem_write_1 = 0;
assign mem_wdata_1 = 0;
assign weight_rdata = 0;

// Memory port 3
// TODO 3
// This is for output data in memory.
// The following assignments are for demonstration.
// You need to assign the correct values.
// Hint: Since the read/write data is 32bits,
// this memory address will be different to mem_addr_0 and mem_addr_1.
assign mem_valid_2 = 1;
assign mem_addr_2 = 0;
assign mem_write_2 = 0;
assign mem_wdata_2 = 0;
assign output_rdata = 0;

/**
 *	This block handles the MMAP request.
 *
 *	Signal description:
 *	[31:0] addr:	MMAP request address
 *	[31:0] wdata:	Data write from master
 *	[31:0] rdata:	Data read from module
 *	[3:0] wstrb:	each bit enables 8-bit write to the 32-bit data
 *	valid:		MMAP request from master
 *	ready:		MMAP request is handled
 */
always @(posedge clk or negedge rst_n)
begin
    if (!rst_n) begin
        start <= 0;
        ready <= 0;
        finish <= 0;
        conv <= 0;
        N <= 0;
        C <= 0;
        H <= 0; 
        W <= 0;
        R <= 0;
        S <= 0;
        M <= 0;
        P <= 0;
        Q <= 0;
        input_offset <= 0;
        weight_offset <= 0;
        output_offset <= 0;
    end else begin
        ready <= 0;
        rdata <= 0;
        finish <= (finish == 0) ? Ready : 1;
        start <= (finish == 1) ? 0 : start;
        if (en && valid && !ready) begin
            if (!wstrb) begin   //read
                case (dnn_addr)
		            // TODO 4
                    // Implement your MMAP read routine here
                    DNN_MMAP_READ_FINISH: begin
                        rdata <= finish;
                        ready <= 1;
                        finish <= (finish == 1) ? 0 : finish;
                    end
                    default: $display("DNN_MMAP: read invalid reg: %h(%h)", dnn_addr, addr);
                endcase
            end else begin      //write
                case (dnn_addr)
		            // TODO 5
                    // Implement your MMAP write routine here
                    DNN_MMAP_WRITE_CONV: begin
                        conv <= wdata[0];
                        ready <= 1;
                    end

                    DNN_MMAP_WRITE_N: begin
                        N <= wdata;
                        ready <= 1;
                    end

                    default: $display("DNN_MMAP: write invalid reg: %h(%h)", dnn_addr, addr);
                endcase
            end
        end
    end
end

// your homework 3 design
top top01(
    .clk(clk),
    .rst_n(rst_n),
    .valid(start),
    .conv(conv),
    .Wait(), // we only use in pcpi
    .Ready(Ready),
    .input_addr(input_addr),
    .input_rdata(input_rdata),
    .weight_addr(weight_addr),
    .weight_rdata(weight_rdata),
    .output_wen(output_wen),
    .output_addr(output_addr),
    .output_rdata(output_rdata),
    .output_wdata(output_wdata),
    .N(N),
    .C(C),
    .H(H),
    .W(W),
    .R(R),
    .S(S),
    .M(M),
    .P(P),
    .Q(Q)
);

endmodule

// TODO 6
// You can replace the top module with your design in HW #3 here.
module top (
    input clk,
    input rst_n,
    input valid,
    input conv,
    output reg Wait,
    output reg Ready,
    output [15:0] input_addr,
    input [31:0] input_rdata,
    output [15:0] weight_addr,
    input [31:0] weight_rdata,
    output output_wen,
    output [15:0] output_addr,
    input  signed [31:0] output_rdata,
    output signed [31:0] output_wdata,
    input [31:0] N,
    input [31:0] C,
    input [31:0] H,
    input [31:0] W,
    input [31:0] R,
    input [31:0] S,
    input [31:0] M,
    input [31:0] P,
    input [31:0] Q
);

// Replace this block with your code.
always @(posedge clk) begin
    Ready <= valid;
end

endmodule
